import simpy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rc("font",family='MicroSoft YaHei',weight="bold")

class VTOLPort:
    def __init__(self, env, config):
        self.env = env
        self.config = config

        # 初始化四种流程服务率
        self.mu_vl = 1 / max(config['ssd_v']/config['V_vl'] + config['t_land'], config['L_vl']/config['V_vl'] + config['t_land'])
        self.mu_vt = 1 / max(config['ssd_v']/config['V_vt'] + config['t_takeoff'], config['L_vt']/config['V_vt'] + config['t_takeoff'])
        self.mu_hl = 1 / max(config['ssd_h']/config['V_hl'] + config['t_land'], config['L_hl']/config['V_hl'] + config['t_land'])
        self.mu_ht = 1 / max(config['ssd_h']/config['V_ht'] + config['t_takeoff'], config['L_ht']/config['V_ht'] + config['t_takeoff'])

        # 计算总服务率
        self.mu_land_unsaturation = (config['c_vl'] * self.mu_vl + config['c_hl'] * self.mu_hl) * config['land_corridors']
        self.mu_takeoff = (config['c_vt'] * self.mu_vt + config['c_ht'] * self.mu_ht) * config['takeoff_corridors']
        self.mu_land_saturation = (config['avg_park_time'] + config['t_land'])

        # 初始化资源
        # Resource可以同时由有限数量的进程使用。进程要求使用这些Resource（或“拥有”这些资源），并且必须在完成后释放Resource
        self.parking_spots = simpy.Resource(env, capacity=config['number_max']) # 停机位
        self.land_corridors = simpy.Resource(env, capacity=config['land_corridors']) # 降落走廊数量
        self.takeoff_corridors = simpy.Resource(env, capacity=config['takeoff_corridors']) # 起飞走廊数量

        # Store允许生产和使用Python对象的资源
        self.landing_queue = simpy.Store(env)
        self.takeoff_queue = simpy.Store(env)

        # 状态跟踪
        self.state_history = []
        self.occupancy_history = []
        self.queue_history = []

        # 总降落飞行器数量
        self.total_land_compelet = 0

        # 启动监控进程
        env.process(self.monitor_system())
    
    def monitor_system(self):
        """监控系统状态"""
        while True:
            state = {
                'time': self.env.now,
                'parking_occupied': self.parking_spots.count,
                'land_occupied': self.land_corridors.count,
                'takeoff_occupied': self.takeoff_corridors.count,
                'landing_queue': len(self.landing_queue.items),
                'takeoff_queue': len(self.takeoff_queue.items),
                'vl_queue': 0,
                'hl_queue': 0,
                'vt_queue': 0,
                'ht_queue': 0
            }
            self.state_history.append(state)
            yield self.env.timeout(300) # 每分钟记录一次

    def landing_process(self, aircraft):
        """处理飞机降落"""
        # with self.parking_spots.request() as req:
        yield self.landing_queue.put(aircraft)  # 进入降落队列
        start_wait = self.env.now

        # 同时请求停机位资源和降落走廊资源
        req_parking_spots = self.parking_spots.request()
        req_land_corridors = self.land_corridors.request()
        yield simpy.AllOf(self.env, [req_parking_spots, req_land_corridors]) 

        aircraft = yield self.landing_queue.get()   # 得到降落资源退出降落队列

        # 记录等待时间
        wait_time = self.env.now - start_wait
        aircraft['wait_time'] = wait_time

        # 执行降落操作
        landing_type = aircraft['landing_type']
        if landing_type == 'vl':
            service_time = 1/self.mu_vl
        else:
            service_time = 1/self.mu_hl

        yield self.env.timeout(service_time)

        # 降落完成，飞机进入停机位
        aircraft['status'] = 'parked'
        self.total_land_compelet += 1
        aircraft['park_time'] = self.env.now - start_wait
        self.land_corridors.release(req_land_corridors) # 释放降落空域走廊资源

        # 触发起飞流程
        self.env.process(self.takeoff_process(aircraft, req_parking_spots))

    def takeoff_process(self, aircraft, req_parking_spots):
        """处理飞机起飞"""
        # 飞机在停机位停留一段时间
        yield self.env.timeout(np.random.poisson(self.config['avg_park_time']))

        # 随机选择起飞类型
        takeoff_type = np.random.choice(
            ['vt', 'ht'],
            p=[self.config['c_vt'], self.config['c_ht']]
        )
        aircraft['takeoff_type'] = takeoff_type

        # 加入起飞队列
        yield self.takeoff_queue.put(aircraft)
        # 等待起飞
        start_wait = self.env.now
        # yield self.env.process(self.process_takeoff(aircraft))

        # 等待起飞走廊可用
        req_takeoff_corridors = self.takeoff_corridors.request()
        yield req_takeoff_corridors

        # 得到起飞资源，退出起飞队列
        aircraft = yield self.takeoff_queue.get()
        #记录起飞等待时间
        aircraft['takeoff_wait'] = self.env.now - start_wait

        # 执行起飞操作
        takeoff_type = aircraft['takeoff_type']
        if takeoff_type == 'vt':
            service_time = 1/self.mu_vt
        else:
            service_time = 1/self.mu_ht

        self.parking_spots.release(req_parking_spots)   # 释放停机位资源
        yield self.env.timeout(service_time)

        # 起飞完成
        aircraft['status'] = 'departed'
        aircraft['depart_time'] = self.env.now
        self.takeoff_corridors.release(req_takeoff_corridors)   # 释放起飞走廊资源

    def run_simulation(self, sim_time):
        """运行模拟"""
        # 启动飞机到达进程
        self.env.process(self.aircraft_arrival())

        # 运行模拟
        self.env.run(until=sim_time)

        # 分析结果
        self.analyze_results()

    def aircraft_arrival(self):
        """生成飞机到达时间"""
        arrival_rates = {
            'vl': self.config['lambda_vl'],
            'hl': self.config['lambda_hl']
        }

        while True:
            # 随机选择到达类型
            arrival_type = np.random.choice(
                ['vl', 'hl'], 
                p=[arrival_rates[t]/sum(arrival_rates.values()) for t in arrival_rates]
            )

            # 生成到达间隔时间
            # interarrival_time = np.random.exponential(1/arrival_rates[arrival_type])
            interarrival_time = np.random.poisson(1/arrival_rates[arrival_type])
            # interarrival_time = 1/arrival_rates[arrival_type]
            yield self.env.timeout(interarrival_time)

            # 创建飞机对象
            aircraft = {
                'id' : f"{arrival_type}_{np.random.randint(1000, 9999)}",
                'arrival_time' : self.env.now,
                'status' : 'arrived',
                'landing_type': arrival_type,
                'takeoff_type': None
            }

            # 加入降落队列
            self.env.process(self.landing_process(aircraft))
            
    def analyze_results(self):
        """分析模拟结果"""
        parking_utilization = np.mean([s['parking_occupied'] for s in self.state_history])
        land_utilization = np.mean([s['land_occupied'] for s in self.state_history])
        takeoff_utilization = np.mean([s['takeoff_occupied'] for s in self.state_history])
        avg_landing_queue = np.mean([s['landing_queue'] for s in self.state_history])
        avg_takeoff_queue = np.mean([s['takeoff_queue'] for s in self.state_history])

        print("\n=== 模拟结果 ===")
        print(f"停机位占用数量：{parking_utilization:.2}")
        print(f"降落空域走廊占用数量：{land_utilization:.2}")
        print(f"起飞空域走廊占用数量：{takeoff_utilization:.2}")
        print(f"平均降落队列长度：{avg_landing_queue:.2f}")
        print(f"平均起飞队列长度：{avg_takeoff_queue:.2f}")
        print(f"总降落飞行器数量：{self.total_land_compelet}")
        
        # 绘制图表
        self.plot_results()

    def plot_results(self):
        """绘制结果图表"""
        times = [s['time']/3600 for s in self.state_history]
        parking = [s['parking_occupied'] for s in self.state_history]
        land_corridors = [s['land_occupied'] for s in self.state_history]
        takeoff_corridors = [s['takeoff_occupied'] for s in self.state_history]
        landing_q = [s['landing_queue'] for s in self.state_history]
        takeoff_q = [s['takeoff_queue'] for s in self.state_history]

        plt.figure(figsize=(15, 10))

        plt.subplot(5, 1, 1)
        plt.plot(times, parking, color='g')
        plt.title('停机位占用情况')
        plt.ylabel('占用数量')
        plt.xlim((0, 24))
        plt.grid(True)

        plt.subplot(5, 1, 2)
        plt.plot(times, landing_q, 'r')
        plt.title('降落队列长度')
        plt.ylabel('队列长度')
        plt.xlim((0, 24))
        plt.grid(True)

        plt.subplot(5, 1, 3)
        plt.plot(times, land_corridors)
        plt.title('降落空域走廊占用情况')
        plt.ylabel('占用数量')
        plt.xlim((0, 24))
        plt.grid(True)

        plt.subplot(5, 1, 4)
        plt.plot(times, takeoff_q, 'indianred')
        plt.title('起飞队列长度')
        plt.ylabel('队列长度')
        plt.xlim((0, 24))
        plt.grid(True)

        plt.subplot(5, 1, 5)
        plt.plot(times, takeoff_corridors, color='royalblue')
        plt.title('起飞空域走廊占用情况')
        plt.ylabel('占用数量')
        plt.xlabel('时间（小时）')
        plt.xlim((0, 24))
        plt.grid(True)

        # plt.subplot(2, 1, 1)
        # plt.plot(times, landing_q, 'r')
        # plt.title('降落队列长度')
        # plt.ylabel('队列长度')
        # plt.xlim((0, 24))
        # plt.grid(True)

        # plt.subplot(2, 1, 2)
        # plt.plot(times, land_corridors)
        # plt.title('降落空域走廊占用情况')
        # plt.ylabel('占用数量')
        # plt.xlabel('时间（小时）')
        # plt.xlim((0, 24))
        # plt.grid(True)

        plt.tight_layout()
        # plt.savefig('./Service_rate_Verification_average.svg', dpi=330, format='svg')
        plt.savefig('./limitation_Service_rate_Verification_poisson.svg', dpi=330, format='svg')
        plt.show()

def run_example_simulation():
    # 配置参数
    config = {
        # 物理参数
        'ssd_v': 40,   # 垂直安全距离（米）
        'ssd_h': 30,    # 水平安全距离
        'L_vl': 30,     # 垂直降落走廊长度
        'L_vt': 20,     # 垂直起飞走廊长度
        'L_hl': 20,    # 水平降落走廊长度
        'L_ht': 20,    # 水平起飞走廊长度
        'V_vl': 4,      # 垂直降落速度 (m/s)
        'V_vt': 4,      # 垂直起飞速度
        'V_hl': 3,      # 水平降落速度
        'V_ht': 3,      # 水平起飞速度
        't_land': 310,     # 垂直降落处理时间 (秒)
        't_takeoff': 310,     # 水平降落处理时间
        
        # 通道数量
        'c_vl': 0.5,     # 垂直降落走廊数量
        'c_vt': 0.5,     # 垂直起飞走廊数量
        'c_hl': 0.5,     # 水平降落走廊数量
        'c_ht': 0.5,     # 水平起飞走廊数量

        'land_corridors' : 8,  # 降落走廊数量
        'takeoff_corridors' : 4,   # 起飞走廊数量
        
        # 到达率 (单位时间内到达的飞机数量，架次/秒)
        # 'lambda_vl': 1/40,
        # 'lambda_hl': 1/40,
        'lambda_vl': 1/188,
        'lambda_hl': 1/188,
        # 'lambda_vt': 10,
        # 'lambda_ht': 10,
        
        # 系统容量
        'number_max': 40,  # 最大停机位数
        'avg_park_time': 60*60*2  # 平均停放时间 (秒)
    }

    # 创建模拟环境
    env = simpy.Environment()
    port = VTOLPort(env, config)

    # 运行模拟（8小时）
    port.run_simulation(sim_time=24*3600)

if __name__ == "__main__":
    run_example_simulation()