import simpy
import numpy as np
import matplotlib.pyplot as plt
from low_altitude_airport import VTOLPort

class AirNetworkSimulation:
    def __init__(self, env, network_config):
        self.env = env
        self.network_config = network_config
        self.airports = {}
        self.aircraft_in_system = []
        self.total_park = 0
        self.total_mu_land_unsaturation = 0
        self.total_mu_land_saturation = 0
        self.total_mu_takeoff = 0

        # 初始化所有机场
        self.initialize_airports()

        # 初始化航线网络
        self.routes = network_config.get('routes', {})

        # 状态跟踪
        self.state_history = []

        self.completed_landings = 0  # 已完成降落的飞行器数量
        self.completed_takeoffs = 0  # 已完成起飞的飞行器数量

        # 周期飞行器计数器
        self.last_landings = 0  # 上一周期完成降落的飞行器数量
        self.last_takeoffs = 0  # 上一周期完成起飞的飞行器数量
        self.last_aircrafts = 0 # 上一周期的飞行器总数
        self.periodic_landings = [] # 记录每个周期的降落数量
        self.periodic_takeoffs = [] # 记录每个周期的起飞数量

        self.congestion_history = []    # 记录拥堵传播指数(CPI)
        self.stability_loss_history = []    # 记录稳定性损失函数(SLF)
        self.slf_smooth = []
        self.composite_congestion_history = []

        # 启动监控进程
        env.process(self.monitor_network())

    def initialize_airports(self):
        """初始化网络中的所有机场"""
        for airport_id, config in self.network_config['airports'].items():
            self.airports[airport_id] = VTOLPort(self.env, config)
            self.total_park += config['number_max']
            self.total_mu_land_unsaturation += self.airports[airport_id].mu_land_unsaturation
            self.total_mu_land_saturation += self.airports[airport_id].mu_land_saturation
            self.total_mu_takeoff += self.airports[airport_id].mu_takeoff

    def monitor_network(self):
        """监控整个网络状态"""
        while True:
            network_state = {
                'time' : self.env.now,
                'airports' : {},
                'aircraft_in_arrival' : len([a for a in self.aircraft_in_system if a['status'] == 'arrival']),
                'total_aircraft' : len(self.aircraft_in_system)
            }

            for airport_id, airport in self.airports.items():
                airport_state = {
                    'parking_occupied' : airport.parking_spots.count,
                    'land_occupied': airport.land_corridors.count,
                    'takeoff_occupied': airport.takeoff_corridors.count,
                    'landing_queue' : len(airport.landing_queue.items),
                    'takeoff_queue' : len(airport.takeoff_queue.items)
                }
                network_state['airports'][airport_id] = airport_state

            # 获取当前累计值
            current_landings = self.completed_landings
            # current_takeoffs = self.completed_takeoffs
            current_aircrafts = network_state['aircraft_in_arrival']

            # 计算本周期增量
            delta_landings = current_landings - self.last_landings
            # delta_takeoffs = current_takeoffs - self.last_takeoffs
            # delta_aircraft = current_aircrafts - self.last_aircrafts

            # 更新记录
            # self.periodic_landings.append(delta_landings)
            # self.periodic_takeoffs.append(delta_takeoffs)
            self.last_landings = current_landings
            # self.last_takeoffs = current_takeoffs
            self.last_aircrafts = current_aircrafts

            # print(current_aircrafts, delta_landings, delta_takeoffs)

            # 计算CPI和SLF
            # cpi, slf, cfri = self.calculate_stability_metrics(delta_aircraft)
            cpi, slf, cfri = self.calculate_stability_metrics(current_aircrafts, delta_landings)

            # smoothed_slf = self.slf_filter.filter(slf)

            self.congestion_history.append(cpi)
            self.stability_loss_history.append(slf)
            self.composite_congestion_history.append(cfri)
        
            self.state_history.append(network_state)
            yield self.env.timeout(600) # 每分钟记录一次

    # def calculate_stability_metrics(self, delta_landings):
    def calculate_stability_metrics(self, delta_aircraft, delta_landings):
        """计算拥堵传播指数"""
        cpi = 0.0
        slf = 0.0
        total_parking_occupied = 0.0

        for airport_id, airport in self.airports.items():
            # 获取当前机场状态
            total_parking_occupied += airport.parking_spots.count
            q_land = len(airport.landing_queue.items)

            # 计算CPI（按队列长度加权，假设权重beta_j=1）
            cpi += 1/3*q_land

        # 计算SLF
        # if total_parking_occupied / self.total_park > 1:
        #     slf = (max((delta_aircraft) -  1 / self.total_mu_land_saturation, 0) + max(delta_landings - 1/self.total_mu_takeoff, 0))
        # else:
        #     slf = (max((delta_aircraft) -  1 / self.total_mu_land_unsaturation, 0) + max(delta_landings - 1/self.total_mu_takeoff, 0))
        slf = (max((delta_aircraft) -  1 / self.total_mu_land_saturation, 0) + max(delta_landings - 1/self.total_mu_takeoff, 0))

        cfri = (airport.parking_spots.count / self.total_park)*cpi + (1 - (airport.parking_spots.count / self.total_park))*slf

        return cpi, slf, cfri

    def run_simulation(self, sim_time):
        """运行网络仿真"""
        # 启动飞机到达进程
        for airport_id in self.airports:
            self.env.process(self.aircraft_arrival(airport_id))

        # 运行模拟
        self.env.run(until=sim_time)

        # 分析结果
        self.analyze_results()

    def is_airport_congestion(self, airport_id):
        """判断机场是否过载"""
        airport = self.airports[airport_id]
        #定义过载条件：队列长度超过阈值或停机位占用率过高
        landing_queue_threshold = 30
        parking_util_threshold = 0.99

        current_parking_util = airport.parking_spots.count / airport.config['number_max']
        current_landing_queue = len(airport.landing_queue.items)

        # return (current_landing_queue > landing_queue_threshold or current_parking_util > parking_util_threshold)
        return current_parking_util > parking_util_threshold
    
    def find_alternative_airport(self, origin_airport_id):
    # def find_alternative_airport(self):
        """寻找可替代的未饱和机场"""
        # 按饱和度排序，选择最不繁忙的机场
        available_airports = []

        for airport_id, airport in self.airports.items():
            if airport_id == origin_airport_id:
                continue
            
            if not self.is_airport_congestion(airport_id):
                # 计算机场负载分数（越低越好）
                parking_util = airport.parking_spots.count / airport.config['number_max']
                queue_length = len(airport.landing_queue.items)
                load_score = 0.7 * parking_util + 0.3 * (queue_length  / 10)

                available_airports.append((airport_id, load_score))
            
            # queue_length = len(airport.landing_queue.items)
            # available_airports.append((airport_id, queue_length))

        if not available_airports:
            return None  # 没有可用替代机场
        
        # 返回队列最短的机场
        available_airports.sort(key=lambda x:x[1])
        return available_airports[0][0]

    def aircraft_arrival(self, origin_airport_id):
        """生成飞机到达时间（外部到达）"""
        airport = self.airports[origin_airport_id]

        base_rates = {
            'vl' : airport.config['lambda_vl'],
            'hl' : airport.config['lambda_hl']
        }

        # 高峰时段到达率 (增加50%)
        peak_rates = {
            'vl': airport.config['lambda_vl'] * 6,
            'hl': airport.config['lambda_hl'] * 6
        }

        # 定义高峰时段
        peak_periods = [
            (8*3600, 10*3600),
            (17*3600, 19*3600)
        ]

        while True:
            current_time = self.env.now
            in_peak = any(start <= current_time < end for start, end in peak_periods)
        
            # 根据当前时段选择到达率
            current_rates = peak_rates if in_peak else base_rates

            # 随机选择到达类型
            arrival_type = np.random.choice(
                ['vl', 'hl'],
                # p=[arrival_rates[t]/sum(arrival_rates.values()) for t in arrival_rates]
                p=[current_rates[t]/sum(current_rates.values()) for t in current_rates]
            )

            # 生成到达时间间隔
            # interarrival_time = np.random.exponential(1/current_rates[arrival_type])
            interarrival_time = np.random.poisson(1/current_rates[arrival_type])
            # interarrival_time = 1/current_rates[arrival_type]
            yield self.env.timeout(interarrival_time)

            # 创建飞机对象
            aircraft = {
                'id' : f"{arrival_type}_{np.random.randint(1000, 9999)}",
                'arrival_time' : self.env.now,
                'status' : 'arrival',
                'landing_type' : arrival_type,
                'takeoff_type' : None,
                'current_airport' : origin_airport_id,
                'destination' : self.choose_destination(origin_airport_id),
                'flight_history' : []
            }

            self.aircraft_in_system.append(aircraft)

            # 检查当前机场是否过载
            if self.is_airport_congestion(origin_airport_id):
                # 尝试寻找其他可降落机场
                alt_airport = self.find_alternative_airport(origin_airport_id)
                if alt_airport:
                    # 修改飞行器目的地为替代机场
                    self.env.process(self.landing_process(aircraft, alt_airport))
                    continue

            self.env.process(self.landing_process(aircraft, origin_airport_id))

    def choose_destination(self, origin):
        """为飞机选择目的地机场"""
        # 简单实现：随机选择不同于起点的机场
        possible_dests = [a for a in self.airports if a != origin]
        return np.random.choice(possible_dests) if possible_dests else origin
    
    def landing_process(self, aircraft, airport_id):
        """处理飞机降落（重写原方法以支持网络）"""
        airport = self.airports[airport_id]

        yield airport.landing_queue.put(aircraft)   # 进入降落队列
        start_wait = self.env.now
        
        yield self.env.timeout(5*60)    # 等待机场回复时间

        # 检查当前机场是否过载
        if self.is_airport_congestion(airport_id):
            # 尝试寻找其他可降落机场
            alt_airport = self.find_alternative_airport(airport_id)
            if alt_airport:
                aircraft = yield airport.landing_queue.get()    # 当前机场拥堵，退出降落队列
                aircraft['status'] = 'arrival'
                yield self.env.timeout(10*60)   # 飞行时间
                # 修改飞行器目的地为替代机场
                self.env.process(self.landing_process(aircraft, alt_airport))

        # 同时请求停机位资源和降落走廊资源
        req_parking_spots = airport.parking_spots.request()
        req_land_corridors = airport.land_corridors.request()
        yield simpy.AllOf(self.env, [req_parking_spots, req_land_corridors])

        aircraft = yield airport.landing_queue.get()    # 得到降落资源退出降落队列
        aircraft['status'] = 'landing'

        # 记录等待时间
        wait_time = self.env.now - start_wait
        aircraft['wait_time'] = wait_time

        # 执行降落操作
        landing_type = aircraft['landing_type']
        if landing_type == 'vl':
            service_time = 1 / airport.mu_vl
        else:
            service_time = 1 / airport.mu_hl

        yield self.env.timeout(service_time)

        # 降落完成
        aircraft['status'] = 'parked'
        aircraft['park_time'] = self.env.now
        self.completed_landings += 1
        airport.land_corridors.release(req_land_corridors)  # 释放降落空域走廊资源

        # 触发起飞流程
        self.env.process(self.takeoff_process(aircraft, airport_id, req_parking_spots))

    def takeoff_process(self, aircraft, airport_id, req_parking_spots):
        """处理飞机起飞（重写原方法以支持网络）"""
        airport = self.airports[airport_id]

        # 飞机在停机位停留时间
        yield self.env.timeout(np.random.exponential(airport.config['avg_park_time']))

        # 随机选择起飞类型
        takeoff_type = np.random.choice(
            ['vt', 'ht'],
            p=[airport.config['c_vt'], airport.config['c_ht']]
        )
        aircraft['takeoff_type'] = takeoff_type

        # 加入起飞队列
        yield airport.takeoff_queue.put(aircraft)
        start_wait = self.env.now

        # 等待起飞走廊可用
        req_takeoff_corridors = airport.takeoff_corridors.request()
        yield req_takeoff_corridors

        # 得到起飞资源，退出起飞队列
        aircraft = yield airport.takeoff_queue.get()
        aircraft['takeoff_wait'] = self.env.now - start_wait    # 记录起飞等待时间

        # 执行起飞操作
        takeoff_type = aircraft['takeoff_type']
        if takeoff_type == 'vt':
            service_time = 1 / airport.mu_vt
        else:
            service_time = 1 / airport.mu_ht

        airport.parking_spots.release(req_parking_spots)    # 释放停机位资源
        yield self.env.timeout(service_time)

        # 起飞完成
        aircraft['status'] = 'in_transit'
        aircraft['depart_time'] = self.env.now
        self.completed_takeoffs += 1
        airport.takeoff_corridors.release(req_takeoff_corridors)    # 释放起飞走廊资源

        # 记录飞行历史
        # aircraft['flight_history'].append({
        #     'airport' : airport_id,
        #     'arrival' : aircraft['arrival_time'],
        #     'departure' : aircraft['depart_time']
        # })

        # # 如果飞机到达目的地则结束；否则飞往下一个机场
        # if airport_id == aircraft['destination']:
        #     aircraft['status'] = 'completed'
        #     self.aircraft_in_system.remove(aircraft)
        # else:
        #     # 计算飞行时间（简化模型）
        #     flight_time = self.calculate_flight_time(airport_id, aircraft['destination'])
        #     yield self.env.timeout(flight_time)

        #     # 更新当前机场和目的地
        #     aircraft['current_airport'] = aircraft['destination']
        #     aircraft['destination'] = self.choose_destination(aircraft['destination'])

        #     # 加入新机场的降落队列
        #     self.env.process(self.landing_process(aircraft, aircraft['current_airport']))

    def calculate_flight_time(self, origin, destination):
        """计算两个机场之间的飞行时间（简化模型）"""
        # 这里可以使用更复杂的模型，如基于距离和速度的计算
        # 现在使用固定平均值加上一些随机变化
        return np.random.exponential(1800) # 平均30分钟
    
    def analyze_results(self):
        """分析网络仿真结果"""
        print("\n=== 空域网络仿真结果 ===")
        
        # 各机场利用率
        for airport_id in self.airports:
            parking_util = np.mean([s['airports'][airport_id]['parking_occupied'] for s in self.state_history])
            landing_q = np.mean([s['airports'][airport_id]['landing_queue'] for s in self.state_history])
            takeoff_q = np.mean([s['airports'][airport_id]['takeoff_queue'] for s in self.state_history])

            print(f"\n机场 {airport_id}:")
            print(f"  平均停机位占用: {parking_util:.2f}")
            print(f"  平均降落队列长度: {landing_q:.2f}")
            print(f"  平均起飞队列长度: {takeoff_q:.2f}")

        # 网络整体统计
        avg_in_transit = np.mean([s['aircraft_in_arrival'] for s in self.state_history])
        print(f"\n网络整体:")
        print(f"    平均在途飞机数量:{avg_in_transit:.2f}")
        print(f"    SLF均值: {np.mean(self.stability_loss_history)}")

        # 绘制图表
        self.plot_results()

    def plot_results(self):
        """绘制网络仿真结果图表"""
        times = [s['time']/3600 for s in self.state_history]  # 转换为小时
        line_styles = ['-.', ':', '--']  # 定义不同的线型
        
        plt.figure(figsize=(15, 12))
    
        # 各机场停机位占用情况
        plt.subplot(6, 1, 1)
        for i, airport_id in enumerate(self.airports):
            parking = [s['airports'][airport_id]['parking_occupied'] for s in self.state_history]
            plt.plot(times, parking, linestyle=line_styles[i], label=f'机场 {airport_id}')
        plt.title('各机场停机位占用情况')
        plt.ylabel('占用数量')
        plt.legend()
        plt.grid(True)

        # 各机场降落队列情况
        plt.subplot(6, 1, 2)
        for i, airport_id in enumerate(self.airports):
            landing_q = [s['airports'][airport_id]['landing_queue'] for s in self.state_history]
            plt.plot(times, landing_q, linestyle=line_styles[i], label=f'机场 {airport_id} 降落队列')
        plt.title('各机场降落队列长度')
        plt.ylabel('队列长度')
        plt.legend()
        plt.grid(True)

        # 各机场降落空域走廊占用情况
        plt.subplot(6, 1, 3)
        for i, airport_id in enumerate(self.airports):
            land_corridors = [s['airports'][airport_id]['land_occupied'] for s in self.state_history]
            plt.plot(times, land_corridors, linestyle=line_styles[i], label=f'机场 {airport_id} 降落空域走廊占用数量')
        plt.title('各机场降落空域走廊占用数量')
        plt.ylabel('占用数量')
        plt.legend()
        plt.grid(True)

        # 各机场起飞队列情况
        plt.subplot(6, 1, 4)
        for i, airport_id in enumerate(self.airports):
            takeoff_q = [s['airports'][airport_id]['takeoff_queue'] for s in self.state_history]
            plt.plot(times, takeoff_q, linestyle=line_styles[i], label=f'机场 {airport_id} 起飞队列')
        plt.title('各机场起飞队列长度')
        plt.ylabel('队列长度')
        # plt.xlabel('时间（小时）')
        plt.legend()
        plt.grid(True)

        # 各机场起飞空域走廊占用情况
        plt.subplot(6, 1, 5)
        for i, airport_id in enumerate(self.airports):
            takeoff_corridors = [s['airports'][airport_id]['takeoff_occupied'] for s in self.state_history]
            plt.plot(times, takeoff_corridors, linestyle=line_styles[i], label=f'机场 {airport_id} 起飞空域走廊占用数量')
        plt.title('各机场起飞空域走廊占用数量')
        plt.ylabel('占用数量')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(6, 1, 6)
        plt.plot(times, self.congestion_history, linestyle=':', label='CPI')
        plt.plot(times, self.stability_loss_history, linestyle='-', label='SLF')
        # plt.plot(times, self.composite_congestion_history, 'black', linestyle='-.', label='cfri')
        plt.title('稳定性判据')
        # plt.ylabel('')
        plt.xlabel('时间（小时）')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('./airnet_work.svg', dpi=330, format='svg')
        plt.show()

def run_network_simulation():
    avg_park_time = 3*3600
    lambda_l = 1 / 400
    # 网络配置参数
    network_config = {
        'airports': {
            'A': {
                # 物理参数
                'ssd_v': 40, 'ssd_h': 30,
                'L_vl': 60, 'L_vt': 60, 'L_hl': 45, 'L_ht': 45,
                'V_vl': 4, 'V_vt': 4, 'V_hl': 3, 'V_ht': 3,
                't_land': 310, 't_takeoff': 310,
                
                # 通道数量
                'c_vl': 0.5, 'c_vt': 0.5, 'c_hl': 0.5, 'c_ht': 0.5,
                # 'land_corridors': land_corridors, 'takeoff_corridors': takeoff_corridors,
                'land_corridors': 8, 'takeoff_corridors': 4,
                
                # 到达率
                # 'lambda_vl': 12/1505, 'lambda_hl': 12/1505,
                'lambda_vl': lambda_l, 'lambda_hl': lambda_l,
                
                # 系统容量
                'number_max': 60, 'avg_park_time': avg_park_time  # 平均停放3小时
            },
            'B': {
                # 物理参数
                'ssd_v': 40, 'ssd_h': 30,
                'L_vl': 60, 'L_vt': 60, 'L_hl': 45, 'L_ht': 45,
                'V_vl': 4, 'V_vt': 4, 'V_hl': 3, 'V_ht': 3,
                't_land': 310, 't_takeoff': 310,
                
                # 通道数量
                'c_vl': 0.5, 'c_vt': 0.5, 'c_hl': 0.5, 'c_ht': 0.5,
                'land_corridors': 6, 'takeoff_corridors': 2,
                
                # 到达率
                # 'lambda_vl': 2/301, 'lambda_hl': 2/301,
                'lambda_vl': lambda_l, 'lambda_hl': lambda_l,
                
                # 系统容量
                'number_max': 50, 'avg_park_time': avg_park_time  # 平均停放2小时
            },
            'C': {
                # 物理参数
                'ssd_v': 40, 'ssd_h': 30,
                'L_vl': 30, 'L_vt': 20, 'L_hl': 30, 'L_ht': 20,
                'V_vl': 4, 'V_vt': 4, 'V_hl': 3, 'V_ht': 3,
                't_land': 310, 't_takeoff': 310,
                
                # 通道数量
                'c_vl': 0.5, 'c_vt': 0.5, 'c_hl': 0.5, 'c_ht': 0.5,
                'land_corridors': 8, 'takeoff_corridors': 4,
                
                # 到达率
                'lambda_vl': lambda_l, 'lambda_hl': lambda_l,

                # 系统容量
                'number_max': 40, 'avg_park_time': avg_park_time
            }
        },
        'routes': {
            # 可以定义特定航线及其属性，当前使用随机选择
        }
    }

    # 创建模拟环境
    env = simpy.Environment()
    network = AirNetworkSimulation(env, network_config)

    # 运行模拟(24小时)
    network.run_simulation(sim_time=24*3600)

if __name__ == "__main__":
    run_network_simulation()