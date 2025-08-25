# AirNetwork 空中交通网络仿真系统

## 概述

AirNetwork 是一个基于 SimPy 的离散事件仿真系统，用于模拟低空垂直起降机场网络中的空中交通流量和拥堵传播。该系统模拟多个机场之间的飞机起降、排队、中转等过程，并提供了网络稳定性指标分析功能。

## 主要功能

- **多机场网络仿真**：支持配置多个具有不同特性的机场
- **动态交通流模拟**：模拟飞机到达、降落、停放、起飞全过程
- **拥堵检测与重路由**：当机场过载时自动寻找替代机场
- **性能指标监控**：实时跟踪停机位占用率、队列长度等关键指标
- **稳定性分析**：计算拥堵传播指数(CPI)和稳定性损失函数(SLF)
- **可视化输出**：生成多维度性能图表

## 系统架构

### 核心类

1. **AirNetworkSimulation**：主仿真类，管理整个网络
2. **VTOLPort**：单个垂直起降机场类（从 low_altitude_airport 导入）

### 关键组件

- **机场资源配置**：停机位、降落走廊、起飞走廊
- **队列管理**：降落队列、起飞队列
- **飞机状态跟踪**：到达、降落中、停放、起飞中、中转
- **拥堵传播分析**：CPI、SLF、CFRI 指标计算

## 配置网络参数
在 run_network_simulation() 函数中修改网络配置：
```
network_config = {
    'airports': {
        '机场ID': {
            # 物理参数
            'ssd_v': 40, 'ssd_h': 30,
            'L_vl': 60, 'L_vt': 60, 'L_hl': 45, 'L_ht': 45,
            'V_vl': 4, 'V_vt': 4, 'V_hl': 3, 'V_ht': 3,
            't_land': 310, 't_takeoff': 310,
            
            # 通道数量和比例
            'c_vl': 0.5, 'c_vt': 0.5, 'c_hl': 0.5, 'c_ht': 0.5,
            'land_corridors': 8, 'takeoff_corridors': 4,
            
            # 到达率
            'lambda_vl': 0.0025, 'lambda_hl': 0.0025,
            
            # 系统容量
            'number_max': 60, 'avg_park_time': 10800  # 3小时
        },
        # 更多机场...
    }
}
```

## 自定义仿真参数
- **仿真时间**：修改 network.run_simulation(sim_time=24*3600) 中的时间值

- **监控频率**：在 monitor_network() 中修改 yield self.env.timeout(600) 的时间间隔

- **高峰时段**：在 aircraft_arrival() 中调整 peak_periods

## 关键算法
- **拥堵检测**：
```
def is_airport_congestion(self, airport_id):
    # 基于停机位占用率判断是否过载
    current_parking_util = airport.parking_spots.count / airport.config['number_max']
    return current_parking_util > 0.99  # 99%占用率阈值
```

- **替代机场选择**：
```
  def find_alternative_airport(self, origin_airport_id):
    # 基于负载评分选择最不繁忙的可用机场
    load_score = 0.7 * parking_util + 0.3 * (queue_length / 10)
```

## 安装依赖

```bash
pip install simpy numpy matplotlib
