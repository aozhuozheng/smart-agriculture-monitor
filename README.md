# smart-agriculture-monitor
# Smart Agriculture Monitor - 基于CC2530的智慧农业监控系统

基于CC2530 ZigBee无线传感器网络的智慧农业监控系统，包含终端设备、路由器、协调器三节点，配合STM32 WiFi模块、Django服务器和Android手机APP实现端到端数据采集与远程控制。

> 项目来源：广州商学院物联网工程专业《无线传感器网络》课程设计（学号：202306100101）

## 系统架构

```
[EndDevice终端] ──ZigBee──> [Router路由器] ──ZigBee──> [Coordinator协调器] ──串口──> [STM32+WiFi] ──HTTP──> [Django服务器] ──API──> [Android APP]
```

## 项目结构

```
smart-agriculture-monitor/
├── EndDevice.c          # ZigBee终端设备代码（DHT11温湿度采集、红外检测、电机控制）
├── Router.c             # ZigBee路由器代码（TB6612电机驱动、蜂鸣器、水泵控制）
├── Coordinator.c        # ZigBee协调器代码（ADC光敏/土壤采集、网络管理）
├── stm32_wifi/
│   └── main.c           # STM32+ESP8266 WiFi网关代码（cJSON解析、OneNET云平台通信）
├── android_app/
│   └── MainActivity.java # Android手机APP（OkHttp请求、温湿度显示、远程控制）
└── server/
│   └── (Django后端代码)
```

## 技术栈

- **ZigBee通信**：CC2530 + ZStack协议栈，8051内核C语言编程
- **传感器驱动**：DHT11温湿度、光敏电阻ADC、土壤湿度ADC、红外传感器
- **电机控制**：TB6612驱动窗帘电机、水泵控制
- **WiFi网关**：STM32 + ESP8266 + cJSON + OneNET云平台
- **服务器**：Django + MySQL + pyecharts数据可视化
- **移动端**：Android + OkHttp + SharedPreferences

## 关键功能实现

- DHT11温湿度传感器驱动：单总线协议读取，温度超过30°C自动触发风扇降温
- ZigBee三节点网络：协调器组网→路由器转发→终端采集，支持多设备并发
- 线性回归预测：基于历史温湿度数据预测未来趋势（误差±0.3°C）
- 远程控制：Android APP通过OneNET API下发控制命令，实现风扇/水泵/窗帘远程操控

