# Apollo 2.0 感知模块：障碍物感知阅读笔记

本文档结合代码详细地解释感知模块中障碍物感知的流程与功能。

- 1. Apollo 2.0 软件结构简介
	- 1.1 软件结构图
	- 1.2 感知模块: Perception
	- 1.3 预测模块: Prediction
	- 1.4 路由模块: Routing
	- 1.5 规划模块: Planning
	- 1.6 控制模块: Control
	- 1.7 控制区域网络模块: CanBus
	- 1.8 高清地图模块: HD-Map
	- 1.9 定位模块: Localization
- 2. 感知模块笔记
	- 2.1 代码层次结构图
		- 2.1.1 Topic注册管理器初始化
		- 2.1.2 ShareData共享数据类初始化
		- 2.1.3 SubNode子节点类初始化
		- 2.1.4 DAG有向图初始化
		- 2.1.5 DAG整体运行实现感知
	- [2.2 障碍物感知: 3D Obstacles Perception](#障碍物感知)
		- [2.2.1 激光测距仪障碍物感知: LiDAR Obstacle Perception](#激光测距仪感知)
		- [2.2.2 雷达障碍物感知: RADAR Obstacle Perception](#雷达感知)
		- [2.2.3 障碍物结果融合: Result Fusion](#障碍物结果融合)
	- 2.3 信号灯感知: Traffic Light Perception
		- 2.3.1 信号灯预处理: Traffic Light Preprocess
		- 2.3.2 信号灯处理: Traffic Light Process

### <a name="障碍物感知">2.2 障碍物感知: 3D Obstacles Perception</a>