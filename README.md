# Apollo 2.0阅读笔记

本文档主要介绍Apollo 2.0软件平台结构，由于专业方向为视觉感知，以及未来从事方向为路径规划，因此阅读笔记主要包含对应的两部分。文档的目录结构为：

- [1. Apollo 2.0 软件结构简介](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md)
	- 1.1 软件结构图
	- 1.2 感知模块: Perception
	- 1.3 预测模块: Prediction
	- 1.4 路由模块: Routing
	- 1.5 规划模块: Planning
	- 1.6 控制模块: Control
	- 1.7 控制区域网络模块: CanBus
	- 1.8 高清地图模块: HD-Map
	- 1.9 定位模块: Localization
- [2. 感知模块笔记](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception.md)
	- 2.1 代码层次结构图
		- 2.1.1 Topic注册管理器初始化
		- 2.1.2 ShareData共享数据类初始化
		- 2.1.3 SubNode子节点类初始化
		- 2.1.4 DAG有向图初始化
		- 2.1.5 DAG整体运行实现感知
	- 2.2 障碍物感知: 3D Obstacles Perception
		- 2.2.1 激光测距仪障碍物感知: LiDAR Obstacle Perception
		- 2.2.2 雷达障碍物感知: RADAR Obstacle Perception
		- 2.2.3 障碍物结果融合: Result Fusion
	- 2.3 信号灯感知: Traffic Light Perception
		- 2.3.1 信号灯预处理: Traffic Light Preprocess
		- 2.3.2 信号灯处理: Traffic Light Process

