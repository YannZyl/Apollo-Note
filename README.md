# Apollo 2.0阅读笔记

本文档主要介绍Apollo 2.0软件平台结构，由于专业方向为视觉感知，以及未来从事方向为路径规划，因此阅读笔记主要包含对应的两部分。文档的目录结构为：

- Apollo 2.0 软件结构简介
	- 软件结构图
	- 感知模块: Perception
	- 预测模块: Prediction
	- 路由模块: Routing
	- 规划模块: Planning
	- 控制模块: Control
	- 控制区域网络模块: CanBus
	- 高清地图模块: HD-Map
	- 定位模块: Localization
- 感知模块笔记
	- 代码层次结构图
		- Topic注册管理器初始化
		- ShareData共享数据类初始化
		- SubNode子节点类初始化
		- DAG有向图初始化
		- DAG整体运行实现感知
	- 障碍物感知: 3D Obstacles Perception
		- 激光测距仪障碍物感知: LiDAR Obstacle Perception
		- 雷达障碍物感知: RADAR Obstacle Perception
		- 障碍物结果融合: Result Fusion
	- 信号灯感知: Traffic Light Perception
		- 信号灯预处理: Traffic Light Preprocess
		- 信号灯处理: Traffic Light Process

links:

- [Apollo 2.0 软件结构简介](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md)
- [感知模块: 代码层次结构](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception_software_arch.md)
- [感知模块: 障碍物感知3D Obstacles Perception](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception_obstacles.md)
- [感知模块: 信号灯感知Traffic Light Perception](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception_traffic_lights.md）

refs：

- 补充: 相机标定(待完善)
- 补充: ROS tf坐标系转换(待完善)