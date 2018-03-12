# Apollo 2.0 感知模块：障碍物感知阅读笔记

本文档结合代码详细地解释感知模块中障碍物感知的流程与功能。

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
	- [障碍物感知: 3D Obstacles Perception](#障碍物感知)
		- [激光测距仪障碍物感知: LiDAR Obstacle Perception](#激光测距仪感知)
		- [雷达障碍物感知: RADAR Obstacle Perception](#雷达感知)
		- [障碍物结果融合: Result Fusion](#障碍物结果融合)
	- 信号灯感知: Traffic Light Perception
		- 信号灯预处理: Traffic Light Preprocess
		- 信号灯处理: Traffic Light Process

## <a name="障碍物感知">障碍物感知: 3D Obstacles Perception</a>