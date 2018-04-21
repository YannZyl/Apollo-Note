# Apollo 2.0阅读笔记

本文档主要介绍Apollo 2.0软件平台，其中各个模块结构与功能的纤细介绍。文档的目录结构为：

- [Apollo 2.0 软件结构简介](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md)
	- [软件结构图](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#软件结构图)
	- [感知模块: Perception](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#感知模块)
	- [预测模块: Prediction](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#预测模块)
	- [路由模块: Routing](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#路由模块)
	- [规划模块: Planning](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#规划模块)
	- [控制模块: Control](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#控制模块)
	- [控制区域网络模块: CanBus](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#控制区域网络模块)
	- [高精地图模块: HD-Map](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#高精地图模块)
	- [定位模块: Localization](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_software_arch.md/#定位模块)
- [感知模块笔记](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_arch.md)
	- [代码层次结构](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md)
		- [Topic注册管理器初始化](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md/#注册管理器初始化)
		- [ShareData共享数据类初始化](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md/#共享数据类初始化)
		- [SubNode子节点类初始化](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md/#子节点类初始化)
		- [DAG有向图初始化](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md/#有向图初始化)
		- [DAG整体运行实现感知](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md/#DAG运行)
		- [消息发布与订阅](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md/#消息发布与接收)
	- [激光雷达障碍物感知: LiDAR Obstacle Perception](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/obstacles_arch.md)
		- [高精地图ROI过滤器](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/obstacles_1_hdmap.md)
		- [基于卷积神经网络分割](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/obstacles_2_cnn.md)
		- [MinBox障碍物边框构建](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/obstacles_3_minibox.md)
		- [HM对象跟踪](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/obstacles_4_hmtrack.md)
		- [跟踪信息融合](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/obstacles_5_fusion.md)
	- 雷达障碍物感知: RADAR Obstacle Perception
	- 障碍物结果融合: Result Fusion
	- [信号灯预处理: Traffic Light Preprocess](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/traffic_lights_preprocess.md)
	- [信号灯处理: Traffic Light Process](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/traffic_lights_process.md)
	- CameraProcessSubnode(未启用，暂不更新)
	- CIPVSubnode(未启用，暂不更新)
	- LanePostProcessingSubnode(未启用，暂不更新)
	- AsyncFusionSubnode(未启用，暂不更新)
	- VisualizationSubnode(未启用，暂不更新)

References：

- 感知模块：相机标定 Camera Calibration(待完善)
- [感知模块：ROS tf坐标系介绍](http://wiki.ros.org/tf/Tutorials#Learning_tf)
- [感知模块：ROS tf坐标系转换Example](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)
- [感知模块：ROS&&PCL PointCloud点云介绍](http://wiki.ros.org/pcl/Overview)
- [感知模块：扫描线算法](https://www.jianshu.com/p/d9be99077c2b)
- [感知模块：并查集算法](https://www.cnblogs.com/shadowwalker9/p/5999029.html)
- [感知模块：二分图匹配-匈牙利算法](https://en.wikipedia.org/wiki/Hungarian_algorithm)
- [感知模块：维特比Viterbi算法](https://zh.wikipedia.org/wiki/%E7%BB%B4%E7%89%B9%E6%AF%94%E7%AE%97%E6%B3%95)