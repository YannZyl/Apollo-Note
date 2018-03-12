# Apollo 2.0 模块结构说明

本文档主要介绍Apollo 2.0软件平台结构，大致描述了系统的模块构成以及每个模块对应的功能。

- [Apollo 2.0 软件结构简介](#总体软件结构)
	- [软件结构图](#软件结构图)
	- [感知模块: Perception](#感知模块)
	- [预测模块: Prediction](#预测模块)
	- [路由模块: Routing](#路由模块)
	- [规划模块: Planning](#规划模块)
	- [控制模块: Control](#控制模块)
	- [控制区域网络模块: CanBus](#控制区域网络模块)
	- [高清地图模块: HD-Map](#高清地图模块)
	- [定位模块: Localization](#定位模块)
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

## <a name="总体软件结构">Apollo 2.0总体软件结构简介</a>
本章主要介绍Apollo 2.0的软件结构，粗略的解释总体组成模块以及每个模块的功能，代码请参考([Apollo 2.0 Github](https://github.com/ApolloAuto/apollo)), 软件框架请参考([Apollo 2.0软件架构](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Apollo_2.0_Software_Architecture.md))。

### <a name="软件结构图">软件结构图</a>

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/Apollo_2_0_Software_Arch.png)

上图每个模块都以独立的ROS node运行，相互之间的消息传递依赖ROS的消息发布与订阅机制。消息订阅(subsrcibe)等同于数据输入(data input)，而消息发布(publish)等同于数据输出(data output)。

### <a name="感知模块">感知模块: Perception</a>

感知模块主要功能是识别周围环境树木，人，路面，交通灯等信息，为后续路径规划，控制等做辅助。感知模块主要包含两个重要的子模块：

- 障碍物检测(3D obstacles perception).
- 交通信号灯检测(Traffic light perception).

感知模块输入来源于汽车物理感知设备，主要包含激光雷达点云数据，视觉摄像头数据。同时，交通信号灯检测也依赖于其他模块提供的数据，包含定位模块数据，高清地图数据(信号灯实时检测是不可行的或者说是不合理不必要的，原因在于太过耗费计算资源，实际只需要定位与高精度地图模块提供相应的位置信息，指示感知模块在什么时间段与路面段做信号灯检测)，E.g.路口需要启动信号灯检测，中间路段只需要障碍物检测，不存在信号灯。

#### <a name="感知模块输入">感知模块输入数据</a>

- Point cloud data/点云数据 (ROS topic /apollo/sensor/velodyne64/comensator/pointCloud2)
- Radar data/雷达数据 (ROS topic /apollo/sensor/conti_radar)
- Image data/图像数据 (ROS topic /apollo/sensor/camera/traffic/image_long & /apollo/sensor/camera/traffic/image_short)
- Coordinate frame transformation information over time/坐标系转换数据 (ROS topic /tf)
- HDMap/高清地图
- Extrinsic parameters of LiDAR sensor calibration/激光雷达传感器矫正外参 (ROS topic /tf_static)
- Extrinsic parameters of radar sensor calibration/雷达传感器矫正外参 (来自外部YAML文件)
- Extrinsic and Intrinsic parameters of all camera calibration/所有相机/长焦+短焦相机的内参和外参 (来自外部YAML文件)
- Velocity of host vehicle/主车体速度 (ROS topic /apollo/localization/pos)

#### <a name="感知模块输出">感知模块输出数据</a>

- 3D障碍物跟踪的方向，速度以及树，人，车辆等分类信息
- 交通信号灯的标定框以及信号灯状态信息

### <a name="预测模块">预测模块: Prediction</a>

预测模块主要功能为从感知模块获取障碍物信息，包括位置，方向，速度，加速度等。最终预测障碍物未来时刻的运动轨迹。当感知模块发布障碍物信息时，自动触发预测模块。

#### <a name="预测模块输入">预测模块输入数据</a>

- Obstacles from perception module/感知模块障碍物信息
- Localization from localization module/定位模块信息

#### <a name="预测模块输出">预测模块输出数据</a>

- Obstacles additionally with predicted trajectories/障碍物预测轨迹

#### <a name="预测模块函数">预测模块函数与功能</a>

- Contrainer/容器
Container存储来自ROS订阅的输入数据，目前版本支持存储感知障碍物(perception onstacles)，车辆定位(vehicle localization)，车辆轨迹规划(vehicle planning)

- Evaluator/评估器
Evaluator对视野中的每个障碍物独立评估器路径与速度。评估器通过使用保存在prediction/data下的模型数据来对每个车道序列进行路径评估，输出车辆在每个车道的概率。

- Predictor/预测器
Predictor最终生成障碍物的预测轨迹，目前支持的预测器有：
	- Lane sequence: 障碍物沿着车道线运动
	- Move sequence: 障碍物遵循汽车动态模式，沿车道线运动(没翻译好，不准确)
	- Free movement: 障碍物自由运动
	- Regional movement: 障碍物在一定区域内运动

### <a name="路由模块">路由模块: Routing</a>

路由模块根据请求(起始地与目的地信息)生成高层的导航信息，通过一系列的车道线与车道信息，指示车辆如何抵达目的地

#### <a name="路由模块输入">路由模块输入数据</a>

- Map data/地图数据
- Routing request/路由请求，包含起始地与目的地位置信息。

#### <a name="路由模块输出">路由模块输出数据</a>

- 路由导航信息

### <a name="规划模块">规划模块: Planning</a>

规划模块为自动驾驶车辆规划时空轨迹。在Apollo 2.0中，规划模块使用多个信息源来规划安全，无碰撞的运动轨迹，所以规划模块几乎可以与其他任何模块交互(localization, vehicle status (position, velocity, acceleration, chassis), map, routing, perception and prediction)。

(1). 初始化时，规划模块使用预测模块输出信息。预测模块输出包含了原始障碍物信息及其运动轨迹信息。为什么规划模块不使用感知模块Perception/3D_obstacles输出信息？因为规划模块订阅的topic为交通信号灯Perception/traffic_light topic，而没有订阅Perception/3D_obstacles topic，所以不能直接使用。

(2). 紧接着，规划模块使用路由模块输出信息。在某些特定情况下，如果不能按照路由模块导航路线行驶，规划模块可以发送请求并触发路由模块进行新的计算，生成新的导航信息。E.g. 路上发生交通事故，需要重新导航新的路径。

(3). 最终，规划模块需要了解定位信息，以及当前车辆状态信息。规划模块有特定的函数进行一定频率的触发。

支持两类主要的规划器Planner：
- RTK replay planner(since Apollo 1.0)
- EM planner (since Apollo 1.5)

#### <a name="规划模块输入">规划模块输入数据</a>

- RTK replay planner
	- Localization/定位信息
	- Recorded RTK trajectory/记录的RTK轨迹(put into the folder modules/planning/data, and change the gflag file name in planning/common/planning_gflags)
- EM planner
	- Localization/定位信息
	- Perception/感知模块信息 traffic light
	- Prediction/预测模块信息
	- HD Map/高清地图信息
	- Routing/路由导航信息

#### <a name="规划模块输出">规划模块输出数据</a>

- 安全合理的运动轨迹信息，供控制模块执行

### <a name="控制模块">控制模块: Control</a>

控制模块通过生成诸如节流阀，制动器，转向的控制指令给CanBus模块，执行来自规划模块的时空轨迹命令。

#### <a name="控制模块输入">控制模块输入数据</a>

- Planning trajectory/规划路径
- Car status/车辆状态
- Localization/定位信息
- Dreamview AUTO mode change request/模式切换请求

#### <a name="控制模块输出">控制模块输出数据</a>

- control command (steering, throttle, brake) to canbus/具体的控制指令

### <a name="控制区域网络模块">控制区域网络模块:  CanBus</a>

类似于下位机，控制区域网络模块讲来自于控制模块的具体控制指令传递给汽车硬件，同时向上层软件反馈车辆硬件信息。

#### <a name="控制区域网络模块输入">控制区域网络模块输入数据</a>

- Control command/控制指令

#### <a name="控制区域网络模块输出">控制区域网络模块输出数据</a>

- Chassis status/机箱状态
- Chassis detail status/机箱具体状态

### <a name="高清地图模块">高清地图模块: HD Map</a>

高清地图模块类似于一个库，不像其他模块使用发布与订阅机制，他频繁的被调用用来检索相关道路的结构信息。

### <a name="定位模块">定位模块: Localization</a>

定位模块从不同的信息源，如GPS(经纬度)，LiDAR(距离)和IMU(加速度等)来评估车辆当前的信息。常用的定位技术有两类：

- 基于RTK(Real-Time Kinematic, 实时动态载波相位差分技术)定位。由OnTimer函数以一定频率触发定位。
- 多传感器融合(Multiple Sensor Fusion, MSF)定位。由一系列传感器触发函数触发。

#### <a name="定位模块输入">定位模块输入数据</a>

- RTK-base method.
	- GPS - Global Position System/全球定位系统
	- IMU - Interial Measurement Unit/惯性测量单元
- MSF method.
	- GPS - Global Position System/全球定位系统
	- IMU - Interial Measurement Unit/惯性测量单元
	- LiDAR - Light Detection And Ranging Sensor/光检测和测距传感器，激光雷达
