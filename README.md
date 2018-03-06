# Apollo 2.0阅读笔记

本文档主要介绍Apollo 2.0软件平台结构，由于专业方向为视觉感知，以及未来从事方向为路径规划，因此阅读笔记主要包含对应的两部分。文档的目录结构为：

<!-- 添加隐藏链接，便于后续小结返回 -->
<a name="目录头"></a>
- [1. Apollo 2.0 软件结构简介](#总体软件结构)
	- [1.1 软件结构图](#软件结构图)
	- [1.2 感知模块: Perception](#感知模块)
	- [1.3 预测模块: Prediction](#预测模块)
	- [1.4 路由模块: Routing](#路由模块)
	- [1.5 规划模块: Planning](#规划模块)
	- [1.6 控制模块: Control](#控制模块)
	- [1.7 控制区域网络模块: CanBus](#控制区域网络模块)
	- [1.8 高清地图模块: HD-Map](#高清地图模块)
	- [1.9 定位模块: Localization](#定位模块)
- [2. 感知模块笔记](#感知模块详解)
	- [2.1 代码层次结构图](#代码层次结构)
		- [2.1.1 Topic注册管理器初始化](#注册管理器初始化)
		- [2.1.2 ShareData共享数据类初始化](#共享数据类初始化)
		- [2.1.3 SubNode子节点类初始化](#子节点类初始化)
		- [2.1.4 DAG有向图初始化](#有向图初始化)
		- [2.1.5 DAG整体运行实现感知](#DAG运行)
	- [2.2 障碍物感知: 3D Obstacles Perception](#障碍物感知)
		- [2.2.1 激光测距仪障碍物感知: LiDAR Obstacle Perception](#激光测距仪感知)
		- [2.2.2 雷达障碍物感知: RADAR Obstacle Perception](#雷达感知)
		- [2.2.3 障碍物结果融合: Result Fusion](#障碍物结果融合)
	- [2.3 信号灯感知: Traffic Light Perception](#信号灯感知)
		- [2.3.1 信号灯预处理: Traffic Light Preprocess](#信号灯预处理)
		- [2.3.2 信号灯处理: Traffic Light Process](#信号灯处理)

## <a name="总体软件结构">1. Apollo 2.0总体软件结构简介</a>
本章主要介绍Apollo 2.0的软件结构，粗略的解释总体组成模块以及每个模块的功能，代码请参考([Apollo 2.0 Github](https://github.com/ApolloAuto/apollo)), 软件框架请参考([Apollo 2.0软件架构](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Apollo_2.0_Software_Architecture.md))。

### <a name="软件结构图">1.1 软件结构图</a>

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/Apollo_2_0_Software_Arch.png)

上图每个模块都以独立的ROS node运行，相互之间的消息传递依赖ROS的消息发布与订阅机制。消息订阅(subsrcibe)等同于数据输入(data input)，而消息发布(publish)等同于数据输出(data output)。

### <a name="感知模块">1.2 感知模块: Perception</a>

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

### <a name="预测模块">1.3预测模块: Prediction</a>

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

### <a name="路由模块">1.4 路由模块: Routing</a>

路由模块根据请求(起始地与目的地信息)生成高层的导航信息，通过一系列的车道线与车道信息，指示车辆如何抵达目的地

#### <a name="路由模块输入">路由模块输入数据</a>

- Map data/地图数据
- Routing request/路由请求，包含起始地与目的地位置信息。

#### <a name="路由模块输出">路由模块输出数据</a>

- 路由导航信息

### <a name="规划模块">1.5 规划模块: Planning</a>

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

### <a name="控制模块">1.6 控制模块: Control</a>

控制模块通过生成诸如节流阀，制动器，转向的控制指令给CanBus模块，执行来自规划模块的时空轨迹命令。

#### <a name="控制模块输入">控制模块输入数据</a>

- Planning trajectory/规划路径
- Car status/车辆状态
- Localization/定位信息
- Dreamview AUTO mode change request/模式切换请求

#### <a name="控制模块输出">控制模块输出数据</a>

- control command (steering, throttle, brake) to canbus/具体的控制指令

### <a name="控制区域网络模块">1.7  控制区域网络模块:  CanBus</a>

类似于下位机，控制区域网络模块讲来自于控制模块的具体控制指令传递给汽车硬件，同时向上层软件反馈车辆硬件信息。

#### <a name="控制区域网络模块输入">控制区域网络模块输入数据</a>

- Control command/控制指令

#### <a name="控制区域网络模块输出">控制区域网络模块输出数据</a>

- Chassis status/机箱状态
- Chassis detail status/机箱具体状态

### <a name="高清地图模块">1. 8 高清地图模块: HD Map</a>

高清地图模块类似于一个库，不像其他模块使用发布与订阅机制，他频繁的被调用用来检索相关道路的结构信息。

### <a name="定位模块">1.9 定位模块: Localization</a>

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

[返回目录](#目录头)

## <a name="感知模块详解">2. 感知模块笔记</a>

本章节主要详细的解析Apollo 2.0感知模块Perception代码与功能结构，相关资料请参考([Perception: 3D Obstacles](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md)和[Percepton: Traffic Light](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/traffic_light.md))

### <a name="代码层次结构">2.1 代码层次结构图</a>

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_software_arch.png)

感知模块框架本质是一个DAG有向图，该图由3类基本元素组成，包括：子节点Sub Node，边Edge和共享数据Share Data。框架中的每一个功能都以子节点SubNode的形式存在，以线程形式运行；子节点之间的共享数据ShareData沿着边Edge有向流动，从产生子节点流向需求子节点。上图中第二行分别初始化共享数据，子节点以及DAG，最终DAG运行执行感知功能。

#### <a name="注册管理器初始化">2.1.1 Topic注册管理器初始化</a>

Topic注册与管理器初始化主要是为Apollo各个模块之间节点通信创建所需要订阅与发布的topic。前提是节点使用的是ROS消息发布与订阅机制(下文将提到)，这些节点会接受各类数据，然后处理数据，最终将数据发布出去给需要用到的另一些节点。总而言之，这类节点有输入或者输出。节点输入意味着需要订阅某些topic，而节点输出代表节点需要发布某些topic，这样就可以完成节点之间的通信。

```
/// file in apollo/modules/perception/perception.cc
Status Perception::Init() {
  AdapterManager::Init(FLAGS_perception_adapter_config_filename);
  ...
}

/// file in apollo/modules/common/adapters/adapter_manager.cc
void AdapterManager::Init(const std::string &adapter_config_filename) {
  // Parse config file
  AdapterManagerConfig configs;
  CHECK(util::GetProtoFromFile(adapter_config_filename, &configs))
      << "Unable to parse adapter config file " << adapter_config_filename;
  AINFO << "Init AdapterManger config:" << configs.DebugString();
  Init(configs);
}
void AdapterManager::Init(const AdapterManagerConfig &configs) {
  ...
  for (const auto &config : configs.config()) {
    switch (config.type()) {
      case AdapterConfig::POINT_CLOUD:
        EnablePointCloud(FLAGS_pointcloud_topic, config);
        break;
      case AdapterConfig::GPS:
        EnableGps(FLAGS_gps_topic, config);
        break;
    ...
  }
}
```

Apollo中所有的topic有：

| topic名称 | 备注说明 |
| --------- | -------- |
| "/apollo/sensor/gnss/odometry" | GPS topic name|
| "/apollo/sensor/gnss/corrected_imu" | "IMU topic name" |
| "/apollo/sensor/gnss/imu" | "Raw IMU topic name" |
| "/apollo/canbus/chassis" | "chassis topic name" |
| "/apollo/canbus/chassis_detail" | "chassis detail topic name" |
| "/apollo/localization/pose" | "localization topic name" |
| "/apollo/planning" | "planning trajectory topic name" |
| "/apollo/monitor" | "ROS topic for monitor" |
| "/apollo/control/pad" | "control pad message topic name" |
| "/apollo/control" | "control command topic name" |
| "/apollo/sensor/velodyne64/compensator/PointCloud2" | "pointcloud topic name" |
| "/apollo/prediction" | "prediction topic name" |
| "/apollo/perception/obstacles" | "perception obstacle topic name" |
| "/apollo/drive_event" | "drive event topic name" |
| "/apollo/perception/traffic_light" | "traffic light detection topic name" |
| "/apollo/routing_request" | "routing request topic name" |
| "/apollo/routing_response" | "routing response topic name" |
| "/apollo/calibration/relative_odometry" | "relative odometry topic name" |
| "/apollo/sensor/gnss/ins_stat" | "ins stat topic name") |
| "/apollo/sensor/gnss/ins_status" | "ins status topic name" |
| "/apollo/sensor/gnss/gnss_status" | "gnss status topic name" |
| "/apollo/monitor/system_status" | "System status topic name" |
| "/apollo/monitor/static_info" | "Static info topic name" |
| "/apollo/sensor/mobileye" | "mobileye topic name" |
| "/apollo/sensor/delphi_esr" | "delphi esr radar topic name" |
| "/apollo/sensor/conti_radar" | "delphi esr radar topic name" |
| "camera/image_raw" | "CompressedImage topic name" |
| "/apollo/sensor/camera/traffic/image_short" | "short camera image topic name" |
| "/apollo/sensor/camera/traffic/image_long" | "long camera image topic name" |
| "/apollo/sensor/gnss/rtk_obs" | "Gnss rtk observation topic name" |
| "/apollo/sensor/gnss/rtk_eph" | "Gnss rtk ephemeris topic name" |
| "/apollo/sensor/gnss/best_pose" | "Gnss rtk best gnss pose" |
| "/apollo/localization/msf_gnss" | "Gnss localization measurement topic name" |
| "/apollo/localization/msf_lidar" | "Lidar localization measurement topic name" |
| "/apollo/localization/msf_sins_pva" | "Localization sins pva topic name" |
| "/apollo/localization/msf_status" | "msf localization status" |
| "/apollo/relative_map" | "relative map" |
| "/apollo/navigation" | "navigation" |

每个topic的注册管理器类创建使用REGISTER_ADAPTER(name)完成，具体的初始化工作则调用宏生成的Enable##name()。进一步具体观察REGISTER_ADAPTER宏部分关键代码:

```
/// file in apollo/modules/common/adapters/adapter_manager.h
#define REGISTER_ADAPTER(name)                                                 \
 public:                                                                       \
  static void Enable##name(const std::string &topic_name,                      \
                           const AdapterConfig &config) {                      \
    instance()->InternalEnable##name(topic_name, config);                      \
  }                                                                            \
  static void Publish##name(const name##Adapter::DataType &data) {             \
    instance()->InternalPublish##name(data);                                   \
  }                                                                            \
  static void Add##name##Callback(name##Adapter::Callback callback) {          \
    CHECK(instance()->name##_)                                                 \
        << "Initialize adapter before setting callback";                       \
    instance()->name##_->AddCallback(callback);                                \
  }                                                                            \
  template <class T>                                                           \
  static void Add##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data), T *obj) {            \
    Add##name##Callback(std::bind(fp, obj, std::placeholders::_1));            \
  }                                                                            \
  template <class T>                                                           \
  static void Add##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data)) {                    \
    Add##name##Callback(fp);                                                   \
  }                                                                            \
 private:                                                                      \
  std::unique_ptr<name##Adapter> name##_;                                      \
  ros::Publisher name##publisher_;                                             \
  ros::Subscriber name##subscriber_;                                           \
  AdapterConfig name##config_;                                                 \
                                                                               \
  void InternalEnable##name(const std::string &topic_name,                     \
                            const AdapterConfig &config) {                     \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, config.message_history_limit())); \
    if (config.mode() != AdapterConfig::PUBLISH_ONLY && IsRos()) {             \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, config.message_history_limit(),  \
                                  &name##Adapter::RosCallback, name##_.get()); \
    }                                                                          \
    if (config.mode() != AdapterConfig::RECEIVE_ONLY && IsRos()) {             \
      name##publisher_ = node_handle_->advertise<name##Adapter::DataType>(     \
          topic_name, config.message_history_limit(), config.latch());         \
    }                                                                          \
                                                                               \
    observers_.push_back([this]() { name##_->Observe(); });                    \
    name##config_ = config;                                                    \
  }      

```

从代码段不难看出，调用一次REGISTER_ADAPTER(name)就会生成该topic的订阅与发布数据成员，以及对应的发布函数，Callback添加函数，如果某个节点需要订阅该topic(即输入是该topic发布的信息)，则只需要使用Add\*Callback把节点的处理函数加入即可自动调用。

#### <a name="共享数据类初始化">2.1.2 ShareData共享数据类初始化</a>

对比上述的ROS消息订阅与发布机制，另一类消息传递机制是依赖手工管理收发消息(下文将提到)。共享数据是针对这类机制设定的，某些节点需要的输入数据保存到共享信息中，节点需要自己调用函数去提取共享数据完成处理。共享数据类初始化主要是创建各个共享数据结构的模板类，在后续DAG初始化工程中调用这些类可以真正实例化共享数据类。

```
/// file in apollo/modules/perception/perception.cc
Status Perception::Init() {
  ...
  RegistAllOnboardClass();
  ...
}
void Perception::RegistAllOnboardClass() {
  /// regist sharedata
  RegisterFactoryLidarObjectData();
  RegisterFactoryRadarObjectData();
  traffic_light::RegisterFactoryTLPreprocessingData();
  ...
}
```

共享数据包含3类，分别为：
- LiDARObjectData/激光测距仪数据，用于障碍物感知/3D Obstacle Perception
- RadarObjectData/雷达数据，用于障碍物感知/3D Obstacle Perception
- TLPreprocessingData/交通灯预处理数据，用于信号灯感知/Traffic Light Perception

以LiDARObjectData注册初始化为例，共享数据初始化分两步：

(1) 对应共享数据容器类注册，注册LidarObjectData

```
/// file in apollo/modules/perception/obstacle/onboard/object_share_data.h
#define OBJECT_SHARED_DATA(data_name)                        \
  class data_name : public CommonSharedData<SensorObjects> { \
   public:                                                   \
    data_name() : CommonSharedData<SensorObjects>() {}       \
    virtual ~data_name() {}                                  \
    std::string name() const override {                      \
      return #data_name;                                     \
    }                                                        \
    ...														 \
  }

OBJECT_SHARED_DATA(LidarObjectData);
...
```

该过程通过宏注册对应的共享数据类，继承CommonSharedData基础类，其中CommonSharedData类包含的元素如下

| 名称 | 返回 | 备注 |
| ---- | ---- | ---- |
| Init() | bool | 初始化标记 |
| name() | bool | 共享数据名称 |
| Reset() | void | 清空所有共享数据，当重置DAGStreaming时，ShareDataManager调用 |
| RemoveStaleData() | void | 清空共享数据map中过时数据，当前时刻-数据时间戳大于人工设定的共享数据过期时间，则清空 |
| Add(const std::string &key, const SharedDataPtr<M> &data) | bool | 根据键-值对添加新的共享数据，key为字符串类型 |
| Add(const CommonSharedDataKey &key, const SharedDataPtr<M> &data) | bool | 根据键-值对添加新的共享数据 ，key为CommonSharedDataKey类型(包含时间戳与设备id) |
| Get(const std::string &key, SharedDataPtr<M> \*data) | bool | 由key获取共享数据，存储进data，key为字符串类型 |
| Get(const CommonSharedDataKey &key, SharedDataPtr<M> \*data) | bool | 由key获取共享数据，存储进data，key为CommonSharedDataKey类型(包含时间戳与设备id) |
| Pop(const std::string &key, SharedDataPtr<M> \*data) | bool | 由key获取共享数据，存储进data，并从map中删除，key为字符串类型 |
| Pop(const CommonSharedDataKey &key, SharedDataPtr<M> \*data) | bool | 由key获取共享数据，存储进data，并从map中删除，key为CommonSharedDataKey类型(包含时间戳与设备id) |
| Remove(const std::string &key) | bool | 根据key删除共享数据，key为字符串类型 |
| Remove(const CommonSharedDataKey &key) | bool | 根据key删除共享数据，key为CommonSharedDataKey类型(包含时间戳与设备id) |
| Size() | unsigned | 共享数据类map中存储的数据量 |
| GetStat() | CommonSharedDataStat | 返回类操作记录，增加数据次数，删除数据次数，获取数据次数 |
| std::map<std::string, SharedDataPtr<M>> SharedDataMap/data_map_ | -- | 共享数据存储容器map |
| std::map<std::string, uint64_t> DataAddedTimeMap/data_added_time_map_ | -- | map中数据加入的时间戳，配合用于删除过时数据 |
| CommonSharedDataStat stat_ | -- | 类操作记录: 增加数据次数，删除数据次数，获取数据次数 |

由上表可知，第一步注册对应的LidarObjectData主要的工作是创建一个Lidar数据的容器类，用以数据的存储，删除与查询。数据以一定格式(ShareData子类)存储在map中，每个数据标有时间戳和设备id，并定时清理旧数据。

(2) 创建共享数据容器类实例与保存类函数，方便实例化LidarObjectData与保存

```
/// file in apollo/modules/perception/obstacle/onboard/object_share_data.h
#define OBJECT_SHARED_DATA(data_name)                        \
  class data_name : public CommonSharedData<SensorObjects> { \
   public:                                                   \
    data_name() : CommonSharedData<SensorObjects>() {}       \
    virtual ~data_name() {}                                  \
    std::string name() const override {                      \
      return #data_name;                                     \
    }                                                        \
    ...                                                      \
  }
REGISTER_SHAREDDATA(LidarObjectData);
...

/// file in apollo/modules/perception/onboard/shared_data.h
#define REGISTER_SHAREDDATA(name) REGISTER_CLASS(SharedData, name)

/// file in apollo/modules/perception/lib/base/registerer.h
typedef std::map<std::string, ObjectFactory *> FactoryMap;
typedef std::map<std::string, FactoryMap> BaseClassMap;
BaseClassMap &GlobalFactoryMap();

#define REGISTER_CLASS(clazz, name)                                           \
  class ObjectFactory##name : public apollo::perception::ObjectFactory {      \
   public:                                                                    \
    virtual ~ObjectFactory##name() {}                                         \
    virtual perception::Any NewInstance() {                                   \
      return perception::Any(new name());                                     \
    }                                                                         \
  };                                                                          \
  inline void RegisterFactory##name() {                                       \
    perception::FactoryMap &map = perception::GlobalFactoryMap()[#clazz];     \
    if (map.find(#name) == map.end()) map[#name] = new ObjectFactory##name(); \
  }
```

总结可知REGISTER_SHAREDDATA宏实际是创建共享数据容器类实例化与保存函数，通过调用该宏生成的函数可以方便的实例化对应的容易类并添加至全局工厂管理类，方便管理所有共享数据实例。E.g. 当在perception.cc的RegistAllOnboardClass中调用RegisterFactoryLidarObjectData()时，实际是实例化对应的容器类LidarObjectData，最终存储进GlobalFactoryMap中，存储的形式为：GlobalFactory[SharedData][LidarObjectData]两级存储。

#### <a name="子节点类初始化">2.1.3 SubNode子节点类初始化</a>

子节点SubNode类初始化与共享数据类初始化相同，主要是创建各个子节点的模板类，在后续DAG初始化工程中调用这些类可以真正实例化子节点类。

```
/// file in apollo/modules/perception/perception.cc
Status Perception::Init() {
  ...
  RegistAllOnboardClass();
  ...
}
void Perception::RegistAllOnboardClass() {
  ...
  /// regist subnode
  RegisterFactoryLidarProcessSubnode();
  RegisterFactoryRadarProcessSubnode();
  RegisterFactoryFusionSubnode();
  traffic_light::RegisterFactoryTLPreprocessorSubnode();
  traffic_light::RegisterFactoryTLProcSubnode();
}
```

子节点SubNode是程序中的某个功能块，每个子节点都是一个线程，在感知模块中，一共存在5个子节点：
- LidarProcessSubnode/激光测距处理子节点，用于障碍物感知/3D Obstacle Perception
- RadarProcessSubnode/雷达处理子节点，用于障碍物感知/3D Obstacle Perception
- FusionSubnode/障碍物结果融合，将上述两类感知结果融合，用于障碍物感知/3D Obstacle Perception
- TLPreprocessSubnode/信号灯预处理子节点，用于信号灯感知/Traffic Light Perception
- TLProcessSubnode/信号灯处理子节点，用于信号灯感知/Traffic Light Perception

以TLPreprocessSubnode注册初始化为例，子节点初始化
```
/// file in apollo/modules/perception/onboard/tl_preprocessor_subnode.h
class TLProcSubnode : public Subnode {
 public:
  ...
 protected:
  ...
 private:
 ...
};
REGISTER_SUBNODE(TLPreprocessorSubnode);

/// file in apollo/modules/perception/onboard/subnode.h
#define REGISTER_SUBNODE(name) REGISTER_CLASS(Subnode, name)

/// file in apollo/modules/perception/lib/base/registerer.h
typedef std::map<std::string, ObjectFactory *> FactoryMap;
typedef std::map<std::string, FactoryMap> BaseClassMap;
BaseClassMap &GlobalFactoryMap();

#define REGISTER_CLASS(clazz, name)                                           \
  class ObjectFactory##name : public apollo::perception::ObjectFactory {      \
   public:                                                                    \
    virtual ~ObjectFactory##name() {}                                         \
    virtual perception::Any NewInstance() {                                   \
      return perception::Any(new name());                                     \
    }                                                                         \
  };                                                                          \
  inline void RegisterFactory##name() {                                       \
    perception::FactoryMap &map = perception::GlobalFactoryMap()[#clazz];     \
    if (map.find(#name) == map.end()) map[#name] = new ObjectFactory##name(); \
  }
```

与前小节类似，REGISTER_SUBNODE宏作用是生成对应的Subnode类，同时创建该Subnode类的实例化与保存函数，通过调用RegisterFactoryTLPreprocessorSubnode可以方便的实例化该子节点类，并保存到全局工厂管理类中，存储的形式为：GlobalFactory[Subnode][TLPreprocessorSubnode]两级存储。

注：代码上的细节，在这里需要对某些类进行全局工厂的存储，既包含SubNode，又包含了ShareData等类。代码上采用两级存储方式，GlobalFactoryMap为<string, map<string, ObjectFactory*>>类型，GlobalFactoryMap[sharedata]存储3类共享数据单例对象，例如，GlobalFactoryMap[sharedata][LidarObjectData]存雷达共享数据对象；GlobalFactoryMap[subnode]存储5类子节点单例对象，例如GlobalFactory[Subnode][TLPreprocessorSubnode]存信号灯预处理数据对象。代码中又是使用Any，又是使用ObjectFactory，还是用了REGISTER_REGISTERER(base_class)和REGISTER_class(clazz, name)宏，本质上是为了对GlobalFactoryMap里面的第二级对象类别进行封装，因为LidarObjectData和TLPreprocessorSubnode是无关类，因此需要将这些无关类封装进Any对象，Any中可以完成封装和装换。ObjectFactory是对各个Any的第二次封装，可以使用Any进行对应类的产生(NewInstance函数)。

TLPreprocessorSubnode继承了Subnode类，可以进一步分析TLPreprocessorSubnod类成分，该类主要包含的元素如下

| 名称 | 返回 | 备注 |
| ---- | ---- | ---- |
| InitInternal() | bool | 内部初始化函数，初始化边界映射函数，共享数据，信号灯预处理器，高清地图等。 |
| InitSharedData() | bool | 初始化共享数据类TLPreprocessingData，在InitInternal函数中被调用 |
| InitPreprocessor() | bool | 初始化预处理器(实际做预处理工作类)，在InitInternal函数中被调用 |
| InitHdmap() | bool | 初始化高清地图类，在InitInternal函数中被调用 |
| AddDataAndPublishEvent(const std::shared_ptr<ImageLights> &data,const CameraId &camera_id, double timestamp) | bool | 将数据存储进共享数据实例的map中，并交由EventManager发布消息信息，在SubCameraImage函数中被调用 |
| SubLongFocusCamera(const sensor_msgs::Image &msg) | void | 选择长焦摄像头回调函数，在InitInternal函数中被调用注册回调函数 |
| SubShortFocusCamera(const sensor_msgs::Image &msg) | void | 选择短焦摄像头回调函数，在InitInternal函数中被调用注册回调函数 |
| SubCameraImage(boost::shared_ptr<const sensor_msgs::Image> msg, CameraId camera_id) | void | 主体函数，获取图像，相机选择，图像处理等 |
| CameraSelection(double ts) | void | 相机选择，在SubCameraImage函数中被调用 |
| VerifyLightsProjection(std::shared_ptr<ImageLights> image_lights) | bool | 验证信号灯映射，在SubCameraImage函数中被调用 |
| GetSignals(double ts, CarPose \*pose, std::vector<apollo::hdmap::Signal> \*signals) | bool |获取汽车姿态与图像信息，，在SubCameraImage函数中被调用 |
| GetCarPose(const double ts, CarPose \*pose) | bool | 获取汽车姿态信息，在GetSignals函数被调用 |
| TLPreprocessor preprocessor_ | -- | 预处理器 |
| TLPreprocessingData \*preprocessing_data_ | -- | 预处理共享数据容器类指针 |
| HDMapInput \*hd_map_ | -- | 高清地图类指针 |
| last_signals_ts_  | -- | 上一次调用GetSignals的时间戳，若相隔太短，直接用上次的信息 |
| std::vector<apollo::hdmap::Signal> last_signals_ | -- | 上一次的信息 |
| last_query_tf_ts_ | -- | 上一次调用CameraSelection时间戳，若相隔太短，则直接调用该步骤，同时跳过GetSignals和CacheLightsProjections |
| last_proc_image_ts_ | -- | 上一次图像处理时间戳，若相隔太短，掉过后续图像选择步骤 |

从TLPreprocessorSubnode成员函数可以看到，子节点类主要为ROS消息发布与订阅机制(或者是定时触发机制)完善回调函数，在回调函数中执行相应的功能，5类子节点都具有相同的类形式，但功能不同。具体的功能在下小节描述。

#### <a name="有向图初始化">2.1.4 DAG有向图初始化</a>

DAG初始化过程主要是构建子节点SubNode，边Edge和共享数据ShareData的一个有向图，并全部实例化得到对应的类对象，关于有向图的三部分内容，程序从config文件读入

```
/// file in apollo/modules/perception/perception.cc
Status Perception::Init() {
  ...
  // work_root: modules/perception
  // dag_config_path: ./conf/dag_streaming.config
  const std::string dag_config_path = apollo::common::util::GetAbsolutePath(
      FLAGS_work_root, FLAGS_dag_config_path);
  ...
}

/// file in apollo/modules/perception/conf/dag_streaming.config
# Define all nodes in DAG streaming.
subnode_config {
    # 64-Lidar Input nodes.
    subnodes {
        id: 1
        name: "LidarProcessSubnode"
        reserve: "device_id:velodyne64;"
        type: SUBNODE_IN
    }
    ...
}

# Define all edges linked nodes.
edge_config {
    # 64-Lidar LidarProcessSubnode -> FusionSubnode
    edges {
        id: 101
        from_node: 1
        to_node: 31
        events {
            id: 1001
            name: "lidar_fusion"
        }
    }
    ...
}

data_config {
    datas {
        id: 1
        name: "LidarObjectData"
    }
    ...
}
```

(1) 在DAG三部分初始化过程中，上小节"子节点SubNode类初始化"仅仅创建该类实例化的函数，5个子节点分别对应5个线程，每个线程设置对应的回调函数，当有输入的时候(收到ROS订阅的消息topic或者定时触发)，自动触发子节点功能，但未真正的实例化该类。在DAG有向图初始化中子节点的初始化工作包含SubNode配置记录(节点id，入度id，出度id)，子节点实例化，通过调用先前创建的函数可以实例化子节点，并保存在GlobalFactory[Subnode][TLPreprocessorSubnode]两级存储中。

```
/// file in apollo/modules/perception/onboard/dag_streaming.cc
bool DAGStreaming::InitSubnodes(const DAGConfig& dag_config) {
  const DAGConfig::SubnodeConfig& subnode_config = dag_config.subnode_config();
  const DAGConfig::EdgeConfig& edge_config = dag_config.edge_config();
  
  map<SubnodeID, DAGConfig::Subnode> subnode_config_map;
  map<SubnodeID, vector<EventID>> subnode_sub_events_map;
  map<SubnodeID, vector<EventID>> subnode_pub_events_map;
   
  for (auto& subnode_proto : subnode_config.subnodes()) {
    std::pair<map<SubnodeID, DAGConfig::Subnode>::iterator, bool> result =
        subnode_config_map.insert(std::make_pair(subnode_proto.id(), subnode_proto));
    ...
  }

  for (auto& edge_proto : edge_config.edges()) {
    SubnodeID from = edge_proto.from_node();
    SubnodeID to = edge_proto.to_node();
    ...
    for (auto& event_proto : edge_proto.events()) {
      subnode_pub_events_map[from].push_back(event_proto.id());
      subnode_sub_events_map[to].push_back(event_proto.id());
    }
  }

  // Generate Subnode instance.
  for (auto pair : subnode_config_map) {
    const DAGConfig::Subnode& subnode_config = pair.second;
    const SubnodeID subnode_id = pair.first;
    Subnode* inst = SubnodeRegisterer::GetInstanceByName(subnode_config.name());
    ...
    bool result = inst->Init(
        subnode_config, &event_manager_, &shared_data_manager_,
        subnode_sub_events_map[subnode_id], subnode_pub_events_map[subnode_id]);
    ...
    subnode_map_.emplace(subnode_id, std::unique_ptr<Subnode>(inst));
  }
  ...
}
```

上述过程前两个for达到记录Edge配置作用，真正意义上的实例化工作由SubnodeRegisterer::GetInstanceByName和Init完成，该过程代码如下

```
/// file in apollo/modules/perception/lib/base/registerer.h
#define REGISTER_REGISTERER(base_class)                               \
  class base_class##Registerer {                                      \
   public:                                                            \
    static base_class *GetInstanceByName(const ::std::string &name) { \
      FactoryMap &map = perception::GlobalFactoryMap()[#base_class];  \
      FactoryMap::iterator iter = map.find(name);                     \
      Any object = iter->second->NewInstance();                       \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
};
```

(2) 在DAG初始化过程中，边Edge初始化需要依赖EvenManager完成，其实本质上Edge是数据的流动过程，也就是消息的订阅与发布过程，所以Apollo中使用EvenManager统一管理各个topic消息的订阅与发布

```
/// file in apollo/modules/perception/onboard/dag_streaming.cc
bool DAGStreaming::Init(const string& dag_config_path) {
  ...
  if (!event_manager_.Init(dag_config.edge_config())) {
    AERROR << "failed to Init EventManager. file: " << dag_config_path;
    return false;
  }
  ...
}

/// file in aploo/modules/perception/onboard/event_manager.cc 
bool EventManager::Init(const DAGConfig::EdgeConfig &edge_config) {
  ...
  for (const DAGConfig::Edge &edge : edge_config.edges()) {
    for (const DAGConfig::Event event_pb : edge.events()) {
      ...
      event_queue_map_[event_pb.id()].reset(
          new EventQueue(FLAGS_max_event_queue_size));

      EventMeta event_meta;
      event_meta.event_id = event_pb.id();
      event_meta.name = event_pb.name();
      event_meta.from_node = edge.from_node();
      event_meta.to_node = edge.to_node();
      event_meta_map_.emplace(event_pb.id(), event_meta);
    }
  }
  ...
}

/// file in apollo/modules/perception/onboard/dag_streaming.h
class EventManager {
  ...
private:
  using EventQueue = FixedSizeConQueue<Event>;
  using EventQueueMap = std::unordered_map<EventID, std::unique_ptr<EventQueue>>;
  using EventMetaMap = std::unordered_map<EventID, EventMeta>;

  EventQueueMap event_queue_map_;
  EventMetaMap event_meta_map_;
};
```
由代码分析可知，EvenManager类包含两个主要的成员变量，分别保存<事件id，消息队列>的event_queue_map_，以及保存<事件id，事件信息>的event_meta_map_(用于调试，打印信息)。一个事件的成分包含id和name。一个完整的Edge总体保存了事件信息(id，name)，入度节点(from_node)，出度节点(to_node)

(3) 最后的共享数据ShareData初始化依赖，共享数据的初始化与子节点初始化相似，主要是做数据的记录以及ShareData的实例化

```
/// file in apollo/modules/perception/onboard/dag_streaming.cc
bool DAGStreaming::Init(const string& dag_config_path) {
 ...
  if (!InitSharedData(dag_config.data_config()) || !InitSubnodes(dag_config)) {
    return false;
  }
  ...
}
bool DAGStreaming::InitSharedData(
    const DAGConfig::SharedDataConfig& data_config) {
  return shared_data_manager_.Init(data_config);
}

/// file in apollo/modules/perception/onboard/shared_data_manager.cc
bool SharedDataManager::Init(const DAGConfig::SharedDataConfig &data_config) {
  for (auto &proto : data_config.datas()) {
    SharedData \*shared_data = SharedDataRegisterer::GetInstanceByName(proto.name());
    auto result = shared_data_map_.emplace(
        shared_data->name(), std::unique_ptr<SharedData>(shared_data));
  }
  return true;
}
```

#### <a name="DAG运行">2.1.5 DAG整体运行实行感知</a>

当DAG完成子节点SubNode，边Edge以及共享数据ShareData的单例对象初始化时，下一步就是启动各个节点的多线程进行工作。运行DAG的过程如下

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/dag_run_process.png)

上图展示了DAG运行的流程图，其中Run函数是线程指定的运行函数。

```
/// file in apollo/modules/perception/lib/base/thread.h && thread.cc
class Thread {
  void Thread::Start() {
    pthread_attr_t attr;
    CHECK_EQ(pthread_attr_init(&attr), 0);
    CHECK_EQ(
      pthread_attr_setdetachstate(
          &attr, joinable_ ? PTHREAD_CREATE_JOINABLE : PTHREAD_CREATE_DETACHED),
      0);
    CHECK_EQ(pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr), 0);
    CHECK_EQ(pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, nullptr), 0);

    int result = pthread_create(&tid_, &attr, &ThreadRunner, this);
    CHECK_EQ(result, 0) << "Could not create thread (" << result << ")";
    CHECK_EQ(pthread_attr_destroy(&attr), 0);
    started_ = true;
}

  static void* ThreadRunner(void* arg) {
    Thread* t = reinterpret_cast<Thread*>(arg);
    t->Run();
    return nullptr;
  }
};
```

最末端DAGStreamingMonitor::Run()函数是DAG监视器线程，主要工作是检测拥塞情况(拥塞值大于实现设定的最大允许拥塞值时重置DAG)以及清除超时的共享数据的。

```
/// file in apollo/modules/perception/onboard/dag_streaming.cc
void DAGStreaming::Schedule() {
  monitor_->Start();
  ...
}

void DAGStreamingMonitor::Run() {
  while (!stop_) {
    if (FLAGS_max_allowed_congestion_value > 0) {
      // Timing to check DAGStreaming congestion value.
      int congestion_value = dag_streaming_->CongestionValue();
      if (congestion_value > FLAGS_max_allowed_congestion_value) {
        dag_streaming_->Reset();
      }
    }

    if (FLAGS_enable_timing_remove_stale_data) {
      dag_streaming_->RemoveStaleData();
    }
    sleep(1);
  }
}
```

第二部分是开启所有子节点SubNode的线程。具体每个SubNode的线程函数可以参考Subnode类

```
/// file in apollo/modules/perception/onboard/dag_streaming.cc
void DAGStreaming::Schedule() {
  ...
  // start all subnodes.
  for (auto& pair : subnode_map_) {
    pair.second->Start();
  }
  for (auto& pair : subnode_map_) {
    pair.second->Join();
  }
  monitor_->Join();
}

/// file in apollo/modules/perception/onboard/subnode.cc
void Subnode::Run() {
  ...
  if (type_ == DAGConfig::SUBNODE_IN) {
    AINFO << "Subnode == SUBNODE_IN, EXIT THREAD. subnode:" << DebugString();
    return;
  }

  while (!stop_) {
    Status status = ProcEvents();
    ++total_count_;
    if (status.code() == ErrorCode::PERCEPTION_ERROR) {
      ++failed_count_;
      ...
      continue;
    }
    ...
  }
}

/// file in apollo/modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h
class TLPreprocessorSubnode : public Subnode {
 public:
  /**
   * @brief as a subnode with type SUBNODE_IN
   *         we will use ros callback, so ignore subnode callback
   */
  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }
}
```

分析上述的代码可知，5个子节点的线程由DAG依次开启。而每个SubNode的Run()函数主要做的工作就是从发布队列和订阅队列中逐个处理数据，ProcEvents()函数清楚的展示了这个过程。这里有个细节，在Apollo代码中，模块/线程间消息传递有两种方式：

- 一类是基于ROS消息与订阅机制。以交通信号灯预处理子节点为例，在tl_preprocessor_subnode.h中，ProcEvents()仅返回OK状态，并未处理任何消息，这是由于该节点使用的是ROS消息与订阅机制，当收到消息时自动跳转到回调函数(请参考节点的InitInternal::Add\*Callback函数)，所以其实不需要人工去捕获消息然后处理。如果节点使用ROS消息订阅机制，不需要考虑ProcEvents()，甚至可以不需要考虑EventManager(注：EventManager的Subsrcibe和Publish其实并不是真正消息的订阅与发布，订阅与发布在PrecEvents里面处理(从对应的共享数据中提取数据处理)，EventManager主要是记录消息传递的信息，E.g. 从哪个节点到哪个节点，发布的时间戳等信息，不包含真正的信息内容).

- 另一类消息接受与发布机制依赖于EvenManager来管理消息，就是采用ProEvents()处理，当节点线程循环执行Run函数去接受列表头的第一个消息，交由ProEvents()去处理消息，同时Run函数可以逐个发布发送队列中的消息。

此外，总结一下5个子节点的消息发布类型：

| 子节点名称 | 消息传递机制 |
| ---------- | ------------------- |
| LidarProcessSubnode | ROS发布与订阅机制 |
| RadarProcessSubnode | ROS发布与订阅机制 |
| FusionSubnode | 自定义ProcEvent消息处理机制，从LidarObjectData和RadarObjectData共享数据中手动提取数据 |
| TLPreprocessorSubnode | ROS发布与订阅机制 |
| TLProcSubnode | 自定义ProcEvent消息处理机制，从TLPreprocessingData共享数据中手动提取数据 |

### <a name="障碍物感知">2.2 障碍物感知: 3D Obstacles Perception</a>

### <a name="信号灯感知">2.3 信号灯感知: Traffic Light Perception</a>

交通信号灯模块主要是提供精确的信号灯识别，通常信号灯有三种状态：

- Red/红灯
- Green/绿灯
- Yellow/黄灯

但是在实际生活中，有可能没有检测到信号灯(未知状态)，也有可能信号灯检测到了但是坏了(黑色状态)，为了充分考虑到所有的状态，在代码Apollo中一共设置了5种状态：

- Red/红灯
- Green/绿灯
- Yellow/黄灯
- Black/黑色
- Unknown/未知

在车辆形式的过程中，高清地图模块HD Map需要重复查询前方是否存在信号灯(注：为什么不直接使用实际摄像头拍摄到的图片进行信号灯存在与否的检测，因为HD Map上的路况图像质量必然不会比现实的差，而现实拍摄的路况图像很有可能受天气或者遮挡等影响而产生误差)。当然HD Map还需要与定位模块配合使用。信号灯可以使用4个角点来描述，如果在信号灯(HD Map中检测到信号灯)，而进一步需要将信号灯从3D的世界坐标系转换到2D的图像坐标系进行进一步的信号灯类型检测，这时候使用的是实际摄像头拍摄照片。

之前的Apollo版本使用的是单摄像头解决方案，而且摄像头只有固定的视野域，不能看到所有的信号灯(前方和两侧)。该方案存在很大的限制因素，主要有：

- 必须保证感知域在100米以上
- 交通信号灯的高度或交叉口的宽度差异很大

在Apollo 2.0中，采用了双摄像头扩大车辆的感知域。

- 长焦摄像头(telephoto camera)，焦距为25mm，主要是拍摄前方较远距离的路况。使用长焦摄像头拍摄到的信号灯尺度大，比较清晰，容易被算法检测到。但是该类摄像头也有一定的缺陷，长焦相机能拍到较远处的信号灯，但是信号灯距离车辆过近亦或者信号灯偏向车辆两侧，长焦相机很大可能捕获不到信号灯图像。
- 广角摄像头(wide-range camera)，焦距为6mm，解决长焦相机无法捕获近距离与两侧信号灯的问题。

在实际检测过程中，使用哪个摄像头有信号灯投影决定。下图是长焦相机与广角相机拍摄的实际图片。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/telephoto_camera.jpg)

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/wide_range_camera.jpg)

信号灯感知可以分为两个阶段：

- 预处理阶段 Preprocess
	- 信号灯投影 Traffic light projection
	- 相机选择 Camera selection
	- 图像与缓存信号灯同步 Image and cached lights sync
- 处理节点 Process
	- 整流 Rectify：提供准确的信号灯标定框
	- 识别 Recognize：识别每个标定框对应的信号灯类型
	- 修正 Revise：参考时间序列进行信号灯修正

[返回目录](#目录头)