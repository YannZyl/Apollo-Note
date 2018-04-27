# Apollo 2.0/2.5 预测模块代码层次结构说明

本文档将从代码层面讲解Apollo Prediction预测模块的工作，其实对比Preception模块与Prediction模块，后者更像是前者的一个子节点，换句话说：Prediction模块就只有一个线程工作。这个线程主要的任务就是接受Preception模块FusionSubnode发布的融合数据，其中包含了障碍物的位置、方向、尺寸、类别、速度与加速度等等信息，来预测障碍物未来时刻的运动轨迹。

我们先从`Prediction::Init`函数了解一下这个模块的输入输出，以及用到的管理器等信息，

```c++
Status Prediction::Init() {
  // Initialization of all managers
  AdapterManager::Init(adapter_conf_);
  ContainerManager::instance()->Init(adapter_conf_);
  EvaluatorManager::instance()->Init(prediction_conf_);
  PredictorManager::instance()->Init(prediction_conf_);


  // Set localization callback function
  AdapterManager::AddCallback(&Prediction::OnLocalization, this);
  // Set planning callback function
  AdapterManager::AddPlanningCallback(&Prediction::OnPlanning, this);
  // Set perception obstacle callback function
  AdapterManager::AddPerceptionObstaclesCallback(&Prediction::RunOnce, this);

  return Status::OK();
}
```

从代码中我们可以看到，Prediction确实很像Perception模块的子节点，通过ROS的topic订阅机制，来完成一些任务，这里主要可以得知两类类型的数据：

1. 第一类为管理器类，做的工作主要是负责一些数据或者api函数的管理，比如`AdapterManager`类的作用是管理各个topic对应的发布订阅函数、回调添加函数等。

2. 第二类为输入输出定义，这里可看到Prediction有三类输入，分别是：车辆位置信息(Localization topic)、高精地图与路径规划信息(Planning topic)，视觉感知障碍物信息(PerceptionObstacles topic)。

注意：其实在Apollo代码中`Prediction::OnLocalization`和`Prediction::OnPlanning`两个回调函数其实只是刷新管理器类中的车辆定位与轨迹数据(Insert函数)，因此这两个函数不需要去过分关注，只要关注`Prediction::RunOnce`即可。下面我们将注意描述上述的各个管理器类和回调函数工作内容与流程。

## ContrainerManager管理器

ContrainerManager管理器本质上和Perception里面的共享容器类类似，它的作用就是存储接收到的车辆位置、车辆轨迹等信息。我们看一下ContrainerManager类的函数与初始化过程。

```c++
ContainerManager::Init 
-> ContainerManager::RegisterContainers 
-> ContainerManager::RegisterContainer 
-> ContainerManager::CreateContainer

/// file in apollo/modules/prediction/container/container_manager.cc
std::unique_ptr<Container> ContainerManager::CreateContainer(
    const common::adapter::AdapterConfig::MessageType& type) {
  std::unique_ptr<Container> container_ptr(nullptr);
  if (type == AdapterConfig::PERCEPTION_OBSTACLES) {         // 1. Obstacle container
    container_ptr.reset(new ObstaclesContainer());
  } else if (type == AdapterConfig::LOCALIZATION) {          // 2. localization container
    container_ptr.reset(new PoseContainer());
  } else if (type == AdapterConfig::PLANNING_TRAJECTORY) {   // 3. trajectory container
    container_ptr.reset(new ADCTrajectoryContainer());
  }
  return container_ptr;
}

void ContainerManager::RegisterContainer(
    const common::adapter::AdapterConfig::MessageType& type) {
  containers_[type] = CreateContainer(type);
}

Container* ContainerManager::GetContainer(
    const common::adapter::AdapterConfig::MessageType& type) {
  if (containers_.find(type) != containers_.end()) {
    return containers_[type].get();
  } else {
    return nullptr;
  }
}
```

从上面代码可以看到ContrainerManager管理器存储了3类子容器，分别为：

- containers_[AdapterConfig::PERCEPTION_OBSTACLES] -- 障碍物容器
- containers_[AdapterConfig::LOCALIZATION] -- 车辆位置容器
- containers_[AdapterConfig::PLANNING_TRAJECTORY] -- 车辆规划轨迹容器

获取容器只要调用如下形式的函数:

`dynamic_cast<ObstaclesContainer*>(ContainerManager::instance()->GetContainer(AdapterConfig::PERCEPTION_OBSTACLES))`

接下来我们分别对这三种容器类进行分析

### PoseContainer车辆姿态容器

车辆姿态容器主要存储的是车辆的世界坐标系坐标、世界坐标系(东-北-天)到车辆坐标系(右-前-上)的变换矩阵、速度信息等信息。

```c++
/// file in apollo/modules/prediction/container/pose/pose_container.cc
void PoseContainer::Insert(const ::google::protobuf::Message& message) {
  localization::LocalizationEstimate localization;
  localization.CopyFrom(dynamic_cast<const LocalizationEstimate&>(message));
  Update(localization);
}

void PoseContainer::Update(const localization::LocalizationEstimate& localization) {

  if (obstacle_ptr_.get() == nullptr) {
    obstacle_ptr_.reset(new PerceptionObstacle());
  }
  obstacle_ptr_->Clear();
  obstacle_ptr_->set_id(ID);				
  Point position;		
  position.set_x(localization.pose().position().x());
  position.set_y(localization.pose().position().y());
  position.set_z(localization.pose().position().z());
  obstacle_ptr_->mutable_position()->CopyFrom(position);      // localization in Earch/North/Up referencr frame

  double theta = 0.0;
  if (localization.pose().has_orientation() &&
      localization.pose().orientation().has_qx() &&
      localization.pose().orientation().has_qy() &&
      localization.pose().orientation().has_qz() &&
      localization.pose().orientation().has_qw()) {
    double qw = localization.pose().orientation().qw();
    double qx = localization.pose().orientation().qx();
    double qy = localization.pose().orientation().qy();
    double qz = localization.pose().orientation().qz();       
    theta = ::apollo::common::math::QuaternionToHeading(qw, qx, qy, qz);
  }
  obstacle_ptr_->set_theta(theta);                            // world2vechile transform matrix

  Point velocity;                                             // velocity in Earch/North/Up referencr frame
  velocity.set_x(localization.pose().linear_velocity().x());
  velocity.set_y(localization.pose().linear_velocity().y());
  velocity.set_z(localization.pose().linear_velocity().z());
  obstacle_ptr_->mutable_velocity()->CopyFrom(velocity);

  obstacle_ptr_->set_type(type_);
  obstacle_ptr_->set_timestamp(localization.header().timestamp_sec());
}
```

### ObstaclesContainer障碍物容器

```c++
void ObstaclesContainer::Insert(const ::google::protobuf::Message& message) {
  PerceptionObstacles perception_obstacles;
  perception_obstacles.CopyFrom(dynamic_cast<const PerceptionObstacles&>(message));

  ObstacleClusters::Init();
  for (const PerceptionObstacle& perception_obstacle : perception_obstacles.perception_obstacle()) {
    InsertPerceptionObstacle(perception_obstacle, timestamp_);
  }
}

void ObstaclesContainer::InsertPerceptionObstacle(
    const PerceptionObstacle& perception_obstacle, const double timestamp) {
  Obstacle* obstacle_ptr = obstacles_.GetSilently(id);
  if (obstacle_ptr != nullptr) {
    obstacle_ptr->Insert(perception_obstacle, timestamp);
  } else {
    Obstacle obstacle;
    obstacle.Insert(perception_obstacle, timestamp);
    obstacles_.Put(id, std::move(obstacle));
  }
}
```

从代码中可以看到障碍物容器做的工作更简单，存储障碍物Obstacle使用的是LRU数据结构(Least Recently Used 近期最少使用算法,应用于缓存中的数据淘汰， 其核心思想是"如果数据最近被访问过，那么将来被访问的几率也更高")。

额外补充一下，Apollo中LRU存储机制实现的工具是哈希map(`std::unordered_map<K, Node<K, V>> map_;`)。里面的元素对是<障碍物id,障碍物节点node>，node的里面有k(障碍物id)和V(Obstacle对象)。每个障碍物都以键值对的形式存储在map中。那么如何体现LRU的形式呢，就是Node是以双向链(prev ptr, next ptr)的形式连接在一起的，双向链最前的节点是最早访问的，最末的节点是最近访问的。

LRU存储的更新方式：

1. 当障碍物没有出现过，即首次出现。那么使用`obstacles_.Put`函数，将Obstacle封装成Node，然后加入map中，同时节点放在双向链的最末端，表示最近刚访问过。如果LRU缓存满了，那么就解绑/删除双向链第一个节点(最早访问的节点)，同时删除map中该节点的键值对，在进行末尾添加
2. 当障碍物出现过了，只要进行更新就行。首先更新map中的val值，然后找到双向链中node的位置，先解绑，然后绑到双向链最末端即可。`obstacle_ptr->Insert`函数即可，`obstacles_.GetSilently(id)`返回的是node的V引用类型(&Obstacle)

### ADCTrajectoryContainer轨迹容器

在障碍物容器和车辆姿态容器中，存储的信息已经比较了解，前者是Perception模块检测到的障碍物信息，后者是定位产生的车辆姿态信息。而在轨迹容器中，我们不曾接触过其中的内容，所以这里我们就对ADCTrajectoryContainer中用到的各种概念做一个简介。

```c++
using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::math::LineSegment2d;
using ::apollo::common::math::Polygon2d;
using ::apollo::common::math::Vec2d;
using ::apollo::hdmap::JunctionInfo;
using ::apollo::planning::ADCTrajectory;
```

**1. PathPoint -- apollo/modules/common/proto/pnc_point.proto**

|  属性名  |  protobuf关键字类型  |                 说明               |
|  ------  |  ------------------  |  --------------------------------- |
|     x    |        optional      |     路径点x坐标(世界坐标系ENU)     |
|     y    |        optional      |     路径点y坐标(世界坐标系ENU)     |
|     z    |        optional      |     路径点z坐标(世界坐标系ENU)     |
|   theta  |        optional      |          x-y平面方向/角度          |
|   kappa  |        optional      |              x-y平面曲线           |
|     s    |        optional      |            与起始点的累计距离      |
|  dkappa  |        optional      |  x-y平面曲线一次导数，用于计算曲率 |
|  ddkappa |        optional      |  x-y平面曲线二次倒数，用于计算曲率 |
|  lane_id |        optional      |       PathPoint点所在道路线id      |

**2. TrajectoryPoint -- apollo/modules/common/proto/pnc_point.proto**

|      属性名     |  protobuf关键字类型  |                 说明               |
|  -------------  |  ------------------  |  --------------------------------- |
|    PathPoint    |        optional      |PathPoint类型，常规的PathPoint路径点|
|        v        |        optional      |              轨迹点速度            |
|        a        |        optional      |             轨迹点加速度           |
|  relative_time  |        optional      |      从轨迹起始点的累计相对时间    |

**3. ADCTrajectory -- apollo/modules/planning/proto/planning.proto**
     
|      属性名     |  protobuf关键字类型  |                        说明                            |
|  -------------  |  ------------------  |         ---------------------------------              |
|    header       |        optional      |                 message头部                            |
|total_path_length|        optional      |                 路径总长度                             |
| trajectory_point|        repeated      |       TrajectoryPoint类型，路径轨迹点集合，数据带速度  |
|  estop          |        optional      |           紧急停车标志，包含是紧急停车标志位与停车理由 |
|  path_point     |        repeated      |  PathPoint类型，路径轨迹点集合，数据不带速度，有冗余性 |
|  is_replan      |        optional      |           轨迹是否需要重新规划                         |
|  gear           |        optional      |           特定轨迹齿轮(这里不做了解Canbus模块)         |
|  decision       |        optional      |  车联决策结果，包含沿车道线前进、停车、倒车、车道线变换、车辆信号等信息 |
|  latency_stats  |        optional      |         延时状态信息                                   |
|  adc_path_point |        repeated      |           现由path_point代替                           |
|  adc_trajectory_point |  repeated      |           现由trajectory_point代替                     |
|  signal         |        optional      |  车辆信号，左转、右转or保持不变，近光灯、远光灯、喇叭、双闪等信号灯指令 |
|  right_of_way_status  |  optional      |          当前车道车辆是否有权通行                      |
|  lane_id        |        repeated      |           当前点位置所有车道线id(相对于参考车道)       |
|  engage_advice  |        optional      |          根据当前的计划结果设置合作建议。              |
| critical_region |        optional      |          关心的区域，当对路况不确定时，该项内容为空    |

从上述三个protobuf的message形式可以看到：

- PathPoint是最基本的点表示形式，包含点的坐标，方向曲率等信息；
- TrajectoryPoint是轨迹表示的基本点，封装了PathPoint的基础上，加入了改点的速度与加速度信息，以及距离起始点的累计时间；
- ADCTrajectory就是完整的轨迹信息，不仅包含了轨迹中等时间的轨迹点，也包含了车辆的决策信息，控制信号，车道线信息等。

## EvaluatorManager管理器

## PredictorManager管理器

## 预测模块工作内容与流程
