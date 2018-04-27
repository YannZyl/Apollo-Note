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

总体上讲一个ADCTrajectory就是当前位置到目标位置的一条路径，起点是当前位置，重点是目标位置，里面的信息都是当前时刻的信息，例如lane_id就是当前点(起始点)车辆附近的车道线信息；decision是当前时刻，车辆的一些决策信息。当车辆前进过程中，ADCTrajectory会实时更新，里面的信息都是当前时刻的一些路况和决策信息。


**4. JunctionInfo -- apollo/modules/map/hdmap/hdmap_common.h**

```proto
/// file in apollo/modules/map/proto/map_id.proto
message Id {                  // Global unique ids for all objects (include lanes, junctions, overlaps, etc).
  optional string id = 1;
}
/// file in apollo/modules/map/proto/map_geometry.proto
message Polygon {
  repeated apollo.common.PointENU point = 1;  // East-North-Up coordinate referencr: (x,y,z)
}
/// file in apollo/modules/map/proto/map_junction.proto
message Junction {                  // An junction is the junction at-grade of two or more roads crossing.
  optional Id id = 1;
  optional Polygon polygon = 2;
  repeated Id overlap_id = 3;
}
```

|      成员名     |         返回值       |                        说明                            |
|  -------------  |  ------------------  |         ---------------------------------              |
|    id()         |        ID            |                 返回路口的id                           |
|    junction()   |        Junction      |                 返回路口的结构体(Id,Polygen)           |
| polygon()       |        Polygon2d     |   返回路由的多边形区域，Polygon2d类型，由junction().polygon转换而来(去掉过近的顶点)   |
|  OverlapStopSignIds |  vector<Id>      |           返回停车标志,由junction().overlap_id得到     |
|  Init()         |        void          |  从 Junction message中得到id，polygon和overlap_id      |
|  PostProcess()  |        void          |           确定当前地图上有的物体(overlap)，如车道线、信号灯、停车标志灯       |
|  junction_      |        -             |          路口结构体，存储路口id、路口多边形凸包、当前路口的overlap id             |
|  polygon_       |        -             |  路口多边形凸包，由junction_.polygon()得到，去除过近的顶点 |
|  overlap_stop_sign_ids_  |      -      |  vector<Id>，路口的覆盖物体的id                        |
|  overlap_ids_   |        -             |  vector<Id>，覆盖区域集合，当前地图可能有多快覆盖区域  |

从上述message和类中，可以看到JunctionInfo类，主要是包含了当前路口的多边形凸包、覆盖区域及其区域中的物体id。接下去我们从代码入手，查看一下`ADCTrajectoryContainer::Insert`回调函数做的工作内容：

```c++
/// file in apollo/modules/prediction/container/adc_trajectory/adc_trajectory_container.cc
void ADCTrajectoryContainer::Insert(const ::google::protobuf::Message& message) {
  adc_trajectory_.CopyFrom(dynamic_cast<const ADCTrajectory&>(message));
  // Find junction
  if (IsProtected()) {
    SetJunctionPolygon();
  }
  // Find ADC lane sequence
  SetLaneSequence();
}

void ADCTrajectoryContainer::SetJunctionPolygon() {
  std::shared_ptr<const JunctionInfo> junction_info(nullptr);

  for (int i = 0; i < adc_trajectory_.trajectory_point_size(); ++i) {
    double s = adc_trajectory_.trajectory_point(i).path_point().s();
    if (s > FLAGS_adc_trajectory_search_length) {
      break;
    }
    if (junction_info != nullptr) {
      break;
    }
    double x = adc_trajectory_.trajectory_point(i).path_point().x();
    double y = adc_trajectory_.trajectory_point(i).path_point().y();
    std::vector<std::shared_ptr<const JunctionInfo>> junctions =
        PredictionMap::GetJunctions({x, y}, FLAGS_junction_search_radius);
    if (!junctions.empty() && junctions.front() != nullptr) {
      junction_info = junctions.front();
    }
  }

  if (junction_info != nullptr && junction_info->junction().has_polygon()) {
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    if (vertices.size() >= 3) {
      adc_junction_polygon_ = Polygon2d{vertices};
    }
  }
}

void ADCTrajectoryContainer::SetLaneSequence() {
  for (const auto& lane : adc_trajectory_.lane_id()) {
    if (!lane.id().empty()) {
      adc_lane_seq_.emplace_back(lane.id());
    }
  }
  adc_lane_ids_.clear();
  adc_lane_ids_.insert(adc_lane_seq_.begin(), adc_lane_seq_.end());
}
```

上述代码比较长，我们将介绍一下insert过程的工作，Insert函数主要包含两个工作内容：路口查找和车道线设置。通过分析我们可以得到：

1. 参数`::google::protobuf::Message& message`指的就是`planning::ADCTrajectory`，它是一条规划好的，从当前位置到目标地点的完整路径，路径由一个个的点组成(`path_point`或者`trajectory_point`)

2. `SetJunctionPolygon`作用是为车辆寻找下一个路口，前提条件是当前道路是有权通行的。寻找下一个路口的方法为：

- 遍历路径上的所有点，查看坐标点的s。s越大说明离当前位置越远，当s超过阈值(`FLAGS_adc_trajectory_search_length`，默认10m)，就不考虑这个点了，因为未来太远了，不需要过早地去寻找路口，当靠近的时候可以再寻找该点附近的路口。
- 如果s在10m以内，那么根据这个点的世界坐标(x,y)去高精地图查找该点附近(`FLAGS_junction_search_radius`，默认1m)的路口信息。如果查到了该点附近有路口(!junctions.empty())，那么最近路口就设置为这个路口；否则就查询下一个点的附近路口信息
- 当有查到路口信息时(junction_info != nullptr)，结束循环，先处理这个路口；然后车辆前进一段距离，得到新的AdjTractory去寻找更远的路口。同时设置路口的多边形区域/凸包。

3. `SetLaneSequence`设置当前位置的车道线信息。

## EvaluatorManager管理器

## PredictorManager管理器

## 预测模块工作内容与流程
