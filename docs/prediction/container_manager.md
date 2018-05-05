# ContainerManager管理器

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

## PoseContainer车辆姿态容器

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

## ObstaclesContainer障碍物容器

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

在这里我们额外分析一个知识点，就是上述`obstacle.Insert(perception_obstacle, timestamp);`过程，代码如何将PerceptionObstacle类型插入或者填充到Obstacle类型中？确切的说如何将PerceptionObstacle类型插入到Feature类型中，我们先观察一下Feature这个类包含的元素，Feature是以protobuf的形式声明。

**Feature -- apollo/modules/prediction/proto/feature.proto**
```proto
message Feature {
    // Obstacle ID
    optional int32 id = 1;

    // Obstacle features
    optional apollo.common.Point3D position = 2;        // 物体位置，矢量
    optional apollo.common.Point3D velocity = 3;        // 物体速度，矢量  
    optional apollo.common.Point3D acceleration = 4;    // 物体加速度，矢量
    optional double velocity_heading = 5;               // 物体速度方向，arctan(velocity_y, velocity_x)，由跟踪得到
    optional double speed = 6;                          // 物体速度，标量
    optional double acc = 7;                            // 物体加速度，标量
    optional double theta = 8;                          // 物体偏航角，arctan(direction_y, direction_x)，由MiniBox边框得到
    optional double length = 9;                         // 物体长、宽、高
    optional double width = 10;                         
    optional double height = 11;
    optional double tracking_time = 12;                 // 跟踪时间
    optional double timestamp = 13;                     // 时间戳

    // Obstacle type-specific features
    optional Lane lane = 14;                            // 物体所在车道

    // Obstacle tracked features
    optional apollo.common.Point3D t_position = 16;
    optional apollo.common.Point3D t_velocity = 17 [deprecated = true];
    optional double t_velocity_heading = 18 [deprecated = true];
    optional double t_speed = 19 [deprecated = true];
    optional apollo.common.Point3D t_acceleration = 20 [deprecated = true];
    optional double t_acc = 21 [deprecated = true];

    optional bool is_still = 22 [default = false];
    optional apollo.perception.PerceptionObstacle.Type type = 23;
    optional double label_update_time_delta = 24;
}
```

接下来我们查看`void Obstacle::Insert(const PerceptionObstacle& , const double)`函数流程，这里内容比较多，主要是车道信息插入比较繁琐，主要函数有：

- SetId： 插入id信息
- SetType： 插入物体类别信息
- SetStatus： 插入物体状态信息
  - SetTimestamp： 插入时间戳信息
  - SetPosition： 插入位置信息
  - SetVelocity： 插入速度信息
  - SetAcceleration：插入加速度信息
  - SetTheta: 设置偏航角信息
  - SetLengthWidthHeight： 设置长宽高属性
- SetCurrentLanes： 设置当前车道信息
- SetNearbyLanes：设置附近邻接车道信息
- SetLaneGraphFeature： 设置车况图信息
- InsertFeatureToHistory： 缓存Feature
- SetMotionStatusBySpeed： 设置物体运动状态信息

上述函数中，由于PerceptionObstacle中已经带有了位置、速度等信息，因此前半部分设置比较简单；我们重点介绍关于车道线与车道图的设置，包含`SetCurrentLanes`，`SetNearbyLanes`与`SetLaneGraphFeature`函数，在正式讲解车道信息插入之前，我们首先简单的介绍车道在Apollo中的存储形式，具体的存储过程会在HD Map高精地图章节详细讲解。

车道是一个二维的平面，它具有一定长度，也有不等的宽度，可能是直线段，也可能是弧线。如果存储一条车道，我们使用的思想跟积分类似--分段存储，每段近似一个矩形(其实说梯形更合适，因为宽度可能不一致)。只要给定一条车道的中心线central_curve，两侧道路线就可以大致描述车道形状

```proto
message Lane {
  optional Id id = 1;
  // Central lane as reference trajectory, not necessary to be the geometry central.
  optional Curve central_curve = 2;
  // Lane boundary curve.
  optional LaneBoundary left_boundary = 3;
  optional LaneBoundary right_boundary = 4;
  // in meters.
  optional double length = 5;
  // Speed limit of the lane, in meters per second.
  optional double speed_limit = 6;
  repeated Id overlap_id = 7;
  // All lanes can be driving into (or from).
  repeated Id predecessor_id = 8;
  repeated Id successor_id = 9;
  // Neighbor lanes on the same direction.
  repeated Id left_neighbor_forward_lane_id = 10;
  repeated Id right_neighbor_forward_lane_id = 11;

  optional LaneType type = 12; // oneof in {NONE, CITY_DRIVING, BIKING, SIDEWALK, PARKING}
  optional LaneTurn turn = 13; // oneof in {NO_TURN, LEFT_TURN, RIGHT_TURN, U_TURN}

  repeated Id left_neighbor_reverse_lane_id = 14;
  repeated Id right_neighbor_reverse_lane_id = 15;
  optional Id junction_id = 16;

  // Association between central point to closest boundary.
  repeated LaneSampleAssociation left_sample = 17;
  repeated LaneSampleAssociation right_sample = 18;

  optional LaneDirection direction = 19; // oneof in {FORWARD, BACKWARD, BIDIRECTION}

  // Association between central point to closest road boundary.
  repeated LaneSampleAssociation left_road_sample = 20;
  repeated LaneSampleAssociation right_road_sample = 21;
}

```

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/prediction/container_lane.png)

从上述图中我们可以看到原始的车道是连续的，但是在程序中无法存储连续的车道，因此Apollo采用最常用的离散采样存储，将原始的左右边界和中心线离散化保存，其中每个片段即为一个Segment2d形式的结构体，最终Lane可以使用向量来保存所有的Segment2d段。在保存过程中还有一个比较重要的属性，就是每个Segment2d都有一个start(x1,y1)和end(x2,y2)成员，这是记录该段起始点和结束点的世界坐标系坐标

### SetCurrentLanes当前车道设置

1. 检查是否在路口中 `PredictionMap::InJunction`

```c++
/// file in apollo/modules/prediction/common/prediction_map.cc
bool PredictionMap::InJunction(const Eigen::Vector2d& point, const double radius) {
  auto junction_infos = GetJunctions(point, radius);
  Vec2d vec(point[0], point[1]);
  if (junction_infos.empty()) {
    return false;
  }
  for (const auto junction_info : junction_infos) {
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    Polygon2d junction_polygon{vertices};
    if (junction_polygon.IsPointIn(vec)) {
      return true;
    }
  }
  return false;
}
```

根据物体的位置(x,y)去高精地图HD Map查询该点坐标附近的路口区域。对于每个路口区域进行检查，检查的方法`junction_polygon.IsPointIn(vec)` 在前面RadarPerception中已经有提及，可以参考下图： 

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/radar_roi.png)

2. 检查是否在车道上 `PredictionMap::OnLane`

检查物体是够在车道线上，同样地，根据物体的位置(x,y)去高精地图HD Map查询该点坐标附近的车道信息，在`PredictionMap::OnLane`函数中，使用:

```HDMapUtil::BaseMap().GetLanesWithHeading(hdmap_point, radius, heading, max_lane_angle_diff, &candidate_lanes)```

函数参数解释：

- hdmap_point: 物体坐标，根据这个坐标去查询
- radius： 查询半径，物体到车道中心线小于3米的车道都将被保存下来
- heading： 物体运动方向，
- max_lane_angle_diff: 最大允许偏差方向，车道方向(更准确的说是物体在车道上的投影点方向)和物体运动方向夹角在该值以内，那么这个车道就被保存，夹角过大的车道被删除，
- candidate_lanes： 最终得到的候选车道

GetLanesWithHeading函数完成查询，这个查询的过程很简单：

Step1. 先根据物体当前位置(x,y)，以及设定的查询半径radius去高精地图查询，得到符合条件的所有车道

Step2. 对上述所有的车道，检查物体坐标(x,y)到车道的距离，如果比radius大，则删除，否则进行Step3

Step3. 对Step2中合格(距离小于radius)的车道进行二次筛选，物体坐标投影到车道上得到投影点(x',y')，计算和(x,y)两个点各自方向的夹角，如果大于max_lane_angle_diff，车道被删除。 

Step4. 删除物体不在车道上的那些车道，最后根据夹角由小到大排序，取最前面的max_num_lane条车道作为最终物体车道集合。

上述过程理解比较容易，但是存在两个问题：

Question1：如何计算物体在车道的投影点以及物体到车道的距离？

Question2：如何判断车道是否在车道上？

其实第二个问题比较简单，如何判断物体在车道上，只要判断物体在车道(准确的说车道中心线)上的投影在车道区间内即可。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/prediction/container_onlane.png)

上图很明显的给出了以上两个问题的答案:

a. 如何计算物在车道的投影点和距离，答案很简单，只要遍历Lane里面的所有Segment2d，根据图右边1中的公式计算投影长度|sq|，投影向量sq，如果点p在该Segment2d内，则必有: 0=<|sq|<=|v1|；如果点p在segment外，则有：|sq|<0或|sq|>|v1|。根据图右边2中的公式可以计算物体到Lane的距离|pq|

b. 遍历Lane里面的所有Segment2d，如果物体坐标p在所有的Segment2d段内的投影点都在段外，则可以很明确的判断物体不在该车道内

下面我们看一眼代码验证一下

```c++
/// file in apollo/modules/map/hdmap/hdmap_common.cc
bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s, double *lateral) const {
 
  double min_distance = std::numeric_limits<double>::infinity();
  std::size_t min_index = 0;
  double min_proj = 0.0;
  std::size_t num_segments = segments_.size();
  for (std::size_t i = 0; i < num_segments; ++i) {
    const auto &segment = segments_[i];
    const double distance = segment.DistanceTo(point);          // 计算投影距离|pq|
    if (distance < min_distance) {
      const double proj = segment.ProjectOntoUnit(point);       // 计算投影长度|sq|
      if (proj < 0.0 && i > 0) {                                // 判断投影长度<0，则在段外
        continue;
      }
      if (proj > segment.length() && i + 1 < num_segments) {    // 投影长度>|v1|，则在段外
        const auto &next_segment = segments_[i + 1];
        if ((point - next_segment.start()).InnerProd(next_segment.unit_direction()) >= 0.0) {
          continue;
        }
      }
      min_distance = distance;
      min_index = i;
      min_proj = proj;
    }
  }
}

/// file in apollo/modules/common/math/line_segment2d.cc
double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
  const double x0 = point.x() - start_.x();                 // sp.x，即v2.x
  const double y0 = point.y() - start_.y();                 // sp.y，即v2.y
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y(); 
  if (proj <= 0.0) {            // unit_direction_ = v1 / |v1| = (end.x-start.x, end.y-start.y) / norm(v1)
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }                                         // return abs((v1.x*v2.y-v2.x*v1.y)/sqrt(v1.x*v1.x+v1.y*v1.y))
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
  return unit_direction_.InnerProd(point - start_);         // sq=v1/|v1|*|sq|
}
```

从上面代码我们就不难看到具体的解决方案

### SetNearbyLanes邻接车道设置

### SetLaneGraphFeature车道图结构体设置


## ADCTrajectoryContainer轨迹容器

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
