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

邻接车道设置与当前候选车道设置很相似，由`PredictionMap::NearbyLanesByCurrentLanes`完成原始的车道获取，获取的方式很简单

```c++
/// file in apollo/modules/prediction/common/prediction_map.cc
void PredictionMap::NearbyLanesByCurrentLanes(
    const Eigen::Vector2d& point, const double heading, const double radius,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes,
    const int max_num_lane,
    std::vector<std::shared_ptr<const LaneInfo>>* nearby_lanes) {
  if (lanes.size() == 0) {
    std::vector<std::shared_ptr<const LaneInfo>> prev_lanes(0);
    OnLane(prev_lanes, point, heading, radius, false, max_num_lane,
           FLAGS_max_lane_angle_diff, nearby_lanes);
  } else {
    std::unordered_set<std::string> lane_ids;
    for (auto& lane_ptr : lanes) {
      for (auto& lane_id : lane_ptr->lane().left_neighbor_forward_lane_id()) {
        ...
      }
      for (auto& lane_id : lane_ptr->lane().right_neighbor_forward_lane_id()) {
        ...
      }
    }
  }
}
```

处理过程为：

如果传入的传输lanes为空，那么与调用OnLane函数或者半径radius以内，夹角不超过FLAGS_max_lane_angle_diff的所有lane；否者就直接从hdmap的Lane结构中获取左右两边车道。因为Lane这个protobuf具有

```protobuf
repeated Id left_neighbor_forward_lane_id = 10;
repeated Id right_neighbor_forward_lane_id = 11;

```

### SetLaneGraphFeature车道图结构体设置

车道图结构体设置，主要是讲当前车道和邻接车道信息进行整合，得到当前路况下的一个完整的车道序列。我们通过其定义的protobuf结构了解一下车道序列LaneSequence的一些内容：

```proto
message LanePoint {
    optional apollo.common.Point3D position = 1;
    optional double heading = 2 [default = 0.0];
    optional double width = 3 [default = 0.0];
    optional double relative_s = 4 [default = 0.0];
    optional double relative_l = 5 [default = 0.0];
    optional double angle_diff = 6 [default = 0.0];
}

message LaneSegment {
    optional string lane_id = 1;
    optional double start_s = 2 [default = 0.0];
    optional double end_s = 3 [default = 0.0];
    optional uint32 lane_turn_type = 4 [default = 0];
    repeated LanePoint lane_point = 5;
    optional double total_length = 6 [default = 0.0];
}

message NearbyObstacle {
    optional int32 id = 1;
    optional double s = 2;
}

message LaneSequence {                             
    optional int32 lane_sequence_id = 1;              // 包含序列id
    repeated LaneSegment lane_segment = 2;            // 序列段，与Segments相似，但不同
    repeated NearbyObstacle nearby_obstacle = 3;      // 附近物体信息

    message Features {
        repeated double mlp_features = 1;
    }
    optional Features features = 4;
    optional int32 label = 5 [default = 0];
    optional double probability = 6 [default = 0.0];
    optional double acceleration = 7 [default = 0.0];
    repeated apollo.common.PathPoint path_point = 8;
}

message LaneGraph {
    repeated LaneSequence lane_sequence = 1;  // 车道图有一系列车道序列组成，一个车道序列即为一个Lane
}
```

`Obstacle::SetLaneGraphFeature`函数的作用其实就是把原先的当前车道current_lane和邻接车道nearby_lane封装到LaneSequence里面，最后保存到Feature内。但是这里做了一个处理，即截取当前车道和邻接车道的部分路畅，因为预测是有时间局部性和空间局部性的，那些太长的Lane，只要关注当前位置的邻域即可，具体的邻域如何设置？

```c++
/// file in apollo/modules/prediction/container/obstacles/obstacle.cc
void Obstacle::SetLaneGraphFeature(Feature* feature) {
  double road_graph_distance = std::max(
      speed * FLAGS_prediction_duration +
      0.5 * FLAGS_max_acc * FLAGS_prediction_duration *
      FLAGS_prediction_duration, FLAGS_min_prediction_length);
  // 保存当前车道信息到Feature
  for (auto& lane : feature->lane().current_lane_feature()) {
    ...
  }
  // 保存邻接车道信息到Feature
  for (auto& lane : feature->lane().nearby_lane_feature()) {
    ...
  }

  if (feature->has_lane() && feature->lane().has_lane_graph()) {
    SetLanePoints(feature);
    SetLaneSequencePath(feature->mutable_lane()->mutable_lane_graph());
  }
}
```

可以看到预测的距离(也是图的大小，换句话说也就是物体当前位置邻域半径)计算方式为：$s=vt+0.5at^2$，最基本的物理上的距离计算方式。所以`Obstacle::SetLaneGraphFeature`通过对当前车道CurrentLane和邻接车道NearbyLane截取其邻域，可以降低存储，合情合理。通过截取邻域，那么就可以对截取到的所有Lane(包括CurrentLane和NearbyLane)进行Lane的分割，分割成一个个的LaneSegment，与HD Map中的Segment2d基本类似。(区别：Lane+Segment2d可以存储整条车道，用于HD Map中； LaneSequence+LaneSegment可以存储截取的车道，更加轻量级)

在LaneSequence截取和LaneSegment分割的过程中，代码也相对比较简单，我们可以看一眼代码随便对他做一个解释，便于初学者理解。

```c++
/// file in apollo/master/modules/prediction/common/road_graph.cc
void RoadGraph::ComputeLaneSequence(
    const double accumulated_s, const double start_s,
    std::shared_ptr<const LaneInfo> lane_info_ptr,
    std::vector<LaneSegment>* const lane_segments,
    LaneGraph* const lane_graph_ptr) const {
  // Step 1
  LaneSegment lane_segment;
  lane_segment.set_lane_id(lane_info_ptr->id().id());
  lane_segment.set_start_s(start_s);
  lane_segment.set_lane_turn_type(PredictionMap::LaneTurnType(lane_info_ptr->id().id()));
  if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_) {
    lane_segment.set_end_s(length_ - accumulated_s + start_s);
  } else {
    lane_segment.set_end_s(lane_info_ptr->total_length());
  }
  lane_segment.set_total_length(lane_info_ptr->total_length());

  lane_segments->push_back(std::move(lane_segment));
  // Step 2
  if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_ ||
      lane_info_ptr->lane().successor_id_size() == 0) {
    LaneSequence* sequence = lane_graph_ptr->add_lane_sequence();
    *sequence->mutable_lane_segment() = {lane_segments->begin(),
                                         lane_segments->end()};
    sequence->set_label(0);
  } else {
    const double successor_accumulated_s =
        accumulated_s + lane_info_ptr->total_length() - start_s;
    for (const auto& successor_lane_id : lane_info_ptr->lane().successor_id()) {
      auto successor_lane = PredictionMap::LaneById(successor_lane_id.id());
      ComputeLaneSequence(successor_accumulated_s, 0.0, successor_lane,
                          lane_segments, lane_graph_ptr);
    }
  }
  lane_segments->pop_back();
}

```

我们将上述计算LaneSequence分别两个步骤：

**Step 1 设置LaneSequence的id，起始累计距离，总长度**

注意，下面`if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_)`这个if-else对意思是表明，如果当前物体所在的Lane起始累计距离是start_s，前面计算了邻域为length_($s=vt+0.5at^2$)，路得总长度为total_length。如果`total_length - start_s >= length_`，说明经过这个预测的前进长度以后物体还是在这条Lane上，所以end_s就变为start_s + length_；否者物体就超出了这条Lane，那么end_s也就是Lane的终点total_length. (这里的accumulated_s是需要分段才用到，下面将会提到)

**Step 2 计算LaneSequence**

还是一个if-else结构，当`total_length - start_s >= length_`时，说明物体下一时刻很大可能依旧在这条Lane上，所以当前的LaneSequence就是这条Lane的[start_s, start_s+length_]这段路；如果`total_length - start_s < length_ && lane_info_ptr->lane().successor_id_size() == 0`说明预测前进长度就要超出这条Lane，但是这条Lane没有后继的Lane(言外之意，这条路没有变道的可能)，那么LaneSequence也就只能设置[start_s, total_length]这段路了，其长度是不到length_的。这种情况下LaneGraph只有一个LaneSequence，同时LaneSequence就只有一个LaneSegment

如果`total_length - start_s < length_ && lane_info_ptr->lane().successor_id_size() > 0`说明前进长度要超出这条Lane，但是物体可以变道继续前进，那么这时候一个LaneGraph就有多个LaneSequence，并且LaneSequence就有多个LaneSegment，每个LandSegment的lane_id不一定相同，因为存在物体变道的可能性。这种情况下accumulated_s就起到作用了，他记录了物体前面n个LaneSegment一共的距离，如果物体前面有一个LaneSegment，长度为length_1，那么变道后，只要计算length_ - length_1长度的LaneSegment即可。所以修正后，Step 1中的判断条件就变为`total_length - start_s >= length_ - accumulated_s`，也就是代码中的形式。

下面将举个例子说明上述LaneSequence的计算过程：

1. 假设当前物体所在的车道Lane_1的累计距离start_s=100, 总长度130，这时候经过预测得到的邻域范围为length_=20，由于在前进20米以后不会超出这条Lane，所以得到的LaneSequence只有一个LaneSegment，start_s=100, end_s=120, total_length=130

2. 假设当前物体所在的车道Lane_1的累计距离start_s=100, 总长度110，Lane_1不能变道(successor_id_size()==0)！这时候经过预测得到的邻域范围为length_=20，由于在前进20米以后超出这条Lane，但由于不能变道，所以得到的LaneSequence只有一个LaneSegment，start_s=100, end_s=110, total_length=110

3. 假设当前物体所在的车道Lane_1的累计距离start_s=100, 总长度110，Lane_1可以变道，变道至(Lane_2: start_s=0, total_length=50; Lane_3: start_s=0, total_length=100)！这时候经过预测得到的邻域范围为length_=20，由于在前进20米以后超出这条Lane，但可以变道，所以可以得到2个LaneSequence，每个LaneSequence有2个LaneSegment，分别是：

- LaneSequence 1
  - LaneSegment 1: lane_id=Lane_1, start_s=100, end_s=110, total_length=110
  - LaneSegment 2: lane_id=Lane_2, start_s=0, end_s=10, total_length=50
- LaneSequence 2
  - LaneSegment 1: lane_id=Lane_1, start_s=100, end_s=110, total_length=110
  - LaneSegment 2: lane_id=Lane_3, start_s=0, end_s=10, total_length=100

**其实说白了LaneSequence就是当前情况下物体可能的前进方案，有可能是Lane_1到Lane_2(LaneSequence 1)，也可能是Lane_1到Lane_3(LaneSequence 2)，每种方案中物体经过的路段就是LaneSegment。**

上面是LaneSequence的构建过程，如果当前已经存在了这条物体Lane对应的LaneGraph，那么只要修正一下LaneGraph里面的LaneSequence即可，修正过程只要修改LaneSequence的第一个LaneSegment的start_s即可，后续只要刷新一下里面的数据就行。当完成了LaneGraph的构建，当前车道和邻接车道截取邻域以后，下一步只要对LaneGraph里面LaneSequence中的LaneSegment分段离散化保存即可，E.G. 如果LaneSegment长度为20，那么通过采样，每2m一个点LanePoint，就可以将这条LaneSegment离散化10个点保存。

这个过程主要有`SetLanePoints(feature)`和`SetLaneSequencePath(feature->mutable_lane()->mutable_lane_graph())`函数完成，设置的内容其实与Lane里面的Segment2d相似，

```
lane_point.mutable_position()->set_x(lane_point_pos[0]);      // 等间隔点x坐标
lane_point.mutable_position()->set_y(lane_point_pos[1]);      // 等间隔点y坐标
lane_point.set_heading(lane_point_heading);                   // 采样点运动方便
lane_point.set_width(lane_point_width);                       // 车道宽度
double lane_s = -1.0;              
double lane_l = 0.0;
PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l);
lane_point.set_relative_s(total_s);                     // 相对于车道起始点，累计距离
lane_point.set_relative_l(0.0 - lane_l);                // 点与车道两侧最小距离
lane_point.set_angle_diff(lane_point_angle_diff);       // 物体运动方向与投影点方向夹角
lane_segment->set_lane_turn_type(PredictionMap::LaneTurnType(lane_id));
lane_segment->add_lane_point()->CopyFrom(lane_point);

double delta_theta = apollo::common::math::AngleDiff(second_point->theta(), first_point->theta());
double delta_s = second_point->s() - first_point->s();  
double kappa = std::abs(delta_theta / (delta_s + FLAGS_double_precision));  // LaneSengment中PathPoint之间每1米运动方向变化大小
lane_sequence->mutable_path_point(j)->set_kappa(kappa);
```

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
|   kappa  |        optional      |                x-y平面曲线         |
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
    if (s > FLAGS_adc_trajectory_search_length) {   // 仅考虑搜索半径范围内的轨迹点，太远的不用考虑
      break;
    }
    if (junction_info != nullptr) {                 // 如果找到了距离当前位置最近的路口，成功并退出
      break;
    }
    double x = adc_trajectory_.trajectory_point(i).path_point().x();   // 根据该轨迹点坐标去搜索附近的路口
    double y = adc_trajectory_.trajectory_point(i).path_point().y();
    std::vector<std::shared_ptr<const JunctionInfo>> junctions =
        PredictionMap::GetJunctions({x, y}, FLAGS_junction_search_radius);
    if (!junctions.empty() && junctions.front() != nullptr) {          // 如果找到了设置第一个路口
      junction_info = junctions.front();                               // NOTE: junction_info must be the nearest junction
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
