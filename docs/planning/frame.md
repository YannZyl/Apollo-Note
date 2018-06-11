# 障碍物&主车轨迹处理器：Frame类

在Frame类中，主要的工作还是对障碍物预测轨迹(由Predition模块得到的未来5s内障碍物运动轨迹)和无人车运动轨迹(ReferenceLineProvider类提供)进行融合，确定障碍物和无人车的位置关系，包括横向距离，纵向距离，车辆可以前进到的位置等等信息。在这里我们主要关注两个内容：

1. 障碍物信息的获取策略
2. 无人车参考线ReferenceLineInof初始化(加入障碍物轨迹信息)

# 一. 障碍物信息的获取策略--滞后预测(Lagged Prediction)

在这个步骤中，主要的工作是获取Prediction模块发布的障碍物预测轨迹数据，并且进行后处理工作。首先回顾一下Prediction模块发布的数据格式：

```protobuf
/// file in apollo/modules/prediction/proto/prediction_obstacle.proto
message Trajectory {
  optional double probability = 1;    // probability of this trajectory，障碍物该轨迹运动方案的概率
  repeated apollo.common.TrajectoryPoint trajectory_point = 2;
}

message PredictionObstacle {
  optional apollo.perception.PerceptionObstacle perception_obstacle = 1;
  optional double timestamp = 2;  // GPS time in seconds
  // the length of the time for this prediction (e.g. 10s)
  optional double predicted_period = 3;
  // can have multiple trajectories per obstacle
  repeated Trajectory trajectory = 4;
}

message PredictionObstacles {
  // timestamp is included in header
  optional apollo.common.Header header = 1;
  // make prediction for multiple obstacles
  repeated PredictionObstacle prediction_obstacle = 2;
  // perception error code
  optional apollo.common.ErrorCode perception_error_code = 3;
  // start timestamp
  optional double start_timestamp = 4;
  // end timestamp
  optional double end_timestamp = 5;
}
```

很明显的看到可以使用`prediction_obstacles.prediction_obstacle()`形式获取所有障碍物的轨迹信息，对于每个障碍物prediction_obstacle，可以使用`prediction_obstacle.trajectory()`获取他所有可能运动方案/轨迹。

此外，可以使用`const auto& prediction = *(AdapterManager::GetPrediction());`来获取Adapter中所有已发布的历史消息，最常见的肯定是取最近发布的PredictionObstacles(`prediction.GetLatestObserved()`)，但是Apollo中采用更为精确地障碍物预测获取方式--滞后预测(Lagged Prediction)，除了使用Prediction模块最近一次发布的信息，同时还是用历史信息中的障碍物轨迹预测数据。

```c++
/// file in apollo/modules/planning/planning.cc
void Planning::RunOnce() {
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp, vehicle_state);
}

/// file in apollo/modules/planning/common/frame.cc
Status Frame::Init() {
  // prediction
  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() && !AdapterManager::GetPrediction()->Empty()) {
    if (FLAGS_enable_lag_prediction && lag_predictor_) {      // 滞后预测策略，获取障碍物轨迹信息
      lag_predictor_->GetLaggedPrediction(&prediction_);
    } else {                                                  // 不采用滞后预测策略，直接取最近一次Prediction模块发布的障碍物信息
      prediction_.CopyFrom(AdapterManager::GetPrediction()->GetLatestObserved());
    }
  }
  ...
}
```

采用滞后预测策略获取障碍物轨迹信息的主要步骤可分为：

1. 最近一次发布的数据直接加入PredictionObstacles容器中

```c++
/// file in apollo/modules/planning/common/lag_prediction.cc
void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const {
  // Step A. 最近一次发布的障碍物轨迹预测信息处理
  const auto adc_position = AdapterManager::GetLocalization()->GetLatestObserved().pose().position();
  const auto latest_prediction = (*prediction.begin());        // 记录最近一次Prediction模块发布的信息
  const double timestamp = latest_prediction->header().timestamp_sec(); // 最近一次发布的时间戳
  std::unordered_set<int> protected_obstacles;
  for (const auto& obstacle : latest_prediction->prediction_obstacle()) {  // 获取最近一次发布的数据中，每个障碍物的运动轨迹信息
    const auto& perception = obstacle.perception_obstacle(); 
    double distance = common::util::DistanceXY(perception.position(), adc_position);
    if (perception.confidence() < FLAGS_perception_confidence_threshold &&  // 障碍物置信度必须大于0.5，获取必须是车辆VEHICLE类，否则不作处理
        perception.type() != PerceptionObstacle::VEHICLE) {
      continue;
    }
    if (distance < FLAGS_lag_prediction_protection_distance) {    // 障碍物与车辆之间的距离小于30m，才设置有效
      protected_obstacles.insert(obstacle.perception_obstacle().id());
      // add protected obstacle
      AddObstacleToPrediction(0.0, obstacle, obstacles);
    }
  }
  ...
}
```

从上面的代码可以看到，滞后预测对于最近一次发布的数据处理比较简单，障碍物信息有效只需要满足两个条件：

- 障碍物置信度(Perception模块CNN分割获得)必须大于0.5，或者障碍物是车辆类
- 障碍物与车辆之间的距离小于30m

2. Prediction发布的历史信息后处理

```c++
/// file in apollo/modules/planning/common/lag_prediction.cc
void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const {
  // Step A 最近一次发布的障碍物轨迹预测信息处理
  ...
  // Step B 过往发布的历史障碍物轨迹预测信息处理
  std::unordered_map<int, LagInfo> obstacle_lag_info;
  int index = 0;  // data in begin() is the most recent data
  for (auto it = prediction.begin(); it != prediction.end(); ++it, ++index) {   // 对于每一次发布的信息进行处理
    for (const auto& obstacle : (*it)->prediction_obstacle()) {                 // 获取该次发布的历史数据中，每个障碍物的运动轨迹信息
      const auto& perception = obstacle.perception_obstacle();
      auto id = perception.id();
      if (perception.confidence() < FLAGS_perception_confidence_threshold &&    // 障碍物置信度必须大于0.5，获取必须是车辆VEHICLE类，否则不作处理
          perception.type() != PerceptionObstacle::VEHICLE) {
        continue;
      }
      if (protected_obstacles.count(id) > 0) {          // 如果障碍物在最近一次发布的信息中出现了，那就忽略，因为只考虑最新的障碍物信息
        continue;  // don't need to count the already added protected obstacle
      }
      auto& info = obstacle_lag_info[id];        
      ++info.count;                          // 记录障碍物在所有历史信息中出现的次数
      if ((*it)->header().timestamp_sec() > info.last_observed_time) {      // 保存最近一次出现的信息，因为只考虑最新的障碍物信息
        info.last_observed_time = (*it)->header().timestamp_sec();
        info.last_observed_seq = index;
        info.obstacle_ptr = &obstacle;
      }
    }
  }
  bool apply_lag = std::distance(prediction.begin(), prediction.end()) >= static_cast<int32_t>(min_appear_num_);
  for (const auto& iter : obstacle_lag_info) {
    if (apply_lag && iter.second.count < min_appear_num_) {       // 历史信息中如果障碍物出现次数小于min_appear_num_/3次，次数太少，可忽略。
      continue;
    }
    if (apply_lag && iter.second.last_observed_seq > max_disappear_num_) { // 历史信息中如果障碍物最近一次发布距离现在过远，可忽略。
      continue;
    }
    AddObstacleToPrediction(timestamp - iter.second.last_observed_time,
                            *(iter.second.obstacle_ptr), obstacles);
  }
}
```

所以最后做一个总结，对于历史发布数据，如何判断这些障碍物轨迹信息是否有效。两个步骤：

步骤1：记录历史发布数据中每个障碍物出现的次数(在最近依次发布中出现的障碍物忽略，因为不是最新的数据了)，必须满足两个条件：

- 障碍物置信度(Perception模块CNN分割获得)必须大于0.5，或者障碍物是车辆类
- 障碍物与车辆之间的距离小于30m

步骤2：对于步骤1中得到的障碍物信息，进行筛选，信息有效需要满足两个条件：

- 信息队列中历史数据大于3(min_appear_num_)，并且每个障碍物出现次数大于3(min_appear_num_)
- 信息队列中历史数据大于3(min_appear_num_)，并且障碍物信息上一次发布距离最近一次发布不大于5(max_disappear_num_)，需要保证数据的最近有效性。


# 二. 无人车与障碍物相对位置的设置--ReferenceLineInfo类初始化

从**1. 障碍物信息的获取策略--滞后预测(Lagged Prediction)**中可以得到障碍物短期(未来5s)内的运动轨迹；从ReferenceLineProvider类中我们可以得到车辆的理想规划轨迹。下一步就是将障碍物的轨迹信息加入到这条规划好的参考线ReferenceLine中，确定在什么时间点，无人车能前进到什么位置，需要保证在这个时间点上，障碍物与无人车不相撞。这个工作依旧是在Frame::Init()中完成，主要是完成ReferenceLineInfo类的生成，这个类综合了障碍物预测轨迹与无人车规划轨迹的信息，同时也是最后路径规划的基础类。

```c++
/// file in apollo/modules/planning/common/frame.cc
Status Frame::Init() {
  // Step A prediction，障碍物预测轨迹信息获取，采用滞后预测策略
  ...
  // Step B.1 检查当前时刻(relative_time=0.0s)，无人车位置和障碍物位置是否重叠(相撞)，如果是，可以直接退出
  const auto *collision_obstacle = FindCollisionObstacle();    
  if (collision_obstacle) { 
    AERROR << "Found collision with obstacle: " << collision_obstacle->Id();
    return Status(ErrorCode::PLANNING_ERROR, "Collision found with " + collision_obstacle->Id());
  }
  // Step B.2 如果当前时刻不冲突，检查未来时刻无人车可以在参考线上前进的位置，ReferenceLineInfo生成
  if (!CreateReferenceLineInfo()) {   //
    AERROR << "Failed to init reference line info";
    return Status(ErrorCode::PLANNING_ERROR, "failed to init reference line info");
  }
  return Status::OK();
}
```

如何完成ReferenceLineInfo类的初始化工作，其实比较简单，主要有两个过程：

- 根据无人车规划路径ReferenceLine实例化ReferenceLineInfo类，数量与ReferenceLine一致
- 根据障碍物轨迹初始化ReferenceLineInfo::path_decision_

1. 实例化ReferenceLineInfo类

```c++
/// file in apollo/modules/planning/common/frame.cc
bool Frame::CreateReferenceLineInfo() {
  // Step A 从ReferenceLineProvider中获取无人车的短期内规划路径ReferenceLine，并进行收缩操作
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines, &segments)) {  
    return false;
  }
  // 对于每条ReferenceLine实例化生成一个对应的ReferenceLineInfo
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }
}

/// file in apollo/modules/planning/common/reference_line_info.cc
ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      lanes_(segments) {}
```

这个规程比较简单，就是从ReferenceLineProvider中提取无人车短期内的规划路径，如果不了解，可以查看[(组件)指引线提供器: ReferenceLineProvider](https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/reference_line_provider.md)。然后由一条ReferenceLine&&segments、车辆状态和规划起始点生成对应的ReferenceLineInfo

2. 初始化ReferenceLineInfo类

```c++
/// file in apollo/modules/planning/common/frame.cc
bool Frame::CreateReferenceLineInfo() {
  // Step A 从ReferenceLineProvider中获取无人车的短期内规划路径ReferenceLine，并进行收缩操作
  ...
  // Step B RerfenceLineInfo初始化
  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
      continue;
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}
```

在`ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles)`函数中，主要做的工作也是比较简单，这里做一下总结：

- 检查无人车是否在参考线上

需要满足无人车对应的边界框start_s和end_s在参考线[0, total_length]区间内

- 检查无人车是否离参考线过远

需要满足无人车start_l和end_l在[-kOutOfReferenceLineL, kOutOfReferenceLineL]区间内，其中kOutOfReferenceLineL取10

- 将障碍物信息加入到ReferenceLineInfo类中

除了将障碍物信息加入到类中，还有一个重要的工作就是确定某个时间点无人车能前进到的位置，如下图所示。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/reference_line_info_obstacle.png)

可以看到这个过程其实就是根据障碍物的轨迹(某个相对时间点，障碍物运动到哪个坐标位置)，并结合无人车查询得到的理想路径，得到某个时间点low_t和high_t无人车行驶距离的下界low_s-adc_start_s和上界high_s-adc_start_s

```c++
/// file in apollo/modules/planning/common/reference_line_info.cc
// AddObstacle is thread safe
PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  // 封装成PathObstacle并加入PathDecision
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
  ...
  // 计算障碍物框的start_s, end_s, start_l和end_l
  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(), &perception_sl)) {
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);
  // 计算障碍物是否对无人车行驶有影响：无光障碍物满足以下条件：
  //    1. 障碍物在ReferenceLine以外，忽略
  //    2. 车辆和障碍物都在车道上，但是障碍物在无人车后面，忽略
  if (IsUnrelaventObstacle(path_obstacle)) {
    // 忽略障碍物
  } else {
    // 构建障碍物在参考线上的边界框
    path_obstacle->BuildReferenceLineStBoundary(reference_line_, adc_sl_boundary_.start_s());
  }
  return path_obstacle;
}
/// file in apollo/modules/planning/common/path_obstacle.cc
void PathObstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line, const double adc_start_s) {

  if (obstacle_->IsStatic() || obstacle_->Trajectory().trajectory_point().empty()) {
    ...
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s, &reference_line_st_boundary_)) {
      ...
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}
``` 

可以观察`PathObstacle::BuildTrajectoryStBoundary`函数，我们简单的进行代码段分析：

Step 1. 首先还是对障碍物轨迹点两两选择，每两个点可以构建上图中的object_moving_box以及object_boundary。

```c++
bool PathObstacle::BuildTrajectoryStBoundary(
    const ReferenceLine& reference_line, const double adc_start_s,
    StBoundary* const st_boundary) {
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double total_length = object_length + common::util::DistanceXY(first_point, second_point); //object_moving_box总长度

    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,             // object_moving_box中心
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(center, first_point.theta(), total_length, object_width);
    ... 
    // 计算object_boundary，由object_moving_box旋转一个heading得到，记录障碍物形式段的start_s, end_s, start_l和end_l
    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s, end_s, &object_boundary)) {  
      return false;
    }
  }
}
```

Step 2. 判断障碍物和车辆水平Lateral距离，如果障碍物在参考线两侧，那么障碍物可以忽略；如果障碍物在参考线后面，也可忽略

```c++
// skip if object is entirely on one side of reference line.
    constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor + adc_width / 2.0;

    if (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >   // 障碍物在参考线左侧，那么无人车可以直接通过障碍物，可忽略障碍物
            skip_l_distance ||
        std::fmax(object_boundary.start_l(), object_boundary.end_l()) <   // 障碍物在参考线右侧，那么无人车可以直接通过障碍物，可忽略障碍物
            -skip_l_distance) {
      continue;
    }

    if (object_boundary.end_s() < 0) {  // 障碍物在参考线后面，可忽略障碍物
      continue;
    }
```

Step 3. 计算low_t和high_t时刻的行驶上下界边界框

```c++
const double delta_t = second_traj_point.relative_time() - first_traj_point.relative_time(); // 0.1s
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;
    double high_s = std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {   // 采用渐进逼近的方法，逐渐计算边界框的下界
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap({low_ref, low_ref.heading(), adc_length, adc_width});
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {  // 采用渐进逼近的方法，逐渐计算边界框的上界
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap({high_ref, high_ref.heading(), adc_length, adc_width});
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t = (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) * delta_t);
      polygon_points.emplace_back(    // 计算low_t时刻的上下界
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) * delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(  // 计算high_t时刻的上下界
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
```

Step 4. 计算完所有障碍物轨迹段的上下界框以后，根据时间t进行排序

```c++
 if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = StBoundary(polygon_points);
    }
  } else {
    return false;
  }
```