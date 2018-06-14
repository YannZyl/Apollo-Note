# 障碍物&参考线&交通规则融合器：Frame类

在Frame类中，主要的工作还是对障碍物预测轨迹(由Predition模块得到的未来5s内障碍物运动轨迹)、无人车参考线(ReferenceLineProvider类提供)以及当前路况(停车标志、人行横道、减速带等)信息进行融合。结合路况和障碍物轨迹，最后会给每个障碍物一个标签，这个标签表示该障碍物存在情况下对无人车的影响，例如有些障碍物可忽略，有些障碍物会超车，有些障碍物在运动过程中无人车需要停车等：

1. 障碍物信息的获取策略
2. 无人车参考线ReferenceLineInof初始化(加入障碍物轨迹信息)
3. 依据交通规则对障碍物设定标签

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

总结一下**无人车参考线ReferenceLineInof初始化(加入障碍物轨迹信息)**这步的功能，给定了无人车的规划轨迹ReferenceLine和障碍物的预测轨迹PredictionObstacles，这个过程其实就是计算障碍物在无人车规划轨迹上的重叠部分的位置s，以及驶到这个重叠部分的时间点t，为第三部分每个障碍物在每个车辆上的初步决策进行规划。

# 三. 依据交通规则对障碍物设定标签

二中已经给出了每个障碍物在所有的参考线上的重叠部分的位置与时间点，这些重叠部分就是无人车需要考虑的规划矫正问题，防止交通事故。因为障碍物运动可能跨越多个ReferenceLine，所以这里需要对于每个障碍物进行标定，是否可忽略，是否会超车等。交通规则判定情况一共11种，在文件`modules/planning/conf/traffic_rule_config.pb.txt`中定义，这里我们将一一列举。

1. 后车情况处理--BACKSIDE_VEHICLE
2. 变道情况处理--CHANGE_LANE
3. 人行横道情况处理--CROSSWALK
4. 目的地情况处理--DESTINATION
5. 前车情况处理--FRONT_VEHICLE
6. 禁停区情况处理--KEEP_CLEAR
7. 寻找停车点状态--PULL_OVER
8. 车道线结束情况处理--REFERENCE_LINE_END
9. 重新路由查询情况处理--REROUTING
10. 信号灯情况处理--SIGNAL_LIGHT
11. 停车情况处理--STOP_SIGN

```c++
/// file in apollo/modules/planning/planning.cc
void Planning::RunOnce() {
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp, vehicle_state);

  for (auto& ref_line_info : frame_->reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      continue;
    }
  }
}

/// file in apollo/modules/planning/tasks/traffic_decider/traffic_decider.cc
Status TrafficDecider::Execute(Frame *frame, ReferenceLineInfo *reference_line_info) {
  for (const auto &rule_config : rule_configs_.config()) {   // 对于每条参考线进行障碍物的决策。
    if (!rule_config.enabled()) {
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config);
    if (!rule) {
      continue;
    }
    rule->ApplyRule(frame, reference_line_info);
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}
```

## 3.1 后车情况处理--BACKSIDE_VEHICLE

```c++
/// file in apollo/modules/planning/tasks/traffic_decider/backside_vehicle.cc
Status BacksideVehicle::ApplyRule(Frame* const, ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  if (reference_line_info->Lanes().IsOnSegment()) {  // The lane keeping reference line.
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return Status::OK();
}

void BacksideVehicle::MakeLaneKeepingObstacleDecision(const SLBoundary& adc_sl_boundary, PathDecision* path_decision) {
  ObjectDecisionType ignore;   // 从这里可以看到，对于后车的处理主要是忽略
  ignore.mutable_ignore();
  const double adc_length_s = adc_sl_boundary.end_s() - adc_sl_boundary.start_s(); // 计算"车长""
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) { // 对于每个与参考线有重叠的障碍物进行规则设置
    ...  
  }
}
```

从上述代码可以看到后车情况处理仅仅针对当前无人车所在的参考线，那些临近的参考线不做考虑。Apollo主要的处理方式其实是忽略后车，可以分一下情况：

- 前车在这里不做考虑，由前车情况处理FRONT_VEHICLE来完成

```c++
if (path_obstacle->PerceptionSLBoundary().end_s() >= adc_sl_boundary.end_s()) {  // don't ignore such vehicles.
  continue;
}
```

- 参考线上没有障碍物运动轨迹，直接忽略

```c++
if (path_obstacle->reference_line_st_boundary().IsEmpty()) {
  path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region", path_obstacle->Id(), ignore);
  path_decision->AddLateralDecision("backside_vehicle/no-st-region", path_obstacle->Id(), ignore);
  continue;
}
```

- 忽略从无人车后面来的车辆

```c++
// Ignore the car comes from back of ADC
if (path_obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {  
  path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc", path_obstacle->Id(), ignore);
  path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc", path_obstacle->Id(), ignore);
  continue;
}
```

从代码中可以看到，通过计算min_s，也就是障碍物轨迹距离无人车最近的距离，小于半车长度，说明车辆在无人车后面，可暂时忽略。

- 忽略后面不会超车的车辆

```c++
const double lane_boundary = config_.backside_vehicle().backside_lane_width();  // 4m
  if (path_obstacle->PerceptionSLBoundary().start_s() < adc_sl_boundary.end_s()) {
    if (path_obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
        path_obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
      continue;
    }
    path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s", path_obstacle->Id(), ignore);
    path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s", path_obstacle->Id(), ignore);
    continue;
  }
}
```

从代码中可以看到，第一个if可以选择那些在无人车后(至少不超过无人车)的车辆，第二个if，可以计算车辆与无人车的横向距离，如果超过阈值，那么就说明车辆横向距离无人车足够远，有可能会进行超车，这种情况不能忽略，留到后面的交通规则去处理；若小于这个阈值，则可以间接说明不太会超车，后车可能跟随无人车前进。

## 3.2 变道情况处理--CHANGE_LANE

在变道情况下第一步要找到那些需要警惕的障碍物(包括跟随这辆和超车车辆)，这些障碍物轨迹是会影响无人车的变道轨迹，然后设置每个障碍物设立一个超车的警示(障碍物和无人车的位置与速度等信息)供下一步规划阶段参考。Apollo对于每条参考线每次只考虑距离无人车最近的能影响变道的障碍物，同时设置那些超车的障碍物。

```c++
/// file in apollo/modules/planning/tasks/traffic_decider/change_lane.cc
Status ChangeLane::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // 如果是直行道，不需要变道，则忽略
  if (reference_line_info->Lanes().IsOnSegment()) {
    return Status::OK();
  }
  // 计算警示障碍物&超车障碍物
  guard_obstacles_.clear();
  overtake_obstacles_.clear();
  if (!FilterObstacles(reference_line_info)) {
    return Status(common::PLANNING_ERROR, "Failed to filter obstacles");
  }
  // 创建警示障碍物类
  if (config_.change_lane().enable_guard_obstacle() && !guard_obstacles_.empty()) {
    for (const auto path_obstacle : guard_obstacles_) {
      auto* guard_obstacle = frame->Find(path_obstacle->Id());
      if (guard_obstacle &&  CreateGuardObstacle(reference_line_info, guard_obstacle)) {
        AINFO << "Created guard obstacle: " << guard_obstacle->Id();
      }
    }
  }
  // 设置超车标志
  if (!overtake_obstacles_.empty()) {
    auto* path_decision = reference_line_info->path_decision();
    const auto& reference_line = reference_line_info->reference_line();
    for (const auto* path_obstacle : overtake_obstacles_) {
      auto overtake = CreateOvertakeDecision(reference_line, path_obstacle);
      path_decision->AddLongitudinalDecision(
          TrafficRuleConfig::RuleId_Name(Id()), path_obstacle->Id(), overtake);
    }
  }
  return Status::OK();
}
```

1. 超车&警示障碍物计算

- 没有轨迹的障碍物忽略

```c++
if (!obstacle->HasTrajectory()) {
  continue;
}
```

- 无人车前方的车辆忽略，对变道没有影响

```c++
if (path_obstacle->PerceptionSLBoundary().start_s() > adc_sl_boundary.end_s()) {
  continue;
}
```

- 跟车在一定距离(10m)内，将其标记为超车障碍物

```c++
if (path_obstacle->PerceptionSLBoundary().end_s() <
        adc_sl_boundary.start_s() -
            std::max(config_.change_lane().min_overtake_distance(),   // min_overtake_distance: 10m
                     obstacle->Speed() * min_overtake_time)) {        // min_overtake_time: 2s
  overtake_obstacles_.push_back(path_obstacle);
}
```

- 障碍物速度很小或者障碍物最后不在参考线上，对变道没影响，可忽略

```c++
if (last_point.v() < config_.change_lane().min_guard_speed()) {
  continue;
}
if (!reference_line.IsOnRoad(last_point.path_point())) {
  continue;
}
```

- 障碍物最后一规划点在参考线上，但是超过了无人车一定距离，对变道无影响

```c++
SLPoint last_sl;
if (!reference_line.XYToSL(last_point.path_point(), &last_sl)) {
  continue;
}
if (last_sl.s() < 0 || last_sl.s() > adc_sl_boundary.end_s() + kGuardForwardDistance) {
  continue;
}
```

2. 创建警示障碍物类

创建警示障碍物类本质其实就是预测障碍物未来一段时间内的运动轨迹，代码在`ChangeLane::CreateGuardObstacle`中很明显的给出了障碍物轨迹的预测方法。预测的轨迹是在原始轨迹上进行拼接，即在最后一个轨迹点后面再次进行预测，这次预测的假设是，障碍物沿着参考线形式。几个注意点：

预测长度：障碍物预测轨迹重点到无人车前方100m(`config_.change_lane().guard_distance()`)这段距离

障碍物速度假定：这段距离内，默认障碍物速度和最后一个轨迹点速度一致`extend_v`，并且验证参考线前进

预测频率: 没两个点之间的距离为障碍物长度，所以两个点之间的相对时间差为：`time_delta = kStepDistance / extend_v`

3. 创建障碍物超车标签

```c++
/// file in apollo/modules/planning/tasks/traffic_decider/change_lane.cc
ObjectDecisionType ChangeLane::CreateOvertakeDecision(
    const ReferenceLine& reference_line, const PathObstacle* path_obstacle) const {
  ObjectDecisionType overtake;
  overtake.mutable_overtake();
  const double speed = path_obstacle->obstacle()->Speed();
  double distance = std::max(speed * config_.change_lane().min_overtake_time(),  // 设置变道过程中，障碍物运动距离
                             config_.change_lane().min_overtake_distance());
  overtake.mutable_overtake()->set_distance_s(distance);   
  double fence_s = path_obstacle->PerceptionSLBoundary().end_s() + distance; 
  auto point = reference_line.GetReferencePoint(fence_s);               // 设置变道完成后，障碍物在参考线上的位置
  overtake.mutable_overtake()->set_time_buffer(config_.change_lane().min_overtake_time()); // 设置变道需要的最小时间
  overtake.mutable_overtake()->set_distance_s(distance);               // 设置变道过程中，障碍物前进的距离
  overtake.mutable_overtake()->set_fence_heading(point.heading());
  overtake.mutable_overtake()->mutable_fence_point()->set_x(point.x()); // 设置变道完成后，障碍物的坐标
  overtake.mutable_overtake()->mutable_fence_point()->set_y(point.y());
  overtake.mutable_overtake()->mutable_fence_point()->set_z(0.0);
  return overtake;
}
```

## 3.3 人行横道情况处理--CROSSWALK

对于人行横道部分，根据礼让规则当行人或者非机动车距离很远，无人车可以开过人行横道；当人行横道上有人经过时，必须停车让行。

```c++
/// file in apollo/modules/planning/tasks/traffic_decider/crosswalk.cc
Status Crosswalk::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // 检查是否存在人行横道区域
  if (!FindCrosswalks(reference_line_info)) {
    return Status::OK();
  }
  // 为每个障碍物做标记，障碍物存在时，无人车应该停车还是直接驶过
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}
```

1. 检查在每个人行横道区域内，是否存在障碍物需要无人车停车让行

- 如果车辆已经驶过人行横道一部分了，那么就忽略，不需要停车

```c++
// skip crosswalk if master vehicle body already passes the stop line
double stop_line_end_s = crosswalk_overlap->end_s;    
if (adc_front_edge_s - stop_line_end_s > config_.crosswalk().min_pass_s_distance()) {  // 车头驶过人行横道一定距离，min_pass_s_distance：1.0
  continue;
}
```

遍历每个人行道区域的障碍物(行人和非机动车)，并将人行横道区域扩展，提高安全性。

- 如果障碍物不在扩展后的人行横道内，则忽略。(可能在路边或者其他区域)

```C++
// expand crosswalk polygon
// note: crosswalk expanded area will include sideway area
Vec2d point(perception_obstacle.position().x(),
            perception_obstacle.position().y());
const Polygon2d crosswalk_poly = crosswalk_ptr->polygon();
bool in_crosswalk = crosswalk_poly.IsPointIn(point);
const Polygon2d crosswalk_exp_poly = crosswalk_poly.ExpandByDistance(
         config_.crosswalk().expand_s_distance());
bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);

if (!in_expanded_crosswalk) {
  continue;
}
```

计算障碍物到参考线的横向距离`obstacle_l_distance `，是否在参考线上(附近)`is_on_road `，障碍物轨迹是否与参考线相交`is_path_cross `

- 如果横向距离大于疏松距离。如果轨迹相交，那么需要停车；反之直接驶过(此时轨迹相交无所谓，因为横向距离比较远)

```c++
if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {  // stop_loose_l_distance: 5.0m
  // (1) when obstacle_l_distance is big enough(>= loose_l_distance),  
  //     STOP only if path crosses
  if (is_path_cross) {
    stop = true;
  }
}
```

- 如果横向距离小于紧凑距离。如果障碍物在参考线或者轨迹相交，那么需要停车；反之直接驶过

```c++
else if (obstacle_l_distance <= config_.crosswalk().stop_strick_l_distance()) { // stop_strick_l_distance: 4.0m
  // (2) when l_distance <= strick_l_distance + on_road(not on sideway),
  //     always STOP
  // (3) when l_distance <= strick_l_distance + not on_road(on sideway),
  //     STOP only if path crosses
  if (is_on_road || is_path_cross) {
    stop = true;
  }
} 
```

- 如果横向距离在紧凑距离和疏松距离之间，直接停车

```c++
else {
  // TODO(all)
  // (4) when l_distance is between loose_l and strick_l
  //     use history decision of this crosswalk to smooth unsteadiness
  stop = true;
}
```

如果存在障碍物需要无人车停车让行，最后可以计算无人车的加速度(是否来的及减速，若无人车速度很快减速不了，那么干脆直接驶过)。计算加速的公式还是比较简单

$$ 0 - v^2 = 2as $$

s为当前到车辆停止驶过的距离，物理公式，由函数`util::GetADCStopDeceleration`完成。

2. 对那些影响无人车行驶的障碍物构建虚拟墙障碍物类以及设置停车标签

什么是构建虚拟墙类，其实很简单，单一的障碍物是一个很小的框，那么无人车在行驶过程中必须要与障碍物保持一定的距离，那么只要以障碍物为中心，构建一个长度为0.1，宽度为车道宽度的虚拟墙，只要保证无人车和这个虚拟墙障碍物不相交，就能确保安全。

```c++
// create virtual stop wall
std::string virtual_obstacle_id =
      CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, crosswalk_overlap->start_s);
if (!obstacle) {
  AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
  return -1;
}
PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
if (!stop_wall) {
  AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
  return -1;
}
```

虽然看起来函数跳转比较多，但是其实与二中的障碍物PredictionObstacle封装成Obstacle一样，无非是多加了一个区域框Box2d而已。最后就是对这些虚拟墙添加停车标志

```c++
// build stop decision
const double stop_s =         // 计算停车位置的累计距离，stop_distance：1m，人行横道前1m处停车
      crosswalk_overlap->start_s - config_.crosswalk().stop_distance();
auto stop_point = reference_line.GetReferencePoint(stop_s);
double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

ObjectDecisionType stop;
auto stop_decision = stop.mutable_stop();
stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CROSSWALK);
stop_decision->set_distance_s(-config_.crosswalk().stop_distance());
stop_decision->set_stop_heading(stop_heading);                 // 设置停车点的角度/方向
stop_decision->mutable_stop_point()->set_x(stop_point.x());    // 设置停车点的坐标
stop_decision->mutable_stop_point()->set_y(stop_point.y());
stop_decision->mutable_stop_point()->set_z(0.0);

for (auto pedestrian : pedestrians) {
  stop_decision->add_wait_for_obstacle(pedestrian);  // 设置促使无人车停车的障碍物id
}

auto* path_decision = reference_line_info->path_decision();
path_decision->AddLongitudinalDecision(   
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);
```

## 3.4 目的地情况处理--DESTINATION

到达目的地情况下，障碍物促使无人车采取的行动无非是靠边停车或者寻找合适的停车点(距离目的地还有一小段距离，不需要激励停车)。决策规则比较简单：

1. 主要的决策逻辑

- 若当前正处于PULL_OVER状态，则继续保持

```c++
auto* planning_state = GetPlanningStatus()->mutable_planning_state();
if (planning_state->has_pull_over() && planning_state->pull_over().in_pull_over()) {
  PullOver(nullptr);
  ADEBUG << "destination: continue PULL OVER";
  return 0;
}
```

- 检查车辆当前是否需要进入ULL_OVER，主要参考和目的地之间的距离以及是否允许使用PULL_OVER

```c++
const auto& routing = AdapterManager::GetRoutingResponse()->GetLatestObserved();
const auto& routing_end = *routing.routing_request().waypoint().rbegin();
double dest_lane_s = std::max(       // stop_distance: 0.5，目的地0.5m前停车
      0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
      config_.destination().stop_distance()); 
common::PointENU dest_point;
if (CheckPullOver(reference_line_info, routing_end.id(), dest_lane_s, &dest_point)) {
  PullOver(&dest_point);
} else {
  Stop(frame, reference_line_info, routing_end.id(), dest_lane_s);
}
```

2. CheckPullOver检查机制(Apollo在DESTINATION中不启用PULL_OVER)

- 若在目的地情况下不启用PULL_OVER机制，则返回false

```c++
if (!config_.destination().enable_pull_over()) {
  return false;
}
```

- 若目的地不在参考线上，返回false

```c++
const auto dest_lane = HDMapUtil::BaseMapPtr()->GetLaneById(hdmap::MakeMapId(lane_id));
const auto& reference_line = reference_line_info->reference_line();
// check dest OnRoad
double dest_lane_s = std::max(
    0.0, lane_s - FLAGS_virtual_stop_wall_length -
    config_.destination().stop_distance());
*dest_point = dest_lane->GetSmoothPoint(dest_lane_s);
if (!reference_line.IsOnRoad(*dest_point)) {
  return false;
}
```

- 若无人车与目的地距离太远，则返回false

```c++
// check dest within pull_over_plan_distance
common::SLPoint dest_sl;
if (!reference_line.XYToSL({dest_point->x(), dest_point->y()}, &dest_sl)) {
  return false;
}
double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
double distance_to_dest = dest_sl.s() - adc_front_edge_s;
// pull_over_plan_distance: 55m
if (distance_to_dest > config_.destination().pull_over_plan_distance()) {
  // to far, not sending pull-over yet
  return false;
}
```

3. 障碍物PULL_OVER和STOP标签设定

```c++
int Destination::PullOver(common::PointENU* const dest_point) {
  auto* planning_state = GetPlanningStatus()->mutable_planning_state();
  if (!planning_state->has_pull_over() || !planning_state->pull_over().in_pull_over()) {
    planning_state->clear_pull_over();
    auto pull_over = planning_state->mutable_pull_over();
    pull_over->set_in_pull_over(true);
    pull_over->set_reason(PullOverStatus::DESTINATION);
    pull_over->set_status_set_time(Clock::NowInSeconds());

    if (dest_point) {
      pull_over->mutable_inlane_dest_point()->set_x(dest_point->x());
      pull_over->mutable_inlane_dest_point()->set_y(dest_point->y());
    }
  }
  return 0;
}
```

Stop标签设定和**人行横道情况处理**中STOP一致，创建虚拟墙，并封装成新的PathObstacle加入该ReferenceLineInfo的PathDecision中。

## 3.5 前车情况处理--FRONT_VEHICLE

在存在前车的情况下，障碍物对无人车的决策影响有两个，一是跟车等待机会超车(也有可能一直跟车，视障碍物信息决定)；而是停车(障碍物是静态的，无人车必须停车)

1. 跟车等待机会超车处理

当前车是运动的，而且超车条件良好，那么就存在超车的可能，这里定义的超车指代的需要无人车调整横向距离以后超车，直接驶过前方障碍物的属于正常行驶，可忽略障碍物。要完成超车行为需要经过4个步骤：

- 正常驾驶(DRIVE)
- 等待超车(WAIT)
- 超车(SIDEPASS)
- 正常行驶(DRIVE)

若距离前过远，那么无人车就处理第一个阶段正常行驶，若距离比较近，但是不满足超车条件，那么无人车将一直跟随前车正常行驶；若距离近但是无量速度很小(堵车)，那么就需要等待；若满足条件了，就立即进入到超车过程，超车完成就又回到正常行驶状态。

Step 1. 检查超车条件--`FrontVehicle::FindPassableObstacle`

这个过程其实就是寻找有没有影响无人车正常行驶的障碍物(无人车可能需要超车)，遍历PathDecision中的所有PathObstacle，依次检查以下条件：

- 是否是虚拟或者静态障碍物。若是，那么就需要采取停车策略，超车部分对这些不作处理

```c++
if (path_obstacle->obstacle()->IsVirtual() || !path_obstacle->obstacle()->IsStatic()) {
  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
        << "] VIRTUAL or NOT STATIC. SKIP";
  continue;
}
```

- 检车障碍物与无人车位置关系，障碍物在无人车后方，直接忽略。

```c++
const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
if (obstacle_sl.start_s() <= adc_sl_boundary.end_s()) {
  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
         << "] behind ADC. SKIP";
  continue;
}
```

- 检查横向与纵向距离，若纵向距离过远则可以忽略，依旧进行正常行驶；若侧方距离过大，可直接忽略障碍物，直接正常向前行驶。

```c++
const double side_pass_s_threshold = config_.front_vehicle().side_pass_s_threshold(); // side_pass_s_threshold: 15m
if (obstacle_sl.start_s() - adc_sl_boundary.end_s() > side_pass_s_threshold) {
  continue;
}
const double side_pass_l_threshold = config_.front_vehicle().side_pass_l_threshold(); // side_pass_l_threshold: 1m
if (obstacle_sl.start_l() > side_pass_l_threshold || 
    obstacle_sl.end_l() < -side_pass_l_threshold) {
    continue;
}
```

`FrontVehicle::FindPassableObstacle`函数最后会返回第一个找到的限制无人车行驶的障碍物

Step 2. 设定超车各阶段标志--`FrontVehicle::ProcessSidePass`

- 若上阶段处于SidePassStatus::UNKNOWN状态，设置为正常行驶

- 若上阶段处于SidePassStatus::DRIVE状态，并且障碍物阻塞(存在需要超车条件)，那么设置为等待超车；否则前方没有障碍物，继续正常行驶

```c++
case SidePassStatus::DRIVE: {
  constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s
  const auto& adc_planning_point = reference_line_info->AdcPlanningPoint();
  if (!passable_obstacle_id.empty() &&            // 前方有障碍物则需要等待
      adc_planning_point.v() < kAdcStopSpeedThreshold) {
    sidepass_status->set_status(SidePassStatus::WAIT);
    sidepass_status->set_wait_start_time(Clock::NowInSeconds());
  }
  break;
}
```

- 若上阶段处于SidePassStatus::WAIT状态，若前方已无障碍物，设置为正常行驶；否则视情况而定：

a. 前方已无妨碍物，设置为正常行驶

b. 前方有障碍物，检查已等待的时间，超过阈值，寻找左右有效车道超车；若无有效车道，则一直堵车等待。

```c++
double wait_start_time = sidepass_status->wait_start_time();   
double wait_time = Clock::NowInSeconds() - wait_start_time;  // 计算已等待的时间

if (wait_time > config_.front_vehicle().side_pass_wait_time()) { // 超过阈值，寻找其他车道超车。side_pass_wait_time：30s
}
```

首先查询HDMap，计算当前ReferenceLine所在的车道Lane。若存在坐车道，那么设置做超车；若不存在坐车道，那么检查右车道，右车道如果是机动车道(CITY_DRIVING)，不是非机动车道(BIKING)，不是步行街(SIDEWALK)，也不是停车道(PAKING)，那么可以进行右超车。

```c++
if (enter_sidepass_mode) {
  sidepass_status->set_status(SidePassStatus::SIDEPASS);
  sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
  sidepass_status->clear_wait_start_time();
  sidepass_status->set_pass_side(side);   // side知识左超车还是右超车。取值为ObjectSidePass::RIGHT或ObjectSidePass::LEFT
}
```

- 若上阶段处于SidePassStatus::SIDEPASS状态，若前方已无障碍物，设置为正常行驶；反之继续处于超车过程。

```c++
case SidePassStatus::SIDEPASS: {
  if (passable_obstacle_id.empty()) {
    sidepass_status->set_status(SidePassStatus::DRIVE);
  }
  break;
}
```

2. 停车处理

- 首先检查障碍物是不是静止物体(非1中的堵车情况)。若是虚拟的或者动态障碍物，则忽略，由超车模块处理

```c++
if (path_obstacle->obstacle()->IsVirtual() || !path_obstacle->obstacle()->IsStatic()) {
  continue;
}
```

- 检查障碍物和无人车位置，若障碍物在无人车后，忽略，由后车模块处理

```c++
const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
if (obstacle_sl.end_s() <= adc_sl.start_s()) {
    continue;
}
```

- 障碍物已经在超车模块中被标记，迫使无人车超车，那么此时就不需要考虑该障碍物

```c++
// check SIDE_PASS decision
if (path_obstacle->LateralDecision().has_sidepass()) {
  continue;
}
```

- 检查是否必须要停车条件，车道没有足够的宽度来允许无人车超车

```c++
double left_width = 0.0;
double right_width = 0.0;
reference_line.GetLaneWidth(obstacle_sl.start_s(), &left_width, &right_width);

double left_driving_width = left_width - obstacle_sl.end_l() -           // 计算障碍物左侧空余距离
                                config_.front_vehicle().nudge_l_buffer();
double right_driving_width = right_width + obstacle_sl.start_l() -       // 计算障碍物右侧空余距离，+号是因为车道线FLU坐标系左侧l是负轴，右侧是正轴
                                 config_.front_vehicle().nudge_l_buffer();
if ((left_driving_width < adc_width && right_driving_width < adc_width) ||
        (obstacle_sl.start_l() <= 0.0 && obstacle_sl.end_l() >= 0.0)) {
  // build stop decision
  double stop_distance = path_obstacle->MinRadiusStopDistance(vehicle_param);
  const double stop_s = obstacle_sl.start_s() - stop_distance;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  if (obstacle_type == PerceptionObstacle::UNKNOWN_MOVABLE ||
      obstacle_type == PerceptionObstacle::BICYCLE ||
      obstacle_type == PerceptionObstacle::VEHICLE) {
    stop_decision->set_reason_code(StopReasonCode::STOP_REASON_HEAD_VEHICLE);
  } else {
      stop_decision->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  }
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);
  path_decision->AddLongitudinalDecision("front_vehicle", path_obstacle->Id(), stop);
}
```

6. 禁停区情况处理--KEEP_CLEAR
7. 寻找停车点状态--PULL_OVER
8. 车道线结束情况处理--REFERENCE_LINE_END
9. 重新路由查询情况处理--REROUTING
10. 信号灯情况处理--SIGNAL_LIGHT
11. 停车情况处理--STOP_SIGN