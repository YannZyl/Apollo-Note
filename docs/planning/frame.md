# 障碍&主车轨迹后处理器：Frame类

# 1. Prediction模块发布数据获取与处理(障碍物预测轨迹后处理)

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