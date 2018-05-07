# PredictorManager管理器

ContainerManager管理器得到了每个障碍物的LaneGraph，里面包含了若干LaneSequence，每个LaneSequence指代障碍物运动的轨迹，或者叫做运动方案。而一个LaneSequence包含若干LaneSegment，这些LaneSegment就是该LaneSequence的运动车道，或者叫做路段。

EvaluatorManager管理器对每个障碍物的每条运动路线(LaneSequence)进行评估，评估指标共62项，包括22项障碍物特征，40项车道特征。最后得到每个LaneSequence的概率。

当经过EvaluatorManager评估以后，就可以根据规则得到障碍物有可能的运动轨迹(运动方案)，这时候我们就可以针对每个障碍物生成一条短时间内的运动轨迹。这些运动轨迹可以交由Planning模块进行路径重规划。这里的障碍物短时间内运动轨迹是什么概念？障碍物当前位置(x,y)，还有速度，加速度，速度方向。由ContainerManager可知，我们对LaneSegment是经过离散采样保存的，所以障碍物短时间内的运动轨迹是指：当前位置到LaneSequence的第一个LanePoint的运动轨迹。如果一个障碍物的LaneGraph由N条LaneSequence，那么该障碍物就对应了m条轨迹，这m条轨迹是概率最大的那几条(具体挑选规则在下文提到)

```proto
/// file in apollo/modules/prediction/conf/prediction_conf.pb.txt
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: MLP_EVALUATOR
  predictor_type: MOVE_SEQUENCE_PREDICTOR
}
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: OFF_LANE
  predictor_type: FREE_MOVE_PREDICTOR
}
obstacle_conf {
  obstacle_type: PEDESTRIAN
  predictor_type: FREE_MOVE_PREDICTOR
}
obstacle_conf {
  obstacle_type: BICYCLE 
  obstacle_status: ON_LANE
  evaluator_type: MLP_EVALUATOR
  predictor_type: MOVE_SEQUENCE_PREDICTOR
}
obstacle_conf {
  obstacle_type: BICYCLE
  obstacle_status: OFF_LANE
  predictor_type: FREE_MOVE_PREDICTOR
}
obstacle_conf {
  obstacle_type: UNKNOWN
  obstacle_status: ON_LANE
  evaluator_type: MLP_EVALUATOR
  predictor_type: LANE_SEQUENCE_PREDICTOR
}
obstacle_conf {
  obstacle_type: UNKNOWN
  obstacle_status: OFF_LANE
  predictor_type: FREE_MOVE_PREDICTOR
}
```

还是看一下预测器的多累，上面看到一共用到3类预测器：

Lane sequence(LANE_SEQUENCE_PREDICTOR): 沿着车道(中心线)行驶，比较简单地方法
Move sequence(MOVE_SEQUENCE_PREDICTOR): 根据其运动模式，形式向车道中心线后沿着车道行驶
Free movement(FREE_MOVE_PREDICTOR)：自由移动


```c++
/// file in apollo/modules/prediction/predictor/predictor_manager.cc
void PredictorManager::Run(const PerceptionObstacles& perception_obstacles) {
  for (const auto& perception_obstacle : perception_obstacles.perception_obstacle()) {
   
    PredictionObstacle prediction_obstacle;
    prediction_obstacle.set_timestamp(perception_obstacle.timestamp());
    Obstacle* obstacle = obstacles_container->GetObstacle(id);
    if (obstacle != nullptr) {
      // Step 1. get predictor
      // Step 2. predict
      if (predictor != nullptr) {
        predictor->Predict(obstacle);                                 // 计算每个障碍物的短时间运动轨迹
        for (const auto& trajectory : predictor->trajectories()) {
          prediction_obstacle.add_trajectory()->CopyFrom(trajectory); // 将轨迹加入prediction_obstacle
        }
      }
    }
    prediction_obstacle.set_predicted_period(FLAGS_prediction_duration);
    prediction_obstacle.mutable_perception_obstacle()->CopyFrom(perception_obstacle);

    prediction_obstacles_.add_prediction_obstacle()->CopyFrom(prediction_obstacle); // 将prediction_obstacle加入到向量中，即保存该障碍物的所有可能轨迹
  }
}
```

从上面代码很容易看到预测阶段对每个障碍物分别预测其轨迹，而且一个障碍物的轨迹不止一条(有些LaneSequence概率都比较大)。接下来我们看一下`predictor->Predict(obstacle);`这个过程，我们选取MOVE_SEQUENCE_PREDICTOR模式进行分析。

```c++
/// file in apollo/modules/prediction/predictor/move_sequence/move_sequence_predictor.cc
void MoveSequencePredictor::Predict(Obstacle* obstacle) {
  const Feature& feature = obstacle->latest_feature();
  // Step 1. 对该障碍物LaneGraph中每个LaneSequence根据其evaluator的概率进行预测，是否和规划好的轨迹重叠
  int num_lane_sequence = feature.lane().lane_graph().lane_sequence_size();
  std::vector<bool> enable_lane_sequence(num_lane_sequence, true);
  FilterLaneSequences(feature, lane_id, &enable_lane_sequence);
  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = feature.lane().lane_graph().lane_sequence(i);

    // Step 2. 画出短时间内障碍物运动的轨迹
    std::vector<TrajectoryPoint> points;
    DrawMoveSequenceTrajectoryPoints(*obstacle, sequence, FLAGS_prediction_duration, FLAGS_prediction_period, &points);
    // Step 3. 生成轨迹并保存
    Trajectory trajectory = GenerateTrajectory(points);
    trajectory.set_probability(sequence.probability());
    trajectories_.push_back(std::move(trajectory));
  }
}
```

整个过程相对来说比较简单，但是每个过程有一些难点，这里我们简单地可以将Predict函数分为三个过程：

1. 对各个LaneSequence根据其上阶段evalute的结果(每个运动方案的概率)进行筛选，首先障碍物是智能体，他运动的轨迹也要参考别的障碍物，如果他运动的轨迹和主车当前位置过近，那么是很危险的，所以正常思维判断，这个LaneSequence是不可行的。举个例子，主车在车道1上行驶，障碍物在车道2行驶，两车相距2米，下时刻突然障碍物变道到主车的车道1是不太可能的。

2. 对那么不会造成相撞的障碍物运动方案进行短时间运动轨迹的生成，生成思路很简单，插值。

3. 轨迹保存，这个就比较简单`DrawMoveSequenceTrajectoryPoints`函数画出了一系列的运动点，只要把它封装成轨迹点后放入Trajectory类即可。

## LaneSequence过滤--FilterLaneSequences

这里我们简单地看一下步骤1和步骤2的实现方案。步骤1 LaneSequence的筛选预过滤步骤如下：

1. 计算每个LaneSequence对应障碍物的运动，(保持前进、左转还是右转)。

计算当前位置的车道： lane_id (feature.lane().lane_feature().lane_id())
LaneSequence第一个LaneSegment对应的车道线: first_lane_id(feature.lane().lane_graph().lane_sequence(i).lane_segment(0).lane_id())

如果first_lane_id == lane_id，则障碍物保持前进；如果first_lane_id在lane_id左侧，那么障碍物左转；如果first_lane_id在lane_id右侧，那么障碍物右转

2. 计算ADCTrajectory到LaneSequence的距离

分一下几种情况

- 如果LaneSequence和ADCTrajectory没有重叠，那么返回浮点数最大值即可

- 如果LaneSequence和ADCTrajectory存在重叠，那么计算ADCTrajectory第一个点p1(也就是主车的位置)到LaneSequence的第一个LanePoint：p2之间的距离，计算方法也比较简单，将ADCTrajectory第一个点投影到LaneSegment对应的车道上，返回两个点的累计距离差 renturn fabs(p1.s-p2.s)

3. 最后对上述距离做是否有效的统计

如果距离小于一个阈值(FLAGS_lane_change_dist，默认10米)，那么作为智能体障碍物的LaneSequence，正常情况下是不会选择这条行驶路线的

4. 计算上述有效LaneSequence的最大概率以及转弯最大概率

这个过程比较简单，在代码中可以清晰地看出。至于为什么要额外计算转弯的最大概率，一种可能的原因是evaluate评估得到的结果更多的是偏向于直行，也就是那么多LaneSequence中直行的概率是很大的，而转弯的概率比较小，如果直接根据最大概率一锤定音，那么很难出现转弯的情况。如果把转弯单独拿出来，这会更加障碍物的评估准确率。最终得到的有效LaneSequence可能有多条，所以每个障碍物对应的Trajectory可能有多条。

## 轨迹曲线生成--DrawMoveSequenceTrajectoryPoints

在Move Sequence中，轨迹曲线的生成比较繁琐，主要是分别建立主方向和侧方向的多项式函数(函数形式以及其参数目前没有看明白，暂时先留空)，主要的计算过程分为：

1. 计算障碍物到车道中心线的时间

```c++
Eigen::Vector2d position(feature.position().x(), feature.position().y());
double time_to_lat_end_state = std::max(FLAGS_default_time_to_lat_end_state, ComputeTimeToLatEndConditionByVelocity(obstacle, lane_sequence));
```

这个相对来说不难理解，因为障碍物当前位置由x和y方向的速度vx和vy，同时可以计算得到障碍物到LaneSequence的第一个LanePoint所在Lane的侧面距离relative_l，已经LaneSequence的第一个LanePoint的方向lane_heading，那么只要计算vx和vy在lane_heading垂直方向的分量`v_l = v_y * std::cos(lane_heading) - v_x * std::sin(lane_heading)`，然后relative_l / v_l即可

2. 计算lane_heading侧方向(Lateral)的多项式表达式 (目前暂时没看明白多项式的由来)

3. 计算lane_heading主方向(Longitudinal)的多项式表达式 (目前暂时没看明白多项式的由来)

4. 最后一步就是根据两个多项式进行插值，可以看到Apollo对这段短时间的轨迹一共插值50个点`total_num = static_cast<size_t>(total_time / period)`，并且分装成PathPoint

```c++
  for (size_t i = 0; i < total_num; ++i) {
	TrajectoryPoint trajectory_point;
	PathPoint path_point;
	path_point.set_x(point.x());
	path_point.set_y(point.y());
	path_point.set_z(0.0);
	path_point.set_theta(theta);
	path_point.set_lane_id(lane_id);
	trajectory_point.mutable_path_point()->CopyFrom(path_point);
	trajectory_point.set_v(lane_speed);
	trajectory_point.set_a(lane_acc);
	trajectory_point.set_relative_time(relative_time);
	points->emplace_back(std::move(trajectory_point));

	while (lane_s > PredictionMap::LaneById(lane_id)->total_length() &&
           lane_segment_index + 1 < lane_sequence.lane_segment_size()) {
      lane_segment_index += 1;
      lane_s = lane_s - PredictionMap::LaneById(lane_id)->total_length();
      lane_id = lane_sequence.lane_segment(lane_segment_index).lane_id();
    }
  }
```

代码最后的while作用就是针对多LaneSegment的LaneSequence进行变道处理。


最终总结Predictor的作用：根据LaneSequence的概率，以及预先规划好的ADCTrajectory对LaneSequence进行筛选，去掉那些不合理的行驶方案，最后得到一些短时间轨迹的集合。
