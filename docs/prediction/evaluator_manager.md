# EvaluatorManager管理器

在上一节ContainerManager管理器我们提到了，Container主要包含PoseContainer车辆姿态容器，ObstaclesContainer障碍物容器和ADCTrajectoryContainer轨迹容器。其中：

1. ObstaclesContainer障碍物容器主要是存储障碍物信息以及障碍物所在的车道信息。

通过上一节的介绍我们知道，障碍物信息部分来源于Perception感知模块，而车道信息来源于HD map高精地图模块。通过一系列的LaneGraph、LaneSequence和LaneSegment构建，我们可以得到以下信息：

- LaneGraph其实是根据障碍物的速度，预测一个运动的范围($s=vt+0.5at^2$)，通过对这个范围内的当前车道CurrentLane和邻接车道NearByLane进行截取，得到一个LaneGraphe，每个LaneGraph是以唯一标识符id来识别的，这个id就是Obstacle.lane_id(障碍物所在车道id)，LaneGraph又是由多个LaneSequence组成

- LaneSequence是这个预测时间段内，障碍物可能的运动路径(Lane_1到Lane_2，也可能是Lane_1到Lane_3)。每个运动路径方案中，都会包含若干段LaneSegment。如果物体不变道，那么LaneSegment.size()=1，否则size会大于1

- LaneSegment是一段运行轨迹，通过离散采样保存，例如每2米采样一个点，规定一个LaneSequence中所有的LaneSegment点最多不超过20个。每个点都记录了他的世界系坐标(x,y)，运动方向heading，车道宽度width，障碍物累计长度relative_s，投影距离relative_l，车道方向与物体运动方向夹角angle_diff。同时会额外设置LaneSequence的PathPoint。

如果不清楚上述内容，可以重新参考[ContainerManager管理器](https://github.com/YannZyl/Apollo-Note/blob/master/docs/prediction/container_manager.md)

本章内容其实比较简单，一句话概括就是：**计算LaneGraph中每个LaneSequence的概率，换句话说就是评估物体运动每个方案的概率！**

首先我们看一下预测使用的模型：

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

从上面的配置文件我们可以看到：

1. Apollo中对物体运动的预测是区分开来的，车辆VECHICLE，行人PEDESTRIAN，自行车BICYCLE和未知物体UNKNOWN分开预测

2. 运动概率评估使用的是MLP多层感知器模型，这个模型比较简单，由输入层，隐藏层和输出层组成。Apollo还额外提供了RNN和COST模型，但是未使用，因此这里不做介绍


```c++
/// file in apollo/modules/prediction/evaluator/vehicle/mlp_evaluator.cc
void MLPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  LaneGraph* lane_graph_ptr = latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  // 对每个LaneSequence进行概率评估
  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr = lane_graph_ptr->mutable_lane_sequence(i);
    std::vector<double> feature_values;
    ExtractFeatureValues(obstacle_ptr, lane_sequence_ptr, &feature_values);
    double probability = ComputeProbability(feature_values);
    double centripetal_acc_probability = ValidationChecker::ProbabilityByCentripedalAcceleration(*lane_sequence_ptr, speed);
    probability *= centripetal_acc_probability;
    lane_sequence_ptr->set_probability(probability);
  }
}

void MLPEvaluator::ExtractFeatureValues(Obstacle* obstacle_ptr, LaneSequence* lane_sequence_ptr, std::vector<double>* feature_values) {
  // Step 1. 计算障碍物特征
  if (it == obstacle_feature_values_map_.end()) {
    SetObstacleFeatureValues(obstacle_ptr, &obstacle_feature_values);
    obstacle_feature_values_map_[id] = obstacle_feature_values;
  } else {
    obstacle_feature_values = it->second;
  }
  // Step 2. 计算车道特征
  std::vector<double> lane_feature_values;
  SetLaneFeatureValues(obstacle_ptr, lane_sequence_ptr, &lane_feature_values);
  // Step 3. 两类特征合并作为MLP网络输入
  feature_values->insert(feature_values->end(), obstacle_feature_values.begin(), obstacle_feature_values.end());
  feature_values->insert(feature_values->end(), lane_feature_values.begin(), lane_feature_values.end());
}
```

算法会对LaneGraph的每个LaneSequence进行评估，知道了评估使用的模型，那么接下去就介绍一下模型的输入。在MLP_Evaluator里面，网络的输入矩阵包含两类：障碍物特征ObstacleFeature与车道特征LaneFeature，下面我们将逐一介绍。

## 障碍物特征ObstacleFeature

障碍物特征一共22维(OBSTACLE_FEATURE_SIZE = 22)，在计算该特征时，首先需要对应障碍物信息的历史记录，可以从存储Obstacle的LRU中获取，取距离当前时刻5秒内的所有历史数据。

```c++
double duration = obstacle_ptr->timestamp() - FLAGS_prediction_duration;
```

最后组成成分及其计算方式分别为：

1. 物体方向均值：theta_filtered

theta_filtered是最近5个历史数据的障碍物运动方向均值: `ComputeMean(thetas, 0, curr_size - 1)`

2. 物体方向均值：theta_mean 

theta_filtered是所有历史数据的障碍物运动方向均值: `ComputeMean(thetas, 0, hist_size - 1)`

3. 物体方向均值差：theta_filtered - theta_mean 

4. 物体与车道方向夹角：angle_diff

```c++
double angle_curr = ComputeMean(thetas, 0, curr_size - 1);
double angle_prev = ComputeMean(thetas, curr_size, 2 * curr_size - 1);
double angle_diff = (hist_size >= 2 * curr_size) ? angle_curr - angle_prev : 0.0;
```

这里使用了统计学技术，假设给定10次测量a1,a2,...,a10，差值求解方法为：

diff = [(a6-a1)+(a7-a2)+...+(a10-a5)]/5 = (a6+a7+...+a10)/5 - (a1+a2+...+a5)/5

5. 物体与车道夹角偏差比率：angle_diff_rate 

`angle_diff_rate = angle_diff / (delta_t * curr_size);`

这里使用了统计学技术，假设给定10次测量a1,a2,...,a10，每两次测量的时间间隔是t，差值比率(偏导数)求解方法为：

diff_ratio = [(a6-a1)/5t+(a7-a2)/5t+...+(a10-a5)/5t]/5 = (a6+a7+...+a10)/(5\*5t) - (a1+a2+...+a5)/(5\*5t)

6. 车道方向均值：lane_l_filtered 

theta_filtered是最近5个历史数据的障碍物运动方向均值: `ComputeMean(lane_ls, 0, curr_size - 1)`

7. 车道方向均值：lane_l_mean 

theta_filtered是所有历史数据的障碍物运动方向均值: `ComputeMean(lane_ls, 0, hist_size - 1)`

8. 车道方向均值差：lane_l_filtered - lane_l_mean

9. 障碍物距离车道距离差： lane_l_diff

```c++
double lane_l_curr = ComputeMean(lane_ls, 0, curr_size - 1);
double lane_l_prev = ComputeMean(lane_ls, curr_size, 2 * curr_size - 1);
double lane_l_diff = (hist_size >= 2 * curr_size) ? lane_l_curr - lane_l_prev : 0.0;
```

计算方法与物体与车道方向夹角angle_diff相同，都是求前N次历史数据与后N次数据的距离差

10. 障碍物距离车道距离差利率：lane_l_diff_rate

计算方法与物体与车道夹角偏差比率angle_diff_rate相同 

11. 障碍物速度均值：speed_mean

计算所有时刻的速度均值，计算方式为：`double speed_mean = ComputeMean(speeds, 0, hist_size - 1);`

12. 障碍物加速度：acc

```c++
double acc = 0.0;
if (static_cast<int>(speeds.size()) >= 3 * curr_size && delta_t > std::numeric_limits<double>::epsilon()) {
  double speed_1 = ComputeMean(speeds, 0, curr_size - 1);
  double speed_2 = ComputeMean(speeds, curr_size, 2 * curr_size - 1);
  double speed_3 = ComputeMean(speeds, 2 * curr_size, 3 * curr_size - 1);
  acc = (speed_1 - 2 * speed_2 + speed_3) / (curr_size * curr_size * delta_t * delta_t);
}
```

这里使用了统计学技术，假设给定9次测量a1,a2,...,a9，每两次测量的时间间隔为t，加速求解方法为：

acc = {[(a7-a4)/3t-(a4-a1)/3t]/t + [(a8-a5)/3t-(a5-a2)/3t]/t + [(a9-a6)/3t-(a6-a3)/3t]/t}/3

acc = (a7+a1-2a4)/(3t\*3t)+(a8+a2-2a5)/(3t\*3t)+(a9+a3-2a6)/(3t\*3t)

计算方式同代码一致

13. 障碍物距离车道左边界线距离：dist_lbs

最近历史数据即可，无需求平均，计算方式：`dist_lbs.front()`

14. 障碍物距离车道左边界线距离比率：dist_lb_rate

`double dist_lb_rate = (timestamps.size() > 1) ? (dist_lbs.front() - dist_lbs.back()) / time_diff : 0.0;`

主要含义为每秒障碍物距离车道左边界线变化量

15. 当前时刻，障碍物距离车道左边界线距离比率：dist_lb_rate_curr

```c++
double dist_lb_rate_curr = 0.0;
if (hist_size >= 2 * curr_size && delta_t > std::numeric_limits<double>::epsilon()) {
  double dist_lb_curr = ComputeMean(dist_lbs, 0, curr_size - 1);
  double dist_lb_prev = ComputeMean(dist_lbs, curr_size, 2 * curr_size - 1)
  dist_lb_rate_curr = (dist_lb_curr - dist_lb_prev) / (curr_size * delta_t);
}
```

同 5.物体与车道夹角偏差比率和10. 障碍物距离车道距离差利率 计算方式一致。

16. 障碍物距离车道右边界线距离：dist_rbs

最近历史数据即可，无需求平均，计算方式：`dist_rbs.front()`

17. 障碍物距离车道右边界线距离比率：dist_lb_rate

`double dist_rb_rate = (timestamps.size() > 1) ? (dist_rbs.front() - dist_rbs.back()) / time_diff : 0.0;`

主要含义为每秒障碍物距离车道右边界线变化量

18. 当前时刻，障碍物距离车道右边界线距离比率：dist_rb_rate_curr

```c++
double dist_rb_rate_curr = 0.0;
if (hist_size >= 2 * curr_size && delta_t > std::numeric_limits<double>::epsilon()) {
  double dist_rb_curr = ComputeMean(dist_rbs, 0, curr_size - 1);
  double dist_rb_prev = ComputeMean(dist_rbs, curr_size, 2 * curr_size - 1)
  dist_rb_rate_curr = (dist_rb_curr - dist_rb_prev) / (curr_size * delta_t);
}
```

同 5物体与车道夹角偏差比率、10障碍物距离车道距离差利率和15计算方式一致。

19-22. lane_types的onehot标签

NO_TURN:    [1,0,0,0]
LEFT_TURN:  [0,1,0,0]
RIGHT_TURN: [0,0,1,0]
U_TURN:     [0,0,0,1]

## 车道特征LaneFeature

车道特征计算比较简单，由于车道LaneSequence包含若干LaneSegment，而每个LaneSegment又被离散化采样得到若干LanePoint。由于每条LaneSequence包含的LanePoint数量不一致(但总体来说LanePoint数量必须小于20，由参数设定)，因此这里使用一个简单地方法：从LaneSequence的第一个LaneSegment开始遍历，计算LaneSegment中的每个LanePoint的4个属性，直到维度满40停止，也就是最多访问LaneSequence的10个PanePonit。

1. LanePoint位置与方向夹角的sin值

```c++
double diff_x = lane_point.position().x() - feature.position().x();
double diff_y = lane_point.position().y() - feature.position().y();
double angle = std::atan2(diff_x, diff_y);
feature_values->push_back(std::sin(angle - heading));
```

2. 障碍物与该Segment所属Lane的投影距离

`feature_values->push_back(lane_point.relative_l());`

3. LanePoint的方向

`feature_values->push_back(lane_point.heading());`

4. LanePoint方向与障碍物方向夹角

`feature_values->push_back(angle_diff);`

如果经过上述处理，得到的特征维度没到40维，那么使用最末尾的4维去填充身下的维度即可


## 概率计算  MLPEvaluator::ComputeProbability

这个比较简单，就是传统的MLP前向计算，给定输入inputs(nxm)，隐藏层权值W(mxk)，偏置b(kx1)，那么隐藏层输出计算公式为：

1. 首先输入正则化，需要去均值，方差归一。

inputs = (inputs - mean) / std

2. 前向计算

outputs = inputs * W + b (\*为矩阵乘法)

激活函数sigmoid：

sigmoid(x) = 1/(1+exp(-x))

激活函数tanh：

tanh(x) = 1 - 2\*sigmoid(x)

激活函数relu：

relu(x) = max(0, x)

输出得到原始的probability

3. 向心加速度概率计算

```c++
/// file in apollo/modules/prediction/common/validation_checker.cc
double ValidationChecker::ProbabilityByCentripedalAcceleration(const LaneSequence& lane_sequence, const double speed) {
  double centripetal_acc_cost_sum = 0.0;
  double centripetal_acc_cost_sqr_sum = 0.0;
  for (int i = 0; i < lane_sequence.path_point_size(); ++i) {
    const PathPoint& path_point = lane_sequence.path_point(i);
    // 根据向心加速度公式: an = v^2 / R
    double centripetal_acc = speed * speed * path_point.kappa();
    double centripetal_acc_cost = centripetal_acc / FLAGS_centripedal_acc_threshold;
    centripetal_acc_cost_sum += centripetal_acc_cost;
    centripetal_acc_cost_sqr_sum += centripetal_acc_cost * centripetal_acc_cost;
  }
  double mean_cost = centripetal_acc_cost_sqr_sum / (centripetal_acc_cost_sum + FLAGS_double_precision);
  return std::exp(-FLAGS_centripetal_acc_coeff * mean_cost);
}
```

最后LaneSequence的概率为probability\*centripetal_acc_probability