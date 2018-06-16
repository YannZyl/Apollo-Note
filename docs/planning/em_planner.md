# EM规划器：EMPlanner

EM规划器，也是规划模块Planning的核心，主要的任务是根据：

1. 感知模块Perception和预测模块Prediction得到的障碍物以及未来时间(e.g. 5s内)的运动轨迹
2. 当前路况，禁停区、减速带、停车标志、人行横道等等

来最终规划无人车的最优行驶路径。从参考线提供器ReferenceLineProvider可以看到，当无人车面临变道时，就会存在多条参考线(变道时间可随机扰动，变道早可能进入车道1，变道晚就可能进入车道2)。那么EM规划器就需要计算每条参考线上的最优行驶路径，最后进行综合，得到开销(也就是cost)最小的一条路径。EM规划器对无人车的控制规划进行拆分，分为2类：

1. 路径规划(也就是行驶位置, path-time)
2. 速度规划(speed)

那么对于路径规划和速度规划，可以有两种方法，分别为：

1. 动态规划(dynamic programming)
2. 二次规划(quadratic programming + Spline interpolation，这个和参考线平滑很相似)

上述两种规划方案，Apollo采用的是第一种动态规划法，这种方法的原理很简单，我们这里以速度规划为例，进行简单的流程简述：

step 1：初始化规划起点，可以是车辆当前坐标。然后每隔一段距离(例如10m)规划一次位置，一共向前规划如40m。

step 2：在下个位置侧向进行一些点采样(例如7个)，也就是计算在车道不同宽度位置采样。然后计算每个采样点到前一层所有点的cost，就可以得到其实规划点到当前采样点的最短路径与开销cost。

step 3：前进到最小cost对应的采样点，然后重复2对下个位置横向采样并计算开销cost。最终从终点开始，反向选择一条cost最小的路径。

使用数学的形式来表示，进一步理解动态规划的思想。假设规划起始点为s，需要向前规划M米，那么可以每个d米进行一次采样(也就是纵向采样)，每次采样位置可以横向选择若干个点(横向采样)，表示无人车可以前进到这个车道宽度位置，最终找到一条从起始点到终点的最优行驶路径。

为了更加生动形象的理解，我们以神经网络为类比模型。规划起始点s就是神经网络的输入，只有一个点；规划终止点x其实就是神经网络的输出，但是可以有若干点，这些点在参考线上的长度一致，但车道宽度可以不一致，也就是说可以规划到路中央，也可以规划到路边。纵向每隔d米采样其实就是类似于一个个隐藏层，每个位置横向采样就是该隐藏层的不同神经元。网络中相邻层之间神经元的连接权值就是从前一个点行驶到另一个点所需要的开销cost。那么计算一条最优路径path(s)可以表示为：

$$ mincost(path(s,x^l)) = min_i min_j mincost(path(a,x_j^{l-1})) + cost(x_j^{l-1}, x_i^{l}) $$

公式中$ mincost(path(s,x^l)) $表示从规划起始点s到规划终点(也就是level层输出的最小开销)，只需要取输出层中具有最小开销的点即可，也就对应$min_i$，其中i从0开始到输出层的数量。起始点到终点i个神经元的最小开销，可以分别计算起始点到倒数第二层每个神经元的最小开销 $mincost(path(a,x_j^{l-1}))$，在加上这个神经元到终点i神经元的开销$cost(x_j^{l-1}, x_i^{l})$，最后取倒数第二层中j个神经元计算最小值，即为整个最优路径的cost，这就是动态规划的原理。神经网络中每个点都保存着起始点到该点的最小开销cost。

在计算开销cost的过程中，需要考略三个因素：无人车位置(最好就是验证参考线前进，横向不停地调整会增加cost)，静态障碍物以及动态障碍物(这两类已经在Frame中完成设置，已经考虑到感知物体和路况)。

下面我们就对路径规划和速度规划做一个详细的了解。代码中使用了动态规划法对路径进行规划；同时使用动态规划与二次规划+样条线插值对速度进行规划

```proto
planner_type : EM
em_planner_config {
    task : DP_POLY_PATH_OPTIMIZER
    task : PATH_DECIDER
    task : DP_ST_SPEED_OPTIMIZER
    task : SPEED_DECIDER
    task : QP_SPLINE_ST_SPEED_OPTIMIZER
}
```

## 路径规划--DpPolyPathOptimizer

先看一下代码结构：

```c++
/// file in apollo/modules/planning/planner/em/em_planner.cc
Status EMPlanner::Plan(const TrajectoryPoint& planning_start_point, Frame* frame) {
  for (auto& reference_line_info : frame->reference_line_info()) {       // 先分别对每条参考线进行规划
    auto cur_status = PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);  // 规划主函数
}
Status EMPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  for (auto& optimizer : tasks_) {               // 对每条参考线分别进行所有任务的规划，包括速度规划、路径位置规划、位置决策器规划等。
    ret = optimizer->Execute(frame, reference_line_info);
  }
}
/// file in apollo/modules/planning/tasks/path_optimizer.cc
apollo::common::Status PathOptimizer::Execute(Frame* frame, ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = Process(    // 最终会调用路径规划类的Process函数
      reference_line_info->speed_data(), reference_line_info->reference_line(),
      frame->PlanningStartPoint(), reference_line_info->mutable_path_data());
  return ret;
}
/// file in apollo/planning/tasks/dp_poly_path/dp_poly_path_optimizer.cc
Status DpPolyPathOptimizer::Process(const SpeedData &speed_data,
                                    const ReferenceLine &,
                                    const common::TrajectoryPoint &init_point,
                                    PathData *const path_data) {
  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);
  if (!dp_road_graph.FindPathTunnel(          // 寻找参考线最优的行驶路径，由DPRoadGraph::FindPathTunnel完成。
          init_point,
          reference_line_info_->path_decision()->path_obstacles().Items(),
          path_data)) {
    ...
  }
}
```

从上面的代码结构，可以进一步验证开始我们的想法，规划的方法是先独立对所有参考线进行单独规划，每条参考线可以得到一个最优的行进路线；最后综合所有参考线的最优行进路线，选择"全局"最优的路线。下面就对`DPRoadGraph::FindPathTunnel
`函数进行解析，大体的逻辑也很简单，主要分3步：

Step A. 计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向速度ddl

Step B. 获取当前参考线下最优的前进路线

Step C. 将最优前进路线封装成path_data

```c++
/// file in apollo/modules/planning/tasks/dp_poly_path/dp_road_graph.cc
bool DPRoadGraph::FindPathTunnel(
    const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    PathData *const path_data) {
  // Step A. 计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向速度ddl
  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
    return false;
  }
  // Step B. 获取当前参考线下最优的前进路线
  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    return false;
  }
  // Step C. 将最优前进路线封装成path_data
  std::vector<common::FrenetFramePoint> frenet_path;
  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    ...
  }
  FrenetFramePath tunnel(frenet_path);
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(tunnel);
  return true;
}
```

步骤A和C没什么难点，B步骤稍微有点繁琐，下面我们将逐步进行分析。在计算当前参考线下最优的前进路线时，正如开始所说的，可以每隔一段距离，就在车道横向采样一些点，表示无人车在车道不同宽度行驶时坐标，然后计算两个距离之间cost最小的坐标对，这就代表当前位置到下一位置的最优路径。这样就需要一步步解决以下问题：

**Q1：如何采样坐标点？--`SamplePathWaypoints`函数**

这个问题在开始已经说过，规划首先就需要考虑：

- 规划未来多久的距离？

答案是未来近40m的距离，可以通过观察代码:

```c++
const float kMinSampleDistance = 40.0;
const float total_length = std::fmin(
   init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),
   reference_line_.Length());
```

具体还需要考虑规划起始点的无人车速度以及参考线的长度。

- 每隔多久进行一次采样(纵向采样)？每次采样车道横向不同位置采样几个点(横向采样)？

答案也比较简单，大约每个10-20m，对车道不同宽度进行采样。

```c++
const size_t num_sample_per_level =   // 每个位置，横向采样7个点
      FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()  // navigator_sample_num_each_level: 3
                                : config_.sample_points_num_each_level();    // sample_points_num_each_level: 7
  const bool has_sidepass = HasSidepass();
  constexpr float kSamplePointLookForwardTime = 4.0;
  const float step_length =            // 将step_length裁剪到[20,40]米之间
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                          config_.step_length_min(), config_.step_length_max());
  const float level_distance =        // 每隔level_distance进行一次横向采样，纵向采样的间隔差不多10-20m
      (init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;
  float accumulated_s = init_sl_point_.s();
  float prev_s = accumulated_s;
```

上面代码有一个词--level，其实就是纵向采样点，每个level中会横向采样若干点。当然，可以更紧凑的纵向与横向采样，但是在计算cost会增加过大的计算量，所以需要经过权衡。

- 纵向&&横向怎么样进行采样？

首先如果无人车当前状态正在寻找停车点，也就是PULL_OVER状态，那么采样点其实只要设置为PULL_OVER的计算得到的起始点即可，当前前提条件是需要进入到他的可操作区域。

```c++
if (status->planning_state().has_pull_over() &&
      status->planning_state().pull_over().in_pull_over()) {
    status->mutable_planning_state()->mutable_pull_over()->set_status(
        PullOverStatus::IN_OPERATION);
    if (init_sl_point_.s() > start_point_sl.s()) {     // 表示无人车已进入PULL_OVER可操作区域
      const auto &stop_point = status->planning_state().pull_over().stop_point();
      SLPoint stop_point_sl;
      if (!reference_line_.XYToSL(stop_point, &stop_point_sl)) {
        AERROR << "Fail to change xy to sl.";
        return false;
      }
      std::vector<common::SLPoint> level_points(1, stop_point_sl); // 这时候只要设置停车点为下一时刻(纵向)的采样点即可，那么横向采样点就只有一个，就是停车位置
      points->emplace_back(level_points);
      return true;
    }
  }
```

然后循环进行纵向+横向采样，

```c++
for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    // 计算纵向每个位置，有效的左右边界，kBoundaryBuff是无人车与车道线边界线的间距。需要保持无人车在车道内行驶。
    reference_line_.GetLaneWidth(s, &left_width, &right_width);
    constexpr float kBoundaryBuff = 0.20;
    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;   // 计算右有效宽度
    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;     // 计算左有效宽度

    float kDefaultUnitL = 1.2 / (num_sample_per_level - 1);  // 每个位置上横向采样时，采样点之间的距离，差不多0.2m
    if (reference_line_info_.IsChangeLanePath() &&           // 如果当前参考线是变道，且变道不安全(无人车前后一定距离内有障碍物)
        !reference_line_info_.IsSafeToChangeLane()) {        // ，那么增加采样点间隔，这样可以下一时刻减少变道的时间。
      kDefaultUnitL = 1.0;
    }

    const float sample_l_range = kDefaultUnitL * (num_sample_per_level - 1); // 横向采样的宽度
    float sample_right_boundary = -eff_right_width;   // 计算FLU坐标系下，右边界
    float sample_left_boundary = eff_left_width;      // 计算FLU坐标系下，左边界
    const float kLargeDeviationL = 1.75;
    if (reference_line_info_.IsChangeLanePath() ||    // 修正左右边界，如果参考线需要变道，并且已经便宜车道了，那么就将横向采样区间向变道方向平移。
        std::fabs(init_sl_point_.l()) > kLargeDeviationL) {
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());  // 左变道，修正左边界
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());     // 右变道，修正右边界
      if (init_sl_point_.l() > eff_left_width) {      // 左变道，修正右边界，向左平移，因为左变道车道线右边的区域不用考虑
        sample_right_boundary = std::fmax(sample_right_boundary, init_sl_point_.l() - sample_l_range);
      }
      if (init_sl_point_.l() < eff_right_width) {     // 右变道，修正左边界，向右平移，因为右变道车道线左边的区域不用考虑
        sample_left_boundary = std::fmin(sample_left_boundary, init_sl_point_.l() + sample_l_range);
      }
    }
    // 具体采样
    ...
}
```

最后进行每个纵向位置的横向区间采样，采样分两个多类情况：

情况1：如果当前参考线需要变道，并且变道不安全，那么横向采样点就设置为第二条参考线的位置，直接走第二条参考线。

```c++
if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {
   sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
} 
```

情况2：如果当前参考线需要侧方绕行(SIDEPASS)，即从旁边超车。若是可以进行左边超车，那么横向采样点设置为左边界+超车距离；右边超车，横向采样点设置为右边界+超车距离

```c++
else if (has_sidepass) {
    // currently only left nudge is supported. Need road hard boundary for
    // both sides
    switch (sidepass_.type()) {
      case ObjectSidePass::LEFT: {
        sample_l.push_back(eff_left_width + config_.sidepass_distance());
        break;
      }
      case ObjectSidePass::RIGHT: {
        sample_l.push_back(-eff_right_width - config_.sidepass_distance());
        break;
      }
      default:
        break;
    }
}
```

情况3：正常行驶情况下，从横向区间[sample_right_boundary , sample_left_boundary]大小为sample_l_range进行均匀采样

```c++
else {
  common::util::uniform_slice(sample_right_boundary, sample_left_boundary,
                                  num_sample_per_level - 1, &sample_l);
}
```

计算每个横向采样点的相对偏移距离l和累计距离s，封装成一个level，最后所有level封装成way_points

```c++
for (std::size_t i = 0; accumulated_s < total_length; ++i) {            // 纵向采样
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) {                      // 每个纵向位置采样得到的所有横向采样点，封装成一个level
      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      level_points.push_back(std::move(sl));
    }
    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {    // 对于不变道但是要超车的情况，额外增加一个车道线中心采样点
      auto sl_zero = common::util::MakeSLPoint(s, 0.0);     
      level_points.push_back(std::move(sl_zero));
    }
    if (!level_points.empty()) {                  // level不为空，封装进最终的way+points
      points->emplace_back(level_points);
    }
```

**Q2：如何计算两个level之间两两横向采样点之间的开销？--`UpdateNode`函数和`TrajectoryCost`类**

一个注意点，Q1中采样其实没有把规划起始点加进去，代码中做了一个修正。

```c++
path_waypoints.insert(path_waypoints.begin(), std::vector<common::SLPoint>{init_sl_point_});
```

1. 障碍物处理

在规划中，需要考虑到障碍物的轨迹信息，也就是需要考虑未来每个时刻，障碍物出现的位置，只要将障碍物每个时间点出现的位置，作为开销计算项即可。

```c++
TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);
```

未来每个时间点计算方法也很简单：

```c++
const float total_time = std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);
num_of_time_stamps_ = static_cast<uint32_t>(std::floor(total_time / config.eval_time_interval()));
```

上述`FLAGS_prediction_total_time`其实就是PRediction模块中障碍物轨迹预测总时间，5s。`eval_time_interval`其实就是Prediction模块中障碍物两个预测轨迹点的时间差，0.1s。所以一共差不多50个位置点。最后对参考线上的每个障碍物在每个时间点设定位置标定框。

对于每个障碍物，进行如下条件判断：

情况1：如果是无人车可忽略的障碍物或者迫使无人车停车的障碍物，就不需要考虑。因为前者对无人车前进无影响；后者情况无人车智能停车，根本不需要前进了。

情况2：如果无人车和某个时间点的障碍物横向距离比较大，那么无人车可以毫无影响的前进，所以该类障碍物也可以忽略。

情况3：如果障碍物是虚拟障碍物，那么无人车可以毫无影响的前进，所以该类障碍物也可以忽略。

情况4：如果障碍物是静止的，那么将其加入静止障碍物队列

情况5：如果障碍物时运动的，那么就需要计算障碍物在该时间点的位置，并将其加入动态障碍物队列

```c++
std::vector<Box2d> box_by_time;
for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {    // 计算每个时间点的位置，转换成标定框加入动态障碍物队列
  TrajectoryPoint trajectory_point =
        ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());
  Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
  constexpr float kBuff = 0.5;
  Box2d expanded_obstacle_box =
       Box2d(obstacle_box.center(), obstacle_box.heading(),
                  obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
  box_by_time.push_back(expanded_obstacle_box);
}
dynamic_obstacle_boxes_.push_back(std::move(box_by_time));
```

2. level之间两两计算cost

```c++
bool DPRoadGraph::GenerateMinCostPath(
  std::list<std::list<DPRoadGraphNode>> graph_nodes;    // 这是最后的前向遍历图，类似于神经网络结构，N个level，每个level若干横向采样点，两层level之间的采样点互相连接。
  graph_nodes.emplace_back();                           //    而且网络中的每个node，都保存了，从规划起始点到该节点的最小cost，以及反向连接链(cost最小对应的parent)
  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());  // 输入层(规划起始点)加入网络图
  auto &front = graph_nodes.front().front();            // 规划起始点：init_point
  size_t total_level = path_waypoints.size();           // 网络层数，level数量

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {  // 网络两两level之间计算连接cost
    const auto &prev_dp_nodes = graph_nodes.back();     // 前一层level
    const auto &level_points = path_waypoints[level];   // 当前层level中的所有横向采样点(类似神经元)
    graph_nodes.emplace_back();
    for (size_t i = 0; i < level_points.size(); ++i) {  // 计算当前层level中与前一层所有计算的连接权值，也就是cost
      const auto &cur_point = level_points[i];
      graph_nodes.back().emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes.back().back();
      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
        ...
      } else {    // 计算前一层prev_dp_nodes和当前层的节点cur_node的开销cost，取prev_dp_nodes中与cur_node开销cost最小的节点，设置为最优路径
        UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front, &cur_node); 
      }
    }
  }
}
```

经过注释，可以很容易的理解整个网络神经元(节点)之间连接cost的流程，那么Update函数中，如何计算前一层和当前层的某个节点之间的开销。其实比较简单，举个例子。现有：

a. 前一层prev_dp_nodes中的某个节点prev_dp_node，它有4个属性：累计距离s0，横向偏移距离l0，横向速度dl0，横向加速度ddl0。

b. 当前层某个节点cur_node，同样4个属性：累计距离s1，横向偏移距离l1，横向速度dl1，横向加速度ddl1。

两个节点之间累积距离就是上述提到level_distance，大约10-20m。那么接下来的工作就是个参考线平滑一样，使用六次多项式进行两个点之间的多项式拟合，y=f(s)，自变量是累计距离差s，因变量是横向便宜距离l。经过定义两个点可以数值化为：

prev_dp_node:

l0 = f(0) = prev_dp_node.sl_point.l()

dl0 = f'(0)

ddl0 = f''(0)

cur_node: 

delta_s = cur_node.cur_point.s() - prev_dp_node.prev_sl_point.s()

l1 = f(delta_s) = cur_node.cur_point.l()

dl1 = f'(delta_s) = 0

ddl1 = f''(delta_s) = 0

匹配过程与[Prediction障碍物轨迹预测](https://github.com/YannZyl/Apollo-Note/blob/master/docs/prediction/predictor_manager.md/#poly_fit)

那么cur_node与prev_dp_node相连得到的cost为：

```c++
const auto cost = trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                   level, total_level) +  prev_dp_node.min_cost;
cur_node->UpdateCost(&prev_dp_node, curve, cost);
```

其中UpdateCost是刷新最小cost，这样可以得到与cur_node相连最小的cost，以及对应的prev_dp_node。

最后一个问题：计算开销需要分为哪几项？

**Q3：两两node之间的cost如何计算？包含多少项？**

