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

step 2：在下个位置侧向进行一些点采样(例如7个)，也就是计算在车道不同宽度位置采样。然后计算当前位置(例如4个)所有采样点到下一个位置所有采样点(7个)，两两之间的开销cost。选择cost最小的作为行驶路径。

step 3：前进到最小cost对应的采样点，然后重复2对下个位置横向采样并计算开销cost，选择cost最小再次前进，重复直到规划终点。

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

**Q1：如何采样坐标点？**

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
  const float level_distance =        // 每隔level_distance进行一次横向采样，差不多10-20m进行一次采样
      (init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;
  float accumulated_s = init_sl_point_.s();
  float prev_s = accumulated_s;
```

上面代码有一个词--level，其实就是纵向采样点，每个level中会横向采样若干点。当然，可以更紧凑的纵向与横向采样，但是在计算cost会增加过大的计算量，所以需要平衡。

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