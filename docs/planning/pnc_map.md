# 规划与控制地图: Pnc Map

pnc map其实和高精地图hd map没有关系，后者是专门为规划与控制模块设计的库函数，在hd map层次之上，负责一些地图相关信息的处理。例如查询车辆可能的形式路由段(list<RouteSegments>)，然后对每个路由段合成一个路径Path，这是pnc map最重要的功能。

pnc map目前被封装在指引线提供器ReferenceLineProvider中，但是由于其功能比较集中，我们单独将他拿出来讲解。规划控制地图pnc map主要的功能有三个：

1. 更新路由信息。这部分接受Routing模块的路径查询响应，将其响应信息处理存储到地图类中。

2. 短期路径段查询。根据Routing规划路径以及当前车辆的位置，计算当前车辆可行驶的车道区域。

3. 路径段生成最终路径。针对2中每个可行驶的车道路由段，生成一条路径Path，可以后续生成参考线Reference Line。

## 1. 更新路由信息 (`PncMap::UpdateRoutingResponse`)

更新路由信息的函数原型为：

```c++
bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing)
```

从参数可以看到，更新阶段其实是将路径查询的响应结果进行初步的剥离，剥离出查询点(`routing.routing_request().waypoint()`)以及对应的查询结果(`routing.road()`)。下面我们从代码入手，对更新路由信息阶段进行分析。从代码的功能性上分析，我们将这个阶段的内容分为两个流程：响应结果的剥离与查询点处理。

**1. 响应结果剥离**

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();
  // Step 1
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size(); ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size(); ++lane_index) {
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment = ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Fail to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }
  ...
}
```

从上述代码可以响应结果剥离工作比较简单，就是对这条完整路径进行RoadSegment，Passage，LaneSegment的存储，依旧使用第一小节中的图:

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/routing_result.png)

最后得到的结果：

```c++
all_lane_ids_: ['lane 1', 'lane 1', 'lane 1',       // RoadSegment 0, Passage 0, LaneSegment 0-2
                'lane 2', 'lane 2', 'lane 2',       // RoadSegment 0, Passage 1, LaneSegment 0-2

                'lane 1', 'lane 1', 'lane 1',       // RoadSegment 1, Passage 0, LaneSegment 0-2
                'lane 2', 'lane 2', 'lane 2',       // RoadSegment 1, Passage 1, LaneSegment 0-2

                'lane 1', 'lane 1', 'lane 1',       // RoadSegment 2, Passage 0, LaneSegment 0-2
                'lane 2', 'lane 2', 'lane 2',]      // RoadSegment 2, Passage 1, LaneSegment 0-2

route_indices_: [LaneSegment{id: 'lane 1', s: [100,110], index: [0,0,0]},     // RoadSegment 0, Passage 0, LaneSegment 0
                 LaneSegment{id: 'lane 1', s: [110,120], index: [0,0,1]},     // RoadSegment 0, Passage 0, LaneSegment 1
                 LaneSegment{id: 'lane 1', s: [120,130], index: [0,0,2]},     // RoadSegment 0, Passage 0, LaneSegment 2
                 LaneSegment{id: 'lane 2', s: [240,250], index: [0,1,0]},     // RoadSegment 0, Passage 1, LaneSegment 0
                 LaneSegment{id: 'lane 2', s: [250,260], index: [0,1,1]},     // RoadSegment 0, Passage 1, LaneSegment 1
                 LaneSegment{id: 'lane 2', s: [260,270], index: [0,1,2]},     // RoadSegment 0, Passage 1, LaneSegment 2

                 LaneSegment{id: 'lane 1', s: [130,140], index: [1,0,0]},     // RoadSegment 1, Passage 0, LaneSegment 0
                 LaneSegment{id: 'lane 1', s: [140,150], index: [1,0,1]},     // RoadSegment 1, Passage 0, LaneSegment 1
                 LaneSegment{id: 'lane 1', s: [150,160], index: [1,0,2]},     // RoadSegment 1, Passage 0, LaneSegment 2
                 LaneSegment{id: 'lane 2', s: [270,280], index: [1,1,0]},     // RoadSegment 1, Passage 1, LaneSegment 0
                 LaneSegment{id: 'lane 2', s: [280,290], index: [1,1,1]},     // RoadSegment 1, Passage 1, LaneSegment 1
                 LaneSegment{id: 'lane 2', s: [290,300], index: [1,1,2]},     // RoadSegment 1, Passage 1, LaneSegment 2

                 LaneSegment{id: 'lane 1', s: [160,170], index: [2,0,0]},     // RoadSegment 2, Passage 0, LaneSegment 0
                 LaneSegment{id: 'lane 1', s: [170,180], index: [2,0,1]},     // RoadSegment 2, Passage 0, LaneSegment 1
                 LaneSegment{id: 'lane 1', s: [180,190], index: [2,0,2]},     // RoadSegment 2, Passage 0, LaneSegment 2
                 LaneSegment{id: 'lane 2', s: [300,310], index: [2,1,0]},     // RoadSegment 2, Passage 1, LaneSegment 0
                 LaneSegment{id: 'lane 2', s: [310,320], index: [2,1,1]},     // RoadSegment 2, Passage 1, LaneSegment 1
                 LaneSegment{id: 'lane 2', s: [320,330], index: [2,1,2]}]     // RoadSegment 2, Passage 1, LaneSegment 2
```

**2. 查询点处理**

从上述图中可以看到，该次查询的查询点waypoint一共两个，起点(红色星星)和终点(蓝色星星)。这一步需要对这两个waypoint进行处理，计算这两个waypoint分别在上述的那些LaneSegment中，准确的说是上述all_lane_ids_中每个LaneSegment包含有哪些waypoint。判断是否在对应的LaneSegment中的代码为

```c++
/// file in apollo/modules/map/pnc_map/route_segments.cc
bool RouteSegments::WithinLaneSegment(const LaneSegment &lane_segment, const routing::LaneWaypoint &waypoint) {
  return lane_segment.lane && lane_segment.lane->id().id() == waypoint.id() &&
         lane_segment.start_s - kSegmentationEpsilon <= waypoint.s() &&
         lane_segment.end_s + kSegmentationEpsilon >= waypoint.s();
}
```

从代码中可以看到，waypoint在lane_segment中需要满足条件：

1. waypoint和lane_segment的所在的车道lane的id必须一致
2. waypoint的累计距离s必须在lane_segment的start_s和end_s之间。

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  // Step 1
  ...
  // Step 2
  for (std::size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           RouteSegments::WithinLaneSegment(route_indices_[j].segment, request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(LaneWaypoint(route_indices_[j].segment.lane, request_waypoints.Get(i).s()), j);
      ++i;
    }
  }
}
```

最终得到的结果：

```c++
routing_waypoint_index_: [
                          // waypoint 1(start point), s=105, j=0(route_indices_[0]: RoadSegment 0,Passage 0,LaneSegment 0)
                          LaneWaypoint{id: 'lane 1', s: 105, j: 0},      

                          // waypoint 2(end point), s=185, j=15(route_indices_[15]: RoadSegment 2,Passage 0,LaneSegment 2)
                          LaneWaypoint{id: 'lane 1', s: 185, j: 15}
                         ]    
```

## 2. 短期路径段查询 (`PncMap::GetRouteSegments`)

查询路径查询的目的其实与Prediction模块的Obstacle的LaneGraph的LaneSequence生成是一样的，Planning模块的短期路径查询是根据当前主车的位置，去查询无人驾驶汽车可以行使的车道段(Passage && LaneSegment)；而后者是为预测障碍物的运动轨迹(LaneSequence)。在这个查询阶段，必须要保证已经存在RoutingResponse，所以在`PncMap::GetRouteSegments`函数运行时，必须保证在之前已经更新过路由信息`PncMap::UpdateRoutingResponse`。

短期路径查询的函数原型为：

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments)
```

这是pnc map最重要的功能，从函数参数分析，我们可以看到`GetRouteSegments`接受的参数最重要的是车辆的状态(包含车辆的位置，速度，偏航角，加速度等信息)，`backward_length`和`forward_length`是路径短生成过程中前向与后向修正距离，目前暂时不用考虑。而返回的是短期内(注意是短期内，长期调度不确定因素太多，无法实现)车辆的运动轨迹`route_segments`，这是std::list<RouteSegments>类型的，与上上节的`LaneGraph::LineSequence`类似，每个元素都是代表当前车辆的一种运动方案。

那么存在如下问题：

1. 在Prediction障碍物短期运动方案规划中，"短期"是指当前障碍位置到LaneSequence的第一个最近Lanepoint的位置的路段(在LanePoint构建时，每条LaneSequence最多持有20个LanePoint，每两个LanePoint之间的距离为2m)，所以Prediction中的短期大致预测在2m这个邻域范围内障碍物的运动轨迹。那么Planning模块对无人车短期的路径查询，"短期"是多少范围或者是什么概念？

2. 如何预测或者是查询主车路径？

第一个问题比较简单，Planning对主车"短期"路径查询的距离段由函数后向查询距离`backward_length`和前向查询距离`forward_length`决定，所以查询的路长为当前主车前后`backward_length + forward_length`距离的路段。

额外补充一下，在指引线提供器中，调用该函数，后向查询距离`FLAGs_look_backward_distance`默认为30m，前向查询距离取决于主车速度，若速度很大 `linear_velocity() * FLAGS_look_forward_time_sec > FLAGS_look_forward_short_distance`，其中`FLAGS_look_forward_time_sec`默认8s，`FLAGS_look_forward_short_distance`默认150m)，前向查询设置为`look_forward_long_distance`，默认250m；否者速度小时设置为`FLAGS_look_forward_short_distance`，150m。

在主车路径查询问题中，我们简单地将过程分为几个子流程，分别为：

### A. 更新pnc map中无人车状态

这里的无人车状态不是无人车自身坐标，速度等信息，而是在上述更新路由信息过程中得到的`route_indices_`中，无人车在哪个LaneSegment中，距离无人车最近的下一个查询点waypoint的信息。

```c++
/// file in apollo/master/modules/map/pnc_map/pnc_map.cc
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  // Step 1. update vehicle state
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map";
    return false;
  }
  ...
}
```

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/routing_segments.png)

这部分由`UpdateVehicleState`函数完成，这个函数完成的工作就是计算上图中的红色部分：包括当前车辆在对应车道上的投影`adc_waypoint_`，车辆投影点所在LaneSegment在`route_indices_`中的索引`adc_route_index_`，下一个最近查询点在`routing_waypoint_index_`中的索引`next_routing_waypoint_index_`。

**这里纠正一下，上图中的`adc_route_index_`写错了，不是6，而是15**

- 计算车辆在对应车道上的投影`adc_waypoint_`

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  // Step 1. 计算当前车辆在对应车道上的投影adc_waypoint_
  adc_state_ = vehicle_state;
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    return false;
  }
  ...
}
```

由于代码量比较多，而且代码难度不高，所以`GetNearestPointFromRouting`函数就不贴代码，这里描述一下函数计算的流程：

1. 根据当前车辆坐标(x,y)以及速度方向heading，去高精地图hd map查询车辆附近的车道(车道方向必须与heading方向夹角在90度以内，意味着都是相同方向的车道，超过90度可能是反向车道)

2. 计算查询得到的车道和`range_lane_ids_`或者`all_lane_ids_`的交集，也就是查找RoutingResponse.road().passage()中的附近的车道。

3. 对2中过滤得到的车道进行车辆坐标投影，可计算车辆到各个车道的距离，去最小距离的车道作为最终的投影车道，得到`adc_waypoint_`的车道id和累积距离s，其实也可以计算投影点，但是这里没有计算。

- 计算索引`adc_route_index_`和`next_routing_waypoint_index_`

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  // Step 1. 计算当前车辆在对应车道上的投影adc_waypoint_
  ...
  // Step 2. 计算车辆投影点所在LaneSegment在route_indices_`的索引`dc_route_index_`  
  int route_index = GetWaypointIndex(adc_waypoint_);
  // track how many routing request waypoints the adc have passed.
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);
}
```

`GetWaypointIndex`函数也是比较简单，只要根据`adc_waypoint_`的车道id和累计距离前向或者后向查询`route_indices_`，如果某个LaneSegment和`adc_waypoint_`的车道id一致，并且累计距离s在LaneSegment的[start_s, end_s]区间，就可以得到投影点所在的索引`adc_route_index_`。

`UpdateNextRoutingWaypointIndex`是从当前车辆的索引开始，向后查找最近的查询点waypoint(必须是同一个LaneSegment)，得到这个查询点所在的LaneSegment索引`next_routing_waypoint_index_`。这个过程代码稍微有点难理解，这里简单地做一个说明。

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  // 情况1. 车道倒车，后向查找，下一个查询点waypoint对应的索引查找
  // search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index > cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index == cur_index &&
         adc_waypoint_.s < routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // 情况2. 车道前进，前向查找，下一个查询点waypoint对应的索引查找
  // search forwards
  // 第1步，查找最近包含有waypoint的LaneSegment 
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index < cur_index) {
    ++next_routing_waypoint_index_;
  }
  // 第2步，查找下一个最近的waypoint对应的索引
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index == routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >= routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}
```

**举例1**

现在有以下这么个情况：

```
|-wp0---------------------vp--------|-------wp1-----wp2---------------wp3-----|-------------------------wp4-----|
\             LaneSegment k         /\            LaneSegment k+1            /\         LaneSegment k+2         /
```

其中vp为当前车辆在LaneSegment中的投影，wp0, wp1, wp2, wp3分别是路径查询的4个waypoint(必须经过的点。现在我们要查询车辆下一个最近的必经点waypoint，那么结果应该是wp1！假设车辆当前是前进状态，`cur_index`值为k，那么：

可以判断，函数执行前，`next_routing_waypoint_index_`值为0(查询wp0时得到)

第1步，查找最近包含有waypoint的LaneSegment，最终得到的结果`next_routing_waypoint_index_`仍然为0，因为`routing_waypoint_index_[next_routing_waypoint_index_].index`就是wp0.index，值为k，与`cur_index`相等。

第2步，查找下一个最近的waypoint对应的索引，最终的结果`next_routing_waypoint_index_`变为1，因为`adc_waypoint_.s >= wp0.s`。

**举例2**

现在有以下这么个情况(车辆行驶到了LaneSegment k+2，并且已经经过了wp1，wp2和wp3)：

```
|-wp0-------------------------------|----wp1--------wp2---------------wp3-----|-----vp------------------wp4-----|
\              aneSegment k         /\            LaneSegment k+1            /\         LaneSegment k+2         /
```

其中vp为当前车辆在LaneSegment中的投影，wp0, wp1, wp2, wp3分别是路径查询的4个waypoint(必须经过的点)，其中wp0，wp1，wp2和wp3已经经过了，wp4待车辆经过。现在我们要查询车辆下一个最近的必经点waypoint，那么结果应该是wp4！假设车辆当前是前进状态，`cur_index`值为k+2，那么：

可以判断，函数执行前，`next_routing_waypoint_index_`值为3(查询wp3时得到)

第1步，查找最近包含有waypoint的LaneSegment，最终得到的结果`next_routing_waypoint_index_`变为4，因为`routing_waypoint_index_[next_routing_waypoint_index_].index`就是wp3.index，值为k+1，`wp3.index < cur_index`。

第2步，查找下一个最近的waypoint对应的索引，`next_routing_waypoint_index_`仍然为4，因为`adc_waypoint_.s < wp4.s`  

- 到达目的地标志

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  // Step 1. 计算当前车辆在对应车道上的投影adc_waypoint_
  ...
  // Step 2. 计算车辆投影点所在LaneSegment在route_indices_`的索引`dc_route_index_`  
  int last_index = GetWaypointIndex(routing_waypoint_index_.back().waypoint);
  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1 ||
      (!stop_for_destination_ && last_index == routing_waypoint_index_.back().index)) {
    stop_for_destination_ = true;
  }
}
```

如果`next_routing_waypoint_index_`是终点的索引，表示已经到达目的地。

### B. 计算临近通道

在上一步更新车辆状态中，根据路由查询响应以及车辆状态可以得到当前车辆在规划路径中的位置`adc_waypoint_`，以及下一个必经查询点的索引`next_routing_waypoint_index_`。接下来这一步就是查询当前位置下，附近的通道。这部分功能由函数`GetNeighborPassages`实现。

```c++
/// file in apollo/master/modules/map/pnc_map/pnc_map.cc
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  // Step 1. update vehicle state
  ...
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road = routing_.road(road_index);
  // Step 2. compute neighbor passages
  // raw filter to find all neighboring passages
  auto drive_passages = GetNeighborPassages(road, passage_index);
}
```

代码实现比较容易看懂，这里我们简单地总结一下获取邻接可驶入通道的原则：

1. 如果当前通道(Passage)是直行道(`change_lane_type==routing::FORWARD`)，无法变道，那么直接返回车辆所在的车道

```c++
if (source_passage.change_lane_type() == routing::FORWARD) {
  return result;
}
```

2. 如果当前通道已经准备退出(`can_exit=-True`)，如上图中各个Passage中的LaneSegment3，车辆已经准备进入下一个Passage，不需要变道，直接返回车辆所在的车道

```c++
if (source_passage.can_exit()) {  // no need to change lane
  return result;
}
```

3. 如果下一个毕竟查询点(`routing_waypoint_index_[next_routing_waypoint_index_].waypoint`)在当前通道中，不需要变道，直接返回车辆所在的车道

```c++
RouteSegments source_segments;
if (!PassageToSegments(source_passage, &source_segments)) {
  return result;
}
if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
    source_segments.IsWaypointOnSegment(routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
  return result;
}
```

4. 否则，如果车辆再左转车道或者右转车道，从高精地图hd map中查询当前车道对应左侧或者右侧的所有车道线，然后去和当前RoadSegment.passage()去做对比，找到两者共同包含的车道，就是最终的邻接车道。

```c++
  std::unordered_set<std::string> neighbor_lanes;
  if (source_passage.change_lane_type() == routing::LEFT) {   // 当前passage是左转通道
    for (const auto &segment : source_segments) {     // 查询当前Passage中每个LaneSegment所在车道的邻接左车道
      for (const auto &left_id : segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) { 当前passage是右转通道
    for (const auto &segment : source_segments) {     // 查询当前Passage中每个LaneSegment所在车道的邻接右车道
      for (const auto &right_id : segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {       // 查询当前RoadSegment中所有Passage::LaneSegment的所属车道，有交集就添加到结果中
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}
```

### C. 创建车辆当前可行驶区域

在上一步中，我们找到了当前车辆在规划轨迹中的邻接车道。这一步就对每个邻接车道做一个是否可驶入的检查，并做道路段截取。也就是制定出无人车在当前情况下可能行驶的区域，每个可行驶通道将划分出一个道路区间，上面已经提到过，道路区间的长度由前向查询距离`forward_length`和后向查询距离`backward_length`决定，短期内规划的可行驶道路段长度为`forward_length+backward_length`。

最终的路由段RouteSegments生成只需要对每个邻接可驶入的通道进行Segments生成即可。

```c++
/// file in apollo/master/modules/map/pnc_map/pnc_map.cc
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  // Step 1. update vehicle state
  ...
  // Step 2. compute neighbor passages
  ...
  // Step 3. compute route segment
  for (const int index : drive_passages) {
    ...
  }
  return !route_segments->empty();
}
```

Passage生成对应的RouteSegments分为4个步骤，分别为：

**Step 3.1 将当前车辆的坐标投影到Passage**

这部分是比较常见的，也是之前模块反反复复计算的，直接贴源码：

```c++
// step 3.1 project vehicle pos into passage
PointENU nearest_point = MakePointENU(adc_state_.x(), adc_state_.y(), adc_state_.z());
if (index == passage_index) {
  nearest_point = adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s);
}
common::SLPoint sl;
LaneWaypoint segment_waypoint;
if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
  continue;
}
```

**Step 3.2 检查Passage是否可驶入**

在B计算临近通道时，我们根据当前的位置以及车道变道情况得到了一系列临近车道，但是这些邻接车道是否可驶入没有被检查，这部分就对车道行驶的合法进行检查。

```c++
// step 3.2 check if an drive to passage
if (index != passage_index) {
  if (!segments.CanDriveFrom(adc_waypoint_)) {
    continue;
  }
}
```

合法性检查代码也比较简单，我们这里用规则来总一下，当前车辆所在车道(lane a)的位置为`adc_waypoint_`，要判断能否行驶到passage，规则制定如下：

1. 如果这个点`adc_waypoint_`就在passage对应的LaneSegments中，那么合法可驶入。

```c++
/// file in apollo/modules/map/pnc_map/route_segments.cc
bool RouteSegments::CanDriveFrom(const LaneWaypoint &waypoint) const {
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  // 0 if waypoint is on segment, ok
  if (IsWaypointOnSegment(waypoint)) {
    return true;
  }
}
```

2. 如果这个点投影到passage中，发现投影点均不在passage对应的LaneSegments中，那么就说明当前车辆已经不再这个passage道路段中了，不可驶入；如果满足以上条件，但是点到这个passage的投影距离过大，说明车辆横向距离太大，可能垮了好几个车道，那么不可驶入。

```c++
bool RouteSegments::CanDriveFrom(const LaneWaypoint &waypoint) const {
  // 1. should have valid projection.
  LaneWaypoint segment_waypoint;
  common::SLPoint route_sl;
  bool has_projection = GetProjection(point, &route_sl, &segment_waypoint);
  if (!has_projection) {                     // 车辆无法投影到passage中，不可驶入
    return false;
  }
  constexpr double kMaxLaneWidth = 10.0;
  if (std::fabs(route_sl.l()) > 2 * kMaxLaneWidth) {     // 车辆横向距离passage过大，不可驶入
    return false;
  }
}
```

3. 检测当前车辆所在车道和投影到passage中对应的LaneSegment所属的车道方向是否一致。必须小于90度，否则就是反向车道，不能直接驶入。

```c++
bool RouteSegments::CanDriveFrom(const LaneWaypoint &waypoint) const {
  // 2. heading should be the same.
  double waypoint_heading = waypoint.lane->Heading(waypoint.s);
  double segment_heading = segment_waypoint.lane->Heading(segment_waypoint.s);
  double heading_diff = common::math::AngleDiff(waypoint_heading, segment_heading);
  if (std::fabs(heading_diff) > M_PI / 2) {
    ADEBUG << "Angle diff too large:" << heading_diff;
    return false;
  }
}
```

4. 当前车辆所在车道和投影到passage中对应的LaneSegment所属车道必须是相邻的，不能跨车道驶入(每次只能变道一次，无法连续变道)。

```c++
bool RouteSegments::CanDriveFrom(const LaneWaypoint &waypoint) const {
  // 3. the waypoint and the projected lane should not be separated apart.
  double waypoint_left_width = 0.0;
  double waypoint_right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &waypoint_left_width, &waypoint_right_width);
  double segment_left_width = 0.0;
  double segment_right_width = 0.0;
  segment_waypoint.lane->GetWidth(segment_waypoint.s, &segment_left_width, &segment_right_width);
  auto segment_projected_point = segment_waypoint.lane->GetSmoothPoint(segment_waypoint.s);
  double dist = common::util::DistanceXY(point, segment_projected_point);
  const double kLaneSeparationDistance = 0.3;
  if (route_sl.l() < 0) {  // waypoint at right side
    if (dist > waypoint_left_width + segment_right_width + kLaneSeparationDistance) {
      AERROR << "waypoint is too far to reach: " << dist;
      return false;
    }
  } else {  // waypoint at left side
    if (dist > waypoint_right_width + segment_left_width + kLaneSeparationDistance) {
      AERROR << "waypoint is too far to reach: " << dist;
      return false;
    }
  }
}
```

这个过程也是比较简单，`waypoint`是车辆在他当前车道中心线的投影点；`segment_waypoint`是车辆在passage中某个LaneSegment所属车道中心线上的投影点，`route_sl.l`是投影距离，小于0车辆在右侧，大于0车辆在左侧。判断条件比较简单：

- 如果车辆在passage右边，`route_sl.l()<0`，那么可以近似的使用`waypoint_left_width+segment_right_width+kLaneSeparationDistance`来估计当前车辆所在车道和投影到passage中对应的LaneSegment所属车道中心线之间的距离，`kLaneSeparationDistance`是中间分割线宽度。如果投影距离dist大于这个距离，则设置为无法变道，因为距离太大，无法一次性变道完成。

- 如果车辆在passage左边，`route_sl.l()>0`，那么可以近似的使用`waypoint_right_width+segment_left_width+kLaneSeparationDistance`来估计当前车辆所在车道和投影到passage中对应的LaneSegment所属车道中心线之间的距离，`kLaneSeparationDistance`是中间分割线宽度。同理。如果投影距离dist大于这个距离，则设置为无法变道，因为距离太大，无法一次性变道完成。

**Step 3.3 生成RouteSegmens**

这部分就是对上述通过可驶入合法性检查的车道进行道路段的生成，同时使用`backward_length`和`backward_length`前后扩展道路段。

```c++
// step 3.3 extend segment use backward_length and forward_length
route_segments->emplace_back();
const auto last_waypoint = segments.LastWaypoint();
if (!ExtendSegments(segments, sl.s() - backward_length, sl.s() + forward_length, &route_segments->back())) {
  return false;
}
```

从代码可以看到，原本车辆在passage中的投影点累计距离为sl.s(**注意这个s是投影点在passage段起点的累计距离，而非整个road的累计距离**)，扩展后前向增加`forward_length`，后向增加`backward_length`，扩展的代码也比较简单。主要是针对：

1. 前置车道处理

当车辆投影点所在车道的后向查询起始点小于0，即`s1.s() - backward_length < 0`，此时就需要查询`first_lane_segment`对应的车道前置部分进行截取，如果车道前置部分仍然不够长度`fabs(s1.s() + backward_length)`，那么就需要加入这条车道的前置车道继续截取。

注意这里为什么是`s1.s() - backward_length < 0`，而不是`s1.s() - backward_length < first_lane_segment.s`？

因为sl.s是投影点相对于passage(first_lane_segment.start_s)的累计距离，而不是相对于整个规划路径road(first_passage.first_lane_segment.start_s)的累计距离。所以用小于0来判断，后向查询起始点是否在第一个LaneSegment之前。

```c++
bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  const double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {                       // 当后向查询起始点小于0，说明需要用到这条lane的前置lane
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;       // 或者passage的第一个LaneSegment的所属车道
    double s = first_segment.start_s;     
    double extend_s = -start_s;           // extend_s为需要从前置车道中截取的道路段长度，初始化为-start_s，
    std::vector<LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {    // 每次循环(截取)以后extend_s都会减小，直至到0
      if (s <= kRouteEpsilon) {           // s < 0，则需要在查询这条lane对应的前置车道，进行截取
        lane = GetRoutePredecessor(lane); // 获取当前lane的前置车道
        if (lane == nullptr) {
          break;
        }
        s = lane->total_length();
      } else {                           // 如果s > 0，此时就可以从这条前置lane中截取道路段
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);  // 截取道路段
        extend_s -= length;              // 更新extend_s，如果extend_s>0，说明还需要继续寻找前置道路段截取
        s -= length;
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
}
```


下面部分就是正常的passage中LaneSegment截取，根据start_s和end_s

```c++
bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  // router_s代表已经累计截取到了的LaneSegment长度，如果当前正在截取第3个LaneSegment，那么router_s就是前两个LaneSegment的长度和
  double router_s = 0;            
  for (const auto &lane_segment : segments) {
  	// 计算当前LaneSegment需要截取的start_s和end_s，非最后一段，start_s和end_s就是这个LaneSegment的start_s和end_s，意味着整段截取
    const double adjusted_start_s = std::max(start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s = std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      // 有前置车道的，如果前置最后一段的前置车道和当前LaneSegment的车道相同，那么需要合并(修改end_s即可)；否则新建一段加入list
      if (!truncated_segments->empty() && truncated_segments->back().lane->id().id() == lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s, adjusted_end_s);
      }
    }
    // 判断是否截取结束，如果结束了那么可以退出，否则就需要继续截取，当最后循环最后一次最后一个LaneSegment还是没有结束，那么就需要
    // 新增加后置车道继续处理
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
}
```

2. 后接车道处理

在1中常规的passage中LaneSegment截取结束后，如果router_s仍然小于end_s，就说明车道截取还未结束，还有一段长度`end_s - router_s`的道路段未被截取，此时passage中的LaneSegment已经全部截取完了，所以需要访问最后一个LaneSegment对应的lane，需要继续截取这条lane的后续部分，如果后续部分长度仍然不够，就需要加入这条lane的后接车道继续截取。

```c++
bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && !truncated_segments->empty()) {       // 仍然有未被截取的道路段(长度还没满足)
    auto &back = truncated_segments->back();                    
    if (back.lane->total_length() > back.end_s) {               // 查找最后一个LaneSegment对应的车道，继续从该车道截取
      double origin_end_s = back.end_s;
      back.end_s = std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  if (router_s < end_s) {              // 如果最后一个LaneSegment对应的车道截取完了，还没达到长度要求，虚招这个车道的后接车道，继续截取
    auto last_lane = GetRouteSuccessor(segments.back().lane);
    double last_s = 0.0;               // last_s记录了这个后接车道累计截取到的车道段长度
    while (router_s < end_s - kRouteEpsilon) {
      if (last_lane == nullptr) {
        break;
      }
      if (last_s >= last_lane->total_length() - kRouteEpsilon) {  // 如果累计截取长度达到了lane的总长度，那么需要添加新的后接lane，last_s清零
        last_lane = GetRouteSuccessor(last_lane);
        last_s = 0.0;
      } else {
        const double length = std::min(end_s - router_s, last_lane->total_length() - last_s);
        truncated_segments->emplace_back(last_lane, last_s, last_s + length);
        router_s += length;
        last_s += length;
      }
    }
  }
  return true;
}
```

**Step 3.4 设置RouteSegments属性**

在Step 3.1-3.3中，我们已经完成了一条passage对应的路由段生成，最终就需要添加这个路由段的一些属性，包括:

1. 是否可以退出通道(最后一段Segment决定)

```c++
route_segments->back().SetCanExit(passage.can_exit());
```

2. 下一步的动作(最后一段Segment决定)

```c++
route_segments->back().SetNextAction(passage.change_lane_type())
```

3. RouteSegments的id和是否是目的地

```c++
std::string route_segment_id = std::to_string(road_index) + "_" + std::to_string(index);
route_segments->back().SetId(route_segment_id);
route_segments->back().SetStopForDestination(stop_for_destination_);
```

4. 设置上时刻的状态

```c++
if (index == passage_index) {
  route_segments->back().SetIsOnSegment(true);
  route_segments->back().SetPreviousAction(routing::FORWARD);
} else if (sl.l() > 0) {     // 如果当前车辆在passage左侧，那么车辆肯定需要向右变道到passage
  route_segments->back().SetPreviousAction(routing::RIGHT);
} else {                     // 如果当前车辆在passage右侧，那么车辆肯定需要向左变道到passage
  route_segments->back().SetPreviousAction(routing::LEFT);
}
```

## 3. 路由段RouteSegments到最终路径的生成

在上一个功能产生无人车短期规划路径，也就是行驶区域后，得到一个list<RouteSegments>的类型的结果，其中list中每个元素都是一个可能的形式路径。而最终路径的生成就是将上述的RouteSegments各个段，整合在一起并且拼接出一条完整的路径。这条最终的路径(Path类型)与HD Map中的车道Lane属性有点相似，不光是包含了各个LaneSegment，还有路径上的采样点，每个点的方向heading，以及减速带，交叉口，停车区的覆盖区域。

RouteSegments中的每个段包含的属性有：

- 所属车道id
- 起始点累积距离start_s
- 结束点累积距离end_s

而在高精地图HD Map中，车道Lane的结构存储是拆分成若干LineSegment2d段，每个段都有上述三个属性(lane_id, start_s, end_s)，也包含这个段的方向(unit_direction)。此外，还采样了这条Lane中的若干路径点point作为轨迹来保存，所以每个段LineSegment2d中都包含有一些采样点。采样点机制可以更加细节化刻画这条车道的属性。

但是RouteSegments中的段与HD Map中Lane的LineSegment2d又是不一样的。虽然两者都有上述三个属性，但是前者仅仅为了存储道路段，可以是很长的曲线道路段，因此只需要有lane_id, start_s和end_s即可，表示这条路起始到最终的路段，没有路段的细节信息；而后者是为了将Lane离散化采样存储，所以LineSegment2d是很短的道路段，长度不会太长(否则会丢失信息)，而且只能是直线段，同时额外包含细节信息，比如段的方向heading，段起始点坐标start_pos，段终点坐标end_pos等。

RouteSegments路径点生成的主要目的，是查询HD Map，根据其车道Lane段内的采样点point，将这个大而宏观的的RouteSegment划分成小的离散的轨迹点MapPathPoint，然后这些点两两组合成一个小的LineSegment2d，顺便计算这个段的方向heading(结束点坐标-起始点坐标)，这样就可以将行驶区域与Lane一样，利用更小的"LineSegment2d"来保存。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/generate_path_1.png)

如上图所示例子，图中给出了一个行驶方案。为了方便讲解，此处没有考虑到ExtendSegments，总共包含4个RoadSegments，每个RoadSegments由N个Passage(这里取一个做示例，所以是一个RouteSegments)，每个Passage包含3个LaneSegment。

那么如果没有考虑到前向查询和后向查询的路径段扩充，最终得到的RouteSegments就包含了12段。每段都有所属车道编号lane_id, 段起始累计距离start_s, 段结束累积距离end_s，每个段都可以很长！为了更加细致的描述这条路由路径，我们取每个段所述的车道Lane中的采样点point。

图中紫色point是"lane 1"HD Map中保存的采样点point，蓝色point是"lane 2"HD Map中保存的采样点point。从图中不难发现有7个紫色point在RouteSegments的区域内(对应"lane 1"部分)，5个蓝色point在RouteSegments区域内(对应"lane 2"部分)。所以这个RouteSegments就被离散化12个点进行保存。实际点可能更加密集，效果会更加好。每个点都可计算其点坐标，方向heading，以及该车道的累积距离。这些点就是被封装好的MapPathPoint。

**RouteSegments表示不足之处**：另外还有一个原因是RouteSegments形式不够好，虽然RouteSegments中已经将路径比较好的表示了，但是如果想用{lane_id, start_s, end_s}形式来表示一条路径，其实可以更加方便高效的标识。为什么不把前7个段合并成一个大的LaneSegment，因为他们属于同一个车道"lane 1"，并且是相邻的段；而后面5个段可以合成另一个大的LaneSegment？

所以总结一下：下面的工作主要有4步

A. RouteSegments离散化MapPathPoint

B. 离散LaneSegment2d && 重构RouteSegment的LaneSegment

C. 道路采样点生成

D. 覆盖区域设置

### A. RouteSegments离散化MapPathPoint

针对每个可行驶区域方案(RouteSegments，由多个段组成)，生成段内的MapPathPoint路径点，言外之意将RouteSegments以路径点的形式进一步离散化。

由RouteSegments到vector<MapPathPoint>的生成函数如下，代码也是比较简单，这里不做多介绍。

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {         // 封装中间点point
      points->emplace_back(lane->points()[i], lane->headings()[i], LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) { // 封装段起点waypoint
        points->emplace_back( segment.start() + segment.unit_direction() * (start_s - accumulate_s), lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {     // 封装段终点waypoint
        points->emplace_back( segment.start() + segment.unit_direction() * (end_s - accumulate_s), lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}
```
可以看到，除了原本lane中的point封装成MapPathPoint，还加入了段起点和终点作为新的MapPathPoint，所以可能在接壤或者开始或者结尾部分，某些点的距离过近，PncMap::CreatePathFromLaneSegments函数中使用RemoveDuplicates(&points)完成去冗余点功能。

**所以这里指出一个错误: 上图中的LaneSegment应该改成LaneSegment2d!**

### B. 离散LaneSegment2d && 重构RouteSegment的LaneSegment

在上步骤"A. RouteSegments离散化MapPathPoint"中，已经将原始粗糙的RouteSegments离散化为一个个MapPathPoint，这一步就将这些MapPathPoint两两重组成一个个新LaneSegment2d.

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/generate_path_3.png)

重组过程如上图，这部分由Path::InitPoints和Path::InitLaneSegments函数完成。

```c++
/// file in apollo/modules/map/pnc_map/path.cc
void Path::InitPoints() {
  num_points_ = static_cast<int>(path_points_.size());
  ...
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    Vec2d heading;
    if (i + 1 >= num_points_) {
      heading = path_points_[i] - path_points_[i - 1];
    } else {              
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);  // MapPathPoint两两生成一个LaneSegment2d
      heading = path_points_[i + 1] - path_points_[i];               // LaneSegment2d的方向，段终点-段起点
      s += heading.Length();
    }
    heading.Normalize();          // 方向正则，长度归一
    unit_directions_.push_back(heading);
  }
  ...
}
```

同时代码还重构了原本的RouteSegment形式的LaneSegment，上述的小段由`std::vector<common::math::LineSegment2d> segments_`存储；而重构RouteSegment由`std::vector<LaneSegment> lane_segments_`保存，不要搞混淆。一个是LaneSegment2d，一个是LaneSegment。

下图是重构LaneSegment的代码，与LaneSegment2d其实很相似，就多了一个段拼接(Join函数)的功能：对那些同一车道，相邻的段进行合并。如上图LaneSegment2d_1-6可以合并成第一个的LaneSegment，LaneSegment2d_7-10可以合并成第二个LaneSegment。

```
/// file in apollo/modules/map/pnc_map/path.cc
void Path::InitLaneSegments() {
  if (lane_segments_.empty()) {
    for (int i = 0; i + 1 < num_points_; ++i) {
      LaneSegment lane_segment;      
      // FindLaneSegment是查询首尾两个点是否在同一个段中，如上图的紫色和蓝色交接两个点，不能组成一个段(因为lane_id不一致)。
      if (FindLaneSegment(path_points_[i], path_points_[i + 1], &lane_segment)) { 
        lane_segments_.push_back(lane_segment);
      }
    }
  }
  // 相同车道的相邻段合并
  LaneSegment::Join(&lane_segments_);
}

void LaneSegment::Join(std::vector<LaneSegment>* segments) {
  constexpr double kSegmentDelta = 0.5;
  std::size_t k = 0;  // k表示合并后段的数量，初始化0，合并一次加1
  std::size_t i = 0;  // i表示原始段集合中的索引，初始化0，访问过程不能超过段的最大索引
  while (i < segments->size()) {
    std::size_t j = i;
    // 查找车道id相同的连续的段，最终一个合并的大段，索引从i到j的j-i+1个段
    // LaneSegment 1-6合并；LaneSegment 7-10合并
    while (j + 1 < segments->size() && segments->at(i).lane == segments->at(j + 1).lane) {  
      ++j;
    }
    auto& segment_k = segments->at(k);                 // 生成新的段, 并且修改起始累积距离，结束累积距离
    segment_k.lane = segments->at(i).lane;
    segment_k.start_s = segments->at(i).start_s;       // 合并后的段，起始累积距离为首段的start_s
    segment_k.end_s = segments->at(j).end_s;           // 合并后的段，结束累计距离为末段的end_s
    if (segment_k.start_s < kSegmentDelta) {
      segment_k.start_s = 0.0;
    }
    if (segment_k.end_s + kSegmentDelta >= segment_k.lane->total_length()) {
      segment_k.end_s = segment_k.lane->total_length();
    }
    i = j + 1;
    ++k;
  }
  segments->resize(k);
  segments->shrink_to_fit();  // release memory
}
```

最终重构与和合并后的LaneSegment有两个，如上图的LaneSegment(final)1和2，其中LaneSegment(final)1由原先的LaneSegment1-6合并完成；LaneSegment(final)2由原先的LaneSegment7-10合并完成。

最后做一个总结，LaneSegment2d和LaneSegment区别

1. LaneSegment2d包含起始点start_pos，结束点end_pos，可以由此计算段方向(end_pos - start_pos)。LaneSegment2d一般是小段。
2. LaneSegment主要存储段的起始点累积距离start_s，结束点累积距离end_s，没有段的坐标信息。LaneSegment可以存储任意长度的车道。

所以前者适合用来存储底层车道的数据结构；后者适合存储车道的大致道路区间信息，仅仅长度范围。


###C. 道路采样点生成

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/generate_path_4.png)

跟HD Map中Lane的构建一样，segments_已经保存了这个RouteSegments的各个段，接下来的工作就是等距离采样(累积距离)，离散化保存这条路径，等间隔采样的频率是每`kSampleDistance`采样一个点，默认0.25m。

这部分工作由Path::InitPointIndex和Path::InitWidth完成。代码比较多，但是不是特别难，这里从函数讲解一下计算过程

1. Path::InitPointIndex函数计算每个采样点的上界MapPathPoint

```
/// file in apollo/modules/map/pnc_map/path.cc
void Path::InitPointIndex() {
  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);
  double s = 0.0;
  int last_index = 0;
  // num_sample_points_ = length_ / kSampleDistance + 1
  for (int i = 0; i < num_sample_points_; ++i) {
  	// 向后遍历，得到采样点(对应累积距离为s)的上界MapPathPoint
    while (last_index + 1 < num_points_ && accumulated_s_[last_index + 1] <= s) {  
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;  // 下一个采样点的累积距离，加上0.25
  }
}
```

2. Path::InitWidth可以根据采样点的累计距离，上界和下届MapPathPoint进行插值，使用`GetSmoothPoint`平滑插值后得到采样点的坐标，并且设置每个采样点的左右车道线距离

```c++
/// file in apollo/modules/map/pnc_map/path.cc
void Path::InitWidth() {
  left_width_.clear();
  left_width_.reserve(num_sample_points_);
  right_width_.clear();
  right_width_.reserve(num_sample_points_);

  double s = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    const MapPathPoint point = GetSmoothPoint(s);          // 利用上下界MapPathPoint插值得到采样点的坐标
    if (point.lane_waypoints().empty()) {
      left_width_.push_back(FLAGS_default_lane_width / 2.0);
      right_width_.push_back(FLAGS_default_lane_width / 2.0);
    } else {
      const LaneWaypoint waypoint = point.lane_waypoints()[0];
      CHECK_NOTNULL(waypoint.lane);
      double left_width = 0.0;
      double right_width = 0.0;
      waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);
      left_width_.push_back(left_width - waypoint.l);
      right_width_.push_back(right_width + waypoint.l);
    }
    s += kSampleDistance;
  }
}
```

平滑GetSmoothPoint过程的计算方法如下：

- 计算采样点下界

Step 1. 可以计算大致的采样点坐标

`sample_id = static_cast<int>(s / kSampleDistance);`

Step 2. 计算理论下界 

`low = last_point_index_[sample_id];`

Step 3. 计算理论上界

```c++
high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);
```

Step 4. 二分法计算真实下界以及与下届的累计距离差

```c++
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (accumulated_s_[mid] <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return {low, s - accumulated_s_[low]};
```

- 插值平滑

```c++
MapPathPoint Path::GetSmoothPoint(const InterpolatedIndex& index) const {
  const MapPathPoint& ref_point = path_points_[index.id];               // 采样点下界point
  if (std::abs(index.offset) > kMathEpsilon) {
    const Vec2d delta = unit_directions_[index.id] * index.offset;      // 插值平滑的位移
    MapPathPoint point({ref_point.x() + delta.x(), ref_point.y() + delta.y()}, ref_point.heading());  //采样点坐标
    // 采样点LaneWaypoint设置
    ...
    return point;
  } else {
    return ref_point;
  }
}
```

###D. 覆盖区域设置

最后一步就是对RouteSegment产生的路径上的覆盖区域，如交叉口车道、停车区域、人行横道等区域加入到这条路径中，方便后续做决策。

```c++
void Path::InitOverlaps() {
  GetAllOverlaps(std::bind(&LaneInfo::cross_lanes, _1), &lane_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::signals, _1), &signal_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::yield_signs, _1), &yield_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::stop_signs, _1), &stop_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::crosswalks, _1), &crosswalk_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::junctions, _1), &junction_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::clear_areas, _1), &clear_area_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::speed_bumps, _1), &speed_bump_overlaps_);
}
```

可以看到所有类型的覆盖区域，一个路段存在多个覆盖区域，每个覆盖区域又存在多个物体，所以访问覆盖区域中的物体，代码需要3个循环。

```c++
void Path::GetAllOverlaps(GetOverlapFromLaneFunc GetOverlaps_from_lane,
                          std::vector<PathOverlap>* const overlaps) const {

  // Step 1. 计算所有的覆盖，保存到overlaps_by_id
  overlaps->clear();
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> overlaps_by_id;
  double s = 0.0;
  // 循环1. 对每个LaneSegment分别做覆盖检测
  for (const auto& lane_segment : lane_segments_) {         
    if (lane_segment.lane == nullptr) {
      continue;
    }
    // 循环2. 每个LaneSegment中有多个覆盖区域，分别做检测
    for (const auto& overlap : GetOverlaps_from_lane(*(lane_segment.lane))) {
      const auto& overlap_info = overlap->GetObjectOverlapInfo(lane_segment.lane->id()); // 获得覆盖区域信息
      if (overlap_info == nullptr) {
        continue;
      }

      const auto& lane_overlap_info = overlap_info->lane_overlap_info();
      if (lane_overlap_info.start_s() < lane_segment.end_s && lane_overlap_info.end_s() > lane_segment.start_s) {
      	// 这里adjusted_start_s和adjusted_end_s相对于第一个LaneSegment的start_s来计算的
        const double ref_s = s - lane_segment.start_s;
        const double adjusted_start_s = std::max(lane_overlap_info.start_s(), lane_segment.start_s) + ref_s;
        const double adjusted_end_s = std::min(lane_overlap_info.end_s(), lane_segment.end_s) + ref_s;
        // 循环3. 每个覆盖区域有多类物体，循环访问
        for (const auto& object : overlap->overlap().object()) {
          if (object.id().id() != lane_segment.lane->id().id()) {
          	// overlaps_by_id详细记录了每个物体占据的道路段，哪里开始到哪里结束
            overlaps_by_id[object.id().id()].emplace_back(adjusted_start_s, adjusted_end_s);
          }
        }
      }
    }
    s += lane_segment.end_s - lane_segment.start_s;
  }
  
  // Step 2. 覆盖合并，保存到函数参数overlaps中
  // E.G. overlaps_by_id的人行横道中两块区域可能可以相连
  for (auto& overlaps_one_object : overlaps_by_id) {
    const std::string& object_id = overlaps_one_object.first;
    auto& segments = overlaps_one_object.second;
    std::sort(segments.begin(), segments.end());
    const double kMinOverlapDistanceGap = 1.5;  // in meters.
    for (const auto& segment : segments) {
      // 检查两个覆盖物体id相同(类型一样)，end_s和start_s在一定阈值之内，就直接合并成一块、
      if (!overlaps->empty() && overlaps->back().object_id == object_id &&
          segment.first - overlaps->back().end_s <= kMinOverlapDistanceGap) {
        overlaps->back().end_s = std::max(overlaps->back().end_s, segment.second);
      } else {
        overlaps->emplace_back(object_id, segment.first, segment.second);
      }
    }
  }
  // 覆盖物体根据离主车距离由近及远排序
  std::sort(overlaps->begin(), overlaps->end(),
            [](const PathOverlap& overlap1, const PathOverlap& overlap2) {
              return overlap1.start_s < overlap2.start_s;
            });
}
```

最后总结一下由RouteSegments生成的Path包含的成员：

| 成员变量或函数 | 功能 |
| -------------- | ---- |
| `std::vector<MapPathPoint> path_points_` | 原始RouteSegments的MapPathPoint数量，共num_points_个路径点，上图例子中，num_points_=12，一共12个MapPathPoint |
| `std::vector<common::math::LineSegment2d> segments_` | MapPathPoint两两构成一个LaneSegment2d，离散化保存RouteSegments，长度为num_points_-1=11 |
| `std::vector<double> accumulated_s_` | 每个MapPathPoint的累积距离，长度为num_points_=12 |
| `std::vector<common::math::Vec2d> unit_directions_` | segments_中每个LaneSegment2d的方向，段终点-段起点，正则化长度1，长度为num_points_-1=11 |
| `std::vector<LaneSegment> lane_segments_` |  MapPathPoint两两构成一个LaneSegment2d，但必须保证起点和终点所属车道id一致，最后经过拼接Join可以重构RouteSegments，上图中lane_segments_一共两个，LaneSegment(final)1和LaneSegment(final)2 |
| `std::vector<LaneSegment> lane_segments_to_next_point_` | MapPathPoint两两构成一个LaneSegment2d，允许起点和终点车道不一致，用于后期路径采样时平滑插值。长度为num_points_-1=11 |
| `double length_` | 路径总长度 | 
| `int num_sample_points_` | 采样点数量，length_ / kSampleDistance(0.25) |
| `std::vector<int> last_point_index_` | 记录每个采样点累积距离s对应的上界MapPathPoint，上界s+下届s可以平滑得到采样点坐标，长度为num_sample_points_ |
| `std::vector<double> left_width_` | 每个采样点距离左车道边界距离，长度为num_sample_points_ |
| `std::vector<double> right_width_` | 每个采样点距离右车道边界距离，长度为num_sample_points_ |
| `std::vector<PathOverlap> lane_overlaps_` | 车道交叉覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> yield_sign_overlaps_` | 车道交叉覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> stop_sign_overlaps_` | 停车覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> crosswalk_overlaps_` | 人行道覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> parking_space_overlaps_`| 停车区域覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> junction_overlaps_` | 路口覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> clear_area_overlaps_` | 禁停区覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |
| `std::vector<PathOverlap> speed_bump_overlaps_` | 减速带覆盖区域，元素形式{id, start_s, end_s}，由start_s从小到大排列，即距离主车最近的覆盖区域最靠前存储 |