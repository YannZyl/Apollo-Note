# 规划与控制地图: pnc map

pnc map其实和高精地图hd map没有关系，后者是专门为规划与控制模块设计的库函数，在hd map层次之上，负责一些地图相关信息的处理。例如查询车辆可能的形式路径(list<RouteSegments>)，这是pnc map最重要的功能。

pnc map目前被封装在基准线提供器ReferenceLineProvider中，但是由于其功能比较集中，我们单独将他拿出来讲解。下面我们将从代码入手，看看pnc map实现的功能有哪些。

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

额外补充一下，在基准线提供器中，调用该函数，后向查询距离`FLAGs_look_backward_distance`默认为30m，前向查询距离取决于主车速度，若速度很大 `linear_velocity() * FLAGS_look_forward_time_sec > FLAGS_look_forward_short_distance`，其中`FLAGS_look_forward_time_sec`默认8s，`FLAGS_look_forward_short_distance`默认150m)，前向查询设置为`look_forward_long_distance`，默认250m；否者速度小时设置为`FLAGS_look_forward_short_distance`，150m。

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
  if (source_passage.change_lane_type() == routing::LEFT) {
    for (const auto &segment : source_segments) {     // 查询当前Passage中每个LaneSegment所在车道的邻接左车道
      for (const auto &left_id : segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
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
