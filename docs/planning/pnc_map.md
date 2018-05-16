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

额外补充一下，在基准线提供器中，调用该函数，后向查询距离`FLAGs_look_backward_distance`默认为30m，前向查询距离取决于主车速度，若速度很大 $ linear_velocity() * FLAGS_look_forward_time_sec > FLAGS_look_forward_short_distance $，其中`FLAGS_look_forward_time_sec`默认8s，`FLAGS_look_forward_short_distance`默认150m)，前向查询设置为`look_forward_long_distance`，默认250m；否者速度小时设置为`FLAGS_look_forward_short_distance`，150m。

在主车路径查询问题中，我们简单地将过程分为几个子流程，分别为：

**1. 更新pnc map中无人车状态**

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

第2步，查找下一个最近的waypoint对应的索引，最终的结果`next_routing_waypoint_index_`变为1，因为 $ adc_waypoint_.s >= wp0.s $。

**举例2**

现在有以下这么个情况(车辆行驶到了LaneSegment k+2，并且已经经过了wp1，wp2和wp3)：

```
|-wp0-------------------------------|----wp1--------wp2---------------wp3-----|-----vp------------------wp4-----|
\              aneSegment k         /\            LaneSegment k+1            /\         LaneSegment k+2         /
```

其中vp为当前车辆在LaneSegment中的投影，wp0, wp1, wp2, wp3分别是路径查询的4个waypoint(必须经过的点)，其中wp0，wp1，wp2和wp3已经经过了，wp4待车辆经过。现在我们要查询车辆下一个最近的必经点waypoint，那么结果应该是wp4！假设车辆当前是前进状态，`cur_index`值为k+2，那么：

可以判断，函数执行前，`next_routing_waypoint_index_`值为3(查询wp3时得到)

第1步，查找最近包含有waypoint的LaneSegment，最终得到的结果`next_routing_waypoint_index_`变为4，因为`routing_waypoint_index_[next_routing_waypoint_index_].index`就是wp3.index，值为k+1，$ wp3.index < cur_index $。

第2步，查找下一个最近的waypoint对应的索引，`next_routing_waypoint_index_`仍然为4，因为 $ adc_waypoint_.s < wp4.s $  

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

**2. 计算可驶入的临近通道**

