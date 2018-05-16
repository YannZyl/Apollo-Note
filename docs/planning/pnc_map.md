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

route_indices_: [LaneSegment{id: 'lane 1', s: [100,110], index: [0,0,0]},    // RoadSegment 0, Passage 0, LaneSegment 0
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

从上述图中可以看到，该次查询的查询点waypoint一共两个，起点(红色星星)和终点(蓝色星星)。这一步需要对这两个waypoint进行处理，计算这两个waypoint分别在上述的那些LaneSegment中。判断是否在对应的LaneSegment中的代码为

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
routing_waypoint_index_: [LaneWaypoint{id: 'lane 1', s: 105, j: 0},      // start point, s=105, j=0(route_indices_[0]: RoadSegment 0,Passage 0,LaneSegment 0)
                          LaneWaypoint{id: 'lane 1', s: 185, j: 15},]    // end point, s=185, j=15(route_indices_[15]: RoadSegment 2,Passage 0,LaneSegment 2)
```

## 2. 短期路径段查询 (`PncMap::GetRouteSegments`)

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments)
```

这是pnc map最重要的功能，从函数参数分析，我们可以看到`GetRouteSegments`接受的参数最重要的是车辆的状态(包含车辆的位置，速度，偏航角，加速度等信息)，`backward_length`和`forward_length`是路径短生成过程中前向与后向修正距离，目前暂时不用考虑。而返回的是短期内(注意是短期内，长期调度不确定因素太多，无法实现)车辆的运动轨迹`route_segments`，这是std::list<RouteSegments>类型的，与上上节的`std::list<LineSequence>`类似，每个元素都是代表当前车辆的一种运动方案。

其实在执行`PncMap::GetRouteSegments`函数之前，需要确保执行