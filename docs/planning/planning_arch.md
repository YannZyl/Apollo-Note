# Apollo 2.0/2.5 规划模块代码层次结构说明

本文档将从代码层面讲解Apollo Planning规划模块的工作，规划模块是一个相对比较复杂的模块，他可以接受来自定位Localization、高精地图HD Map、交通灯模块Traffic Light，路由模块Routing以及预测模块Prediction的信息，并且综合上述信息，给出无人驾驶汽车在当前环境下短期内的路径规划。注意这个规划仅仅是短期的。

Apollo针对路径规划的思想是，利用Routing模块产生一条当前位置到终点的路径，这条路径是很理想的，不会考虑实际的路况与环境信息。Planning模块就是利用Routing产生的理想轨迹，结合当前路况下信号灯(Traffic Light模块得到)，障碍物短期内的运动轨迹(Prediction模块得到的LaneSequence及其概率)，来修正短期内无人驾驶汽车的路径(也就是决策)，通过Planning得到的一个修正的路径以后行驶到新的位置，可以根据这个新的位置再次去查询Routing模块，得到新的路由轨迹，以此往复，直到到达终点。

这一节将从代码入手，详细讲解Planning模块的各个细节，小结与Prediction模块类似，首先介绍Planning模块中的各个组件，分析每个组件的功能和算法，最后整体分析Planning模块的工作流程。

Planning模块组件分为以下部分：

1. 车辆状态提供器: VehicleStateProvider

这是最简单的组件，他负责将定位Localization与底盘Chassis信息进行融合，得到当前车辆的状态

2. 规划与控制地图: Planning and Control Map, pnc map

pnc map其实和高精地图hd map没有关系，后者是专门为规划与控制模块设计的库函数，在hd map层次之上，负责一些地图相关信息的处理。例如查询车辆可能的形式路径(list<RouteSegments>)

3. 基准线提供器: Reference Line Provider

基准线提供器其实就是路径的生成，对于一系列的RouteSegments进行平滑与拼接，最终得到无人车形式的基准线，也就是行驶路径。



## 额外补充

最后我们额外补充一下，Planning和Routing模块的核心数据结构。其中包括路由查询RoutingRequest与路由响应RoutingResponse。

```protobuf
/// file in apollo/modules/routing/proto/routing.proto
message LaneWaypoint {
  optional string id = 1;
  optional double s = 2;
  optional apollo.common.PointENU pose = 3;
}

message LaneSegment {
    optional string id = 1;
    optional double start_s = 2;
    optional double end_s = 3;
  }

message RoutingRequest {
  optional apollo.common.Header header = 1;
  // at least two points. The first is start point, the end is final point.
  // The routing must go through each point in waypoint.
  repeated LaneWaypoint waypoint = 2;
  repeated LaneSegment blacklisted_lane = 3;
  repeated string blacklisted_road = 4;
  optional bool broadcast = 5 [default = true];
}
```

从上述的protobuf定义内容可以看到:

1. RoutingRequest里面的waypoint(LaneWaypoint类型)是路径查询的核心，例如我要查询公交站A到学校B的一条路径，那么waypoint就是两个；如果我要查询公交站A到学校B的一条路径，并且我还要经过早餐店C，那么最终的waypoint就是三个。

2. LaneSegment和Prediction中的LaneSegment一样，定义了一条车道的[start_s, end_s]路段区间，使用repeated形式可以完整的离散化定义一条车道。

3. LaneWaypoint可以定义车道上的任意一点，包括所在车道id，所在车道的累计距离s，以及世界坐标系下的坐标pose。

```protobuf
message Measurement {
  optional double distance = 1;
}

enum ChangeLaneType {
    FORWARD = 0;
    LEFT = 1;
    RIGHT = 2;
};

message Passage {
   repeated LaneSegment segment = 1;
   optional bool can_exit = 2;
   optional ChangeLaneType change_lane_type = 3 [default = FORWARD];
}

message RoadSegment {
  optional string id = 1;
  repeated Passage passage = 2;
}

message RoutingResponse {
  optional apollo.common.Header header = 1;
  repeated RoadSegment road = 2;
  optional Measurement measurement = 3;
  optional RoutingRequest routing_request = 4;
  // the map version which is used to build road graph
  optional bytes map_version = 5;
  optional apollo.common.StatusPb status = 6;
}
```

以上是路径查询的返回/响应结果RoutingResponse，其中routing_request是对应发出的查询，measurement是行驶距离，最核心的内容就是road(repeated RoadSegment)，这是一条从起点公交站A到重点学校B，并且经过中间早餐店C的完整路径，由一段段离散化的RoadSegment组成。

RoadSegment类型包含了repeated Passage，这意味着，一个RoadSegment中包含了多个通道，每个通道可以理解为一条车道，一个道路段RoadSegment可以有多条并行向前行驶的车道。而Passage中每条车道可以有多个LaneSegment组成，意味着进一步划分成小的区间段，便于精细化调度。

最终的可视化结果如下图所示

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/routing_result.png)