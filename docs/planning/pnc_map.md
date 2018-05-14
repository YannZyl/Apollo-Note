# 规划与控制地图: pnc map

pnc map其实和高精地图hd map没有关系，后者是专门为规划与控制模块设计的库函数，在hd map层次之上，负责一些地图相关信息的处理。例如查询车辆可能的形式路径(list<RouteSegments>)，这是pnc map最重要的功能。

pnc map目前被封装在基准线提供器ReferenceLineProvider中，但是由于其功能比较集中，我们单独将他拿出来讲解。下面我们将从代码入手，看看pnc map实现的功能有哪些。

## 1. 更新路由信息 (UpdateRoutingResponse)



## 2. 短期路径段查询 (PncMap::GetRouteSegments)

```c++
/// file in apollo/modules/map/pnc_map/pnc_map.cc
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments)
```

这是pnc map最重要的功能，从函数参数分析，我们可以看到`GetRouteSegments`接受的参数最重要的是车辆的状态(包含车辆的位置，速度，偏航角，加速度等信息)，`backward_length`和`forward_length`是路径短生成过程中前向与后向修正距离，目前暂时不用考虑。而返回的是短期内(注意是短期内，长期调度不确定因素太多，无法实现)车辆的运动轨迹`route_segments`，这是std::list<RouteSegments>类型的，与上上节的`std::list<LineSequence>`类似，每个元素都是代表当前车辆的一种运动方案。

其实在执行`PncMap::GetRouteSegments`函数之前，需要确保执行