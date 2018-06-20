# 无人车轨迹规划机制

介绍完前面的模块和组件部分的具体技术，在最后一部分就综合介绍一下Planning规划模块的整体流程。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/planning_progress.png)

整个流程主要分两部分：

1. 参考线提供器独立子线程：

参考线提供的功能已经在[指引线提供器: ReferenceLineProvider](https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/reference_line_provider.md)中讲解完毕。他根据车辆当前的定位信息(来自Localization模块)和底盘信息(来自Chassis模块)，构建[规划与控制地图: Pnc Map](https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/pnc_map.md)。参考线提供器使用Pnc map完成其主要的功能：

- 功能1：初级参考线vector<RouteSegments>生成(未经过平滑的Segment段)
- 功能2：参考线平滑(得到光滑的参考线ReferenceLine)
- 功能3：平滑参考线的拼接得到最终成型的参考线(参考线复用、拼接、收缩等)

2. 信息存储与融合Frame类 && 规划器类EMPlanner

Frame类中存储了当前一次执行Planning模块的全部信息，每次调用Planning模块时都需要重置Frame里面的数据。Frame里面封装了ReferenceLineInfo，这是ReferenceLine的进一步加入了路径规划结果PathData，速度规划SpeedData，最优规划结果的开销cost，最优规划轨迹DiscretizedTrajectory等信息。ReferenceLineInfo中又封装了PathDecision和PathObstacle，这些来自于Perception和Prediction模块。

EM规划器使用ReferenceLineInfo中的参考线信息和障碍物未来时刻的轨迹信息，使用动态规划DP路径规划器得到一条未来(如40m)的最优规划路径PathData。通过得到的最优规划路径PathData(s-l序列)，使用二次规划QP速度规划器得到一条未来(如8s,150m)的最优速度规划路径SpeedData(t-s序列)。最终在ReferenceLine中将两者融合，得到一条最优的前进路线(t-s-l序列)，并且根绝是否成功完成路径规划(失败cost+20000)、是否成功完成速度规划(失败cost+20000)、是否有障碍物在规划路线上逼停无人车(一个障碍物cost+1000)，计算该最优规划路径的总体cost。

最后，在Frame类中可以对其拥有的所有ReferenceLineInfo进行选择，选择具有最小cost的全局最优规划路径。