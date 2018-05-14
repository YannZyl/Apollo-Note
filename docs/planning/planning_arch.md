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