# Apollo 2.0阅读笔记

本文档主要介绍Apollo 2.0软件平台结构，由于专业方向为视觉感知，以及未来从事方向为路径规划，因此阅读笔记主要包含对应的两部分。文档的目录结构为：

<!-- 添加隐藏链接，便于后续小结返回 -->
<a name="目录头"></a>
- [1. Apollo 2.0 软件结构](#总体软件结构)
	- [1.1 软件结构图](#软件结构图)
	- [1.2 感知模块: Perception](#感知模块)
	- [1.3 预测模块: Prediction](#预测模块)
	- [1.4 路由模块: Routing](#路由模块)
	- [1.5 控制模块: Control](#控制模块)
	- [1.6 控制器区域网络模块: CanBus](#控制器区域网络模块)
	- [1.7 高清地图模块: HD-Map](#高清地图模块)
	- [1.8 定位模块: Localization](#定位模块)
- [2. 感知模块 & 交通信号灯模块笔记](#感知模块详解)

## <a name="总体软件结构">1. Apollo 2.0总体软件结构</a>
本章主要介绍Apollo 2.0的软件结构，粗略的解释总体组成模块以及每个模块的功能，代码请参考([Apollo 2.0 Github](https://github.com/ApolloAuto/apollo)), 软件框架请参考([Apollo 2.0软件架构](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Apollo_2.0_Software_Architecture.md))。

### <a name="软件结构图">1.1 软件结构图</a>

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/Apollo_2_0_Software_Arch.png)

上图每个模块都以独立的ROS node运行，相互之间的消息传递依赖ROS的消息发布与订阅机制。消息订阅(sucsrcibe)等同于数据输入(data input)，而消息发布(publish)等同于数据输出(data output)。

### <a name="感知模块">1.2 感知模块</a>

感知模块主要功能是识别周围环境树木，人，路面，交通灯等信息，为后续路径规划，控制等做辅助。感知模块主要包含两个重要的子模块：

- 障碍物检测(3D onstacles perception).
- 交通信号灯检测(Traffic light perception).

感知模块输入来源于汽车物理感知设备，主要包含激光雷达点云数据，视觉摄像头数据。同时，交通信号灯检测也依赖于其他模块提供的数据，包含定位模块数据，高清地图数据(信号灯实时检测是不可行的或者不合理不必要的，原因在于太过耗费计算资源，因此需要定位与高精度地图模块提供相应的位置信息，指示感知模块在什么时间段与路面段做信号灯检测)，E.g.路口需要启动信号灯检测，中间路段只需要障碍物检测，不存在信号灯。

#### <a name="感知模块输入">感知模块输入数据</a>

	-
[返回目录](#目录头)
