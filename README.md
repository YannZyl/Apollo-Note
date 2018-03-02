# Apollo 2.0阅读笔记

本文档主要介绍Apollo 2.0软件平台结构，由于专业方向为视觉感知，以及未来从事方向为路径规划，因此阅读笔记主要包含对应的两部分。文档的目录结构为：

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
![img](https://githb.com/YannZyl/Apollo-Note/blob/master/images/Apollo_2_0_Software_Arch.png)


