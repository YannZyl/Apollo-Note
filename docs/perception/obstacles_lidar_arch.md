# 激光雷达感知模块: LiDAR Obstacles Perception

本文档结合代码详细地解释感知模块中障碍物感知的流程与功能，也可以官方参考文档([Apollo 2.0 Obstacles Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md)

## 硬件简介

Apollo2.0 中激光雷达感知模块使用的是Velodyne 64线激光雷达。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/lidar_pic.png)

**Key Features:**

- 64 Channels
- 120m range
- 2.2 Million Points per Second
- 360° Horizontal FOV
- 26.9° Vertical FOV
- 0.08° angular resolution (azimuth)
- <2cm accuracy
- ~0.4° Vertical Resolution
- User selectable frame rate
- Rugged Design

Webpage for Velodyne HDL-64E S3:
[http://velodynelidar.com/hdl-64e.html](http://velodynelidar.com/hdl-64e.html)

## 功能介绍

任何时刻，激光雷达将会对汽车周围360°范围内捕获三维点云，然后对点云做后续处理。感知模块主要的工作由4块，分别为：

**(1) 高精地图ROI过滤器**

该过程处理在ROI之外的激光雷达点，去除背景对象，如路边建筑物和树木等，剩余的点云留待后续处理(这里的ROI区域是指汽车周围路面区域，路面外区域不用理会)。再具体的处理过程中主要步骤为：

-  数据接收与存储

接收来自激光雷达的原始点云数据，设备id，时间戳ts等信息，并将其信息存入自定义类中。

- 点云坐标系转换

原始点云参考系是Lidar坐标系，在实际处理的过程中需要考虑到方向，所以需要将点云转换到局部坐标系(Lidar ENU坐标系，也叫作东北天坐标系)

- ROI LUT构造与点查询

高精地图查询到的路面轮廓多边形，通过构造LUT，将路面轮廓多边形映射到一个二维表中。这样做目的是当给定一个新的点云坐标，就可以通过转换成二维的映射表坐标，查询以后就可以知道该点是否在路面中。

- 点云筛选

通过上一步构造完成的LUT二维映射表，可以对局部坐标系中的点云就行筛选。每个点云根据现在的坐标去计算LUT二维表格中的坐标，然后查询是否在路面多边形区域内即可。最终筛选出路面ROI区域内的点云做下一步处理。

**(2) 基于卷积神经网络分割**

高精地图 ROI过滤之后，Apollo得到已过滤、只包含属于ROI内的点云，大部分背景障碍物，如路侧的建筑物、树木等均被移除，ROI内的点云被传递到分割模块。分割模块检测和划分前景障碍物，例如汽车，卡车，自行车和行人。具体的处理过程包涵一下步骤：

- 通道特征提取

由(1)可以得到筛选过后的路面区域点云，然后对这个路面区域构建一个512x512的2D俯视投影网格(原本ROI内的点云是车辆70米范围内，现在投影到512个网格中)，每个网格内包含若干点云。最终对网格进行8个变量的统计，得到512x512x8的特征图、

- 基于卷积神经网络的障碍物预测

对于上述得到的1x512x512x8大小特征图进行基于神经网络的分割，输出大小为1x512x512x12，表示每个网格点是否是物体、物体偏移中心、置信度概率、物体类别等信息。

- 障碍物聚类

对于卷积神经网络输出的特征图进行障碍物聚类，聚类依据是上述的物体偏移中心，将有可能是一类的网络进行汇聚，最终得到各类的物体。

- 后期处理

在后期处理中，Apollo首先对所涉及的单元格的积极性和物体高度值，平均计算每个候选群体的检测置信度分数和物体高度。然后，Apollo去除相对于预测物体高度太高的点，并收集每个候选集中的有效单元格的点。 最后，Apollo删除具有非常低的可信度分数或小点数的候选聚类，以输出最终的障碍物集/分段。

**(3) MinBox障碍物边框构建**

上一步CNN分割与后期处理，可以得到lidar一定区域内的障碍物集群。接下去我们将对这些障碍物集群建立其标定框。标定框的作用除了标识物体，还有一个作用就是标记障碍物的长length，宽width，高height。其中规定长length大于宽width，障碍物方向就是长的方向direction。主要流程为

- MinBox构建--计算2DXY平面投影

在(2)中通过聚类可以得到每个类的点云集合，该步骤就是对点云进行投影到2维XY平面，然后计算点云的凸包，即多边形。

- MinBox构建--边框构建

对上述得到的多边形做一个边框构建(尽可能用一个面积小的矩阵去覆盖多边形)


**(4) HM对象跟踪**

该过程其实对物体进行跟踪和运动估计。主要步骤为：

- 预处理

主要做的工作是将原始点云从lidar坐标系转换到局部坐标系。然后开始是将(3)中检测到的物体加入跟踪列表

- 卡尔曼滤波，跟踪物体对象(卡尔曼滤波阶段1： Predict)

对上面已在跟踪列表中的物体进行运动估计，估计出当前时刻的位置与速度。

- 匈牙利算法比配，关联检测物体和跟踪物体

对已在跟踪列表中的物体和当前时刻CNN检测到的物体做关联匹配。例如跟踪列表中A物体对应当前时刻CNN检测到的B物体，跟踪列表中B物体找不到CNN检测到的物体与之对应等等。

- 卡尔曼滤波，更新跟踪物体位置与速度信息(卡尔曼滤波阶段2：Update阶段)

若已跟踪物体--CNN检测新物体匹配成功，则使用卡尔曼滤波更新当前物体的最优位置；如果跟踪列表物体失配，超时可以移出跟踪列表；如果新物体失配，则加入跟踪列表。


## 数据结构与流程图**

首先对激光雷达障碍物感知模块使用的数据结构做一个简单地分析，其核心数据结构为Object和SensorObject，可以分析这两类数据结构，如下表:

| 名称 | 备注 |
| ---- | ---- |
| pcl_util::PointCloudPtr cloud | PCL点云数据，原始sensor_msgs::PointCloud2需经过转化 |
| PolygonDType polygon | 物体凸包/角点，PolygonDType数据类型为pcl_util::PointCloud |
| Eigen::Vector3d direction | 物体主方向，右手坐标系，默认X轴方向，即Eigen::Vector3d(1, 0, 0) |
| double theta | 物体偏航角，默认0.0，即无偏向，向正前方 |
| Eigen::Vector3d center | 物体中心，默认自身坐标系中心，即Eigen::Vector3d::Zero() |
| double length, width, height | 物体有向标定框尺寸，length为朝向主方向轴的大小 |
| std::vector<float> shape_features | 形状特征，用于物体跟踪 |
| float score | 前景概率 |
| ObjectType type | 物体类型，行人、自信车、车辆、未知等 |
| Eigen::Vector3d velocity | 物体运动速度，用于物体跟踪 |
| double tracking_time | 跟踪时间 |
| double latest_tracked_time | 最近一次跟踪时间戳 |
| Eigen::Vector3d anchor_point | 稳定的锚点，比如说重心 |

SensorObject继承了Object类，主要数据结构如下：

| 名称 | 备注 |
| ---- | ---- |
| SensorType sensor_type | 感知数据来源，64/16线激光雷达、雷达、摄像头、未知等 |
| std::string sensor_id | 设备id |
| double timestamp | 时间戳 |
| SeqId seq_num | 数据序列，每次回调seq_num自增1 |
| std::vector<ObjectPtr> objects | 重点，感知数据中的多个物体数据 |
| Eigen::Matrix4d sensor2world_pose | 感知设备自身坐标系相对世界坐标系的系数矩阵 |
| LaneObjectsPtr lane_objects | 车道线信息 |

从上面两张表格基本可以看出激光雷达模块需要做的工作，Object类很完善地存储了单个物体的一些静态与动态信息，静态信息包括物体点云数据，凸包及标定框信息，坐标系坐标/偏向/尺寸信息，物体的类别信息等；动态信息主要是在跟踪过程中物体速度，中心变化等信息。而在一次激光雷达扫描过程中，得到的信息中包含了多类物体，因此SensorObject类中存储了Object的一个向量，刻画了本次扫描分离的物体集合，同时包含了一些标记信息如感知来源、设备id、时间戳、序列等，最后还包含了本次激光雷达扫描到的车道线信息。SensorObject类包含了每次激光雷达扫描得到的所有信息。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/perception_obstacles_lidar.png)

上图展示了激光雷达处理的4个模块，分别为高精地图ROI过滤器、基于卷积神经网络的障碍物分割、MinBox 障碍物边框构建与HM对象跟踪。接下去将一层层解剖代码，分析各个模块的流程的方法。