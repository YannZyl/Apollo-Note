# Apollo 2.0 感知模块：障碍物感知阅读笔记

本文档结合代码详细地解释感知模块中障碍物感知的流程与功能，也可以官方参考文档([Apollo 2.0 Obstacles Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md))

***初次接触三维重建，如有笔误，欢迎指正!***

## 1. 障碍物感知: 3D Obstacles Perception

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/perception_obstacles_framework.png)

上图为子节点之间的关系与数据流动，障碍物感知共存在三个子节点(线程)，分别为：

- 激光雷达处理子节点 LidarProcessSubnode
- 雷达处理子节点 RadarProcessSubnode
- 融合子节点 FusionSubnode

每个子节点的输入数据与输出数据在边上标出。

**(1) 激光雷达子节点**

LidarProcessSubnode::OnPointCloud以ROS消息订阅与发布机制触发回调函数，处理结果保存在LidarObjectData共享数据容器中。主要解决的问题有：

- 高精地图ROI过滤器(HDMap ROI Filter)
- 基于卷积神经网络分割(CNN Segmentation)
- MinBox 障碍物边框构建(MinBox Builder)
- HM对象跟踪(HM Object Tracker)

**(2) 雷达子节点**

RadarProcessSubnode::OnRadar同样以ROS消息订阅与发布机制触发回调函数，处理结果保存在RadarObjectData共享数据容器中。

**(3) 融合子节点**

FusionSubnode::ProcEvents以自定义ProcEvents+EventManeger消息处理机制，从LidarObjectData和RadarObjectData共享数据容器中提取数据，融合并存储在FusionObjectData共享容器中。

## 2. 激光雷达感知 Perception: Lidar Obstacles Perception

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

### 2.1 高精地图ROI过滤器

高精地图ROI过滤器是回调的第一个过程，该过程处理在ROI之外的激光雷达点，去除背景对象，如路边建筑物和树木等，剩余的点云留待后续处理。从最开始的模块框架图可以看到，LidarProcessSubnode子节点接受的输入数据类型是ROS原始的点云数据类型，sensor_msgs::PointCloud2，简单地看一下这个数据结构，也可以参考官方文档[PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)

```c++
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```

sensor_msgs::PointCloud2与第一个版本sensor_msgs::PointCloud有一些区别，支持任意的N维数据，并且支持任意的基础数据类型，也支持密集存储。从上面的说明可以看到，PointCloud2可以支持2D数据结构，每个点N维，每行X个通道，共H列，可以存储图像信息等。fields存储了各个通道的名称，例如x,y,z,r,g,b等，在这里我们使用的PointCloud2类型每个通道只需要xyz三维即可，表示坐标系中的位置。

还有一个细节，激光雷达获取的点云是ROS原始的sensor_msgs::PointClouds类型，而实际处理过程中使用的更多的是PCL库的pcl::PointCloud<T>类型，需要在代码中做一个转换，使用pcl_conversions的pcl::fromROSMsg和pcl::toROSMsg函数即可方便的实现相互转换。

#### 2.1.1 数据转换与ROI生成

在进行高精地图ROI过滤的过程中，第一步是接收来自激光雷达的原始点云数据，设备id，时间戳ts等信息，并将其信息存入上述的SensorObject类中。存储过程中代码中值得关注的两个点分别是传感器到世界坐标系的转换矩阵velodyne_trans以及sensor_msgs::PointCloud2到PCL::PointCloud的转换。

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  ...
  /// get velodyne2world transfrom
  std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
    ...
  }
  out_sensor_objects->sensor2world_pose = *velodyne_trans;

  PointCloudPtr point_cloud(new PointCloud);
  TransPointCloudToPCL(message, &point_cloud);
}
```

当获得原始的点云数据并转换成PCL格式以后，下一步需要从点云中检索ROI区域，这些ROI区域包含路面与路口的驾驶区域。

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// get velodyne2world transfrom
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
  	...
  }
  /// call hdmap to get ROI
  HdmapStructPtr hdmap = nullptr;
  if (hdmap_input_) {
    PointD velodyne_pose = {0.0, 0.0, 0.0, 0};  // (0,0,0)
    Affine3d temp_trans(*velodyne_trans);
    PointD velodyne_pose_world = pcl::transformPoint(velodyne_pose, temp_trans);
    hdmap.reset(new HdmapStruct);
    hdmap_input_->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap);
    PERF_BLOCK_END("lidar_get_roi_from_hdmap");
  }
}
```

路面与路口的驾驶区域需要查询高精地图来完成，该阶段首先使用tf进行坐标系的转换(lidar坐标系到世界坐标系的变换矩阵)，配合velodyne_pose计算得到velodyne_pose_world(lidar在世界坐标系中的坐标)，坐标系具体情况请参考[Apollo坐标系研究](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_coordinate.md)。真正获取ROI使用的是GetROI函数。具体的路口，车道存储形式请参考[高精地图模块](https://github.com/YannZyl/Apollo-Note/blob/master/docs/hdmap.md)

简单分析一下GetVelodyneTrans函数，这个函数功能是产生lidar坐标系到世界坐标系的变换矩阵。实现过程我们可以先简要的看一下在做功能分析：

```c++
/// file in apollo/modules/perception/onboard/transform_input.cc
bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans) {
  ...
  // Step1: lidar refer to novatel(GPS/IMU)
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp);
  } 
  Eigen::Affine3d affine_lidar_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_lidar_3d);
  Eigen::Matrix4d lidar2novatel_trans = affine_lidar_3d.matrix();

  // Step2 notavel(GPS/IMU) refer to world coordinate system
  try {
    transform_stamped = tf2_buffer.lookupTransform(FLAGS_localization_tf2_frame_id, FLAGS_localization_tf2_child_frame_id, query_stamp);
  } 
  Eigen::Affine3d affine_localization_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_localization_3d);
  Eigen::Matrix4d novatel2world_trans = affine_localization_3d.matrix();

  *trans = novatel2world_trans * lidar2novatel_trans;
}
```

点云数据由lidar获取，所以数据都是以激光雷达lidar参考系作为标准参考系，在查询高精地图的时候需要世界坐标系坐标。因此获取变换矩阵分为两步，第一步获取激光雷达lidar坐标系到惯测单元IMU坐标系的变换矩阵；第二步，获取惯测单元IMU坐标系到世界坐标系变换矩阵。从上述的代码中我们明显可以看到有两部分相似度很高的代码组成:

- 计算仿射变换矩阵lidar2novatel_trans，激光雷达lidar坐标系到惯测IMU坐标系(车辆坐标系)变换矩阵。这个矩阵虽然通过ROS的tf模块调用lookupTransform函数计算完成，但是实际是外参决定，在运行过程中保持不变。
```c++
/// file in apollo/modules/localization/msf/params/velodyne_params/velodyne64_novatel_extrinsics_example.yaml
child_frame_id: velodyne64
transform:
  translation:
    x: -0.0581372003122598
    y: 1.459274166013735
    z: 1.24965
  rotation:
    x: 0.02748694630673456
    y: -0.03223034579615043
    z: 0.7065742186090662
    w: 0.706369978261802
header:
  seq: 0
  stamp:
    secs: 1512689414
    nsecs: 0
  frame_id: novatel
```

- 计算仿射变换矩阵novatel2world_trans，惯测单元IMU坐标系(车辆坐标系)相对于世界坐标系的仿射变换矩阵。

- 计算仿射变换矩阵lidar2world_trans，最终两个矩阵相乘得到激光雷达lidar坐标系到世界坐标系的变换矩阵。

#### 2.1.2 坐标变换

>Apollo官方文档引用：对于(高精地图ROI)过滤器来说，高精地图数据接口被定义为一系列多边形集合，每个集合由世界坐标系点组成有序点集。高精地图ROI点查询需要点云和多边形处在相同的坐标系，为此，Apollo将输入点云和HDMap多边形变换为来自激光雷达传感器位置的地方坐标系。

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// get velodyne2world transfrom
  ...
  /// call hdmap to get ROI
  ...
  /// call roi_filter
  PointCloudPtr roi_cloud(new PointCloud);
  if (roi_filter_ != nullptr) {
    PointIndicesPtr roi_indices(new PointIndices);
    ROIFilterOptions roi_filter_options;
    roi_filter_options.velodyne_trans = velodyne_trans;
    roi_filter_options.hdmap = hdmap;
    if (roi_filter_->Filter(point_cloud, roi_filter_options, roi_indices.get())) {
      pcl::copyPointCloud(*point_cloud, *roi_indices, *roi_cloud);
      roi_indices_ = roi_indices;
    } else {
      ...
    }
  }
}
```

坐标转换和接下去ROI LUT构造与点查询的步骤都在HdmapROIFilter这个类里面完成。

这个阶段使用到的变换矩阵就是以上的lidar2world_trans矩阵。看了官方说明，并配合具体的代码，可能会存在一些疑惑。这里给出一些变换的研究心得。坐标变换的实现是在HdmapROIFilter::Filter函数中完成。具体的变换过程如下：

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc
bool HdmapROIFilter::Filter(const pcl_util::PointCloudPtr& cloud,
                            const ROIFilterOptions& roi_filter_options,
                            pcl_util::PointIndices* roi_indices) {
  Eigen::Affine3d temp_trans(*(roi_filter_options.velodyne_trans));
  std::vector<PolygonDType> polygons;
  MergeHdmapStructToPolygons(roi_filter_options.hdmap, &polygons);
  ...
  // Transform polygon and point to local coordinates
  pcl_util::PointCloudPtr cloud_local(new pcl_util::PointCloud);
  std::vector<PolygonType> polygons_local;
  TransformFrame(cloud, temp_trans, polygons, &polygons_local, cloud_local);
  ...
}

void HdmapROIFilter::TransformFrame(
    const pcl_util::PointCloudConstPtr& cloud, const Eigen::Affine3d& vel_pose,
    const std::vector<PolygonDType>& polygons_world,
    std::vector<PolygonType>* polygons_local,
    pcl_util::PointCloudPtr cloud_local) {
  ...
}

```

注意点1: 上面代码中MergeHdmapStructToPolygons函数负责把路口和路面的点云并入到多边形集合polygons，这里roi_filter_options里面的数据都是经过高精地图查询得到的路口和路面信息，是基于世界坐标系的，所以结果合并后polygons也是世界坐标系的数据。

注意点2: 输入的cloud是基于lidar坐标系的点云数据，而下面代码还需要转换成cloud_local、polygons_local，按照注释解释是局部坐标系，那么这个局部坐标系到底是什么坐标系？如果看得懂TransformFrame函数，可以不难发现：**这个所谓"local coordinate system"，其实跟lidar坐标系很相近，他表示以lidar为原点的ENU坐标系**，这个坐标系是以X(东)-Y(北)-Z(天)为坐标轴的二维投影坐标系。在TransformFrame函数中，

```c++
  Eigen::Vector3d vel_location = vel_pose.translation();
  Eigen::Matrix3d vel_rot = vel_pose.linear();
  Eigen::Vector3d x_axis = vel_rot.row(0);
  Eigen::Vector3d y_axis = vel_rot.row(1);
```

vel_location是lidar坐标系相对世界坐标系的平移成分，vel_rot则是lidar坐标系相对世界坐标系的旋转矩阵。那么从lidar坐标系到世界坐标系的坐标变换其实很简单，假设在lidar坐标系中有一个坐标点P(x1,y1,z1)，那么该点在世界坐标系下的坐标P_hat为: P_hat = vel_rot * P + vel_location. 了解了这个变换，接下来观察cloud和polygons的变换代码：

```c++
  polygons_local->resize(polygons_world.size());
  for (size_t i = 0; i < polygons_local->size(); ++i) {
    const auto& polygon_world = polygons_world[i];
    auto& polygon_local = polygons_local->at(i);
    polygon_local.resize(polygon_world.size());
    for (size_t j = 0; j < polygon_local.size(); ++j) {
      polygon_local[j].x = polygon_world[j].x - vel_location.x();
      polygon_local[j].y = polygon_world[j].y - vel_location.y();
    }
  }
```

开始的时候也是很奇怪，为什么最后变换的形式是P_local = P_world - translation. 后来经过研究猜测(有待后续深入阅读证实)路口和路面多边形信息只经过平移达到新的局部ENU坐标系，可以推测其实世界坐标系也是ENU坐标系，所以两个坐标系之间没有旋转成分，直接移除平移就可以从世界坐标系变换到局部ENU坐标系。


**注意其实变换前后polygons_world和polygons_local的高度z是变化的，但是由于polygons被用来做2D投影网格LUT构建，所以对高度z这一维度并不关心，这些就不做z的变化；另外强度i始终不会变化。**

P_world = vel_rot * P_local + translation 
当vel_rot旋转成分为0时: P_local = P_world - translation

```c++
  cloud_local->resize(cloud->size());
  for (size_t i = 0; i < cloud_local->size(); ++i) {
    const auto& pt = cloud->points[i];
    auto& local_pt = cloud_local->points[i];
    Eigen::Vector3d e_pt(pt.x, pt.y, pt.z);
    local_pt.x = x_axis.dot(e_pt);
    local_pt.y = y_axis.dot(e_pt);
  }
```

上述cloud变换代码再次验证了世界坐标系也是ENU坐标系类型的说法，局部ENU坐标系和lidar坐标系共原点但存在一个旋转角度，lidar坐标系到世界坐标系变换的旋转矩阵是vel_rot，那么到局部ENU坐标系的旋转矩阵也应该是vel_rot；其次共原点说明两个坐标系的平移矩阵其实是0。最终变换到局部ENU坐标系的公式就是：P_hat = vel_rot * P + 0。也就是上面看到的公式。

**注意这里为什么只进行x和y坐标的转换，而没有进行高度z和强度i的转换？首先强度在任何坐标系下都是一样的，所以不用进行转换。其次cloud从lidar坐标系转换到以lidar为原点的ENU局部坐标系cloud_local，只有旋转没有平移成分，因为原点一样，xy轴构成的平面是同一个平面，所以高度是一样的，不需要变换z。**

另外补充一点猜测世界坐标系也是ENU类型坐标系的证据：

```c++
/// file in apollo/modules/perception/traffic_light/onboard/hdmap_input.cc
bool HDMapInput::GetSignals(const Eigen::Matrix4d &pointd, std::vector<apollo::hdmap::Signal> *signals) {
  auto hdmap = HDMapUtil::BaseMapPtr();
  std::vector<hdmap::SignalInfoConstPtr> forward_signals;
  apollo::common::PointENU point;
  point.set_x(pointd(0, 3));
  point.set_y(pointd(1, 3));
  point.set_z(pointd(2, 3));
  int result = hdmap->GetForwardNearestSignalsOnLane( point, FLAGS_query_signal_range, &forward_signals);
  ...
}
```

在交通信号灯感知模块中，有一个功能是根据当前车的位置，去查询高精地图，获取该位置处的信号灯信息。上面的代码中GetSignals函数实现了这个功能，输入pointd就是我们上述使用到的lidar2world_trans变换矩阵，代码中point是PointENU类型的点，并设置为变换矩阵的平移成分去世界坐标系查询(实际是根据世界坐标系下面车坐标查询高精地图)。因此可以初步判断，世界坐标系是ENU类型坐标系。

最后有个问题，转换到局部ENU坐标系有什么作用，以及效果。简单地说一下ENU坐标系是带有方向性的，所以转换到该坐标系下的polygeons_world和cloud_local其实是有东南西北的意思，从Apollo官方文档可以看到效果如下，得到当前位置下ROI区域内外的一个矩形框，这时候路面和路口就有东西南北走向的意义。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/roi_lookup_table.png)

**思考：为什么不在车辆坐标系(IMU坐标系)或者lidar坐标系下面进行接下去的分割操作？(这两个坐标系的方向都参考车头的方向，是没有东南西北这些地理位置信息的)。**

#### 2.1.3 ROI LUT构造与点查询

>Apollo官方文档引用：Apollo采用网格显示查找表（LUT），如上图将ROI量化为俯视图2D网格，以此决定输入点是在ROI之内还是之外。如图1所示，该LUT覆盖了一个矩形区域，该区域位于高精地图边界上方，以普通视图周围的预定义空间范围为边界。它代表了与ROI关联网格的每个单元格（如用1/0表示在ROI的内部/外部）。为了计算效率，Apollo使用扫描线算法和位图编码来构建ROI LUT。

>如上图，蓝色线条标出了高精地图ROI的边界，包含路表与路口。红色加粗点表示对应于激光雷达传感器位置的地方坐标系原始位置。2D网格由8\*8个绿色正方形组成，在ROI中的单元格，为蓝色填充的正方形，而之外的是黄色填充的正方形。

>基于ROI LUT，查询每个输入点的关系使用两步认证。对于点查询过程，Apollo数据编译输出如下:
>1. 检查点在ROI LUT矩形区域之内还是之外。
>2. 查询LUT中相对于ROI关联点的相应单元格。
>3. 收集属于ROI的所有点，并输出其相对于输入点云的索引。

以上是Apollo官方文档描述的原话，确实看起来让人似懂非懂，能了解大概的作用流程，但是又对具体的细节毫无掌握。本节我们将从代码解剖上来具体了解所谓的"ROI LUT构造与点查询"。**简单地说这个环节的作用就是：将上述转换到局部ENU坐标系下的路面与路口ROI的二维信息映射到一张2D网格图中，网格图中0表示非路口路面区域，1表示路口遇路面区域，最后判断点云cloud中哪些点在路面ROI内(方便做行人，车辆分割)**

先解释一下一些基本信息概念：

1. 从上面映射到局部ENU坐标系的路口&&路面点云信息，这些点云并不是覆盖所有路口路面区域的，而是路口与路面的凸包，也就是角点。polygons_local里面其实存储了路面与边界线的角点/轮廓信息。cloud_local是所有原始点云映射到ENU局部坐标系过后的信息。
2. 需要将原先路口与路面等多边形的角点存储模式(节省内存)转换到填充模式，用到最常用的算法就是扫描线算法，具体请参考GitHub首页链接。
3. 这个填充模式需要用到一个填充的2d网格，网格的大小范围、网格之间的间距(扫描线之间的大小)等信息，由外部文件定义如下，而最后如何2D网格节省存储开销，就用到了bitmap数据结构：

用户定义的参数可在配置文件`modules/perception/model/hdmap_roi_filter.config`中设置，HDMap ROI Filter 参数使用参考如下表格：

|参数名称      |使用                                                                          |默认     |
|------------------- |----------------------------------------------------------------------- |------------|
|range           | 基于LiDAR传感器点的2D网格ROI LUT的图层范围(说白了就是以lidar为中心，ENU坐标系下，东西方向x的统计范围，南北方向y的统计范围)，如(-70, 70)*(-70, 70) |70.0 米 |
|cell_size       | 用于量化2D网格的单元格的大小，主方向上两条网格线之间的距离                 |0.25 米  |
|extend_dist     | 从多边形边界扩展ROI的距离。                 |0.0 米   |

现在明白了ROI LUT的作用，接下去我们将从代码一步步了解Apollo采用的方案。上面小节讲到使用TransformFrame函数完成原始点云到局部ENU坐标系点云的转换以后得到了cloud_local映射原点云，polygons_local映射路面与路口多边形信息。接下来做的工作就是根据polygons_local构建ROI LUT。构建的过程在FilterWithPolygonMask函数中开启。

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc
bool HdmapROIFilter::Filter(const pcl_util::PointCloudPtr& cloud,
                            const ROIFilterOptions& roi_filter_options,
                            pcl_util::PointIndices* roi_indices) {
  // 1. Transform polygon and point to local coordinates
  TransformFrame(cloud, temp_trans, polygons, &polygons_local, cloud_local);

  return FilterWithPolygonMask(cloud_local, polygons_local, roi_indices);
}

bool HdmapROIFilter::FilterWithPolygonMask(
    const pcl_util::PointCloudPtr& cloud,
    const std::vector<PolygonType>& map_polygons,
    pcl_util::PointIndices* roi_indices) {
  // 2. Get Major Direction as X direction and convert map_polygons to raw
  // polygons
  std::vector<PolygonScanConverter::Polygon> raw_polygons(map_polygons.size());
  MajorDirection major_dir = GetMajorDirection(map_polygons, &raw_polygons);

  // 3. Convert polygons into roi grids in bitmap
  Eigen::Vector2d min_p(-range_, -range_);
  Eigen::Vector2d max_p(range_, range_);
  Eigen::Vector2d grid_size(cell_size_, cell_size_);
  Bitmap2D bitmap(min_p, max_p, grid_size, major_dir);
  bitmap.BuildMap();

  DrawPolygonInBitmap(raw_polygons, extend_dist_, &bitmap);

  // 4. Check each point whether is in roi grids in bitmap
  return Bitmap2dFilter(cloud, bitmap, roi_indices);
}
```

可以看到构建的过程总共分为3部分(其实2,3就能完成构建；4只是check，分类cloud_local中在路面ROI内和外的点云)，接下来逐个分析流程，第2步求polygons_local的主方向比较简单，只要计算多边形点云集合中，x/东西方向与y/南北方向最大值与最小值的差，差越大跨度越大。选择跨度小的方向作为主方向。(这部分代码比较简单，所以不再贴出来)。

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.cc
Bitmap2D::Bitmap2D(const Eigen::Vector2d& min_p, const Eigen::Vector2d& max_p, const Eigen::Vector2d& grid_size, DirectionMajor dir_major) {
  dir_major_ = dir_major;
  op_dir_major_ = opposite_direction(dir_major);
  min_p_ = min_p;
  max_p_ = max_p;
  grid_size_ = grid_size;
}

void Bitmap2D::BuildMap() {
  Eigen::Matrix<size_t, 2, 1> dims = ((max_p_ - min_p_).array() / grid_size_.array()).cast<size_t>();
  size_t rows = dims[dir_major_];
  size_t cols = (dims[op_dir_major_] >> 6) + 1;
  bitmap_ = std::vector<std::vector<uint64_t>>(rows, std::vector<uint64_t>(cols, 0));
}
```

在构建bitmap的过程中，上面的代码对应bitmap初始化，主要做的工作就是设置参数，设置min_p_，max_p_，这两个参数就是对应外部文件的range参数，默认70米。grid_size是网格线之间的距离，默认0.25m。最后可以看到申请了一个bitmap_的2D向量，这个向量有rows行能理解，为什么列cols需要除以2^6次(64)呢？答案是因为他是bitmap，一个unsigned64有64位，每一位bit存储一个网格0/1值，起到节省开销作用。所以bitmap_的大小如果是mxn，那么他可以存储mx64n网格大小的数据。

难点是DrawPolygonInBitmap函数，这也是主要工作完成的函数，在这个函数里面，会真正的构建ROI LUT。我们接着分析他的实现代码：

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_mask.cc
void DrawPolygonInBitmap(const typename PolygonScanConverter::Polygon& polygon, const double extend_dist, Bitmap2D* bitmap) {
  ...
  // 1. Get valid x range
  Interval valid_x_range;
  GetValidXRange(polygon, *bitmap, major_dir, major_dir_grid_size, &valid_x_range);

  // 2. Convert polygon to scan intervals(Most important)
  std::vector<std::vector<Interval>> scans_intervals;
  PolygonScanConverter polygon_scan_converter;
  polygon_scan_converter.Init(major_dir, valid_x_range, polygon, major_dir_grid_size);
  polygon_scan_converter.ConvertScans(&scans_intervals);

  // 3. Draw grids in bitmap based on scan intervals
  double x = valid_x_range.first;
  for (size_t i = 0; i < scans_intervals.size(); x += major_dir_grid_size, ++i) {
    for (const auto& scan_interval : scans_intervals[i]) {
      ...
      bitmap->Set(x, valid_y_range.first, valid_y_range.second);
    }
  }
}

/// file in modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.cc
void PolygonScanConverter::ConvertScans( std::vector<std::vector<Interval>> *scans_intervals) {
  scans_intervals->resize(scans_size_);

  DisturbPolygon();
  ConvertPolygonToSegments();
  BuildEdgeTable();

  // convert polygon to filled table
  ...
}
```

从上面段代码我们来分析一下DrawPolygonInBitmap的流程，首先GetValidXRange函数其实是获取主方向上，最大值和最小值，跟MajorDirection几乎无差异，这里不在分析。有基础的应该容易看懂。接着第二部分是将路面与边界线的多边形有序集合转换成Edge信息，如何转换？

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/perception_obstacles_roi_lut.png)

需要了解一个前提，polygons_local里面存储了路面和路口的多边形信息，对于每个多边形polygon，它里面存储了N个点云(主要使用其xy坐标)，物体的轮廓点是有序的排列的，所以每两个相邻的点可以构建一条边，最后得到一个封闭的轮廓(如上图A所示)。所以每个polygon里面存的形式为：

polygon.data: {P1.x, P1.y, P2.x, P2.y ,..., P7.x, P7.y, P8.x, P8.y}

那么接下去我们就需要将这个角点存储形式彻底转换成填充(点阵)形式，转换的步骤是:

1. 如上图B，DisturbPolygon函数是将polygon里面的坐标，过于靠近网格线(坐标差值在epsion以内)的点稍微推离网格线，原因便于步骤3中处理网格线附近的边，经过推离以后，网格线附近的边要么是不穿过网格线，要么就明显的横穿网络，减少那种差一点就横穿网格线的边，降低判断逻辑。代码中可以很明确的看到这个目的：

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.cc
void PolygonScanConverter::DisturbPolygon() {
  for (auto &pt : polygon_) {
    double &x = pt[major_dir_];
    double d_x = (x - min_x_) / step_;   
    int int_d_x = std::round(d_x);          // get grid line id
    double delta_x = d_x - int_d_x;         // compute distance between point and grid line in major direction 
    if (std::abs(delta_x) < kEpsilon) {
      if (delta_x > 0) {                    // point in the right side of the grid line ==> right move fix(E.g. P2, P4)
        x = (int_d_x + kEpsilon) * step_ + min_x_; 
      } 
      else {                                // point in the left side of the grid line ==> left move fix(E.g. P1, P6)
        x = (int_d_x - kEpsilon) * step_ + min_x_; 
      }
    }
  }
}
```

2. 如上图C，ConvertPolygonToSegments函数将多边形相邻的两个点保存出他们的边segment_(以pair<point,point>形式存储)，同时计算得到这条边的斜率slope_，那么最后计算得到的segment_和slope_里面保存的信息方式为：

segment_: {<P1,P2>, <P2,P3>, <P3,P4>, <P5,P4>, <P6,P5>, <P7,P6>, <P7,P8>};

slope_: {0.2, 0.8, -4, 0.7, 0.4, -1, 10, 8}

这里segment_里面每个元素两个点存储的顺序依赖其主方向上的坐标值，永远是后面一个点的坐标比前面一个点的坐标大。segment_[n].first.x < segment_[n].second.x，在代码中也相对比较简单。

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.cc
void PolygonScanConverter::ConvertPolygonToSegments() {
  for (size_t i = 0; i < vertices_num; ++i) {
    const Point &cur_vertex = polygon_[i];                          // first point info
    const Point &next_vertex = polygon_[(i + 1) % vertices_num];    // second point info
    // store segment which segment_[i].first.x < segment_[i].second.x
    if (cur_vertex[major_dir_] < next_vertex[major_dir_]) {         
      segments_.emplace_back(cur_vertex, next_vertex);
    } else {
      segments_.emplace_back(next_vertex, cur_vertex);
    }
    // compute slope k
    double x_diff = next_vertex[major_dir_] - cur_vertex[major_dir_];    
    double y_diff = next_vertex[op_major_dir_] - cur_vertex[op_major_dir_];
    std::abs(cur_vertex[major_dir_] - next_vertex[major_dir_]) < kEpsilon
        ? slope_.push_back(kInf) : slope_.push_back(y_diff / x_diff);
  }
}
```
3. 如上图D，根据主方向上的valid_range(最大坐标和最小坐标差值)，以及网格线间距grid_size，画出valid_range/grid_size条网格线，然后根据segment_里面的两个点计算每条边跟其后面的网格线的第一个交点E。这里计算E有什么用？为什么只计算第一个交点E，而不计算边S和所有网格线可能的交点？

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.cc
bool PolygonScanConverter::ConvertSegmentToEdge(const size_t seg_id, std::pair<int, Edge> *out_edge) {
  const Segment &segment = segments_[seg_id];
  double min_x = segment.first[major_dir_] - min_x_;        // bias from left boundary
  double min_y = segment.first[op_major_dir_];              // y coordinate

  int x_id = std::ceil(min_x / step_);                      // the next closest grid line
  out_edge->first = x_id;
  Edge &edge = out_edge->second;
  edge.x = x_id * step_;                                    // compute x of interaction
  edge.max_x = segment.second[major_dir_] - min_x_;         // max x of this segment. For any interaction P, if P.x > edge.max_x, P is out of this segment 
  edge.max_y = segment.second[op_major_dir_];
  edge.k = slope_[seg_id];

  if (std::isfinite(edge.k)) {                              // compute y of the interaction
    edge.y = min_y + (edge.x - min_x) * edge.k;
  } else {
    edge.y = min_y;
    if (edge.y > edge.max_y) {
      std::swap(edge.y, edge.max_y);
    }
  }
  if (std::isfinite(edge.k) && edge.max_x < edge.x) {       // return false if interaction is not cross the grid line which his k is finite
    return false;
  }
  return true;
}
```

上面代码相对来说也不是特别难理解，min_x, min_y是计算这个边segment到主方向最左端的距离以及次方向上的坐标。通过x_id=ceil(min_x/step_)可以求出边的起点和后面相交的网格线id，最后用edge.x, edge.y来存储这个交点坐标。这里的edge.max_x，edge.max_y有什么作用？他解决了上述"边与其他后续网格线的交点坐标怎么计算？"这个问题。通过(edge.x, edge.y)作为起点，加上斜率k，就能计算该条边和接下来网格线的交点。

比如已知边segment和下条网格线x_id的交点坐标为(edge.x, edge.y)，那么和下面第二条网格线x_id+1的交点坐标计算方法为：
(x, y) = (edge.x+step_, edge.y+k\*step_)，那么怎么知道这个交点在这条边上还是在边的延长线上(不属于这条边)呢，只要判断计算得到的x小于edge.max_x是否成立，成立边和x_id+1网格线有交点，否则无交点。

另外返回false的条件是什么意思，"k有限大并且边与x_id号网络线交点超过了边的最大坐标(在边的延长线上，边外)"，这说明，这条边与x_id号网格线没有交点，也不是垂直边，无效边！

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/perception_obstacles_roi_lut2.png)

4. 如上图E，第一个问题"计算E有什么用?" 首先我们要知道一个问题：如果多边形只包含凸或者凹，那么网格线穿过多边形，如何计算落在多边形ROI里面的区间？只要计算多边形和该网格线的交点，然后按照y的大小从小到大排列，必定是2n的交点{P1,P2,P3,...,P2n}，那么落入多边形的区间肯定是[P1.y,P2.y] [P3.y, P4.y], .. , [P2n-1.y, P2n.y]。这个可以从上图证实。那么对于3中的交点E可以排序，两两组合最终得到路面与路口区域落在该网格线上的区间。这部分代码有点难，可以慢慢体会：

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.cc
void PolygonScanConverter::UpdateActiveEdgeTable(
    const size_t x_id, std::vector<Interval> \*scan_intervals) {
  size_t valid_edges_num = active_edge_table_.size();
  size_t invalid_edges_num = 0;
  // For each edege in active edge table, check whether it is still valid.
  // Stage 1, compute next interaction with later grid line, if out the line, erase in step3 by setting the slope infinite
  for (auto &edge : active_edge_table_) {
    if (!edge.MoveUp(step_)) {
      --valid_edges_num;
      ++invalid_edges_num;
      edge.y = kInf;
    }
  }
  // Stage 2, add the new edge into vector which compute directly from step 3.
  size_t new_edges_num = 0;
  for (const auto &edge : edge_table_[x_id]) {
    if (std::isfinite(edge.k)) {
      ++valid_edges_num;
      ++new_edges_num;
      active_edge_table_.push_back(edge);
    } else {
      scan_intervals->emplace_back(edge.y, edge.max_y);
    }
  }
  // Stage 3, remove the interactions which out of the segment(p.y greater than p.max_x)
  if (invalid_edges_num != 0 || new_edges_num != 0) {
    std::sort(active_edge_table_.begin(), active_edge_table_.end(),
              [](const Edge &a, const Edge &b) { return a.y < b.y; });
    active_edge_table_.erase(next(active_edge_table_.begin(), valid_edges_num), active_edge_table_.end());
  }
  // Stage 4, compute interval of grid line #x_id
  for (size_t i = 0; i + 1 < active_edge_table_.size(); i += 2) {
    double min_y = active_edge_table_[i].y;
    double max_y = active_edge_table_[i + 1].y;

    scan_intervals->emplace_back(min_y, max_y);
  }
}
```

根据代码段我暂时将ROI区间计算分为4个阶段，第一个阶段就是3中所讨论的问题，前面每条边开始已知跟后一条x_id-1网格线的交点，那么这些边跟后续网格线x_id, x_id+1的交点怎么计算(计算如上图的E13，E53)，这里通过MoveUp(step_)函数向后推演一个step_计算，函数跟上面讲得一致，最后返回时候超过边最大x,超过则丢弃，不超过保留。第二阶段增加是否有新的边由步骤3中直接计算得到(新加入如上图的E1-E7,)；第三阶段就是按照次方向y的值从小到大排列，并删除超出边的那些点；最后一个阶段就是计算落入多边形与id_x网格线相交的区间，

经过BCDE四步骤，就能将基于定点存储形式的路口与路面坐标转化成填充(点阵)形式。最后得到的结果是vector<std::vector<Interval>> \*scans_intervals形式的扫描结果，2D向量，每行对应一个扫描线；每行的vector存储对应网格线的路面ROI区间。最后就只要把scans_intervals这个扫描结果转换成bitmap的点阵就行了，也就是将scans_intervals放入bitmap_的即可。填充部分，代码相对比较简单，这里就省略了，可能以64位bit的形式进行存储有点点让人费解，不过没关系，相信有C++数据结构基础的你一定能理解。

最后要做的就是对原始点云cloud_local进行处理，标记点云中哪些点在ROI以外，哪些点在ROI以内，ROI区域内的点云可以供下一步行人，车辆等物体分割。

```c++
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.cc
bool HdmapROIFilter::Bitmap2dFilter(const pcl::PointCloud<pcl_util::Point>::ConstPtr in_cloud_ptr,
    const Bitmap2D& bitmap, pcl_util::PointIndices* roi_indices_ptr) {
  roi_indices_ptr->indices.reserve(in_cloud_ptr->size());
  for (size_t i = 0; i < in_cloud_ptr->size(); ++i) {
    const auto& pt = in_cloud_ptr->points[i];
    Eigen::Vector2d p(pt.x, pt.y);
    if (bitmap.IsExist(p) && bitmap.Check(p)) {
      roi_indices_ptr->indices.push_back(i);
    }
  }
  return true;
}

/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.cc
bool Bitmap2D::IsExist(const Eigen::Vector2d& p) const {
  if (p.x() < min_p_.x() || p.x() >= max_p_.x()) {
    return false;
  }
  if (p.y() < min_p_.y() || p.y() >= max_p_.y()) {
    return false;
  }
  return true;
}

bool Bitmap2D::Check(const Eigen::Vector2d& p) const {
  Eigen::Matrix<size_t, 2, 1> grid_pt = ((p - min_p_).array() / grid_size_.array()).cast<size_t>();
  Eigen::Matrix<size_t, 2, 1> major_grid_pt(grid_pt[dir_major_], grid_pt[op_dir_major_]);

  size_t x_id = major_grid_pt.x();
  size_t block_id = major_grid_pt.y() >> 6;  // major_grid_pt.y() / 64, which grid line
  size_t bit_id = major_grid_pt.y() & 63;    // major_grid_pt.y() % 64

  const uint64_t& block = bitmap_[x_id][block_id];

  const uint64_t first_one = static_cast<uint64_t>(1) << 63;
  return block & (first_one >> bit_id);
}
```

从上面代码中不难看到其实针对每个基于ENU局部坐标系的点云cloud，根据其x和y去bitmap里面做check，第一check该点是否落在这个网格里面(x:[-range,range], y:[-range,range])，这个检查由isExist函数完成；第二个check，如果该点在LUT网格内，那么check这个点是否在路面ROI内，只要检查其对应的网格坐标去bitmap查询即可，1表示在路面；0表示在路面外，这个检查由Check函数完成。最后cloud点云中每个点是否在路面ROI内全部记录在roi_indices_ptr内，里面本质就是一个向量，大小跟cloud里面的点云数量相等，结果一一对应。

#### 2.1.4 点云筛选

高地图ROI过滤器最后一步就是过滤工作，去除背景点云(ROI区域以外点云)，便于下一步物体分割。点云筛选由PCL库pcl::copyPointCloud函数实现，roi_indices存储ROI区域内点云的id。

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(
  /// call roi_filter
  PointCloudPtr roi_cloud(new PointCloud);
  if (roi_filter_ != nullptr) {
    PointIndicesPtr roi_indices(new PointIndices);
    ROIFilterOptions roi_filter_options;
    roi_filter_options.velodyne_trans = velodyne_trans;
    roi_filter_options.hdmap = hdmap;
    if (roi_filter_->Filter(point_cloud, roi_filter_options,roi_indices.get())) {
      pcl::copyPointCloud(*point_cloud, *roi_indices, *roi_cloud);
      roi_indices_ = roi_indices;
    } else {
      ...
    }
  }
}

/// PCL
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                     const pcl::PointIndices &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  // Do we want to copy everything?
  if (indices.indices.size () == cloud_in.points.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  ...
 
  // Iterate over each point
  for (size_t i = 0; i < indices.indices.size (); ++i)
    cloud_out.points[i] = cloud_in.points[indices.indices[i]];
}
```

最后总结一下高精地图ROI的过程：

1. 高精地图查询，获得路面路口的polygons多边形信息
2. 点云坐标变换。将原始点云cloud从lidar坐标系转到ENU局部坐标系cloud_local；polygons从世界坐标系转到ENU局部坐标系polygons_local；
3. 将顶点存储形式的路面路口polygon信息，转换成填充(点阵)形式的存储方式。扫描线算法转换，bitmap存储
4. 根据ROI LUT查询表，标记原始点云cloud_local是否在ROI内或外面。

### 2.2 基于卷积神经网络分割

>高精地图 ROI过滤之后，Apollo得到已过滤、只包含属于ROI内的点云，大部分背景障碍物，如路侧的建筑物、树木等均被移除，ROI内的点云被传递到分割模块。分割模块检测和划分前景障碍物，例如汽车，卡车，自行车和行人。

该阶段的输入数据来自高精地图ROI过滤器过滤得到的电云数据，最终输出对应于ROI中的障碍物对象数据集。该阶段包含4个子过程：

- 通道特征提取
- 基于卷积神经网络的障碍物预测
- 障碍物集群
- 后期处理

#### 2.2.1 通道特征提取

给定一个点云框架(cloud_roi)，Apollo在地方坐标系中构建俯视图（即投影到X-Y平面）2D网格。基于点的X、Y坐标，相对于LiDAR传感器原点的预定范围内，每个点被量化为2D网格的一个单元。量化后，Apollo计算网格内每个单元格中点的8个统计测量，这将是下一步中传递给CNN的输入通道特征。计算的8个统计测量：

1. 单元格中点的最大高度--max_height_data
2. 单元格中最高点的强度--top_intensity_data
3. 单元格中点的平均高度--mean_height_data
4. 单元格中点的平均强度--mean_intensity_data
5. 单元格中的点数--count_data
6. 单元格中心相对于原点的角度--direction_data
7. 单元格中心与原点之间的距离--distance_data
8. 二进制值标示单元格是空还是被占用--nonempty_data

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// call hdmap to get ROI
  ...
  /// call roi_filter
  ...
  /// call segmentor
  std::vector<ObjectPtr> objects;
  if (segmentor_ != nullptr) {
    SegmentationOptions segmentation_options;
    segmentation_options.origin_cloud = point_cloud;
    PointIndices non_ground_indices;
    non_ground_indices.indices.resize(roi_cloud->points.size());
    std::iota(non_ground_indices.indices.begin(), non_ground_indices.indices.end(), 0);
    if (!segmentor_->Segment(roi_cloud, non_ground_indices, segmentation_options, &objects)) {
      ...
    }
  }
}

/// file in apollo/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects) {
  // generate raw features
  if (use_full_cloud_) {
    feature_generator_->Generate(options.origin_cloud);
  } else {
    feature_generator_->Generate(pc_ptr);
  }
  ...
}
```

从上面代码可以看出，与高精地图ROI过滤器一样，所有的分割操作都在CNNSegmentation这个类里面完成。接下来我们从代码入手，看看怎么样从一个点云的集合{(x,y,z,i)}得到上述8类数据，最终由cloud_local映射过后的点云集合会生成一个[1,8,w,h]的矩阵作为CNN的输入，其中w和h在外部文件中定义，都为512。这里的use_full_cloud_标志其实是处理原始点云or处理roi点云(去掉背景)，默认使用原始点云use_full_cloud_=true。

**这里有一个注意点，原始点云的x和y都有他的范围，即激光雷达的感知范围。有这么一个前提：其实事实上激光雷达检测到360度范围内的点云，如果点云离激光雷达lidar太远，那么这些点其实没必要去处理，处理车辆附近的点云(E.g. 60米范围内)，即可以节省计算资源，又能降低复杂度。而这个筛选的范围由参数point_cloud_range参数控制，默认60米**

(1) 将点云实际的xy坐标映射到输入矩阵HxW平面坐标，并且筛选点云高度

从上面得知，分割阶段处理的点云实际上是激光雷达物理距离x:[-60,60], y:[-60,60]区间内的点云，但是CNN接受的输入大小NCHW是1x8xHxW。所以需要将这个范围内的点云坐标重新映射到HxW这个大小的平面。那么转换其实很简单：

E.g. 1. 如果点X轴坐标px从范围[a,b]，拉伸/压缩映射到范围[c,d]，则映射过后的新坐标px2= c + (d-c)/(b-a) \* (px-a)

E.g. 2. 如果点X轴坐标px从范围[-a,a]，拉伸/压缩映射到范围[0,c]，则映射过后的新坐标px2 = c/2a \* (px-(-a))

接着回到代码，我们看看映射的过程：

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
void FeatureGenerator<Dtype>::Generate(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr) {
  const auto& points = pc_ptr->points;

  map_idx_.resize(points.size());
  float inv_res_x = 0.5 * static_cast<float>(width_) / static_cast<float>(range_);   // E.g.2 inv_res_x == c/2a(a=range_, c=widht_)
  float inv_res_y = 0.5 * static_cast<float>(height_) / static_cast<float>(range_);  // E.g.2 inv_res_x == c/2a(a=range_, c=widht_)

  for (size_t i = 0; i < points.size(); ++i) {
    // 1. remove the cloud points which height is out of the interval [-5.0,5.0]
    if (points[i].z <= min_height_ || points[i].z >= max_height_) {          
      map_idx_[i] = -1;
      continue;
    }
    // * the coordinates of x and y are exchanged here
    int pos_x = F2I(points[i].y, range_, inv_res_x);  // compute mapping coordinate: col
    int pos_y = F2I(points[i].x, range_, inv_res_y);  // compute mapping coordinate: row
    // 2. remove the cloud points which out of the interval x:[-60,60], y:[-60,60]
    if (pos_x >= width_ || pos_x < 0 || pos_y >= height_ || pos_y < 0) {
      map_idx_[i] = -1;
      continue;
    }
    map_idx_[i] = pos_y * width_ + pos_x;
}

/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/util.h
inline int F2I(float val, float ori, float scale) {        // compute mapping coordinate in E.g.2, (px-(-a)) * c/2a
  return static_cast<int>(std::floor((ori - val) * scale));
}
```

从上面代码很容易的看到这个映射过程，以及两个筛选流程：

- 去除高度在5米以上或者-5米以下的点云。信号灯高度差不多在5米以下，因此5米以上可能是建筑物之类的无效点云，可以去除
- 去除xy在60米以外的点云。范围过大，离车过远的点云，即使包含物体，也没必要检测。

(2) 计算单元格中的8类数据

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool FeatureGenerator<Dtype>::Init(const FeatureParam& feature_param, caffe::Blob<Dtype>* out_blob) {
  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      int idx = row * width_ + col;
      // * row <-> x, column <-> y
      float center_x = Pixel2Pc(row, height_, range_);     // compute mapping coordinate: center_x
      float center_y = Pixel2Pc(col, width_, range_);      // compute mapping coordinate: center_y
      constexpr double K_CV_PI = 3.1415926535897932384626433832795;
      direction_data[idx] = static_cast<Dtype>(std::atan2(center_y, center_x) / (2.0 * K_CV_PI)); // compute direction_data(channel 6)
      distance_data[idx] = static_cast<Dtype>(std::hypot(center_x, center_y) / 60.0 - 0.5);       // compute distance_data(channel 7)
    }
  }
  return true;
}

void FeatureGenerator<Dtype>::Generate(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr) {
  for (size_t i = 0; i < points.size(); ++i) {
    // 1. remove the cloud points which height is out of the interval [-5.0,5.0]
    ...
    // 2. remove the cloud points which out of the interval x:[-60,60], y:[-60,60]
    ...
    float pz = points[i].z;    
    float pi = points[i].intensity / 255.0;
    if (max_height_data_[idx] < pz) {        // update max_height_data(channel 1)
      max_height_data_[idx] = pz;
      top_intensity_data_[idx] = pi;		 // update top_intensity_data(channel 2)
    }
    mean_height_data_[idx] += static_cast<Dtype>(pz);    // accumulated  mean_height_data
    mean_intensity_data_[idx] += static_cast<Dtype>(pi); // accumulated mean_intensity_data
    count_data_[idx] += Dtype(1);                        // compute count_data(channel 5)
  }

  for (int i = 0; i < siz; ++i) {
    constexpr double EPS = 1e-6;
    if (count_data_[i] < EPS) {
      max_height_data_[i] = Dtype(0);
    } else {
      mean_height_data_[i] /= count_data_[i];       // compute  mean_height_data(channel 3)
      mean_intensity_data_[i] /= count_data_[i];    // compute  mean_intensity_data(channel 5)
      nonempty_data_[i] = Dtype(1);                 // compute nonempty_data(channel 8)
    }
  }
}

/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/util.h
inline float Pixel2Pc(int in_pixel, float in_size, float out_range) {
  float res = 2.0 * out_range / in_size;
  return out_range - (static_cast<float>(in_pixel) + 0.5f) * res;
}
```

上面的代码经过标记可以很清晰的明白数据的生成过程，其中网格中的点到原点的距离和方向跟实际数据无关，所以在Init函数中早早计算完成了；而其他六类数据需要根据输入计算，因此在Generate函数中计算。其中Pixel2Pc函数其实就是上面坐标映射，换汤不换药，但是有一个问题需要注意，这里额外加上了一个0.5f，这个作用其实就是计算网格的中心点坐标装换。比如第一个网格x坐标是0，那么网格中心点就是0.5(0-1中心)，这里稍微注意下就行，其他一样。

#### 2.2.2 基于卷积神经网络的障碍物预测

基于上述通道特征，Apollo使用深度完全卷积神经网络（FCNN）来预测单元格障碍物属性，包括潜在物体中心的偏移位移（称为中心偏移）、对象性、积极性和物体高度。如图2所示，网络的输入为 W x H x C 通道图像，其中：

- W 代表网格中的列数
- H 代表网格中的行数
- C 代表通道特征数

完全卷积神经网络由三层构成：

- 下游编码层（特征编码器）
- 上游解码层（特征解码器）
- 障碍物属性预测层（预测器）

特征编码器将通道特征图像作为输入，并且随着特征抽取的增加而连续下采样其空间分辨率。 然后特征解码器逐渐对特征图像。上采样到输入2D网格的空间分辨率，可以恢复特征图像的空间细节，以促进单元格方向的障碍物位置、速度属性预测。根据具有非线性激活（即ReLu）层的堆叠卷积/分散层来实现 下采样和 上采样操作。

代码中的执行分割入口函数如下所示，由caffe的Forword函数前向计算得到：

```c++
/// file in apollo/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects) {
  // generate raw features
  ...

  // network forward process
#ifdef USE_CAFFE_GPU
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif
  caffe_net_->Forward();
  PERF_BLOCK_END("[CNNSeg] CNN forward");
}
```

基于卷积神经网络的障碍物分割采用的是UNet的FCNN。具体结构如下：

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/perception_obstacles_segment_unet.png)

接下去我们对整个网络的输入与输出做一个表格的总结：

<table border="1">
<tr>
	<th>类型</th>
	<th>内容</th>
	<th>备注</th>
</tr>
<tr>
	<th rowspan="8">输入 [1,8,512,512]</th>
	<td>channel 0: 单元格中点的最大高度</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 1: 单元格中最高点的强度</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 2: 单元格中点的平均高度</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 3: 单元格中点的平均强度</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 4: 单元格中的点数</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 5: 单元格中心相对于原点的角度</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 6: 单元格中心与原点之间的距离</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 7: 进制值标示单元格是空还是被占用</td>
	<td>掩码mask</td>
</tr>
<tr>
	<th rowspan="6">输出 [1,12,512,512]</th>
	<td>channel 0: category_pt</td>
	<td>是否是物体预测。Sigmoid激活，并与输入channel 7掩码mask相乘</td>
</tr>
<tr>
	<td>channel 1-2: instance_pt</td>
	<td>中心偏移预测</td>
</tr>
<tr>
	<td>channel 3: confidence_pt</td>
	<td>前景物体概率预测。Sigmoid激活</td>
</tr>
<tr>
	<td>channel 4-8: classify_pt</td>
	<td>物体类别预测。Sigmoid激活</td>
</tr>
<tr>
	<td>channel 9-10: heading_pt</td>
	<td>-</td>
</tr>
<tr>
	<td>channel 11: height_pt</td>
	<td>高度预测。</td>
</tr>
</table>

以上神经网络的输出被分为6部分，各部分作用如上表所示，该过程就是传统的CNN分割。

#### 2.2.3 障碍物聚类

>在基于CNN的预测之后，Apollo获取单个单元格的预测信息。利用四个单元对象属性图像，其中包含：
>
>- 中心偏移/instance_pt
>- 对象性/category_pt
>- 积极性/configdence_pt
>- 对象高度/height_pt
>
>为生成障碍物，Apollo基于单元格中心偏移，预测构建有向图，并搜索连接的组件作为候选对象集群。
>
>如下图所示，每个单元格是图的一个节点，并且基于单元格的中心偏移预测构建有向边，其指向对应于另一单元的父节点。
>
>如下图，Apollo采用压缩的联合查找算法（Union Find algorithm ）有效查找连接组件，每个组件都是候选障碍物对象集群。对象是单个单元格成为有效对>象的概率。因此，Apollo将非对象单元定义为目标小于0.5的单元格。因此，Apollo过滤出每个候选对象集群的空单元格和非对象集。
>
![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/obstacle_clustering.png)
>
>(a) 红色箭头表示每个单元格对象中心偏移预测；蓝色填充对应于物体概率不小于0.5的对象单元。
>
>(b) 固体红色多边形内的单元格组成候选对象集群。
>
>由五角星填充的红色范围表示对应于连接组件子图的根节点（单元格）。
>
>一个候选对象集群可以由其根节点彼此相邻的多个相邻连接组件组成。

上述是Apollo 2.0官方文档的描述，听起来还是懵懵懂懂，那么在这章节我们依旧用来代码来解释如何利用CNN分割结果进行障碍物聚类。本小节使用了一个比较简单的数据结构来处理不相交集合的合并问题--并查集(或者联合查找算法, Union Find Sets)，如果你对并查集不了解，可以通过此链接进行初步了解[并查集算法](https://www.cnblogs.com/shadowwalker9/p/5999029.html)

障碍物预测并查集算法由Cluster函数触发：

```c++
/// file in apollo/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects) {
  // generate raw features
  ...
  // network forward process
  ...
  // clutser points and construct segments/objects
  cluster2d_->Cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                      valid_indices, objectness_thresh,
                      use_all_grids_for_clustering);
}
```

下面我们将从代码逐步了解：

1. 并查集建立步骤1: 建立新的并查集--DisjointSetMakeSet

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h
class Cluster2D {
public:
  void Cluster(const caffe::Blob<float>& category_pt_blob,
               const caffe::Blob<float>& instance_pt_blob,
               const apollo::perception::pcl_util::PointCloudPtr& pc_ptr,
               const apollo::perception::pcl_util::PointIndices& valid_indices,
               float objectness_thresh, bool use_all_grids_for_clustering) {

    std::vector<std::vector<Node>> nodes(rows_,std::vector<Node>(cols_, Node()));
    // map points into grids
    ...

    // construct graph with center offset prediction and objectness
    for (int row = 0; row < rows_; ++row) {
      for (int col = 0; col < cols_; ++col) {
        int grid = RowCol2Grid(row, col);
        Node* node = &nodes[row][col];
        DisjointSetMakeSet(node);
        node->is_object = (use_all_grids_for_clustering || nodes[row][col].point_num > 0) &&
            (*(category_pt_data + grid) >= objectness_thresh);
        int center_row = std::round(row + instance_pt_x_data[grid] * scale_);
        int center_col = std::round(col + instance_pt_y_data[grid] * scale_);
        center_row = std::min(std::max(center_row, 0), rows_ - 1);
        center_col = std::min(std::max(center_col, 0), cols_ - 1);
        node->center_node = &nodes[center_row][center_col];
      }
    }
  }
}

/// file in apollo/modules/common/util/disjoint_set.h
template <class T>
void DisjointSetMakeSet(T *x) {
  x->parent = x;
  x->node_rank = 0;
}
```

先从代码流程上看，上述是Cluster函数的第二个阶段，构建新的并查集(第一阶段是上面的"map points into grids"仅仅是数据的一个复制过程，比较简单)。这个阶段主要的函数是DisjointSetMakeSet，该函数下面的一些操作还包括判断该节点是否是物体的一个部分(is_object)，需要is_object大于一个阈值(外部文件定义objectness_thresh，默认0.5)；判断节点指向的中心节点center_node坐标。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/uset_0.png)

回到代码，建立并查集第一步，建立一个新的并查集，其中包含s个单元素集合(对应grid_=512x512个Node)。每个节点的父指针指向自己(如上图)。代码中node_rank其实是用于集合合并，代码采用了按秩合并策略，node_rank代表树的高度上界。

2. 并查集建立步骤2: 产生不相交集合(树)--Traverse

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h
class Cluster2D {
public:
  void Cluster(const caffe::Blob<float>& category_pt_blob,
               const caffe::Blob<float>& instance_pt_blob,
               const apollo::perception::pcl_util::PointCloudPtr& pc_ptr,
               const apollo::perception::pcl_util::PointIndices& valid_indices,
               float objectness_thresh, bool use_all_grids_for_clustering) {

    std::vector<std::vector<Node>> nodes(rows_,std::vector<Node>(cols_, Node()));
    // map points into grids
    ...
    // construct graph with center offset prediction and objectness
    ...
    // traverse nodes
    for (int row = 0; row < rows_; ++row) {
      for (int col = 0; col < cols_; ++col) {
        Node* node = &nodes[row][col];
        if (node->is_object && node->traversed == 0) {
          Traverse(node);
        }
      }
    }
  }
}
```

这个阶段是根据CNN的分割结果，产生若干个不相交的集合(N棵树)

通过每个节点的center_node(由CNN分割输出得到)，可以建立一条从属管理的链，如下图。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/uset_1.png)

图中4个节点的关系，d的中心是c节点，c的中心是b节点，b的中心是a节点，a的中心是自己。那么我们可以根据这个center_node的指针建立一条链表。通过这个center_node我们可以确定每个节点的父节点，也就是如上图左箭头所示。

**这种表示方法有一个问题：当需要查询某个节点的最顶层父节点a，所需要的时间复杂度是树的高度，无法保证在常数时间内完成，所以代码中采用了压缩路径的并查集算法**

如何压缩路径？这里采用子节点直接指向顶层父节点的方法，经过处理也就是上图右边的结果，dcba四个节点都指向顶层父节点--a节点。

下面我们分析Traverse函数，初次这个函数有点让人费解，可以看一下：

```c++
class Cluster2D {
  void Traverse(Node* x) {
    std::vector<Node*> p;
    p.clear();
    while (x->traversed == 0) {
      p.push_back(x);
      x->traversed = 2;
      x = x->center_node;
    }
    if (x->traversed == 2) {
      for (int i = static_cast<int>(p.size()) - 1; i >= 0 && p[i] != x; i--) {
        p[i]->is_center = true;
      }
      x->is_center = true;
    }
    for (size_t i = 0; i < p.size(); i++) {
      Node* y = p[i];
      y->traversed = 1;
      y->parent = x->parent;
    }
  }
};
```

代码中第一个while是从当前节点x向上路由到最顶层父节点(**注意这里的最顶层父节点并不是整整意义上的最顶层，而是最上面的没有遍历过得tranersed=0，因为便利过得父节点其parent指针已经指向最顶层节点了**)，得到的路径放入path。for循环就是修改节点的父节点指针，都指向最顶层达到图右边的效果。

**这里一个比较难理解的环节就是travered标志位，这个标志位乍一看其实看不出什么作用，他的作用其实是标记每棵树的一条支路，后续的合并在这条支路上进行。举个例子:**

输入a节点，如果有链1：a--b--c--d--e。经过Traverse处理，abcde节点父指针指向e，is_traversed=1，并且is_center=true

输入f节点，如果有链2：f--g--c--d--e. 经过Traverse处理，fgcde节点父指针指向e，is_traversed=1，但是fg的is_center=false(没有执行中间的if)，这是因为这棵树上已经存在一条支路abcde用来后续的合并，所以fgcde这条支路就不需要了。

输入h节点，如果有链3：h--i--j--k，经过Traverse处理，hijk节点父指针指向k，is_traversed=1，并且is_center=true，这是第二棵树

其实观察可以发现，每棵树只有一条path的is_center=true，其他支路都是false，所以作用一棵树只与另一颗树的指定支路上合并。传统的并查集仅仅是树的根之间合并，这里增加了根与部分支点间合并，效果更好好。

3. 并查集建立步骤3: 不相交集合(树)合并--DisjointSetUnion

当给定两个节点x，y时，就需要查找，这两个节点的顶层父节点是否一致。如果不一致就需要进行一个集合(树)合并。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/uset_2.png)

上图如果给出两个节点d和g(DisjointSetUnion函数的两个输入)，我们就需要去查询d和g节点的最顶层父节点，这里可以从上一步骤中查询结果，最终发现顶层节点不一致，所以就需要进行一个合并，最终经过合并的结果如图右。

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h
class Cluster2D {
public:
  void Cluster(const caffe::Blob<float>& category_pt_blob,
               const caffe::Blob<float>& instance_pt_blob,
               const apollo::perception::pcl_util::PointCloudPtr& pc_ptr,
               const apollo::perception::pcl_util::PointIndices& valid_indices,
               float objectness_thresh, bool use_all_grids_for_clustering) {

    std::vector<std::vector<Node>> nodes(rows_,std::vector<Node>(cols_, Node()));
    // map points into grids
    ...
    // construct graph with center offset prediction and objectness
    ...
    // traverse nodes, generate collections
    ...
    // merge collection
    for (int row = 0; row < rows_; ++row) {
      for (int col = 0; col < cols_; ++col) {
        Node* node = &nodes[row][col];
        if (!node->is_center) {
          continue;
        }
        for (int row2 = row - 1; row2 <= row + 1; ++row2) {
          for (int col2 = col - 1; col2 <= col + 1; ++col2) {
            if ((row2 == row || col2 == col) && IsValidRowCol(row2, col2)) {
              Node* node2 = &nodes[row2][col2];
              if (node2->is_center) {
                DisjointSetUnion(node, node2);
              }
            }
          }
        }
      }
    }
  }
}
```

**一个注意点，两个节点的最顶层父节点不一致，说明他们不属于同一类，既然不属于一类，那为什么要合并呢？原因很简单，在局部区域内(注意一定是局部区域内)，两棵树虽然最顶层不一致，但可能是CNN的误差造成这一结果，事实上这两个区域内很可能是同一种物体，所以这里对临近区域进行合并**

**局部区域这个概念在代码中的直观反应是合并只在当前节点的3x3网格中进行(row2&&col2
参数，相距太远必然不属于同种物体，根本不需要去合并，因为一棵树上的节点都是属于同一物体的组件) ！！！**

```c++
/// file in apollo/modules/common/util/disjoint_set.h
template <class T>
void DisjointSetMerge(T *x, const T *y) {}

template <class T>
void DisjointSetUnion(T *x, T *y) {
  x = DisjointSetFind(x);
  y = DisjointSetFind(y);
  if (x == y) {
    return;
  }
  if (x->node_rank < y->node_rank) {
    x->parent = y;
    DisjointSetMerge(y, x);
  } else if (y->node_rank < x->node_rank) {
    y->parent = x;
    DisjointSetMerge(x, y);
  } else {
    y->parent = x;
    x->node_rank++;
    DisjointSetMerge(x, y);
  }
}
```

上述代码比较简单，DisjointSetFind(x)是找到x的顶层父节点，然后后续步骤就是顶层父节点的融合。相对比较简单。注意这里的DisjointSetMerge函数是被合并那棵树上子节点的parent修正，因为被合并了，所以这棵树上的子节点的最顶层父节点都要修改，当然可以维护，但是成本比较大(可以参考quick_union算法)，这里不去做修正也没事，无非是增加了一些开销。

4. 合并完以后，每棵树就代表一类物体，做记录

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h
class Cluster2D {
public:
  void Cluster(const caffe::Blob<float>& category_pt_blob,
               const caffe::Blob<float>& instance_pt_blob,
               const apollo::perception::pcl_util::PointCloudPtr& pc_ptr,
               const apollo::perception::pcl_util::PointIndices& valid_indices,
               float objectness_thresh, bool use_all_grids_for_clustering) {

    std::vector<std::vector<Node>> nodes(rows_,std::vector<Node>(cols_, Node()));
    // map points into grids
    ...
    // construct graph with center offset prediction and objectness
    ...
    // traverse nodes, generate collections
    ...
    // merge collection
    ...
    // generate object id
    for (int row = 0; row < rows_; ++row) {
      for (int col = 0; col < cols_; ++col) {
        Node* node = &nodes[row][col];
        if (!node->is_object) {
          continue;
        }
        Node* root = DisjointSetFind(node);
        if (root->obstacle_id < 0) {
          root->obstacle_id = count_obstacles++;
          obstacles_.push_back(Obstacle());
        }
        int grid = RowCol2Grid(row, col);
        id_img_[grid] = root->obstacle_id;
        obstacles_[root->obstacle_id].grids.push_back(grid);
      }
    }
  }
}
```

代码中`root->obstacle_id=count_obstacles++;`这句话就已经很明白的支出，每棵树的root节点就代表一个候选物体对象，整棵树上所有的节点就组成了一个候选对象集群。对象存放在`obstacles_`这个vector<Obstacle>中，该对象包含的候选对象集群(所属网格点)保存在这个`Obstacle.grids`的vector中。

经过这一步处理完，就知道网格是否有物体，如果是物体对象，那么包含那些网格。但是不知道这是什么物体。

#### 2.2.4 后期处理

聚类后，Apollo获得一组候选对象集，每个候选对象集包括若干单元格。 

在后期处理中，Apollo首先对所涉及的单元格的积极性和物体高度值，平均计算每个候选群体的检测置信度分数和物体高度。 然后，Apollo去除相对于预测物体高度太高的点，并收集每个候选集中的有效单元格的点。 最后，Apollo删除具有非常低的可信度分数或小点数的候选聚类，以输出最终的障碍物集/分段。

用户定义的参数可以在`modules/perception/model/cnn_segmentation/cnnseg.conf`的配置文件中设置。 下表说明了CNN细分的参数用法和默认值：


 |参数名称             |使用说明                                           |默认值    |
 |-----------------------------------|--------------------------------------------------------------------------------------------|-----------|
 |objectness_thresh                  |用于在障碍物聚类步骤中过滤掉非对象单元的对象的阈值。 |0.5        |
 |use_all_grids_for_clustering       |指定是否使用所有单元格在障碍物聚类步骤中构建图形的选项。如果不是，则仅考虑占用的单元格。   |true   |
 |confidence_thresh                  |用于在后期处理过程中滤出候选聚类的检测置信度得分阈值。    |0.1    |
 |height_thresh                      |如果是非负数，则在后处理步骤中将过滤掉高于预测物体高度的点。 |0.5 meters |
 |min_pts_num                        |在后期处理中，删除具有小于min_pts_num点的候选集群。 |3   |
 |use_full_cloud                     |如果设置为true，则原始点云的所有点将用于提取通道特征。 否则仅使用输入点云的点（即，HDMap ROI过滤器之后的点）。  |true |
 |gpu_id                             |在基于CNN的障碍物预测步骤中使用的GPU设备的ID。            |0          |
 |feature_param {width}              |2D网格的X轴上的单元格数。                      |512        |
 |feature_param {height}             |2D网格的Y轴上的单元格数。                     |512        |
 |feature_param {range}              |2D格栅相对于原点（LiDAR传感器）的范围。             |60 meters  |

```c++
/// file in apollo/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects) {
  // generate raw features
  ...
  // network forward process
  ...
  // clutser points and construct segments/objects
  ...
  // post process
  cluster2d_->Filter(*confidence_pt_blob_, *height_pt_blob_);

  cluster2d_->Classify(*class_pt_blob_);

  cluster2d_->GetObjects(confidence_thresh, height_thresh, min_pts_num, objects);
}
```

Filter和Classify函数代码很简单，前者计算了每个候选物体集群的平均分数和高度，后者则计算了每个候选物体集群k类物体分类对应的平均置信度分数以及所属物体类别(对应最大平均置信度分数那一类)。这里不贴出来做介绍了。

```c++
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h
class Cluster2D {
public:
  void GetObjects(const float confidence_thresh, const float height_thresh, const int min_pts_num, std::vector<ObjectPtr>* objects) {

    for (size_t i = 0; i < point2grid_.size(); ++i) {
      int grid = point2grid_[i];
      int obstacle_id = id_img_[grid];
      int point_id = valid_indices_in_pc_->at(i);
      // select obstacles which averaged score greater equal than confidence_thresh(0.1)
      // and averaged height in the interval
      if (obstacle_id >= 0 && obstacles_[obstacle_id].score >= confidence_thresh) {
        if (height_thresh < 0 || pc_ptr_->points[point_id].z <= obstacles_[obstacle_id].height + height_thresh) {
          obstacles_[obstacle_id].cloud->push_back(pc_ptr_->points[point_id]);
        }
      }
    }
    
    // select obstacles which has minimal points at least min_pts_num(3)
    for (size_t obstacle_id = 0; obstacle_id < obstacles_.size(); obstacle_id++) {
      Obstacle* obs = &obstacles_[obstacle_id];
      if (static_cast<int>(obs->cloud->size()) < min_pts_num) {
        continue;
      }
      apollo::perception::ObjectPtr out_obj(new apollo::perception::Object);
      out_obj->cloud = obs->cloud;
      out_obj->score = obs->score;
      out_obj->score_type = ScoreType::SCORE_CNN;
      out_obj->type = GetObjectType(obs->meta_type);
      out_obj->type_probs = GetObjectTypeProbs(obs->meta_type_probs);
      objects->push_back(out_obj);
    }
  }
}
```

从上面的代码中可以看到，GetObjects函数就是做最后的删选，物体置信度评分必须大于0.1，CNN分割并且聚类得到的物体平均高度必须和点云检测高度相差到0.5m以内，最后cluster得到的候选物体集群必须包含足够的网格数，大于3个。

GetObjectType是根据候选物体集群的的类别，匹配对应的类。

GetObjectTypeProbs是根据候选物体集群的的类别，计算该类的置信度分数。

### 2.3  MinBox障碍物边框构建

>对象构建器组件为检测到的障碍物建立一个边界框。因为LiDAR传感器的遮挡或距离，形成障碍物的点云可以是稀疏的，并且仅覆盖一部分表面。因此，盒构建器将恢复给定多边形点的完整边界框。即使点云稀疏，边界框的主要目的还是预估障碍物（例如，车辆）的方向。同样地，边框也用于可视化障碍物。
>
>算法背后的想法是找到给定多边形点边缘的所有区域。在以下示例中，如果AB是边缘，则Apollo将其他多边形点投影到AB上，并建立具有最大距离的交点对，这是属于边框的边缘之一。然后直接获得边界框的另一边。通过迭代多边形中的所有边，在以下图4所示，Apollo确定了一个6边界边框，将选择具有最小面积的方案作为最终的边界框。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/object_building.png)

这部分代码看了比较久，有些地方一直没想明白，直到推理了很久才找到了一种合适的说法，下面我们依旧从代码入手，一步步解析障碍物边框构建的流程。

上一步CNN分割与后期处理，可以得到lidar一定区域内的障碍物集群。接下去我们将对这些障碍物集群建立其标定框。标定框的作用除了标识物体，还有一个作用就是标记障碍物的长length，宽width，高height。其中规定长length大于宽width，障碍物方向就是长的方向direction。MinBox构建过程如下：

- 计算障碍物2d投影(高空鸟瞰xy平面)下的多边形polygon(如下图B)
- 根据上述多边形，计算最适边框(如下图C)

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/minbox_framework.png)

大致的代码框架如下：

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// call hdmap to get ROI
  ...
  /// call roi_filter
  ...
  /// call segmentor
  ...
  /// call object builder
  if (object_builder_ != nullptr) {
    ObjectBuilderOptions object_builder_options;
    if (!object_builder_->Build(object_builder_options, &objects)) {
      ...
    }
  }
}

/// file in apollo/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc
bool MinBoxObjectBuilder::Build(const ObjectBuilderOptions& options, std::vector<ObjectPtr>* objects) {
  for (size_t i = 0; i < objects->size(); ++i) {
    if ((*objects)[i]) {
      BuildObject(options, (*objects)[i]);
    }
  }
}
void MinBoxObjectBuilder::BuildObject(ObjectBuilderOptions options, ObjectPtr object) {
  ComputeGeometricFeature(options.ref_center, object);
}
void MinBoxObjectBuilder::ComputeGeometricFeature(const Eigen::Vector3d& ref_ct, ObjectPtr obj) {
  // step 1: compute 2D xy plane's polygen
  ComputePolygon2dxy(obj);
  // step 2: construct box
  ReconstructPolygon(ref_ct, obj);
}
```

上述是MinBox障碍物边框构建的主题框架代码，构建的两个过程分别在`ComputePolygon2dxy`和`ReconstructPolygon`函数完成，下面篇幅我们就具体深入这两个函数，详细了解一下Apollo对障碍物构建的一个流程，和其中一些令人费解的代码段。

#### 2.3.1 MinBox构建--计算2DXY平面投影

这个阶段主要作用是障碍物集群做XY平面下的凸包多边形计算，最终得到这个多边形的一些角点。第一部分相对比较简单，没什么难点，计算凸包是调用PCL库的`ConvexHull`组件(具体请参考[pcl::ConvexHull](http://docs.pointclouds.org/trunk/classpcl_1_1_convex_hull.html))。下面是Apollo的凸包计算代码：

```c++
/// file in apollo/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc
void MinBoxObjectBuilder::ComputePolygon2dxy(ObjectPtr obj) {
  ...
  ConvexHull2DXY<pcl_util::Point> hull;
  hull.setInputCloud(pcd_xy);
  hull.setDimension(2);
  std::vector<pcl::Vertices> poly_vt;
  PointCloudPtr plane_hull(new PointCloud);
  hull.Reconstruct2dxy(plane_hull, &poly_vt);

  if (poly_vt.size() == 1u) {
    std::vector<int> ind(poly_vt[0].vertices.begin(), poly_vt[0].vertices.end());
    TransformPointCloud(plane_hull, ind, &obj->polygon);
  } else {
    ...
  }
}

/// file in apollo/modules/perception/common/convex_hullxy.h
template <typename PointInT>
class ConvexHull2DXY : public pcl::ConvexHull<PointInT> {
public:
  void Reconstruct2dxy(PointCloudPtr hull, std::vector<pcl::Vertices> *polygons) {
    PerformReconstruction2dxy(hull, polygons, true);
  }

  void PerformReconstruction2dxy(PointCloudPtr hull, std::vector<pcl::Vertices> *polygons, bool fill_polygon_data = false) {  
    coordT *points = reinterpret_cast<coordT *>(calloc(indices_->size() * dimension, sizeof(coordT)));
    // step1. Build input data, using appropriate projection
    int j = 0;
    for (size_t i = 0; i < indices_->size(); ++i, j += dimension) {
      points[j + 0] = static_cast<coordT>(input_->points[(*indices_)[i]].x);
      points[j + 1] = static_cast<coordT>(input_->points[(*indices_)[i]].y);
    }
    // step2. Compute convex hull
    int exitcode = qh_new_qhull(dimension, static_cast<int>(indices_->size()), points, ismalloc, const_cast<char *>(flags), outfile, errfile);
    std::vector<std::pair<int, Eigen::Vector4f>, Eigen::aligned_allocator<std::pair<int, Eigen::Vector4f>>> idx_points(num_vertices);
    FORALLvertices {
      hull->points[i] = input_->points[(*indices_)[qh_pointid(vertex->point)]];
      idx_points[i].first = qh_pointid(vertex->point);
      ++i;
    }
    // step3. Sort
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hull, centroid);
    for (size_t j = 0; j < hull->points.size(); j++) {
      idx_points[j].second[0] = hull->points[j].x - centroid[0];
      idx_points[j].second[1] = hull->points[j].y - centroid[1];
    }
    std::sort(idx_points.begin(), idx_points.end(), pcl::comparePoints2D);
    polygons->resize(1);
    (*polygons)[0].vertices.resize(hull->points.size());
    for (int j = 0; j < static_cast<int>(hull->points.size()); j++) {
      hull->points[j] = input_->points[(*indices_)[idx_points[j].first]];
      (*polygons)[0].vertices[j] = static_cast<unsigned int>(j);
    }
  }
}
```

从上面代码的注释我们可以很清楚的了解到这个多边形顶点的求解流程，具体函数由`PerformReconstruction2dxy`函数完成，这个函数其实跟PCL库自带的很像[pcl::ConvexHull<PointInT>::performReconstruction2D/Line76](http://docs.pointclouds.org/trunk/convex__hull_8hpp_source.html)，其实Apollo开发人员几乎将pcl库的`performReconstruction2D`原封不动的搬过来了，去掉了一些冗余额外的信息。这个过程主要有：

1. 构建输入数据，将输入的点云复制到coordT \*points做处理
2. 计算障碍物点云的凸包，得到的结果是多边形顶点。调用`qh_new_qhull`函数
3. 顶点排序，从[pcl::comparePoints2D/Line59](http://docs.pointclouds.org/trunk/surface_2include_2pcl_2surface_2convex__hull_8h_source.html)可以看到排序是角度越大越靠前，atan2函数的结果是[-pi,pi]。所以就相当于是顺时针对顶点进行排序。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/minbox_polygons.png)

这个过程只要自己稍加关注一点就可以了解他的原理和过程，这里不再过度解释每个模块。上图是计算多边形交点的流程示意图。

#### 2.3.2 MinBox构建--边框构建

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/minbox_box.png)

边框构建的大致思想是对过程中1得到的多边形的每一条边，将剩下的所有点都投影到这条边上可以计算边框Box的一条边长度(最远的两个投影点距离)，同时选择距离该条边最远的点计算Box的高，这样就可以得到一个Box(上图case1-7分别是以这个多边形7条边作投影得到的7个Box)，最终选择Box面积最小的边框作为障碍物的边框。上图中case7得到的Box面积最小，所以case7中的Box就是最终障碍物的边框。当边框确定以后，就可以得到障碍物的长度length(大边长)，宽度(小边长)，方向(大边上对应的方向)，高度（点云的平均高度，CNN分割与后期处理阶段得到）。

但是实际这个过程有部分代码块是比较难理解的，而且加入了很多实际问题来优化这个过程。这里我将对这些问题一一进行解释，希望能够让大家理解。根据代码我简单地将这个过程归结为3步：

1. 投影边长的选择(为什么要选择？因为背对lidar那一侧的点云是稀疏的，那一侧的多边形顶点是不可靠的，不用来计算Box)
2. 每个投影边长计算Box

在进入正式的代码详解以前，这里有几个知识点需要我们了解。

假设向量a=(x0,y0)，向量b=(x1,y1)，那么有
- 两个向量的点乘, a·b = x0x1 + y0y1\
- 计算向量a在向量b上的投影: v = a·b/(b^2)·b，投影点的坐标就是v+(b.x, b.y)
- 两个向量的叉乘, axb = |a|·|b|sin(theta) = x0y1 - x1y0，叉乘方向与ab平面垂直，遵循右手法则。**叉乘模大小另一层意义是: ab向量构成的平行四边形面积**

**如果两个向量a，b共起点，那么axb小于0，那么a to b的逆时针夹角大于180度；等于则共线；大于0，a to b的逆时针方向夹角小于180度。**

接下来我们就正式的解剖`ReconstructPolygon`Box构建的代码

(1) Step1：投影边长的选择

```c++
/// file in apollo/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc
void MinBoxObjectBuilder::ReconstructPolygon(const Eigen::Vector3d& ref_ct, ObjectPtr obj) {
  // compute max_point and min_point
  size_t max_point_index = 0;
  size_t min_point_index = 0;
  Eigen::Vector3d p;
  p[0] = obj->polygon.points[0].x;
  p[1] = obj->polygon.points[0].y;
  p[2] = obj->polygon.points[0].z;
  Eigen::Vector3d max_point = p - ref_ct;
  Eigen::Vector3d min_point = p - ref_ct;
  for (size_t i = 1; i < obj->polygon.points.size(); ++i) {
    Eigen::Vector3d p;
    p[0] = obj->polygon.points[i].x;
    p[1] = obj->polygon.points[i].y;
    p[2] = obj->polygon.points[i].z;
    Eigen::Vector3d ray = p - ref_ct;
    // clock direction
    if (max_point[0] * ray[1] - ray[0] * max_point[1] < EPSILON) {
      max_point = ray;
      max_point_index = i;
    }
    // unclock direction
    if (min_point[0] * ray[1] - ray[0] * min_point[1] > EPSILON) {
      min_point = ray;
      min_point_index = i;
    }
  }
}
```

首先我们看到这一段代码，第一眼看过去是计算`min_point`和`max_point`两个角点，那么这个角点到底是什么意思呢？里面这个关于`EPSILON`的比较条件代表了什么，下面我们图。有一个前提我们已经在polygons多边形角点计算中可知：obj的polygon中所有角点都是顺时针按照arctan角度由大到小排序。那么这个过程我们可以从下面的图中了解到作用：

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/minbox_maxminpt.png)

图中叉乘与0(EPSILON)的大小就是根据前面提到的，两个向量的逆时针夹角。从上图我们可以很清晰的看到：**`max_point`和`min_point`就代表了lidar能检测到障碍物的两个极端点！**


```c++
/// file in apollo/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc
void MinBoxObjectBuilder::ReconstructPolygon(const Eigen::Vector3d& ref_ct, ObjectPtr obj) {
  // compute max_point and min_point
  ...
  // compute valid edge
  Eigen::Vector3d line = max_point - min_point;
  double total_len = 0;
  double max_dis = 0;
  bool has_out = false;
  for (size_t i = min_point_index, count = 0; count < obj->polygon.points.size(); i = (i + 1) % obj->polygon.points.size(), ++count) {
    //Eigen::Vector3d p_x = obj->polygon.point[i]
    size_t j = (i + 1) % obj->polygon.points.size();
    if (j != min_point_index && j != max_point_index) {
      // Eigen::Vector3d p = obj->polygon.points[j];
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {
        ...
      }else {
        ...
      }
    } else if ((i == min_point_index && j == max_point_index) || (i == max_point_index && j == min_point_index)) {
      ...
    } else if (j == min_point_index || j == max_point_index) {
      // Eigen::Vector3d p = obj->polygon.points[j];
      Eigen::Vector3d ray = p_x - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {
        ...
      } else {
        ...
      }
    }
  }
}
```

当计算得到`max_point`和`min_point`后就需要执行这段代码，这段代码是比较难理解的，为什么需要对每条边做一个条件筛选？请看下图

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/minbox_edge_selection.png)

**上图中A演示了这段代码对一个汽车的点云多边形进行处理，最后的处理结果可以看到只有Edge45、Edge56、Edge67是有效的，最终会被计入`total_len`和`max_dist`。而且你会发现这些边都是在`line(max_point-min_point)`这条分界线的一侧，而且是靠近lidar这一侧。说明了靠近lidar这一侧点云检测效果好，边稳定；而背离lidar那一侧，会因为遮挡原因，往往很难(有时候不可能)得到真正的顶点，如上图B所示。**

经过这么分析，其实上述的代码理解起来还是比较能接受的，希望能帮到你。

(2) Step2：投影边长Box计算

投影边长Box计算由`ComputeAreaAlongOneEdge`函数完成，分析这个函数的代码：

```c++
/// file in apollo/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc
double MinBoxObjectBuilder::ComputeAreaAlongOneEdge(
    ObjectPtr obj, size_t first_in_point, Eigen::Vector3d* center,
    double* lenth, double* width, Eigen::Vector3d* dir) {
  // first for
  std::vector<Eigen::Vector3d> ns;
  Eigen::Vector3d v(0.0, 0.0, 0.0);      // 记录以(first_in_point,first_in_point+1)两个定点为边，所有点投影，距离这条边最远的那个点
  Eigen::Vector3d vn(0.0, 0.0, 0.0);     // 最远的点在(first_in_point,first_in_point+1)这条边上的投影坐标
  Eigen::Vector3d n(0.0, 0.0, 0.0);      // 用于临时存储
  double len = 0;
  double wid = 0;
  size_t index = (first_in_point + 1) % obj->polygon.points.size();
  for (size_t i = 0; i < obj->polygon.points.size(); ++i) {
    if (i != first_in_point && i != index) {
      // o = obj->polygon.points[i]
      // a = obj->polygon.points[first_in_point]
      // b = obj->polygon.points[first_in_point+1]
      // 计算向量ao在ab向量上的投影，根据公式:k = ao·ab/(ab^2), 计算投影点坐标，根据公式k·ab+(ab.x, ab.y)
      double k =  ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
      k = k / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
      k = k * -1;
      n[0] = (b[0] - a[0]) * k + a[0];
      n[1] = (b[1] - a[1]) * k + a[1];
      n[2] = 0;
      // 计算由ab作为边，o作为顶点的平行四边形的面积,利用公式|ao x ab|，叉乘的模就是四边形的面积，
      Eigen::Vector3d edge1 = o - b;
      Eigen::Vector3d edge2 = a - b;
      double height = fabs(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
      // 利用公式： 面积/length(ab)就是ab边上的高，即o到ab边的垂直距离， 记录最大的高
      height = height / sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1]);
      if (height > wid) {
        wid = height;
        v = o;
        vn = n;
      }
    } else {
      ...
    }
    ns.push_back(n);
  }
}
```

从上面的部分代码可以看得出，`ComputeAreaAlongOneEdge`函数接受的输入包括多边形顶点集合，起始边`first_in_point`。代码将以`first_in_point`和`first_in_point+1`两个顶点构建一条边，将集合中其他点都投影到这条边上，并计算顶点距离这条边的高，也就是垂直距离。最终的结果保存到`ns`中。代码中`k`的计算利用了两个向量点乘来计算投影点的性质；`height`的计算利用了两个向量叉乘的模等于两个向量组成的四边形面积的性质。

```c++
/// file in apollo/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc
double MinBoxObjectBuilder::ComputeAreaAlongOneEdge(
  // first for
  ...
  // second for
  size_t point_num1 = 0;
  size_t point_num2 = 0;
  // 遍历first_in_point和first_in_point+1两个点以外的，其他点的投影高，选择height最大的点，来一起组成Box
  // 这两个for循环是寻找ab边上相聚最远的投影点，因为要把所有点都包括到Box中，所以Box沿着ab边的边长就是最远两个点的距离，可以参考边框构建。
  for (size_t i = 0; i < ns.size() - 1; ++i) {
    Eigen::Vector3d p1 = ns[i];
    for (size_t j = i + 1; j < ns.size(); ++j) {
      Eigen::Vector3d p2 = ns[j];
      double dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
      if (dist > len) {
        len = dist;
        point_num1 = i;
        point_num2 = j;
      }
    }
  }
  // vp1和vp2代表了Box的ab边对边的那条边的两个顶点，分别在v的两侧，方向和ab方向一致。
  Eigen::Vector3d vp1 = v + ns[point_num1] - vn;
  Eigen::Vector3d vp2 = v + ns[point_num2] - vn;
  // 计算中心点和面积
  (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
  (*center)[2] = obj->polygon.points[0].z;
  if (len > wid) {
    *dir = ns[point_num2] - ns[point_num1];
  } else {
    *dir = vp1 - ns[point_num1];
  }
  *lenth = len > wid ? len : wid;
  *width = len > wid ? wid : len;
  return (*lenth) * (*width);
}
```

剩下的代码就是计算Box的四个顶点坐标，以及他的面积Area。

综上所述，Box经过上述(1)(2)两个阶段，可以很清晰的得到每条有效边(靠近lidar一侧，在`min_point`和`max_point`之间)对应的Box四个顶点坐标、宽、高。最终选择Box面积最小的作为障碍物预测Box。这个过程的代码部分在理解上存在一定难度，经过本节的讲解，应该做MinBox边框构建有了一定的了解。

### 2.4 HM对象跟踪

>HM对象跟踪器跟踪分段检测到的障碍物。通常，它通过将当前检测与现有跟踪列表相关联，来形成和更新跟踪列表，如不再存在，则删除旧的跟踪列表，并在识别出新的检测时生成新的跟踪列表。 更新后的跟踪列表的运动状态将在关联后进行估计。 在HM对象跟踪器中，匈牙利算法(Hungarian algorithm)用于检测到跟踪关联，并采用鲁棒卡尔曼滤波器(Robust Kalman Filter) 进行运动估计。

上述是Apollo官方文档对HM对象跟踪的描述，这部分意思比较明了，主要的跟踪流程可以分为:

- 预处理。(lidar->local ENU坐标系变换、跟踪对象创建、跟踪目标保存)
- 卡尔曼滤波器滤波，预测物体当前位置与速度(卡尔曼滤波阶段1：Predict阶段)
- 匈牙利算法比配，关联检测物体和跟踪物体
- 卡尔曼滤波，更新跟踪物体位置与速度信息(卡尔曼滤波阶段2：Update阶段)

进入HM物体跟踪的入口依旧在`LidarProcessSubnode::OnPointCloud`中：

```c++
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// call hdmap to get ROI
  ...
  /// call roi_filter
  ...
  /// call segmentor
  ...
  /// call object builder
  ...
  /// call tracker
  if (tracker_ != nullptr) {
    TrackerOptions tracker_options;
    tracker_options.velodyne_trans = velodyne_trans;
    tracker_options.hdmap = hdmap;
    tracker_options.hdmap_input = hdmap_input_;
    if (!tracker_->Track(objects, timestamp_, tracker_options, &(out_sensor_objects->objects))) {
    ...
    }
  }
}
```

在这部分，总共有三个比较绕的对象类，分别是Object、TrackedObject和ObjectTrack，在这里统一说明一下区别：

- Object类：常见的物体类，里面包含物体原始点云、多边形轮廓、物体类别、物体分类置信度、方向、长宽、速度等信息。**全模块通用**。
- TrackedObject类：封装Object类，记录了跟踪物体类属性，额外包含了中心、重心、速度、加速度、方向等信息。
- ObjectTrack类：封装了TrackedObject类，实际的跟踪解决方案，不仅包含了需要跟踪的物体(TrackedObject)，同时包含了跟踪物体滤波、预测运动趋势等函数。

所以可以看到，跟踪过程需要将原始Object封装成TrackedObject，创立跟踪对象；最后跟踪对象创立跟踪过程ObjectTrack，可以通过ObjectTrack里面的函数来对ObjectTrack所标记的TrackedObject进行跟踪。

#### 2.4.1 预处理


```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc
bool HmObjectTracker::Track(const std::vector<ObjectPtr>& objects,
                            double timestamp, const TrackerOptions& options,
                            std::vector<ObjectPtr>* tracked_objects) {
  // A. track setup
  if (!valid_) {
    valid_ = true;
    return Initialize(objects, timestamp, options, tracked_objects);
  }
  // B. preprocessing
  // B.1 transform given pose to local one
  TransformPoseGlobal2Local(&velo2world_pose);
  // B.2 construct objects for tracking
  std::vector<TrackedObjectPtr> transformed_objects;
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,options);
  ...
}

bool HmObjectTracker::Initialize(const std::vector<ObjectPtr>& objects,
                                 const double& timestamp,
                                 const TrackerOptions& options,
                                 std::vector<ObjectPtr>* tracked_objects) {
  global_to_local_offset_ = Eigen::Vector3d(-velo2world_pose(0, 3), -velo2world_pose(1, 3), -velo2world_pose(2, 3));
  // B. preprocessing
  // B.1 coordinate transformation
  TransformPoseGlobal2Local(&velo2world_pose);
  // B.2 construct tracked objects
  std::vector<TrackedObjectPtr> transformed_objects;
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose, options);
  // C. create tracks
  CreateNewTracks(transformed_objects, unassigned_objects);
  time_stamp_ = timestamp;
  // D. collect tracked results
  CollectTrackedResults(tracked_objects);
  return true;
}
```

预处理阶段主要分两个模块：A.跟踪建立(track setup)和B.预处理(preprocess)。跟踪建立过程，主要是对上述得到的物体对象进行跟踪目标的建立，这是Track第一次被调用的时候进行的，后续只需要进行跟踪对象更新即可。建立过程相对比较简单，主要包含：

1. 物体对象坐标系转换。(原先的lidar坐标系-->lidar局部ENU坐标系/有方向性)
2. 对每个物体创建跟踪对象，加入跟踪列表。
3. 记录现在被跟踪的对象

从上面代码来看，预处理阶段两模块重复度很高，这里我们就介绍`Initialize`对象跟踪建立函数。

(1) 第一步是进行坐标系的变换，这里我们注意到一个平移向量`global_to_local_offset_`，他是lidar坐标系到世界坐标系的变换矩阵`velo2world_trans`的平移成分，前面高精地图ROI过滤器小节我们讲过: **local局部ENU坐标系跟world世界坐标系之间只有平移成分，没有旋转。所以这里取了转变矩阵的平移成分，其实就是world世界坐标系转换到lidar局部ENU坐标系的平移矩阵(变换矩阵)。P_local = P_world + global_to_local_offset_**

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc
void HmObjectTracker::TransformPoseGlobal2Local(Eigen::Matrix4d* pose) {
  (*pose)(0, 3) += global_to_local_offset_(0);
  (*pose)(1, 3) += global_to_local_offset_(1);
  (*pose)(2, 3) += global_to_local_offset_(2);
}
```

从上面的`TransformPoseGlobal2Local`函数代码我们可以得到一个没有平移成分，只有旋转成分的变换矩阵`velo2world_pose`，这个矩阵有什么作用？很简单，**这个矩阵就是lidar坐标系到lidar局部ENU坐标系的转换矩阵。**

(2) 第二步中需要根据前面CNN检测到的物体来创建跟踪对象，也就是将`Object`包装到`TrackedObject`中，那我们先来看一下`TrackedObject`类里面的成分：

| 名称 | 备注 |
| ---- | ---- |
| ObjectPtr object_ptr | Object对象指针 |
| Eigen::Vector3f barycenter | 重心，取该类所有点云xyz的平均值得到 |
| Eigen::Vector3f center | 中心， bbox4个角点外加平均高度计算得到 |
| Eigen::Vector3f velocity | 速度，卡尔曼滤波器预测得到 |
| Eigen::Matrix3f velocity_uncertainty | 不确定速度 |
| Eigen::Vector3f acceleration | 加速度 | 
| ObjectType type | 物体类型，行人、自行车、车辆等 |
| float association_score | -- |

从上面表格可以看到，`TrackedObject`封装了`Object`，并且只增加了少量速度，加速度等额外信息。

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc
void HmObjectTracker::ConstructTrackedObjects(
    const std::vector<ObjectPtr>& objects,
    std::vector<TrackedObjectPtr>* tracked_objects, const Eigen::Matrix4d& pose,
    const TrackerOptions& options) {
  int num_objects = objects.size();
  tracked_objects->clear();
  tracked_objects->resize(num_objects);
  for (int i = 0; i < num_objects; ++i) {
    ObjectPtr obj(new Object());
    obj->clone(*objects[i]);
    (*tracked_objects)[i].reset(new TrackedObject(obj));                  // create new TrackedObject with object
    // Computing shape featrue
    if (use_histogram_for_match_) {
      ComputeShapeFeatures(&((*tracked_objects)[i]));                     // compute shape feature
    }
    // Transforming all tracked objects
    TransformTrackedObject(&((*tracked_objects)[i]), pose);               // transform coordinate from lidar frame to local ENU frame
    // Setting barycenter as anchor point of tracked objects
    Eigen::Vector3f anchor_point = (*tracked_objects)[i]->barycenter;
    (*tracked_objects)[i]->anchor_point = anchor_point;
    // Getting lane direction of tracked objects
    pcl_util::PointD query_pt;                                            // get lidar's world coordinate equals lidar2world_trans's translation part  
    query_pt.x = anchor_point(0) - global_to_local_offset_(0);
    query_pt.y = anchor_point(1) - global_to_local_offset_(1);
    query_pt.z = anchor_point(2) - global_to_local_offset_(2);
    Eigen::Vector3d lane_dir;
    if (!options.hdmap_input->GetNearestLaneDirection(query_pt, &lane_dir)) {
      lane_dir = (pose * Eigen::Vector4d(1, 0, 0, 0)).head(3);            // get nearest line direction from hd map
    }
    (*tracked_objects)[i]->lane_direction = lane_dir.cast<float>();
  }
}
```

`ConstructTrackedObjects`是由物体对象来创建跟踪对象的代码，这个过程相对来说比较简单易懂，没大的难点，下面就解释一下具体的功能。

- 针对`vector<ObjectPtr>& objects`中的每个对象，创建对应的`TrackedObject`，并且计算他的shape feature，这个特征计算比较简单，先计算物体xyz三个坐标轴上的最大和最小值，分别将其划分成10等份，对每个点xyz坐标进行bins投影与统计。最后的到的特征就是[x_bins,y_bins,z_bins]一共30维，归一化(除点云数量)后得到最终的shape feature。
- `TransformTrackedObject`函数进行跟踪物体的方向、中心、原始点云、多边形角点、重心等进行坐标系转换。lidar坐标系变换到local ENU坐标系。
- 根据lidar的世界坐标系坐标查询高精地图HD map计算车道线方向

(3) 第三步就是讲第二步中创建的跟踪对象(TrackedObject)建立跟踪，正式进行跟踪(加入进ObjectTrack)。

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc
void HmObjectTracker::CreateNewTracks(
    const std::vector<TrackedObjectPtr>& new_objects,
    const std::vector<int>& unassigned_objects) {
  // Create new tracks for objects without matched tracks
  for (size_t i = 0; i < unassigned_objects.size(); i++) {
    int obj_id = unassigned_objects[i];
    ObjectTrackPtr track(new ObjectTrack(new_objects[obj_id]));
    object_tracks_.AddTrack(track);
  }
}
```

同时函数`CollectTrackedResults`会将当前正在跟踪的对象(世界坐标系坐标形式)保存到向量中，该部分代码比较简单就不贴出来了。

#### 2.4.2 卡尔曼滤波，跟踪物体对象(卡尔曼滤波阶段1： Predict)

在预处理阶段，每个物体Object类经过封装以后，产生一个对应的ObjectTrack过程类，里面封装了对应要跟踪的物体(TrackedObject，由Object封装而来)。这个阶段的工作就是对跟踪物体TrackedObject进行卡尔曼滤波并预测其运动方向。

首先，在这里我们简单介绍一下卡尔曼滤波的一些基础公式，方便下面理解。

-----------------------------------------------------------------------------------------
一个系统拥有一个状态方程和一个观测方程。观测方程是我们能宏观看到的一些属性，在这里比如说汽车重心xyz的位置和速度；而状态方程是整个系统里面的一些状态，包含能观测到的属性(如汽车重心xyz的位置和速度)，也可能包含其他一些看不见的属性，这些属性甚至我们都不能去定义它的物理意义。**因此观测方程的属性是状态方程的属性的一部分**现在有：

状态方程: $X_t = A_{t,t-1}X_{t-1} + W_t$, 其中$W_t \to N(0,Q) $

观测方程: $Z_t = C_tX_t + V_t$, 其中$V_t \to N(0,R) $

卡尔曼滤波分别两个阶段，分别是预测Predict与更新Update：

- Predict预测阶段
	- 利用上时刻t-1最优估计$X_{t-1}$预测当前时刻状态$X_{t,t-1} = A_{t,t-1}X_{t-1}$，这个$X_{t,t-1}$不是t时刻的最优状态，只是估计出来的状态
	- 利用上时刻t-1最优协方差矩阵$P_{t-1}$预测当前时刻协方差矩阵$P_{t,t-1} = A_{t,t-1}P_{t-1}{A_{t,t-1}}^T + Q$，这个$P_{t,t-1}$也不是t时刻最优协方差
- Update更新阶段
	- 利用$X_{t,t-1}$估计出t时刻最优状态$X_t = X_{t,t-1} + H_t[Z_t - C_tX_{t,t-1}]$, 其中$H_t = P_{t,t-1}{C_t}^T[C_tP_{t,t-1}{C_t}^T + R]^{-1}$
	- 利用$P_{t,t-1}$估计出t时刻最优协方差矩阵$P_t = [I - H_tC_t]P_{t,t-1}$

最终t从1开始递归计算k时刻的最优状态$X_k$与最优协方差矩阵$P_t$

--------------------------------------------------------------------------------------------

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc
bool HmObjectTracker::Track(const std::vector<ObjectPtr>& objects,
                            double timestamp, const TrackerOptions& options,
                            std::vector<ObjectPtr>* tracked_objects) {
  // A. track setup
  ...
  // B. preprocessing
  // B.1 transform given pose to local one
  ...
  // B.2 construct objects for tracking
  ...
  // C. prediction
  std::vector<Eigen::VectorXf> tracks_predict;
  ComputeTracksPredict(&tracks_predict, time_diff);
  ...
}

void HmObjectTracker::ComputeTracksPredict(std::vector<Eigen::VectorXf>* tracks_predict, const double& time_diff) {
  // Compute tracks' predicted states
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (int i = 0; i < no_track; ++i) {
    (*tracks_predict)[i] = tracks[i]->Predict(time_diff);   // track every tracked object in object_tracks_(ObjectTrack class) 
  }
}
```

从代码中我们可以看到，这个过程其实就是对object_tracks_列表中每个物体调用其Predict函数进行滤波跟踪(object_tracks_是上阶段Object--TrackedObject--ObjectTrack的依次封装)。接下去我们就对这个Predict函数进行深层次的挖掘和分析，看看它实现了卡尔曼过滤器的那个阶段工作。

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.cc
Eigen::VectorXf ObjectTrack::Predict(const double& time_diff) {
  // Get the predict of filter
  Eigen::VectorXf filter_predict = filter_->Predict(time_diff);
  // Get the predict of track
  Eigen::VectorXf track_predict = filter_predict;
  track_predict(0) = belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
  track_predict(1) = belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
  track_predict(2) = belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
  track_predict(3) = belief_velocity_(0);
  track_predict(4) = belief_velocity_(1);
  track_predict(5) = belief_velocity_(2);
  return track_predict;
}

/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.cc
Eigen::VectorXf KalmanFilter::Predict(const double& time_diff) {
  // Compute predict states
  Eigen::VectorXf predicted_state;
  predicted_state.resize(6);
  predicted_state(0) = belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
  predicted_state(1) = belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
  predicted_state(2) = belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
  predicted_state(3) = belief_velocity_(0);
  predicted_state(4) = belief_velocity_(1);
  predicted_state(5) = belief_velocity_(2);
  // Compute predicted covariance
  Propagate(time_diff);
  return predicted_state;
}

void KalmanFilter::Propagate(const double& time_diff) {
  // Only propagate tracked motion
  ity_covariance_ += s_propagation_noise_ * time_diff * time_diff;
}
```

**从上面两个函数可以明显看到这个阶段就是卡尔曼滤波器的Predict阶段。同时可以看到**：

1. `track_predict/predicted_state`相当于卡尔曼滤波其中的$X_{t,t-1}$, `belief_anchor_point_`和`belief_velocity_`相当于$X_t$, `ity_covariance_`同时存储$P_t$和$P_{t,t-1}$(Why?可以从上面的卡尔曼滤波器公式看到$P_t$在估测完$P_{t,t-1}$以后就没用了，所以可以覆盖存储，节省部分空间)

2. 状态方程和观测方程其实本质上是一样，也就是相同维度的。都是6维，分别表示重心的xyz坐标和重心xyz的速度。同时在这个应用中，短时间间隔内。当前时刻重心位置=上时刻重心位置 + 上时刻速度\*时间差，所以可知卡尔曼滤波器中$A_{t,t-1}\equiv1$, $Q = I\*ts^2$

3. 该过程工作:首先利用上时刻的最优估计`belief_anchor_point_`和`belief_velocity_`(等同于$X_{t-1}$)估计出t时刻的状态`predicted_state`(等同于$X_{t,t-1}$); 然后估计当前时刻的协方差矩`ity_covariance_`($P_{t-1}$和$P_{t,t-1}$交替存储)。

#### 2.4.3 匈牙利算法比配，关联检测物体和跟踪物体

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc
bool HmObjectTracker::Track(const std::vector<ObjectPtr>& objects,
                            double timestamp, const TrackerOptions& options,
                            std::vector<ObjectPtr>* tracked_objects) {
  // A. track setup
  ...
  // B. preprocessing
  // B.1 transform given pose to local one
  ...
  // B.2 construct objects for tracking
  ...
  // C. prediction
  ...
  // D. match objects to tracks
  std::vector<TrackObjectPair> assignments;
  std::vector<int> unassigned_objects;
  std::vector<int> unassigned_tracks;
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  if (matcher_ != nullptr) {
    matcher_->Match(&transformed_objects, tracks, tracks_predict, &assignments, &unassigned_tracks, &unassigned_objects);
  }
  ...
}

/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/hungarian_matcher.cc
void HungarianMatcher::Match(std::vector<TrackedObjectPtr>* objects,
                             const std::vector<ObjectTrackPtr>& tracks,
                             const std::vector<Eigen::VectorXf>& tracks_predict,
                             std::vector<TrackObjectPair>* assignments,
                             std::vector<int>* unassigned_tracks,
                             std::vector<int>* unassigned_objects) {
  // A. computing association matrix
  Eigen::MatrixXf association_mat(tracks.size(), objects->size());
  ComputeAssociateMatrix(tracks, tracks_predict, (*objects), &association_mat);

  // B. computing connected components
  std::vector<std::vector<int>> object_components;
  std::vector<std::vector<int>> track_components;
  ComputeConnectedComponents(association_mat, s_match_distance_maximum_,
                             &track_components, &object_components);
  // C. matching each sub-graph
  ...
}
```

这个阶段主要的工作是匹配CNN分割+MinBox检测到的物体和当前ObjectTrack的跟踪物体。主要的工作为：

- A. Object和TrackedObject之间关联矩阵`association_mat`计算
- B. 子图划分，利用上述的关联矩阵和设定的阈值(两两评分小于阈值则互相关联，即节点之间链接)，将矩阵分割成一系列子图
- C. 匈牙利算法进行二分图匹配，得到cost最小的(Object,TrackedObject)连接对

1. 关联矩阵`association_mat`计算

经过CNN分割与MinBox构建以后可以得到N个Object，而目前跟踪列表中有M个TrackedObject，所以需要构建一个NxM的关联矩阵，矩阵中每个元素(即关联评分)的计算共分为5个子项。

- 重心位置坐标距离差异评分
- 物体方向差异评分
- 标定框尺寸差异评分
- 点云数量差异评分
- 外观特征差异评分

最终以0.6，0.2，0.1，0.1，0.5的权重加权求和得到关联评分。

```c++
/// file in apollo/modules/perception/obstacle/lidar/tracker/hm_tracker/track_object_distance.cc
float TrackObjectDistance::ComputeDistance(const ObjectTrackPtr& track,
                                           const Eigen::VectorXf& track_predict,
                                           const TrackedObjectPtr& new_object) {
  // Compute distance for given trakc & object
  float location_distance = ComputeLocationDistance(track, track_predict, new_object);
  float direction_distance = ComputeDirectionDistance(track, track_predict, new_object);
  float bbox_size_distance = ComputeBboxSizeDistance(track, new_object);
  float point_num_distance = ComputePointNumDistance(track, new_object);
  float histogram_distance = ComputeHistogramDistance(track, new_object);

  float result_distance = s_location_distance_weight_ * location_distance +            // s_location_distance_weight_ = 0.6
                          s_direction_distance_weight_ * direction_distance +          // s_direction_distance_weight_ = 0.2
                          s_bbox_size_distance_weight_ * bbox_size_distance +          // s_bbox_size_distance_weight_ = 0.1
                          s_point_num_distance_weight_ * point_num_distance +          // s_point_num_distance_weight_ = 0.1
                          s_histogram_distance_weight_ * histogram_distance;           // s_histogram_distance_weight_ = 0.5
  return result_distance;
}
```

各个子项的计算方式，这里以文字形式描述，假设：
Object重心坐标为(x1,y1,z1)，方向为(dx1,dy1,dz1)，bbox尺寸为(l1,w1,h1), shape featrue为30维向量sf1，包含原始点云数量n1
TrackedObject重心坐标为(x2,y2,z2)，方向为(dx2,dy2,dz2)，bbox尺寸为(l2,w2,h2), shape featrue为30维向量sf2，包含原始点云数量n2

则有：

- 重心位置坐标距离差异评分location_distance计算
$location_distance = \sqrt{{x1 - x2}^2 + {y1 - y2}^2}$ 
如果速度太大，则需要用方向向量去正则惩罚，具体可以参考代码


- 物体方向差异评分direction_distance计算
方向差异其实就是计算两个向量的夹角:
$cos\theta = a·b/(|a|·|b|)$
夹角越大，差异越大，cos值越小
夹角越大，差异越大，cos值越大

最后使用1-cos计算评分，差异越小，评分越大。

- 标定框尺寸差异评分bbox_size_distance计算
代码中首先计算两个量`dot_val_00`和`dot_val_01`：

```c++
/// file in apollo/master/modules/perception/obstacle/lidar/tracker/hm_tracker/track_object_distance.cc
float TrackObjectDistance::ComputeBboxSizeDistance(const ObjectTrackPtr& track, const TrackedObjectPtr& new_object) {
  double dot_val_00 = fabs(old_bbox_dir(0) * new_bbox_dir(0) + old_bbox_dir(1) * new_bbox_dir(1));
  double dot_val_01 = fabs(old_bbox_dir(0) * new_bbox_dir(1) - old_bbox_dir(1) * new_bbox_dir(0));
  bool bbox_dir_close = dot_val_00 > dot_val_01;

  if (bbox_dir_close) {
    float diff_1 = fabs(old_bbox_size(0) - new_bbox_size(0)) / std::max(old_bbox_size(0), new_bbox_size(0));
    float diff_2 = fabs(old_bbox_size(1) - new_bbox_size(1)) / std::max(old_bbox_size(1), new_bbox_size(1));
    size_distance = std::min(diff_1, diff_2);
  } else {
    float diff_1 = fabs(old_bbox_size(0) - new_bbox_size(1)) / std::max(old_bbox_size(0), new_bbox_size(1));
    float diff_2 = fabs(old_bbox_size(1) - new_bbox_size(0)) / std::max(old_bbox_size(1), new_bbox_size(0));
    size_distance = std::min(diff_1, diff_2);
  }
  return size_distance;
}
```

这两个量有什么意义？这里简单解释一下，从计算方式可以看到：
其实`dot_val_00`是两个坐标的点积，数学计算形式上是方向1投影到方向2向量上得到向量3，最后向量3模乘以方向2模长，这么做可以估算方向差异。因为，当方向1和方向2两个向量夹角靠近0或180度时，投影向量很长，`dot_val_00`这个点积的值会很大。**`dot_val_00`越大说明两个方向越接近。**
同理`dot_val_01`上文我们提到过，差积的模可以衡量两个向量组成的四边形面积大小，这么做也可以估算方向差异。因为，当方向1和方向2两个向量夹角靠近90度时，组成的四边形面积最大，`dot_val_01`这个差积的模会很大。**`dot_val_00`越大说明两个方向越背离。**

- 点云数量差异评分point_num_distance计算
$point_num_distance = |n1-n2|/max(n1,n2)$

- 外观特征差异评分histogram_distance计算
$histogram_distance = \sum_{m=0}^{30} |sf1[m]-sf2[m]|$