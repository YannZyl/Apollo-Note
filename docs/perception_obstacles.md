# Apollo 2.0 感知模块：障碍物感知阅读笔记

本文档结合代码详细地解释感知模块中障碍物感知的流程与功能，也可以官方参考文档([Apollo 2.0 Obstacles Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md))

## <a name="障碍物感知">障碍物感知: 3D Obstacles Perception</a>

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles_framework.png)

上图为子节点之间的关系与数据流动，障碍物感知共存在三个子节点(线程)，分别为：

- 激光雷达处理子节点 LidarProcessSubnode
- 雷达处理子节点 RadarProcessSubnode
- 融合子节点 FusionSubnode

每个子节点的输入数据与输出数据在边上标出。

(1) 激光雷达子节点LidarProcessSubnode::OnPointCloud以ROS消息订阅与发布机制触发回调函数，处理结果保存在LidarObjectData共享数据容器中。主要解决的问题有：

- 高精地图ROI过滤器(HDMap ROI Filter)
- 基于卷积神经网络分割(CNN Segmentation)
- MinBox 障碍物边框构建(MinBox Builder)
- HM对象跟踪(HM Object Tracker)

(2) 雷达子节点RadarProcessSubnode::OnRadar同样以ROS消息订阅与发布机制触发回调函数，处理结果保存在RadarObjectData共享数据容器中。

(3) 融合子节点FusionSubnode::ProcEvents以自定义ProcEvents+EventManeger消息处理机制，从LidarObjectData和RadarObjectData共享数据容器中提取数据，融合并存储在FusionObjectData共享容器中。

### <a name="激光雷达感知">激光雷达感知 Perception: Lidar Obstacles PErception</a>

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

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles_lidar.png)

上图展示了激光雷达处理的4个模块，分别为高精地图ROI过滤器、基于卷积神经网络的障碍物分割、MinBox 障碍物边框构建与HM对象跟踪。接下去将一层层解剖代码，分析各个模块的流程的方法。

#### <a name="高精地图ROI过滤">高精地图ROI过滤器</a>

高精地图ROI过滤器是回调的第一个过程，从最开始的模块框架图可以看到，LidarProcessSubnode子节点接受的输入数据类型是ROS原始的点云数据类型，sensor_msgs::PointCloud2，简单地看一下这个数据结构，也可以参考官方文档[PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)

```
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

(1) 数据转换与存储

在进行高精地图ROI过滤的过程中，第一步是接收来自激光雷达的原始点云数据，设备id，时间戳ts等信息，并将其信息存入上述的SensorObject类中。存储过程中代码中值得关注的两个点分别是传感器到世界坐标系的转换矩阵velodyne_trans以及sensor_msgs::PointCloud2到PCL::PointCloud的转换。

```
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

(2) ROI生成

当获得原始的点云数据并转换成PCL格式以后，下一步需要从点云中检索ROI区域，这些ROI区域包含路面与路口的驾驶区域。

```
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

路面与路口的驾驶区域需要查询高精地图来完成，该阶段首先使用tf进行坐标系的转换(车辆/局部坐标系到世界坐标系的变换矩阵)，配合velodyne_pose计算得到velodyne_pose_world世界坐标系坐标，坐标系具体情况请参考[Apollo坐标系研究](https://github.com/YannZyl/Apollo-Note/blob/master/docs/apollo_coordinate.md)。真正获取ROI使用的是GetROI函数。具体的路口，车道存储形式请参考[高精地图模块](https://github.com/YannZyl/Apollo-Note/blob/master/docs/hdmap.md)

这个分析分析一下比较GetVelodyneTrans函数，这个函数的实现过程我们可以先简要的看一下在做功能分析：

```
/// file in apollo/modules/perception/onboard/transform_input.cc
bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans) {
  ...
  // Step1: lidar refer to novatel(GPS/IMU)
  if (!tf2_buffer.canTransform(FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp, ...) {
  	...
  }
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp);
  } 
  Eigen::Affine3d affine_lidar_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_lidar_3d);
  Eigen::Matrix4d lidar2novatel_trans = affine_lidar_3d.matrix();

  // Step2 notavel(GPS/IMU) refer to world coordinate system
  if (!tf2_buffer.canTransform(FLAGS_localization_tf2_frame_id, FLAGS_localization_tf2_child_frame_id, ...) {
    ...
  }
  try {
    transform_stamped = tf2_buffer.lookupTransform(FLAGS_localization_tf2_frame_id, FLAGS_localization_tf2_child_frame_id, query_stamp);
  } 
  Eigen::Affine3d affine_localization_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_localization_3d);
  Eigen::Matrix4d novatel2world_trans = affine_localization_3d.matrix();

  *trans = novatel2world_trans * lidar2novatel_trans;
}
```

从上述的代码中我们明显可以看到有两部分相似度很高的代码组成，第一部分是计算仿射变换矩阵lidar2novatel_trans，这个矩阵的计算通过ROS的tf模块调用lookupTransform函数完成。这部分的仿射变换矩阵是计算激光摄像头到GPS接收器相对于车辆坐标系的变换；第二部分是计算GPS接收器相对于世界坐标系的仿射变换矩阵。最后矩阵相乘得到车辆坐标系激光雷达到世界坐标系的仿射变换矩阵。

(3) 高精地图ROI过滤器

高精地图 ROI 过滤器（往下简称“过滤器”）处理在ROI之外的激光雷达点，去除背景对象，如路边建筑物和树木等，剩余的点云留待后续处理。该过程共有三个子过程。

- 坐标变换。由激光雷达测量得到的原始点云是基于局部坐标系的，而到经过高精地图处理得到路面和路口的ROI是世界坐标系的，所以需要经过一个坐标系转换，仿射变换矩阵就是上述的velodyne_trans。