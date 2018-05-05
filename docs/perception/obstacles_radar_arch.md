# 雷达感知模块: Radar Obstacles Perception

本文档将逐步讲解Apollo在障碍物检测过程中，雷达感知的软硬件以及检测流程。

## 硬件简介

Apollo 2.0中雷达感知模块使用的是Continental AG的ARS408-21雷达

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/radar_pic.jpg)

Webpage for ARS408-21[https://www.continental-automotive.com/Landing-Pages/Industrial-Sensors/Products/ARS-408-21](https://www.continental-automotive.com/Landing-Pages/Industrial-Sensors/Products/ARS-408-21)

## 检测流程介绍

其实可以从Apollo官方文档看到，相比Radar感知，Apollo更注重的是Lidar激光雷达感知。本文档将简单地介绍一下雷达物体检测的流程。雷达感知中，输入的数据有三种，分别为：

- 物理雷达数据
- 定位数据
- 高精地图

换句话说，Radar Perception的处理函数触发是以ROS topic订阅机制完成的，而输出的数据保存在`RadarObjectData`这个共享容器中。输入输出可以从对应的初始化函数中找到

```c++
/// file in apollo/modules/perception/obstacle/onboard/radar_process_subnode.cc
bool RadarProcessSubnode::InitInternal() {
  // init share data RadarObjectData and HD map
  if (!InitFrameDependence()) {
  }
  // init radar detector
  if (!InitAlgorithmPlugin()) {
  }
  // add callback: OnRadar, main process function 
  AdapterManager::AddContiRadarCallback(&RadarProcessSubnode::OnRadar, this);
  // add callback: OnLocalization get car localtion
  AdapterManager::AddLocalizationCallback(&RadarProcessSubnode::OnLocalization,this);
}
```

可以明显看到Radar感知输入与输出的信息。在Apollo中Radar感知的流程与Lidar感知相比简化了一些，主要分为：

1. 预处理工作
2. 高精地图ROI过滤器
3. 物体检测
4. 物体跟踪

### Step 1. 预处理工作

第一步预处理工作主要是矫正雷达数据的时间戳、获取雷达到世界坐标系的变换矩阵

```c++
/// file in apollo/modules/perception/obstacle/onboard/radar_process_subnode.cc
void RadarProcessSubnode::OnRadar(const ContiRadar &radar_obs) {
  // 0. correct radar timestamp
  ...
  // 1. get radar pose
  std::shared_ptr<Matrix4d> velodyne2world_pose = std::make_shared<Matrix4d>();
  if (!FLAGS_use_navigation_mode &&
      !GetVelodyneTrans(timestamp, velodyne2world_pose.get())) {
    ..
  }
  std::shared_ptr<Matrix4d> radar2world_pose = std::make_shared<Matrix4d>();
  std::shared_ptr<Matrix4d> radar2car_pose = std::make_shared<Matrix4d>();

  if (!FLAGS_use_navigation_mode) {
    *radar2world_pose = *velodyne2world_pose * short_camera_extrinsic_ * radar_extrinsic_;
  } else {
    CalibrationConfigManager *config_manager =
        Singleton<CalibrationConfigManager>::get();
    CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
    // Eigen::Matrix4d camera_to_car = calibrator->get_camera_extrinsics();
    *radar2car_pose = radar_extrinsic_;
  }
}
```

从上述代码中可以看到，Apollo中获取雷达到世界坐标系的变换矩阵依旧依赖于Velodyne激光雷达的转换矩阵，变换公式为：

```c++ 
*radar2world_pose = *velodyne2world_pose * short_camera_extrinsic_ * radar_extrinsic_;
```

即雷达相对于世界坐标系变换矩阵 = 激光雷达相对于世界坐标系变换矩阵 \* 相机相对于激光雷达变换矩阵 \* 雷达相对于相机变换矩阵。

可以看到Apollo中大部分的坐标系变换都是以激光雷达为基础的，`velodyne2world_pose`被频繁计算，这里可以得到比较重要的信息：激光雷达在整体硬件系统中占据着相当重要的地位，而激光雷达的位置是相对来说比较精确地，因此其他硬件设备位置都是以激光雷达为参考系。


### Step 2. 高精地图ROI过滤器

这部分其实与激光雷达Lidar感知的高精地图ROI过滤器比较类似，本节Radar的ROI过滤器是Lidar的简化版本，主要工作我们可以看代码。

```c++
/// file in apollo/modules/perception/obstacle/onboard/radar_process_subnode.cc
void RadarProcessSubnode::OnRadar(const ContiRadar &radar_obs) {
  // 0. correct radar timestamp
  ...
  // 1. get radar pose
  ...
  std::vector<PolygonDType> map_polygons;
  RadarDetectorOptions options;
  // Current Localiztion, radar postion.
  if (!FLAGS_use_navigation_mode) {
    PointD position;
    position.x = (*radar2world_pose)(0, 3);
    position.y = (*radar2world_pose)(1, 3);
    position.z = (*radar2world_pose)(2, 3);
    // 2. Get map polygons.
    HdmapStructPtr hdmap(new HdmapStruct);
    if (FLAGS_enable_hdmap_input && hdmap_input_ &&
        !hdmap_input_->GetROI(position, FLAGS_front_radar_forward_distance, &hdmap)) {
      // NOTE: if call hdmap failed, using empty map_polygons.
    }

    if (roi_filter_ != nullptr) {
      roi_filter_->MergeHdmapStructToPolygons(hdmap, &map_polygons);
    }

    // 3. get car car_linear_speed
    if (!GetCarLinearSpeed(timestamp, &(options.car_linear_speed))) {
      return;
    }
  }
}
```

从代码中我们看到，第一步也是一致的，首先计算雷达在世界坐标系中的位置(可以知道其实雷达在世界坐标系，就是radar2world_pose的平移成分)，所以调用`!hdmap_input_->GetROI(position, FLAGS_front_radar_forward_distance, &hdmap)`，position是雷达的世界坐标系坐标，FLAGS_front_radar_forward_distance是查询的范围，在激光雷达中查询的范围是60m，而在雷达感知中查询的范围为120m。

第二步还是和Lidar感知一致，通过HD map查询到的信息，获取路面ROI区域的多变形轮廓。由函数`roi_filter_->MergeHdmapStructToPolygons(hdmap, &map_polygons);`完成。

第三部就是计算当前时刻车辆的速度，这个计算方式相对来说也比较简单，Apollo中采用的并不是实时的速度计算，而是采用另一中比较简单地计算方式。选择这个方案的原因是：所有使用ROS topic订阅与回调机制的处理方式，都是有延迟的。例如雷达发布原始消息的时间戳是ts1，而真正进行 `RadarProcessSubnode::OnRadar`回调时具有一定的延迟，时间戳变成ts2，所以不能使用ts2去实时查询汽车速度，而应该使用ts1去查询。查询历史数据最好的方式就是**缓存与匹配**。

具体的查询流程为：

- 首先，Apollo中注册了Localization topic，所以ROS会定时的发布车辆位置与速度坐标等信息(发布的频率取决于定位模块Msg的发布频率)，在雷达Radar模块中维持一个容量为40的消息队列`localization_buffer_`。每当定位模块发布消息的时候，执行`RadarProcessSubnode::OnLocalization`回调函数，将定位消息存入`localization_buffer_`中。

- 当需要计算车辆速度时，根据雷达原始消息发布时的时间戳`timestamp`，去查询这个缓存队列，找到第一个比tempstamp晚(大)的缓存数据，选择这个时刻的速度作为汽车的速度。

```c++
bool RadarProcessSubnode::GetCarLinearSpeed(double timestamp,  Eigen::Vector3f *car_linear_speed) {
  MutexLock lock(&mutex_);
  if (localization_buffer_.empty()) {
    return false;
  }
  if (localization_buffer_.front().first - 0.1 > timestamp) {
    return false;
  }
  if (localization_buffer_.back().first + 0.1 < timestamp) {
    return false;
  }
  // loop to find nearest
  double distance = 1e9;
  int idx = localization_buffer_.size() - 1;
  for (; idx >= 0; --idx) {
    double temp_distance = fabs(timestamp - localization_buffer_[idx].first);
    if (temp_distance >= distance) {
      break;
    }
    distance = temp_distance;
  }

  idx = std::max(0, idx);
  auto velocity = localization_buffer_[idx].second.pose().linear_velocity();
  (*car_linear_speed)[0] = velocity.x();
  (*car_linear_speed)[1] = velocity.y();
  (*car_linear_speed)[2] = velocity.z();
  return true;
}
```

另外，前面三个if是做的校验，确保缓冲队列不为空、确保缓存队列中最早的数据不比雷达信号发布时的时间晚、确保缓存队列中最晚的数据不比雷达信号发布时的时间早。总而言之就是雷达信号的时间戳timestamp一定要在缓存数据的时间戳范围内，过早或过晚都将查询不到速度。

### Step 3. 物体检测

雷达Radar感知相对于激光雷达Lidar感知更加简单，是因为Apollo使用Radar就可以分辨出自行车、行人等类别。而激光雷达Lidar需要手动去CNN分割识别障碍物，MiniBox构建标定框等；同时Radar也可以自行完成跟踪，不需要使用匈牙利算法对Object进行匹配等。

```c++
/// file in apollo/modules/perception/obstacle/onboard/radar_process_subnode.cc
void RadarProcessSubnode::OnRadar(const ContiRadar &radar_obs) {
  // 0. correct radar timestamp
  ...
  // 1. get radar pose
  ...
  if (!FLAGS_use_navigation_mode) {
    // 2. Get map polygons.
    ..
    // 3. get car car_linear_speed
    ..
  }
  // 4. Call RadarDetector::detect.
  bool result = radar_detector_->Detect(radar_obs_proto, map_polygons, options, &radar_objects->objects);
  // publish data
  PublishDataAndEvent(timestamp, radar_objects);
}
```

从上面代码看到，所有的检测与跟踪由`radar_detector_->Detect`函数完成，我们可以简单地将检测过程分为几步：

**1. 预处理步骤**

预处理步骤包括设置车辆速度、坐标系变换矩阵等工作。

```c++
bool ModestRadarDetector::Detect(
    const ContiRadar &raw_obstacles,
    const std::vector<PolygonDType> &map_polygons,
    const RadarDetectorOptions &options,
    std::vector<std::shared_ptr<Object>> *objects) {
  if (options.radar2world_pose == nullptr) {
    return false;
  } else {
    radar_pose = *(options.radar2world_pose);
  }
  Eigen::Vector2d main_velocity;
  if (FLAGS_use_navigation_mode) {
    main_velocity[0] = 0;
    main_velocity[1] = 0;
  } else {
    main_velocity[0] = options.car_linear_speed[0];
    main_velocity[1] = options.car_linear_speed[1];
  }
}
```

自动导航模式下车速设为0，以自身为参考系；手动模式下，车速就是上述`RadarProcessSubnode::GetCarLinearSpeed`检测到的速度。

**2. 物体构建**

```c++
bool ModestRadarDetector::Detect(
    const ContiRadar &raw_obstacles,
    const std::vector<PolygonDType> &map_polygons,
    const RadarDetectorOptions &options,
    std::vector<std::shared_ptr<Object>> *objects) {
  // preparation
  ...
  SensorObjects radar_objects;
  object_builder_.Build(raw_obstacles, radar_pose, main_velocity, &radar_objects);
  radar_objects.timestamp =
      static_cast<double>(raw_obstacles.header().timestamp_sec());
  radar_objects.sensor_type = SensorType::RADAR;
}
```

该过程由`object_builder_.Build`函数完成，主要的工作就是原始障碍物跟踪与记录、每个障碍物是否是背景、障碍物速度、障碍物重心位置、障碍物方向、障碍物偏航角等信息，这些信息在Lidar感知中由CNN分割、HM跟踪与卡尔曼滤波阶段完成。下面我们就研究一下障碍物构建的主要工作流程：

Step1. 对每个障碍物，根据其id去查找上时刻的跟踪列表`continuous_ids_`，列表记录了上时刻<id,tracktime>的跟踪对。如果查询成功，跟踪时间加1；否者新加入跟踪列表，跟踪时间设置为1。

Step2. 判断障碍物是否是背景，判断条件为：

- 如果跟踪时间小于阈值`delay_frames_`(默认4次)，障碍物出现次数太少，那么暂时设为背景
- 如果Radar检测到为汽车`ContiObjectType::CONTI_CAR`和卡车`ContiObjectType::CONTI_TRUCK`
	- 如果Radar置信度小于阈值`params.probexist_vehicle`(默认0.9)，那么设置为背景
	- 否则如果障碍物在车前longitude和车侧lateral两侧位置的速度与距离均方差小于阈值，则设为背景
	- 否则如果跟踪时间(出现次数)小于`delay_frames_`，设为背景
- 如果Radar检测到为行人`ContiObjectType::CONTI_PEDESTRIAN`
	- 如果Radar置信度小于阈值`params.probexist_pedestrian`(默认0.25)，那么设置为背景
	- 否则如果障碍物在车前longitude和车侧lateral两侧位置的速度与距离均方差小于阈值，则设为背景
- 如果Radar检测到为摩托车`ContiObjectType::CONTI_MOTOCYCLE`或者自行车`ContiObjectType::CONTI_BICYCLE`
	- 如果Radar置信度小于阈值`params.probexist_bicycle`(默认0.25)，那么设置为背景
	- 否则如果障碍物在车前longitude和车侧lateral两侧位置的速度与距离均方差小于阈值，则设为背景
- 如果Radar检测到为`ContiObjectType::CONTI_POINT`，`ContiObjectType::CONTI_WIDE`或者`ContiObjectType::CONTI_UNKNOWN`
	- 如果Radar置信度小于阈值`params.probexist_unknown`(默认0.99)，那么设置为背景
	- 否则如果障碍物在车前longitude和车侧lateral两侧位置的速度与距离均方差小于阈值，则设为背景
	- 否则如果跟踪时间(出现次数)小于`delay_frames_`，设为背景
- 如果Radar检测到的障碍物跟踪状态为已删除`ContiMeasState::CONTI_DELETED`, 已预测完毕`ContiMeasState::CONTI_PREDICTED`以及将要删除`ContiMeasState::CONTI_DELETED_FOR`，则设为背景

上述判定规则在文件`apollo/modules/perception/obstacle/radar/modest/conti_radar_util.cc`中`ContiRadarUtil::IsFp`函数定义。

Step3. 计算世界坐标系中障碍物位置和速度

```c++
bool ObjectBuilder::Build(const ContiRadar &raw_obstacles,
                          const Eigen::Matrix4d &radar_pose,
                          const Eigen::Vector2d &main_velocity,
                          SensorObjects *radar_objects) {
    Eigen::Matrix<double, 4, 1> location_r;
    Eigen::Matrix<double, 4, 1> location_w;
    location_r << raw_obstacles.contiobs(i).longitude_dist(),
        raw_obstacles.contiobs(i).lateral_dist(), 0.0, 1.0;
    location_w = radar_pose * location_r;  				// transform obstacle's localization to world frame
    Eigen::Vector3d point;
    point = location_w.topLeftCorner(3, 1);
    object_ptr->center = point;       					// obstacle's center
    object_ptr->anchor_point = object_ptr->center; 		// onstacle's anchor_point/graycenter
    Eigen::Matrix<double, 3, 1> velocity_r;
    Eigen::Matrix<double, 3, 1> velocity_w;
    velocity_r << raw_obstacles.contiobs(i).longitude_vel(),	
        raw_obstacles.contiobs(i).lateral_vel(), 0.0;
    velocity_w = radar_pose.topLeftCorner(3, 3) * velocity_r;  // transform obstacle's velocity to world frame
}
```

同时计算障碍物在两个方向上的绝对速度，只要与车辆速度矢量相加即可

```c++
//  calculate the absolute velodity
object_ptr->velocity(0) = velocity_w[0] + main_velocity(0);
object_ptr->velocity(1) = velocity_w[1] + main_velocity(1);
object_ptr->velocity(2) = 0;
```

通过车辆和障碍物的绝对速度以及他们的方向，进一步进行校验，如果障碍物和无人汽车运动方向夹角在[pi/4,3\*pi/4], [-3\*pi/4,-pi/4]则标记为背景。当运动方向夹角在45°~135°/-45°~-135°之间时，可以认为障碍物和车辆比较安全，因为夹角偏大，暂时不会碰撞。夹角过小时，可能是同向或者方向行驶，需要考虑到安全问题，以免发生碰撞。(这里有值得更加合理的解释)

Step4. 设置障碍物各类属性

```c++
object_ptr->length = 1.0;
object_ptr->width = 1.0;
object_ptr->height = 1.0;
object_ptr->type = ObjectType::UNKNOWN;
object_ptr->score_type = ScoreType::SCORE_RADAR;
object_ptr->score = static_cast<float>(raw_obstacles.contiobs(i).probexist());
double local_theta = raw_obstacles.contiobs(i).oritation_angle() / 180.0 * M_PI;
Eigen::Vector3f direction = Eigen::Vector3f(cos(local_theta), sin(local_theta), 0); 
direction = radar_pose.topLeftCorner(3, 3).cast<float>() * direction; // transform direction to world frame
object_ptr->direction = direction.cast<double>();
object_ptr->theta = std::atan2(direction(1), direction(0));
```

不过正如[博客](https://read01.com/mE3M8Ny.html#.WtRvL2pubDc)所说，这里很奇怪，Radar直接将物体长宽高设置为1，而且type类别设置为UNKNOWN，但是在Step2背景校验中`contiobs.obstacle_class()`来获得雷达监测到的类型，这里有些矛盾，很可能是Apollo对雷达准确度可够可靠。

```c++
RadarUtil::MockRadarPolygon(point, object_ptr->length, object_ptr->width,
                                theta, &(object_ptr->polygon));
```

最后一步是根据中心点center与长宽求解，物体框的坐标。

**3. ROI过滤**

```c++
bool ModestRadarDetector::Detect(
    const ContiRadar &raw_obstacles,
    const std::vector<PolygonDType> &map_polygons,
    const RadarDetectorOptions &options,
    std::vector<std::shared_ptr<Object>> *objects) {
  // roi filter
  auto &filter_objects = radar_objects.objects;
  RoiFilter(map_polygons, &filter_objects);
  // treatment
  radar_tracker_->Process(radar_objects);
  CollectRadarResult(objects);
}
```

Radar感知的ROI过滤没有Lidar感知的复杂，这里没有位图存储与扫面线算法，这里使用更简单的算法，前提还是一样如下图。可以很明显得到一个结论，

```c++
template <typename PointT>
  static bool IsXyPointIn2dXyPolygon(const PointT &point, const PolygonDType &polygon) {
    bool in_poly = false;
    double x1, x2, y1, y2;
    int nr_poly_points = static_cast<int>(polygon.points.size());
    double xold = polygon.points[nr_poly_points - 1].x;
    double yold = polygon.points[nr_poly_points - 1].y;
    for (int i = 0; i < nr_poly_points; i++) {
      double xnew = polygon.points[i].x;
      double ynew = polygon.points[i].y;
      if (xnew > xold) {
        x1 = xold;
        x2 = xnew;
        y1 = yold;
        y2 = ynew;
      } else {
        x1 = xnew;
        x2 = xold;
        y1 = ynew;
        y2 = yold;
      }
      if ((x1 < point.x) == (point.x <= x2) &&
          (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
        in_poly = !in_poly;
      }
      xold = xnew;
      yold = ynew;
    }
    return in_poly;
  }

  template <typename PointT>
  static bool IsXyPointInHdmap(const PointT &p, const std::vector<PolygonDType> &polygons) {
    bool in_flag = false;
    for (std::size_t j = 0; j < polygons.size(); j++) {
      if (IsXyPointIn2dXyPolygon<PointT>(p, polygons[j])) {
        in_flag = true;
        break;
      }
    }
    return in_flag;
  }
```

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/radar_roi.png)

从图中右半部分我们可以很明显的看到：

1. 使用`(point.y-y1)*(x2-x1) ? (y2-y1)*(point.x-x1)`可以很清楚的判断障碍物坐标在(xold,xnew)直线上方还是下方，具体原理如上图右半部分。
2. 如上图a-d，无论障碍物如果在polygon内部还是外部，必然可以找到偶数对边，使得边和障碍物在同一x或者y维度。

- 如果障碍物在polygon内部，必然有一对直线，满足障碍物坐标分别在直线上方和下方，这时候代码中in_poly必然会经过奇数取反(在直线下方时不满足条件，需要取反)，所以最终in_poly=True；
- 如果障碍物在polygon外部。那么每一对直线，要么在障碍物下方，要么在障碍物上面，所以in_poly必然经过偶数次取反，最终in_poly依然为False

注意，这只是粗略的计算是否落入多边形内部。参考Lidar感知的算法对物体构建MiniBox多边形，这里没有考虑到遮挡的问题，若存在遮挡，那么背离Radar的一面的polygon必然不会准确，因此无法直接使用这种方法计算。


最后Radar的检测结果将会以自定义的共享数据类型发布，发布的结果其实与Lidar感知相似。障碍物的尺寸、方向、速度等信息。