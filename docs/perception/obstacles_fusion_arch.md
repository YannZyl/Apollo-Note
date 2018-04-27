# 融合感知模块: Fusion Obstacles Perception

本文档主要对融合子节点FusionSubnode任务做一个介绍。前面介绍道其实Lidar激光雷达感知和Radar雷达感知模块都具有物体识别、跟踪的功能。而本文档所谓的融合其实是对上述两个模块发布的自定义数据LidarObjectData(DataType: SensorObjects)、RadarObjectData(DataType: SensorObjects)做一个整理的梳理，将两种设备检测得到的SensorObjects合并在一起，放在PerceptionObstacles里面。

PerceptionObstacles是protobuf形式，ROS消息订阅机制的发布，因为需要序列化操作，protobuf很好的提供了这个功能。另外一点是，FusionSubnode子节点输入时从共享数据容器LidarObjectData和RadarObjectData中自主提取数据，输出是以ROS消息发布机制发布，发布函数为`apollo::common::adapter::AdapterManager::PublishPerceptionObstacles`

下面可以看一下PerceptionObstacles的数据结构:

```proto
// file apollo/in modules/perception/proto/perception_obstacle.proto
message PerceptionObstacle {
  optional int32 id = 1;  // obstacle ID.
  optional Point position = 2;  // obstacle position in the world coordinate
                                // system.
  optional double theta = 3;  // heading in the world coordinate system.
  optional Point velocity = 4;  // obstacle velocity.

  // Size of obstacle bounding box.
  optional double length = 5;  // obstacle length.
  optional double width = 6;  // obstacle width.
  optional double height = 7;  // obstacle height.

  repeated Point polygon_point = 8;  // obstacle corner points.
  // duration of an obstacle since detection in s.
  optional double tracking_time = 9;

  enum Type {
    UNKNOWN = 0;
    UNKNOWN_MOVABLE = 1;
    UNKNOWN_UNMOVABLE = 2;
    PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behaviour.
    BICYCLE = 4;  // bike, motor bike
    VEHICLE = 5;  // Passenger car or truck.
  };
  optional Type type = 10;  // obstacle type
  optional double timestamp = 11;  // GPS time in seconds.

  // Just for offline debuging, onboard will not fill this field.
  // Format like : [x0, y0, z0, x1, y1, z1...]
  repeated double point_cloud = 12 [packed = true];

  optional double confidence = 13 [default = 1.0];
  enum ConfidenceType {
      CONFIDENCE_UNKNOWN = 0;
      CONFIDENCE_CNN = 1;
      CONFIDENCE_RADAR = 2;
  };
  optional ConfidenceType confidence_type = 14 [default = CONFIDENCE_CNN];
}

message PerceptionObstacles {
  repeated PerceptionObstacle perception_obstacle = 1;  // An array of obstacles
  optional apollo.common.Header header = 2;  // Header
  optional apollo.common.ErrorCode error_code = 3 [default = OK];
  optional LaneMarkers lane_marker = 4;
  optional CIPVInfo cipv_info = 5; // closest in path vehicle
}
```

可以看到上面protobuf里面定义的内容其实与SensorObjects类的内容很相似，相当于对SensorObjects对了一个序列化的protobuf模板。里面包含了检测物体的所有信息。

FusionSubnode所做的工作与Lidar、Radar两个节点的跟踪很相似。这里大致的看一下Fusion节点的流程。

```c++
/// file in apollo/modules/perception/obstacle/onboard/fusion_subnode.cc
Status FusionSubnode::ProcEvents() {
  for (auto event_meta : sub_meta_events_) {
  	// A. get lidar and radar's data
    std::vector<Event> events;
    if (!SubscribeEvents(event_meta, &events)) {
    }
    // B. process: fusion
    Process(event_meta, events);
    // C. public obstacle message
    PerceptionObstacles obstacles;
    if (GeneratePbMsg(&obstacles)) {
      common::adapter::AdapterManager::PublishPerceptionObstacles(obstacles);
    }
  }
}
```

从代码中可以看到主要的分为：

1. 从接受列表中获取fusion节点的接收event数据
2. 根据上述event数据，从LidarObjectData和RadarObjectData两个共享数据容器中获取数据，并处理
3. 发布信息

下面我们将介绍第二部分Process函数流程

## Step 1. 收集Lidar和Radar发布的数据

```c++
/// file in apollo/modules/perception/obstacle/onboard/fusion_subnode.cc
Status FusionSubnode::Process(const EventMeta &event_meta, const std::vector<Event> &events) {
  // A. collect data
  std::vector<SensorObjects> sensor_objs;
  if (!BuildSensorObjs(events, &sensor_objs)) {
  }
  // B. fusion
  if (!fusion_->Fuse(sensor_objs, &objects_)) {
  }
}

bool FusionSubnode::BuildSensorObjs(
    const std::vector<Event> &events,
    std::vector<SensorObjects> *multi_sensor_objs) {
  for (auto event : events) {
    std::shared_ptr<SensorObjects> sensor_objects;
    if (!GetSharedData(event, &sensor_objects)) {
      return false;
    }
    // Make sure timestamp and type are filled.
    sensor_objects->timestamp = event.timestamp;
    if (event.event_id == lidar_event_id_) {
      sensor_objects->sensor_type = SensorType::VELODYNE_64;
    } else if (event.event_id == radar_event_id_) {
      sensor_objects->sensor_type = SensorType::RADAR;
    } else if (event.event_id == camera_event_id_) {
      sensor_objects->sensor_type = SensorType::CAMERA;
    } else {
      AERROR << "Event id is not supported. event:" << event.to_string();
      return false;
    }
    sensor_objects->sensor_id = GetSensorType(sensor_objects->sensor_type);
    multi_sensor_objs->push_back(*sensor_objects);
  }
  return true;
}
```

从上述代码可以看出，使用`FusionSubnode::BuildSensorObjs`函数，将Lidar和Radar感知发布的数据，全部收集在一起，放在`multi_sensor_objs`向量中，并注明了每个数据的来源(感知设备)，是Lidar还是Radar(Camera为Apollo2.5内容，正在开发过程中，这里暂时不做介绍)。

## Step 2. Process数据处理

```c++
/// file in apollo/modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc
bool ProbabilisticFusion::Fuse(
    const std::vector<SensorObjects> &multi_sensor_objects,
    std::vector<std::shared_ptr<Object>> *fused_objects) {

  std::vector<PbfSensorFramePtr> frames;
  double fusion_time = 0;
  {
    sensor_data_rw_mutex_.lock();
    bool need_to_fusion = false;
    // 1. collect sensor objects data
    for (size_t i = 0; i < multi_sensor_objects.size(); ++i) {
      auto sensor_type = multi_sensor_objects[i].sensor_type;
      if (GetSensorType(multi_sensor_objects[i].sensor_type) == publish_sensor_id_) {
        sensor_manager_->AddSensorMeasurements(multi_sensor_objects[i]);
      } else if (started_) {
        sensor_manager_->AddSensorMeasurements(multi_sensor_objects[i]);
      }
    }

    // 2.query related sensor frames for fusion
    sensor_manager_->GetLatestFrames(fusion_time, &frames);
    sensor_data_rw_mutex_.unlock();
  }

  {
    fusion_mutex_.lock();
    // 3.peform fusion on related frames
    for (size_t i = 0; i < frames.size(); ++i) {
      FuseFrame(frames[i]);
    }
    // 4.collect results
    CollectFusedObjects(fusion_time, fused_objects);
    fusion_mutex_.unlock();
  }
}
```

从代码中可以看到，整体的融合分4步骤，分别为：

1. 数据收集，这里收集函数`AddSensorMeasurements`做法是，对每个Object根据其SensorType(velodyne_64/lidar,radar)，时间戳timestamp信息，存入二级map: `std::map<string sensortype, map<int64, SensorObjects>> sensors_`里面，可以通过`sensors_[sensor_id]`获取每个感知器的数据，可以通过`sensors_[sensor_id][timestamp]`获取这个感知器在该时间戳下(某时刻)的检测到的物体集合。

2. 查询某一个时刻最近的物体集合，集体我们可以查看代码：

```c++
/// file in apollo/modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc
void PbfSensorManager::GetLatestFrames(const double time_stamp,
                                       std::vector<PbfSensorFramePtr> *frames) {
  frames->clear();
  for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
    PbfSensor *sensor = it->second;
    PbfSensorFramePtr frame = sensor->QueryLatestFrame(time_stamp);
    if (frame != nullptr) {
      frames->push_back(frame);
    }
  }
  std::sort(frames->begin(), frames->end(),
            [](const PbfSensorFramePtr &p1, const PbfSensorFramePtr &p2) {
              return p1->timestamp < p2->timestamp;
            });
}

/// file in apollo/modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor.cc
PbfSensorFramePtr PbfSensor::QueryLatestFrame(const double time_stamp) {
  PbfSensorFramePtr latest_frame = nullptr;
  for (size_t i = 0; i < frames_.size(); ++i) {
    if (frames_[i]->timestamp > latest_query_timestamp_ &&
        frames_[i]->timestamp <= time_stamp) {
      latest_frame = frames_[i];
      latest_query_timestamp_ = frames_[i]->timestamp;
    }
  }
  return latest_frame;
}
```

所以你这个阶段的工作很明显，针对每个感知器`for (auto it = sensors_.begin(); it != sensors_.end(); ++it)`，从每个感知器的存储数据(上述步骤1中的`sensors_[sensor_id]`)。取数据的规则是：找到二级map中该感知器与查询时间戳`time_stamp`最近的数据，这个数据的时间戳必须小于融合时间`time_stamp`。

如果有两个感知设备lidar和radar，那么最后的结果frames中就有两个元素，分别是lidar与radar在时刻time_stamp附近的检测物体集合。

3. 数据融合。

```c++
for (size_t i = 0; i < frames.size(); ++i) {
  FuseFrame(frames[i]);
}
```

从代码中，我们可以看到这里所谓的数据融合，其实并不是真正意义上的，融合radar和lidar检测到的物体，而是分别融合两个感知器的数据，也就是这两类检测数据是完全独立的，Fusion的工作只是将两类数据写入到一个容器中，lidar和radar数据(SensorObjects)之间无关联。

```c++
/// file in apollo/modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc
void ProbabilisticFusion::FuseFrame(const PbfSensorFramePtr &frame) {
  std::vector<std::shared_ptr<PbfSensorObject>> &objects = frame->objects;
  std::vector<std::shared_ptr<PbfSensorObject>> background_objects;
  std::vector<std::shared_ptr<PbfSensorObject>> foreground_objects;
  // classify foregroud objects and background objects
  DecomposeFrameObjects(objects, &foreground_objects, &background_objects);
  // fuse foreground objects
  FuseForegroundObjects(&foreground_objects, ref_point, frame->sensor_type, frame->sensor_id, frame->timestamp);
  // remove lost tracks
  track_manager_->RemoveLostTracks();
}

void ProbabilisticFusion::FuseForegroundObjects(
    std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
    Eigen::Vector3d ref_point, const SensorType &sensor_type,
    const std::string &sensor_id, double timestamp) {

  matcher_->Match(tracks, *foreground_objects, options, &assignments,
                  &unassigned_tracks, &unassigned_objects,
                  &track2measurements_dist, &measurement2tracks_dist);

  UpdateAssignedTracks(&tracks, *foreground_objects, assignments,
                       track2measurements_dist);

  UpdateUnassignedTracks(&tracks, unassigned_tracks, track2measurements_dist,
                         sensor_type, sensor_id, timestamp);

  if (FLAGS_use_navigation_mode) {
    if (is_camera(sensor_type)) {
      CreateNewTracks(*foreground_objects, unassigned_objects);
    }
  } else {
    CreateNewTracks(*foreground_objects, unassigned_objects);
  }
}
```

从代码中我们可以看到，其实这里的`ProbabilisticFusion::FuseForegroundObjects`几乎和Lidar Perception中的track过程几乎是一样的，包含：

- 匈牙利匹配(关联举矩阵association_mat计算、子图划分与二分图匹配)
- 更新跟踪列表(包括已适配的TrackedObject和未适配的TrackedObject)，加入新的Object到跟踪列表

4. 收集并存储被跟踪的物体，放入`std::vector<std::shared_ptr<Object>> *fused_objects`作为返回。

## Step 3. 数据发布

对上述的`std::vector<std::shared_ptr<Object>> *fused_objects`写入PerceptionObstacles，最终通过`apollo::common::adapter::AdapterManager::PublishPerceptionObstacles`函数发布到ROS。如果需要利用到这些数据，可以在你自己的函数中通过`AdapterManager::AddPerceptionObstaclesCallback`添加该topic的回调函数mfunc来实现数据的调用。

E.g. 

```c++
void Myclass::Myclass(){
  AdapterManager::AddPerceptionObstaclesCallback(&Myclass::Myfunc, this);
}

void Myclass::Myfunc(const PerceptionObstacles& perception_obstacles) {
  // process
}
```