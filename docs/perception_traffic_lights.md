# Apollo 2.0 感知模块：交通信号灯感知阅读笔记

本文档结合代码详细地解释感知模块中交通信号灯感知的流程与功能，也可以官方参考文档([Apollo 2.0 Traffic Light Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/traffic_light.md))

## 1. 信号灯感知: Traffic Light Perception

交通信号灯模块主要是提供精确的信号灯识别，通常信号灯有三种状态：

- Red/红灯
- Green/绿灯
- Yellow/黄灯

但是在实际生活中，有可能没有检测到信号灯(未知状态)，也有可能信号灯检测到了但是坏了(黑色状态)，为了充分考虑到所有的状态，在代码Apollo中一共设置了5种状态：

- Red/红灯
- Green/绿灯
- Yellow/黄灯
- Black/黑色
- Unknown/未知

在车辆形式的过程中，高精地图模块HD Map需要重复查询前方是否存在信号灯。当然HD Map还需要与定位模块配合使用。信号灯可以使用4个角点来描述，高精地图查询得到的信号灯结果(HD Map中查询到信号灯每个点有xyz三维构成)，需要将信号灯坐标从3D的世界坐标系转换到2D的图像坐标系进行进一步的信号灯状态检测。

之前的Apollo版本使用的是单摄像头解决方案，而且摄像头只有固定的视野域，不能看到所有的信号灯(前方和两侧)。该方案存在很大的限制因素，主要有：

- 必须保证感知域在100米以上
- 交通信号灯的高度或交叉口的宽度差异很大

在Apollo 2.0中，采用了双摄像头扩大车辆的感知域。

- 长焦摄像头(telephoto camera)，焦距为25mm，主要是拍摄前方较远距离的路况。使用长焦摄像头拍摄到的信号灯尺度大，比较清晰，容易被算法检测到。但是该类摄像头也有一定的缺陷，长焦相机能拍到较远处的信号灯，但是信号灯距离车辆过近亦或者信号灯偏向车辆两侧，长焦相机很大可能捕获不到信号灯图像。
- 广角摄像头(wide-range camera)，焦距为6mm，解决长焦相机无法捕获近距离与两侧信号灯的问题。

在实际检测过程中，使用哪个摄像头有信号灯投影决定。下图是长焦相机与广角相机拍摄的实际图片。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/telephoto_camera.jpg)

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/wide_range_camera.jpg)

信号灯感知可以分为两个阶段：

- 预处理阶段 Preprocess(选择合适的camera使用)
	- 相机选择与信号灯投影 Camera selection && Traffic light projection：选择合适的相机
	- 图像与信号灯缓存同步 Image and cached lights sync： 信号同步到缓存
- 处理节点 Process(使用上述选定的camera，获取图像，检测信号灯状态)
	- 整流 Rectify：提供准确的信号灯标定框
	- 识别 Recognize：识别每个标定框对应的信号灯状态
	- 修正 Revise：参考时间序列进行信号灯状态修正

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/framework_traffic_lights.png)

## 2. 信号灯预处理: Traffice Light Preprocess

预处理阶段主要的工作是为信号灯处理阶段Traffic Light Process选择合适的相机。针对每个路况下车辆的定位与姿态，查询高精地图HD Map得到该路况下的信号灯情况(是否存在，存在的话位置多少)，然后将3D世界坐标系下的信号灯坐标分别投影到长焦和广角摄像头下的2D图像坐标系。接着根据投影过后新的信号灯是否全部暴露在摄像头下(映射信号灯标定框在图像内部)，来选择合适的摄像头。最终将摄像头id，采集到的真实路况图像，投影过后信号灯坐标等信息发布给Process子节点进行信号灯的状态识别。

预处理阶段不需要对每一帧图像进行信号灯查询，因为信号灯状态变化是低频的，持续时间相对较长，而且计算资源受限。通常情况下两个摄像头的图像同时到达，但是一般只处理一个摄像头的图像，因此相机的选择是很有必要的。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_preprocess.png)

上图中虚线框为实际回调函数及其间接调用函数逻辑。在代码中，两类回调函数将会调用SubCameraImage函数进行处理，而处理过程主要分3步，分别为：相机选择，缓存同步和信息校验。

### 2.1 相机选择 CameraSelection

选择使用合适的摄像头，不接触具体的图像信息。

在图像坐标系中，HD Map查询得到的交通信号灯结果可以表示为<灯id，标定框>的一个组合，标定框由边界上的4个点组成，每个点坐标为(x,y,z)，而映射到摄像头中的2D图像坐标系，每个点坐标为(x,y)。那么交通信号灯singal info可以以数学的方式表示为(只要给定车辆位置，可以通过高精地图HD Map查询4个点的世界坐标系)：

```c++
signal info:
id {
  id: "xxx"
}
boundary {
  point { x: ...  y: ...  z: ...  }
  point { x: ...  y: ...  z: ...  }
  point { x: ...  y: ...  z: ...  }
  point { x: ...  y: ...  z: ...  }
}
```

相机选择阶段输入通过ROS topic订阅得到，包括：

- 长焦相机topic：/apollo/sensor/camera/traffic/image_long
- 广角相机topic：/apollo/sensor/camera/traffic/image_short
- 定位信息topic，/tf
- 高精地图查询API

分析信号灯预处理子节点的回调函数，两个函数对别对应长焦与广角相机的图像预处理：

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_preprocessor_subnode.cc 
void TLPreprocessorSubnode::SubLongFocusCamera(const sensor_msgs::Image &msg)
void TLPreprocessorSubnode::SubShortFocusCamera(const sensor_msgs::Image &msg)
``` 

可以看到两个订阅的topic产生的输入是ROS自带sensor_msgs的Image类型，查看[官方文档](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)。可知Image类包含如下信息：

```c++
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#
uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
```

从文档中得知，对ROS的sensor_msgs::Image访问，可以通过height，width和data来获取图像数据。代码中需要将ROS的sensor_msgs类型的Image转换成opencv的Image类，该类在包含的关键数据为：

```c++
/// file in apollo/modules/perception/traffic_light/base/image.h
class Image {
 public:
  bool Init(const double &ts, const CameraId &device_id, const cv::Mat &mat);
  bool Init(const double &ts, const CameraId &device_id, boost::shared_ptr<const sensor_msgs::Image> image_data);
  bool GenerateMat();
 private:
  double timestamp_ = 0.0;                  // Image's timestamp
  CameraId camera_id_ = CameraId::UNKNOWN;  // camera's id
  cv::Mat mat_;                             // Image's data
  boost::shared_ptr<const sensor_msgs::Image> image_data_;
  ...
};
typedef std::shared_ptr<Image> ImageSharedPtr;
```

可以看到自定义的Image类封装了时间戳ts，相机device_id以及ROS sensor_msgs::Image，通过使用GenerateMat将ROS的Image转化为opencv的Mat，保存在成员变量mat_中。

同时还存在的相关数据结构有LightRegion, LightStatus, Light, ImageLights，可以进一步分析这些数据结构

```c++
/// file in apollo/modules/perception/traffic_light/base/light.h
struct LightRegion {
  // roi is marked by map & projection, it may be too large or not accuracy.
  cv::Rect projection_roi;
  // rectified_roi is the region marked by Rectifier, it should be accuracy
  cv::Rect rectified_roi;
  ...
};

struct LightStatus {
  // Traffic light color status.
  TLColor color = UNKNOWN_COLOR;
  ...
};

struct Light {
  apollo::hdmap::Signal info;  //  Light info in the map.
  LightRegion region;          //  Light region on the image.
  LightStatus status;          //  Light Status.
};
typedef std::shared_ptr<Light> LightPtr;
typedef std::vector<LightPtr> LightPtrs;

// file in apollo/modules/perception/traffic_light/base/image_lights.h
struct ImageLights {
  std::shared_ptr<Image> image;
  CarPose pose;
  std::shared_ptr<LightPtrs> lights;
  // record the lights outside the lights.
  std::shared_ptr<LightPtrs> lights_outside_image;
  CameraId camera_id = UNKNOWN;
  ...
  size_t num_signals = 0;
};
typedef std::shared_ptr<ImageLights> ImageLightsPtr;
```

从上面数据结构看到，LightRegion包含整流前后的信号灯标定框(坐标系映射过后的2D图像坐标系信号灯坐标)，LightStatus包含灯状态，Light封装了上述两部分，同时包含了原始的HDmap查询信息(原始的3D世界坐标系信号灯坐标)。LightPtr为单个Light的智能指针，LightPtrs则是若干Light智能指针的向量集合。而最终的ImageLights包含了图像信息，信号灯信息，汽车位置信息以及各类时间戳。


相机选择阶段的基本流程为：

1. 如果上一次查询距离当前时间很近(小于一个阈值，配置文件设定为0.2s，低频率调用)，则跳过本次相机选择，沿用上一次相机id对应的配置。

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_preprocessor_subnode.cc
void TLPreprocessorSubnode::CameraSelection(double ts) {
  const double current_ts = TimeUtil::GetCurrentTime();
  if (last_query_tf_ts_ > 0.0 && current_ts - last_query_tf_ts_ < query_tf_inverval_seconds_) {
    AINFO << "skip current tf msg, img_ts: " << GLOG_TIMESTAMP(ts);
    return;
  }
  ...
}

/// file in apollo/modules/perception/model/traffic_light/subnodes.config
float_params {
  name: "query_tf_inverval_seconds"
  value: 0.2
}
```

2. 获取车辆位置信息(使用ROS的tf，可以参考wiki的[tf_tutorials](http://wiki.ros.org/tf/Tutorials#Learning_tf))，同时根据车辆信息查询高精地图，获取信号灯等信息.(该部分与定位模块&&高精地图模块相关)。使用定位信息和高精地图查询到的信号灯信息，并对综合信息进行缓存并且将高精地图产生的3D世界坐标洗映射到2D图像坐标系(缓存信息包含：摄像头id，Hdmap得到的图像，信号灯信息等等)。

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_preprocessor_subnode.cc
void TLPreprocessorSubnode::CameraSelection(double ts) {
  // get car pose and traffic light signals
  CarPose pose;
  std::vector<Signal> signals;
  if (!GetSignals(ts, &pose, &signals)) {
    return;
  }
  // cache light projection
  if (!preprocessor_.CacheLightsProjections(pose, signals, ts)) {
    ...
  } else {
    ...
  }
  last_query_tf_ts_ = current_ts;
}

/// file in apollo/modules/perception/traffic_light/preprocessor/tl_preprocessor.cc
bool TLPreprocessor::CacheLightsProjections(const CarPose &pose, const std::vector<Signal> &signals, const double timestamp){
  // lights projection info. to be added in cached array
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  // default select long focus camera
  image_lights->camera_id = LONG_FOCUS;
  image_lights->timestamp = timestamp;
  image_lights->pose = pose;
  image_lights->is_pose_valid = true;
  // calculate lights_on_image and lights_outside_image each camera
  ProjectLights(pose, signals, static_cast<CameraId>(cam_id), lights_on_image[cam_id].get(), lights_outside_image[cam_id].get());
  // set camera id
  SelectImage(pose, lights_on_image, lights_outside_image, &(image_lights->camera_id));
  cached_lights_.push_back(image_lights);
}
```

从上述代码可以看到信号缓存过程，其实是收集来自高精地图的信号灯信息，然后一并将这些信号灯的东西与其他的相机id，时间戳ts，汽车姿态信息pose等一并进行缓存，方便接下去利用过往的信息进行信号灯状态矫正。在代码中，允许最大缓存信号灯信息数量为max_cached_lights_size=100，该阶段当缓冲队列溢出时，删除队列头最早的缓存信息

上面的代码ProjectLights函数负责对高精地图查询结果signals(vector)进行坐标系变换，并且得到映射后的信号灯2D坐标，判断哪些信号灯在2个摄像头的图像区域以外，哪些信号灯在图像区域内。如果某信号灯经过坐标系变换后在长焦摄像头图像内，那么就可以考虑使用长焦摄像头进行实际路况下的图像采集，采集到的图像中很大可能可以捕获到该信号灯。ProjectLights函数最终得到的结果存储在lights_on_image(vector)和lights_outside_image(vector)，每个向量里面都保存了原始signal以及变换后的2D坐标signal。

E.g. 如果signal A坐标系映射后标定框在长焦摄像头下但不在广角摄像头下，那么可以将signal A保存在lights_on_image[0]下，同时复制一份保存在lights_outside_image[1]，0号索引代表长焦摄像头，1号索引代表广角摄像头。

E.g. 如果signal B坐标系映射后标定框同时存在长焦摄像头和广角摄像头下，那么可以将signal B保存在lights_on_image[0]下，同时复制一份保存在lights_on_image[1]，0号索引代表长焦摄像头，1号索引代表广角摄像头。

```c++
void TLPreprocessor::SelectImage(const CarPose &pose,
                                 const LightsArray &lights_on_image_array,
                                 const LightsArray &lights_outside_image_array,
                                 CameraId *selection) {
  *selection = static_cast<CameraId>(kShortFocusIdx);
  // check from long focus to short focus
  for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
    // Step 1
    if (!lights_outside_image_array[cam_id]->empty()) {
      continue;
    }
    // Step 2
    bool ok = true;
    // find the short focus camera without range check
    if (cam_id != kShortFocusIdx) {
      for (const LightPtr &light : *(lights_on_image_array[cam_id])) {
        if (IsOnBorder(cv::Size(projection_image_cols_, projection_image_rows_), light->region.projection_roi,
                       image_border_size[cam_id])) {
          ok = false;
          break;
        }
      }
    }
    if (ok) {
      *selection = static_cast<CameraId>(cam_id);
      break;
    }
  }
```

上述为SelectImage函数，利用前面的ProjectLights函数对各个信号灯是否在长焦相机和广角相机中检测结果(lights_on_image和lights_outside_image)，选择合适的camera。该函数的很容易理解，主要工作如下，遍历2个摄像头所对应的lights_on_image和lights_outside_image保存的ImageLights：

a) 如果该摄像头的lights_outside_image不为空，即存在某些信号灯映射过后不会暴露在该摄像头下，那么放弃这个摄像头。因为按理说每次都只取一个摄像头的图像进行处理，所以依赖一个摄像头不能处理全部信号灯，放弃该摄像头。

b) 经过a)步骤的处理，可以得到若干摄像头，这些摄像头存在一个共性：都能看到映射过后的所有信号灯。接来下就需要从中选择一个摄像头使用，默认是用广角摄像头(短焦)，如果长焦也能看到所有的信号灯，那么就需要给定一个判断前提：如果信号灯的标定框都在图像有效区域(代码中使用的投影图像大小为1080p/1920x1080，默认图像四周100以内的像素块为边界区，不能被使用，即真正的有效区域为[100:980,100:1820]。如果信号灯标定框落在这个区域之外，则视为无效处理)，则可以选择长焦摄像头，否则选择短焦摄像头。

### 2.2 信号灯缓存同步

在相机选择与信号灯缓存映射过程中，对于车辆不同位置查询高精地图产生的signals会进行一个相机选择，选择合适的能保证看到所有信号灯的相机。本应该就可以直接发布对应的摄像头id，Process子节点根据对应的摄像头id进行信号灯状态检测。但是现实存在一个问题，CameraSelection是低频调用的(0.2s执行一次)，每次执行回调函数的时候CameraSelection不一定都会被执行，也就是说回调过程不一定会执行相机选择！这种情况下如何确定下阶段需要使用的摄像头？这就导致不执行CameraSlection函数的回调过程需要进行一次确认，确认通过才能发布信息，否则本次回调不发布信息，确认需要用到的数据就是历次相机选择过程保存的缓存信息。(本次的回调和缓存信息比对时，可能时间戳很相近但是camera id可能会不一样，也可能是camera id一样但时间戳差异很大，也可能没有缓存记录，这种情况下就不能确定当前时刻该使用哪个摄像头。)

因此信号灯缓存同步阶段的任务其实是一个check，针对那些没有执行CameraSelection函数的回调过程，根据每次产生的时间戳ts和相机id对，去和CameraSelection过程中缓存的ImageLights进行比对，

(a) 如果缓存队列中的100个记录和当前的记录时间戳ts很相近但是camera id不一样; 也可能是camera id一样但时间戳ts差异很大; 也可能根本没有缓存记录，这些情况下就不能确定当前时刻该使用哪个摄像头，本次回调不发布信息。

(b) 如果缓存队列中的100个记录和当前的记录camera id相同，并且时间戳差异很小，则该camera可以作为Process阶段被使用，发布camera信息

针对这个确认过程，我们只有HD Map查询得到的3D信号灯世界坐标信息，汽车姿态以及本次回调对应的camera id。由于没有执行CameraSelection，因此既不知道这些信号灯的2D图像坐标系信息，也不知道最终使用哪个摄像头。所以需要通过SyncImage查询缓存来确定最终使用的camera。

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_preprocessor_subnode.cc
void TLPreprocessorSubnode::SubCameraImage(boost::shared_ptr<const sensor_msgs::Image> msg, CameraId camera_id) {
  if (!preprocessor_.SyncImage(image, &image_lights, &should_pub)) {
    ...
  }
}

/// file in apollo/modules/perception/traffic_light/preprocessor/tl_preprocessor.cc
bool TLPreprocessor::SyncImage(const ImageSharedPtr &image, ImageLightsPtr *image_lights, bool *should_pub) {
  // case 1: 近期没有接收到信号灯信号(E.g. 直行道路无信号灯，则该阶段不发布信息，也就不需要做信号灯检测)
  if (fabs(image_ts - last_no_signals_ts_) < no_signals_interval_seconds_) {}
  // case 2: 时间戳异常，找不到匹配的缓存记录
  else if (image_ts < cached_lights_.front()->timestamp) {}
  else if (image_ts > cached_lights_.back()->timestamp) {}
  // case 3: 找不到时间戳差异较小的缓存记录
  else if (!find_loc) {}
}
```

总结一下SynImage失败的原因(本次回调不发布信息)：
- 没有/tf(汽车定位信号)，因此也不具有signals信号
- 时间戳漂移
- Image未被选择，找不到匹配的camera或者找不到时间戳差异较小的缓存记录

### 2.3 信息校验

最后注意一个问题，当前时刻虽然利用camera id和时间戳ts找到了合适的camera，并不能立即结束Preprocess阶段，这个时候还有一个工作，就是再次check，保证此刻由高精地图查询到的signals里面的信号灯都暴露在这个摄像头内。代码中调用VerifyLightsProjection函数进行二次验证，实际上是简介再次调用ProjectLights验证。Why check again？因为SynImage仅仅根据时间戳ts和相机id查询缓存，如果匹配失败则无所谓，不会发布任何信息直接return false；如果匹配成功则会产生候选相机，但是这个过程并不会去验证信号灯signals映射后是否会全部暴露在这个候选相机下(本次回调没有执行CameraSelection的情况下)，所以就需要再次验证信号灯情况。使用简洁的关系图表示如下：

- 本次回调执行CameraSelection(低频调用，少数情况下执行)
  - 信号灯映射ProjectLight，判断signals中信号灯映射过后是否暴露在2个摄像头下
  - 相机选择SelectImage，根据信号灯映射坐标是否暴露在摄像头下，产生候选摄像头
  - 同步缓存
  - 缓存查询SynImage(无所谓)，查询结果就是缓存中的最后一条记录
  - 二次映射验证VerifyLightsProjection(无所谓)，验证结果就是缓存中的最后一条记录
- 本次回调不执行CameraSelection(大多数回调可能不执行)
  - 缓存查询SynImage(必须要)，匹配缓存，找到合适的记录
  - 二次映射验证VerifyLightsProjection(必须要)，验证本次signals映射坐标

当二次验证也通过时，就可以发布信息给Process阶段进行后续处理。

```c++
/// file in apollo/modules/perception/traffic_light/preprocessor/tl_preprocessor.cc
void TLPreprocessorSubnode::SubCameraImage(boost::shared_ptr<const sensor_msgs::Image> msg, CameraId camera_id) {
  ...
  AddDataAndPublishEvent(image_lights, camera_id, image->ts())
}

bool TLPreprocessorSubnode::AddDataAndPublishEvent(
    const std::shared_ptr<ImageLights> &data, const CameraId &camera_id,
    double timestamp) {
  // add data down-stream
  std::string device_str = kCameraIdToStr.at(camera_id);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_str, &key)) {
    return false;
  }
  if (!preprocessing_data_->Add(key, data)) {  //TLPreprocessingData *preprocessing_data_
    data->image.reset();
    return false;
  }
  // pub events
  for (size_t i = 0; i < this->pub_meta_events_.size(); ++i) {
    const EventMeta &event_meta = this->pub_meta_events_[i];
    Event event;
    event.event_id = event_meta.event_id;
    event.reserve = device_str;
    event.timestamp = timestamp;
    this->event_manager_->Publish(event);
  }
  return true;
}
```

代码显示信息发布过程是人为的写入共享数据类队列中TLPreprocessingData，而event_manager做信息记录。

## 3. 信号灯处理: Traffic Light Process

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_process.png)

在信号灯预处理Preprocess子节点完成工作并发布信息时，信号灯处理阶段Process节点将会开启正常处理流程。具体的思路是：某一时刻，在预处理Preprocess阶段利用车辆定位和姿态信息，查询高精地图得到当前location和pose下面的信号灯信息signals，经过相机选择与信号灯映射，将高精地图3D世界坐标系中的信号灯坐标映射到摄像头2D图像坐标系中信号灯坐标。在Process阶段，就需要利用这些坐标，配合采集到的真实图像， 进行信号灯的状态检测。

上图是信号灯处理流程图，从代码层面来看，处理阶段并不是采用ROS topic的消息订阅机制，而是采用共享数据类存储的方法进行输入输出提取与存储(具体说明请参考[DAG运行](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception_software_arch.md/#DAG运行))。处理阶段工作比较简单，主要是使用预处理阶段建议的摄像头，以及由高精地图查询得到信号灯在该摄像头下2D图像坐标系中的标定框project_roi，匹配真实路况图像(摄像头提取)，获取真实图像下信号灯的标定框，进行整流，识别与校验。最终得到各个信号灯的状态信息。

### 3.1 整流器 Rectifier

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/rectifier.jpg)

如上图，在高精地图3D坐标到摄像头2D坐标转换的过程中，得到2D信号灯坐标实际并不准确，存在一定程度的坐标偏差。图中蓝色标定框是3D到2D转换过程中得到的映射结果，实际上应该对应右边绿灯信号灯的标定框，奈何存在较大不可控的误差。如何解决映射过程中坐标的漂移？在代码中采用了ROI生成+信号灯重新检测的方案：

1. 首先，将映射的标定框中心点固定，宽高以一定的scale扩大(Apollo中由crop_scale控制，默认2.5倍)得到更大的ROI区域(图中黄色区域)，这个ROI区域非常大概率的包含了信号灯
2. 然后，使用检测网络(ResNet-RFCN)从这个ROI区域中检测得到信号灯的真实坐标。由于ROI相对较小，检测速度比较快。

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_proc_subnode.cc
bool TLProcSubnode::ProcEvent(const Event &event) {
  // get data from sharedata which pulish by Preprocess SubNode
  ...
  // verify image_lights from cameras
  ...
  // using rectifier to rectify the region.
  const double before_rectify_ts = TimeUtil::GetCurrentTime();
  if (!rectifier_->Rectify(*(image_lights->image), rectify_option, (image_lights->lights).get())) {
    return false;
  }
  ...
}
```

整流环节输入包含检测使用的摄像头id，映射过后的2D图像坐标系信号灯标定框，摄像头拍摄到的真实路况图像。这些图像都是从共享数据ShareData容器类中获得，而发布这些信息的就是ProProcess SubNode，上小节可以得知。

```c++
/// file in apollo/modules/perception/traffic_light/rectify/unity_rectify.cc
bool UnityRectify::Rectify(const Image &image, const RectifyOption &option, std::vector<LightPtr> *lights) {
  for (auto &light : lights_ref) {
    cv::Rect cbox;
    crop_->GetCropBox(ros_image.size(), lights_ref, &cbox);
    ...
  }
}

/// file in apollo/modules/perception/traffic_light/rectify/cropbox.cc
void CropBox::GetCropBox(const cv::Size &size, const std::vector<LightPtr> &lights, cv::Rect *cropbox) {
  float center_x = (xr + xl) / 2;
  float center_y = (yb + yt) / 2;
  float resize_width = (xr - xl) * crop_scale_;
  float resize_height = (yb - yt) * crop_scale_;
  float resize = std::max(resize_width, resize_height);
  resize_width = resize_height = (resize < min_crop_size_) ? min_crop_size_ : resize;
  // clamp
  xl = center_x - resize_width / 2;
  xl = (xl < 0) ? 0 : xl;
  yt = center_y - resize_height / 2;
  yt = (yt < 0) ? 0 : yt;
  xr = center_x + resize_width / 2;
  xr = (xr >= cols) ? cols - 1 : xr;
  yb = center_y + resize_width / 2;
  yb = (yb >= rows) ? rows - 1 : yb;
}
```

从上面代码就可以不难理解Apollo对于每个映射过后的信号灯坐标漂移问题处理方式，最终使用crop_scale扩大标定框得到ROI区域，接下来就是使用检测网络对该区域进行信号灯检测。

```c++
/// file in apollo/modules/perception/traffic_light/rectify/unity_rectify.cc
bool UnityRectify::Rectify(const Image &image, const RectifyOption &option, std::vector<LightPtr> *lights) {
  for (auto &light : lights_ref) {
    // CropBox via crop_scale
    ...
    // dection form croped box
    detect_->SetCropBox(cbox);
    detect_->Perform(ros_image, &detected_bboxes);
    for (size_t j = 0; j < detected_bboxes.size(); ++j) {
      cv::Rect &region = detected_bboxes[j]->region.rectified_roi;
      float score = detected_bboxes[j]->region.detect_score;
      region.x += cbox.x;
      region.y += cbox.y;
    }
    // bbox select
    select_->Select(ros_image, lights_ref, detected_bboxes, &selected_bboxes);
  }
}
```

以上代码清晰的反映了ROI检测的流程，对CropBox产生的ROI(cbox)进行网络的检测，得到结果detected_bboxes包含了ROI内所有的信号灯信息，然后对每个信号灯标定框进行筛选，去除落在ROI以外的bbox，然后矫正标定框，加上cbox.x和y映射到整体Image的x和y。最后对所有的hdmap_bboxes和detect_boxes进行匹配，得到整流过后的标定框。另外一些细节问题：

- 检测网络使用的是基于ResNet50的RFCN，每张输入的图片经过预处理，长边保证大于等一个阈值(由crop_min_size控制，默认300)，网络输出分类信息和标定框信息，分类信息包含四类(DetectionClassId)：

	- UNKNOWN_CLASS: 未知类型，即不存在信号灯
	- VERTICAL_CLASS: 竖型信号灯
	- QUADRATE_CLASS: 方型信号灯
	- HORIZONTAL_CLASS: 横型信号灯

```c++
/// file in apollo/modules/perception/traffic_light/rectify/select.cc
void GaussianSelect::Select(const cv::Mat &ros_image,
                            const std::vector<LightPtr> &hdmap_bboxes,
                            const std::vector<LightPtr> &refined_bboxes,
                            std::vector<LightPtr> *selected_bboxes) {
  // find bbox with max area in refined_bboxes
  auto max_area_refined_bbox =
      std::max_element(refined_bboxes.begin(), refined_bboxes.end(),
                       [](const LightPtr lhs, const LightPtr rhs) {
                         return lhs->region.rectified_roi.area() <
                                rhs->region.rectified_roi.area();
                       });

  // cv::Mat_<int> cost_matrix(hdmap_bboxes.size(), refined_bboxes.size());
  std::vector<std::vector<double> > score_matrix( hdmap_bboxes.size(), std::vector<double>(refined_bboxes.size(), 0));
  for (size_t row = 0; row < hdmap_bboxes.size(); ++row) {
    cv::Point2f center_hd = GetCenter(hdmap_bboxes[row]->region.rectified_roi);
    auto width_hd = hdmap_bboxes[row]->region.rectified_roi.width;
    for (size_t col = 0; col < refined_bboxes.size(); ++col) {
      cv::Point2f center_refine = GetCenter(refined_bboxes[col]->region.rectified_roi);
      auto width_refine = refined_bboxes[col]->region.rectified_roi.width;

      // use gaussian score as metrics of distance and width
      // distance_score:
      //    larger distance => 0.
      //    smaller distance => 1
      double distance_score = static_cast<double>(Get2dGaussianScore(center_hd, center_refine, 100, 100));
      // width score:
      //   larger width diff => 0
      //   smaller width diff => 1
      double width_score = static_cast<double>(Get1dGaussianScore(width_hd, width_refine, 100));

      // normalized area score
      // larger area => 1
      double area_score = 1.0 * refined_bboxes[col]->region.rectified_roi.area() / (*max_area_refined_bbox)->region.rectified_roi.area();

      // when numerator=1， denominator is very small,
      // converting to int might reduce to same value, here uses 1000
      // + 0.05 to prevent score = 0
      // score * weight h
      score_matrix[row][col] = (0.05 + 0.4 * refined_bboxes[col]->region.detect_score +  0.2 * distance_score + 0.2 * width_score + 0.2 * area_score);
    }
  }
```

- Select函数是hdmap_bbox与detect_bbox标定框选择与匹配函数。由信号灯映射坐标经过wrap，crop得到的ROI可能包含多个信号灯情况，而实际上每个映射坐标只对应ROI中的某一个bbox，如何进行匹配？在代码中可以看到匹配方法。hdmap给出了m个映射bbox，实际ROI中RFCN检测到了n个bbox，那么可以构建一个mxn的矩阵，矩阵中每个元素代表两两相似度评估。评估标准是：hdmap_bbox和detect_bbox中心和宽度距离信息、detect_bbox检测评分score、面积信息。Get2dGaussianScore和Get1dGaussianScore是标准的二维/一维高斯函数。

	- hdmap_bbox和detect_bbox中心和宽度月相近，评分越大，占比0.2, 0.2
	- detect_bbox的RFCN检测评分score越大，评分越大，占比0.4
	- detect_bbox面积越大，评分越大，占比0.2

- 当然还有一种情况，如果RFCN在ROI实际检测过程中没有检测到任何的信号灯dectect_bbox为空，则就把该信号灯状态直接置为unknown，跳过后续的recognizer和reviser阶段，因为没必要再检测。

### 3.1 识别器 Recognizer

识别器主要工作是对整流器Rectifier得到的整流映射bbox(上述与hdmap_bbox匹配的detect_bbox)，识别过程比较简单，针对竖型，横型(不使用)，方型采用不同的检测网络，本质区别在于输入大小不一致。竖型接受的输入大小为96x32，使用白天模型；方型接受的输入大小为64x64，使用夜晚模型。

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_proc_subnode.cc
bool TLProcSubnode::ProcEvent(const Event &event) {
  // get data from sharedata which pulish by Preprocess SubNode
  ...
  // verify image_lights from cameras
  ...
  // using rectifier to rectify the region.
  ...
  // recognize_status
  const double before_recognization_ts = TimeUtil::GetCurrentTime();
  if (!recognizer_->RecognizeStatus(*(image_lights->image), RecognizeOption(), (image_lights->lights).get())) {
    return false;
  }
  ...
}

/// file in apollo/modules/perception/traffic_light/recognizer/unity_recognize.cc 
bool UnityRecognize::RecognizeStatus(const Image &image, const RecognizeOption &option, std::vector<LightPtr> *lights) {
  for (LightPtr light : *lights) {
    if (light->region.is_detected) {
      candidate[0] = light;
      if (light->region.detect_class_id == QUADRATE_CLASS) {
        classify_night_->Perform(ros_image, &candidate);
      } else if (light->region.detect_class_id == VERTICAL_CLASS) {
        classify_day_->Perform(ros_image, &candidate);
      } else {
        AINFO << "Not support yet!";
      }
    } else {
      light->status.color = UNKNOWN_COLOR;
      light->status.confidence = 0;
    }
  }
  return true;
}
```

另外一个细节，识别过程中白天和夜晚两个模型使用的是比较简单的CNN(5convs+2fc)，输出的信号灯四类状态：
	
- BLACK: 黑色，故障或者正在转换
- RED: 红色
- YELLOW: 黄色
- GREEN: 绿色

为了确保较高的可靠性，最终的检测结果对应的score必须大于一定的阈值才算有效，否则置为BLACK。阈值由classify_threshold控制，默认为0.5

### 3.3 校验器 Reviser

由识别器识别得到的信号灯状态并不一定准确，有时候识别得到信号灯是黑色BLACK，但是实际可能是由红色RED向绿色GREEN装换过程。所以校验器的工作就是利用缓存信息对当前信号灯状态做一个校验。具体的方法比较简单：如果Recognizer是别的是红灯RED或者绿灯GREED，直接存储缓存；如果识别到了黑色BLACK和未知UNKNOWN，则需要从缓存中提取近期时间段内的信号灯状态数据进行比对，进行下一步校验。

另外一个先验，信号灯变换总是有一定规律的，一般是红RED-绿GREED-黄YELLOW，反复循环。如果缓存中上时刻监测到的是红色RED，而现在时刻确实YELLOW，这是不可能的，所以刷新状态变为红色RED，直到检测到信号灯状态变为绿色GREEN才能前进。

```c++
/// file in apollo/modules/perception/traffic_light/onboard/tl_proc_subnode.cc
bool TLProcSubnode::ProcEvent(const Event &event) {
  // get data from sharedata which pulish by Preprocess SubNode
  ...
  // verify image_lights from cameras
  ...
  // using rectifier to rectify the region.
  ...
  // recognize_status
  ...
  // using rectifier to rectify the region.
  if (!rectifier_->Rectify(*(image_lights->image), rectify_option, (image_lights->lights).get())) {
    return false;
  }
}

/// file in apollo/modules/perception/traffic_light/reviser/color_decision.cc
bool ColorReviser::Revise(const ReviseOption &option, std::vector<LightPtr> *lights) {
  for (size_t i = 0; i < lights_ref.size(); ++i) {
  	std::string id = lights_ref[i]->info.id().id();
    switch (lights_ref[i]->status.color) {
      default:
      case BLACK:
      case UNKNOWN_COLOR:
        if (color_map_.find(id) != color_map_.end() && option.ts > 0 && option.ts - time_map_[id] < blink_time_) {
          lights_ref[i]->status.color = color_map_[id];
        } 
        break;
      case YELLOW:
        // if YELLOW appears after RED, revise it to RED
        if (color_map_.find(id) != color_map_.end() && option.ts > 0 && color_map_.at(id) == RED) {
          lights_ref[i]->status.color = color_map_.at(id);
          color_map_[id] = RED;
          time_map_[id] = option.ts;
          break;
        }
      case RED:
      case GREEN:
        if (time_map_.size() > 10) {
          color_map_.clear();
          time_map_.clear();
        }
        color_map_[id] = lights_ref[i]->status.color;
        time_map_[id] = option.ts;
        break;
    }
  }
  return true;
}
```

具体的校验规则很简单：

- 如果检测到的是BLACK或者UNKNOWN，查询缓存，缓存中由上时刻该信号灯(id必须一致)信息，并且两个时间差小与一个阈值(由blink_time_控制，默认1.5s)，则使用缓存中的状态作为当前信号灯状态。
- 如果检测到是YELLOW，查看缓存，如果缓存中上时刻状态是RED，那么刷新变成RED；否则保留YELLOW状态
- 如果是RED或者GREEN，则刷新缓存，缓存中对应id的信号灯状态修改为当前状态。当然如果缓存时间过长，那么已经过掉路口了，对应的信号灯就没意义了，这时候可以清楚缓存，重新保存。
