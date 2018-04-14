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



# 信号灯处理: Traffic Light Process

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_process.png)

在信号灯预处理Preprocess子节点完成工作并发布信息时，信号灯处理阶段Process节点将会开启正常处理流程。具体的思路是：某一时刻，在预处理Preprocess阶段利用车辆定位和姿态信息，查询高精地图得到当前location和pose下面的信号灯信息signals，经过相机选择与信号灯映射，将高精地图3D世界坐标系中的信号灯坐标映射到摄像头2D图像坐标系中信号灯坐标。在Process阶段，就需要利用这些坐标，配合采集到的真实图像， 进行信号灯的状态检测。

上图是信号灯处理流程图，从代码层面来看，处理阶段并不是采用ROS topic的消息订阅机制，而是采用共享数据类存储的方法进行输入输出提取与存储(具体说明请参考[DAG运行](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception_software_arch.md/#DAG运行))。处理阶段工作比较简单，主要是使用预处理阶段建议的摄像头，以及由高精地图查询得到信号灯在该摄像头下2D图像坐标系中的标定框project_roi，匹配真实路况图像(摄像头提取)，获取真实图像下信号灯的标定框，进行整流，识别与校验。最终得到各个信号灯的状态信息。

## 整流器 Rectifier

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

## 识别器 Recognizer

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

## 校验器 Reviser

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
  // revise status
  if (!reviser_->Revise(ReviseOption(event.timestamp),image_lights->lights.get())) {
    ...
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

## 信息发布

当做完最终的校验以后就得到了信号灯的确定状态，最后一步需要将信号灯状态发布出去。

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
  // revise status
  ...
  PublishMessage(image_lights);
}

bool TLProcSubnode::PublishMessage(const std::shared_ptr<ImageLights> &image_lights) {
 
  TrafficLightDetection result;
  AdapterManager::FillTrafficLightDetectionHeader("traffic_light", &result);
  // 1. set timestamp
  auto *header = result.mutable_header();
  uint64_t img_timestamp = static_cast<uint64_t>(image_lights->image->ts() * 1e9);  
  header->set_camera_timestamp(img_timestamp);

  // 2. add traffic light result
  for (const auto &light : *lights) {
    TrafficLight *light_result = result.add_traffic_light();
    light_result->set_id(light->info.id().id());
    light_result->set_confidence(light->status.confidence);
    light_result->set_color(light->status.color);
  }

  // 3. set contain_lights
  result.set_contain_lights(image_lights->num_signals > 0);

  // add info to TrafficLightDebug
  ...

  AdapterManager::PublishTrafficLightDetection(result);
  return true;
}
```

从上面代码我们可以得到如下结论：

1. Traffic Light Process子节点最终发布结果是以ROS消息订阅与发布机制完成的，`AdapterManager::PublishTrafficLightDetection`中注册了各类topic，其中包括TrafficLightDetection类型的topic。

2. 发布的消息内容包含有：时间戳、信号灯id、信号灯状态置信度、信号灯颜色、是否包含信号灯等信息。

```proto
// file in apollo/modules/perception/proto/traffic_light_detection.proto
message TrafficLight {
    enum Color {
        UNKNOWN = 0;
        RED = 1;
        YELLOW = 2;
        GREEN = 3;
        BLACK = 4;
    };
    optional Color color = 1;
    // Traffic light string-ID in the map data.
    optional string id = 2;
    // How confidence about the detected results, between 0 and 1.
    optional double confidence = 3 [default = 1.0];
    // Duration of the traffic light since detected.
    optional double tracking_time = 4;
}

message TrafficLightDetection {
    optional apollo.common.Header header = 2;
    repeated TrafficLight traffic_light = 1;
    optional TrafficLightDebug traffic_light_debug = 3;
    optional bool contain_lights = 4;
}
```