# 基于卷积神经网络分割

>高精地图 ROI过滤之后，Apollo得到已过滤、只包含属于ROI内的点云，大部分背景障碍物，如路侧的建筑物、树木等均被移除，ROI内的点云被传递到分割模块。分割模块检测和划分前景障碍物，例如汽车，卡车，自行车和行人。

该阶段的输入数据来自高精地图ROI过滤器过滤得到的电云数据，最终输出对应于ROI中的障碍物对象数据集。该阶段包含4个子过程：

- 通道特征提取
- 基于卷积神经网络的障碍物预测
- 障碍物集群
- 后期处理

## Step 1. 通道特征提取

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

### (1) 将点云实际的xy坐标映射到输入矩阵HxW平面坐标，并且筛选点云高度

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

### (2) 计算单元格中的8类数据

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

## Step 2. 基于卷积神经网络的障碍物预测

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

## Step 3. 障碍物聚类

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

### (1) 并查集建立步骤1: 建立新的并查集--DisjointSetMakeSet

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

### (2) 并查集建立步骤2: 产生不相交集合(树)--Traverse

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

### (3) 并查集建立步骤3: 不相交集合(树)合并--DisjointSetUnion

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

### (4) 合并完以后，每棵树就代表一类物体，做记录

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

## Step 4. 后期处理

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