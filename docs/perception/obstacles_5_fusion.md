# 跟踪对象信息融合

在上一步HM对象跟踪步骤中，Apollo对每个时刻检测到的Object与跟踪列表中的TrackedObject进行匈牙利算法的二分图匹配，匹配结果分三类：

1. 如果成功匹配，那么使用卡尔曼滤波器更新信息(重心位置、速度、加速度)等信息
2. 如果失配，缺少对应的TrackedObject，将Object封装成TrackedObject，加入跟踪列表
3. 对于跟踪列表中当前时刻目标缺失的TrackedObject(Object都无法与之匹配)，使用上时刻速度，跟新当前时刻的重心位置与加速度等信息(无法使用卡尔曼滤波更新，缺少观测状态)。对于那些时间过长的丢失的跟踪目标，将他们从跟踪队列中删除。

而在跟踪对象信息融合的阶段，Apollo的主要工作是，给定一个被跟踪的物体序列: (Time_1,TrackedObject_1),(Time_2,TrackedObject_2),...,(Time_n,TrackedObject_n)

每次执行LidarSubNode的回调，都会刷新一遍跟踪列表，那么一个被跟踪物体的信息也将会被刷新(重心位置，速度，加速度，CNN物体分割--前景概率，CNN物体分割--各类别概率)。N次调用都会得到N个概率分布(N个CNN物体分割--前景概率score，N个CNN物体分割--各类物体概率Probs)，我们需要在每次回调的过程中确定物体的类别属性，当然最简单的方法肯定是`argmax(Probs)`。但是CNN分割可能会有噪声，所以最好的办法是将N次结果联合起来进行判断！

跟踪物体的属性可以分为4类：

- UNKNOWN--未知物体
- PEDESTRIAN--行人
- BICYCLE--自行车辆
- VEHICLE--汽车车辆

E.g. 5次跟踪的结果显示某物体的类别/Probs分别为：

|  跟踪时间戳  |  UNKNOWN概率  | PEDESTRIAN概率  |  BICYCLE概率  |  VEHICLE概率 |  Argmax结果 |
|  frame1  |  0.1  |  0.2  |  0.6  |  0.1  |  BICYCLE  |
|  frame2  |  0.1  |  0.1  |  0.7  |  0.1  |  BICYCLE  |
|  frame3  |  0.1  |  0.2  |  0.2  |  0.5  |  VEHICLE  |
|  frame4  |  0.1  |  0.2  |  0.6  |  0.1  |  BICYCLE  |
|  frame5  |  0.1  |  0.2  |  0.6  |  0.1  |  BICYCLE  |

如果直接对每次跟踪使用`argmax(Probs)`直接得到结果，有时候会有误差，上表frame3的时候因为误差结果被认为是汽车，所以需要根据前面的N次跟踪结果一起联合确定物体类别属性。Apollo使用维特比算法求解隐状态概率。

这里做一个简单地描述，具体请参考[维特比Viterbi算法](https://zh.wikipedia.org/wiki/%E7%BB%B4%E7%89%B9%E6%AF%94%E7%AE%97%E6%B3%95):

维特比算法前提是状态链是马尔可夫链，即下一时刻的状态仅仅取决于当前状态。(假设隐状态数量为m，观测状态数量为n)，隐状态分别为s1,s2,s3,....sm; 可观测状态分别为o1,o2,...,on。 则有：

状态转移矩阵P(mxm): P[i,j]代表状态i到状态j转移的概率。$\sum_{j=0}^m P[i,j] = 1$
发射概率矩阵R(mxn): R[i,j]代表隐状态i能被观测到为j现象的概率。 $\sum_{j=0}^n P[i,j] = 1$

现在假设初始时刻的m个隐状态概率为：(s0_1, s0_2, ..., s0_m)，第一时刻观测到的可观察状态为ok，那么如何求第一时刻的隐状态：

1. 上时刻隐状态si，第一时刻隐状态sj的联合概率为：

$$ p(s1_j, s0_i) = p(prv_state=s0_i) * P(sj|si) $$

因此可以得到p(s1_1, s0_i), p(s1_2, s0_i), ..., p(s1_m, s0_i)，也可以得到p(s1_j, s0_1), p(s1_j, s0_2), p(s1_j, s0_3),..., p(s1_j, s0_m)。最终是一个mxm的联合概率矩阵。


2. 同时上时刻隐状态si，第一时刻隐状态sj情况下可观测到观察状态ok的概率为：

$$ p(ok|s1_j, s0_i) = p(s1_j, s0_i) * R(ok|sj) $$

同理可得到p(ok|s1_0, s0_i), p(ok|s1_1, s0_i), p(ok|s1_2, s0_i),..., p(ok|s1_m, s0_i)，也是一个mxm的条件概率矩阵。

若最终的条件概率矩阵中p(ok|s1_jj, s0_ii) 值最大，那么就可以得出结论这个时刻的隐状态为s_jj。

Apollo也采取类似的Viterbi算法做隐状态的修正。

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
  ...
  /// call type fuser
  if (type_fuser_ != nullptr) {
    TypeFuserOptions type_fuser_options;
    type_fuser_options.timestamp = timestamp_;
    if (!type_fuser_->FuseType(type_fuser_options, &(out_sensor_objects->objects))) {
      ...
    }
  }
}

/// file in apollo/modules/perception/common/sequence_type_fuser/sequence_type_fuser.cc
bool SequenceTypeFuser::FuseType(
    const TypeFuserOptions& options,
    std::vector<std::shared_ptr<Object>>* objects) {
  if (options.timestamp > 0.0) {
    sequence_.AddTrackedFrameObjects(*objects, options.timestamp);  // step1. add current TrackedObject into sequence
    std::map<int64_t, std::shared_ptr<Object>> tracked_objects;
    for (auto& object : *objects) {
      if (object->is_background) {                // step2. check whether is background
        object->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
        object->type = ObjectType::UNKNOWN_UNMOVABLE;
        continue;
      }
      const int& track_id = object->track_id;
      // step3. crop sequence to generate a new sequence which length smaller or equal than temporal_window_(20)
      sequence_.GetTrackInTemporalWindow(track_id, &tracked_objects, temporal_window_);
      // step4. CRF to rectify object type use Viterbi algorithm
      if (!FuseWithCCRF(&tracked_objects)) {
        ...
      }
    }
  }
}
```

从上述代码可以看到使用CRF进行隐状态(物体类别)修正，主要步骤为：

1. 将TrackedObject加入到sequence(sequence数据结构为map<int, map<int64_, vecotr<ObjectPtr>>>二级存储模式，sequence[track_id]可以得到对应id的物体所有时刻的序列，sequence[track_id][frame]为frame时刻track_id号物体的)

2. 筛选，对于背景物体直接标记为UNKNOWN::UNMOVABLE类别

3. 新序列生成，sequence[track_id]里面的已存储的记录过多，而修正只需要最近一段时间的序列即可，所以使用`GetTrackInTemporalWindow`函数可以获取近期`temporal_window_`(默认20)以内的所有物体跟踪序列

4. CRF隐状态矫正。使用上述new_sequence配合Viterbi算法进行矫正。

上述的难点就在于Step 4，因此本节从代码分析描述CRF修正的原理。在`FuseWithCCRF`函数中共分两步

- Step1. 状态平滑(物体状态整流)
- Step2. Viterbi算法推离状态

```c++
/// file in apollo/modules/perception/common/sequence_type_fuser/sequence_type_fuser.cc
bool SequenceTypeFuser::FuseWithCCRF(std::map<int64_t, std::shared_ptr<Object>>* tracked_objects) {
  /// Step1. rectify object type with smooth matrices
  fused_oneshot_probs_.resize(tracked_objects->size());
  std::size_t i = 0;
  for (auto& pair : *tracked_objects) {
    std::shared_ptr<Object>& object = pair.second;
    if (!RectifyObjectType(object, &fused_oneshot_probs_[i++])) {
      AERROR << "Failed to fuse one shot probs in sequence.";
      return false;
    }
  }
}

bool SequenceTypeFuser::RectifyObjectType(const std::shared_ptr<Object>& object, Vectord* log_prob) {
  Vectord single_prob;
  fuser_util::FromStdVector(object->type_probs, &single_prob);
  auto iter = smooth_matrices_.find("CNNSegClassifier");

  static const Vectord epsilon = Vectord::Ones() * 1e-6;
  single_prob = iter->second * single_prob + epsilon;
  fuser_util::Normalize(&single_prob);

  double conf = object->score;
  single_prob = conf * single_prob + (1.0 - conf) * confidence_smooth_matrix_ * single_prob;
  fuser_util::ToLog(&single_prob);
  *log_prob += single_prob;
  return true;
}
```

从上述代码可以得到以下结论：

1. 原始CNN分割与后处理得到当前跟踪物体对应4类的概率为`object->type_probs`
2. 原始CNN分割与后处理得到当前跟踪物体前景概率为`conf=object->score`，那么背景的概率为`1-conf`
3. 平滑公式为：
single_prob = iter->second * single_prob + epsilon
single_prob = conf * single_prob + (1.0 - conf) * confidence_smooth_matrix_ * single_prob

iter->second(CNNSegClassifier Matrix)矩阵为：

0.9095 0.0238 0.0190 0.0476
0.3673 0.5672 0.0642 0.0014
0.1314 0.0078 0.7627 0.0980
0.3383 0.0017 0.0091 0.6508

confidence_smooth_matrix_(Confidence)矩阵为：

1.00 0.00 0.00 0.00
0.40 0.60 0.00 0.00
0.40 0.00 0.60 0.00
0.50 0.00 0.00 0.50

----------------------------------------------------------------------

Viterbi算法推理代码如下：

```c++
bool SequenceTypeFuser::FuseWithCCRF(std::map<int64_t, std::shared_ptr<Object>>* tracked_objects) {
  /// rectify object type with smooth matrices
  ...
  /// use Viterbi algorithm to infer the state
  std::size_t length = tracked_objects->size();
  fused_sequence_probs_.resize(length);
  state_back_trace_.resize(length);
  fused_sequence_probs_[0] = fused_oneshot_probs_[0];
  /// add prior knowledge to suppress the sudden-appeared object types.
  fused_sequence_probs_[0] += transition_matrix_.row(0).transpose();
  for (std::size_t i = 1; i < length; ++i) {
    for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {
      double max_prob = -DBL_MAX;
      std::size_t id = 0;
      for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {
        const double prob = fused_sequence_probs_[i - 1](left) +
                            transition_matrix_(left, right) * s_alpha_ +
                            fused_oneshot_probs_[i](right);
        if (prob > max_prob) {
          max_prob = prob;
          id = left;
        }
      }
      fused_sequence_probs_[i](right) = max_prob;
      state_back_trace_[i](right) = id;
    }
  }
  std::shared_ptr<Object> object = tracked_objects->rbegin()->second;
  RecoverFromLogProb(&fused_sequence_probs_.back(), &object->type_probs,
                     &object->type);

  return true;
}
```

上述代码中，`fused_oneshot_probs_`是每个时刻独立的4类概率，经过平滑和log(·)处理。`transition_matrix_`是状态转移矩阵P，维度为4x4，经过log(·)处理，`fused_sequence_probs_`为Viterbi算法推理后的修正状态(也就是真实的隐状态)。

那么根据上时刻的真实的隐状态`fused_sequence_probs_[i-1]`和状态转移矩阵`transition_matrix_`，可以求解开始提到的mxm(4x4)联合状态矩阵，其中矩阵中元素fused_sequence_probs_[i][j]的求解方式为：

p(si_j, si-1_k) = p(prv_state=si-1_k) * P(sj|sk)

对应代码：

```c++
const double prob = fused_sequence_probs_[i - 1](left) +
                    transition_matrix_(left, right) * s_alpha_ +
                    fused_oneshot_probs_[i](right);
```

上述存在几个问题：

- 问题1： 为什么代码用的加法？

因为代码中是将乘法转换到log(·)做运算，上述已提到`transition_matrix_`和`fused_oneshot_probs_`都是经过log(·)处理。那么上述公式等价于：

log p(si_j, si-1_k) = log p(prv_state=si-1_k) + log P(sj|sk)

只要最终将log p(si_j, si-1_k) 经过 exp(·)处理还原真实的概率即可。

- 问题2. `s_alpha_`是什么意思？

有待后续深入研究，这里还不曾研究透。

- 问题3. 为什么代码会额外多乘一个概率p(ok|sj) -- `used_oneshot_probs_[i](right)`

Apollo中对于当前状态的推理就是求解前后两个时刻关联最紧密(上时刻各状态中与当前时刻状态sj变换概率最大的状态si)，本质就是求解开始例子中提到联合概率矩阵。在开始的例子中，我们举例隐状态s共m个。紧接着计算前后两个时刻状态的联合概率矩阵。在这个例子中，隐状态是onehot类型，也就是[0,0,0,si-1=1,0,0,0]，那么当我们的隐状态不是onehot，也就是这种类型[0.1,0.1,0.1,si-1=0.6,0,0,0.1]，那么如何求解联合概率矩阵？

做法也很类似，只需要将原先计算目标：

p(si_j, si-1_k) = p(prv_state=sk) \* P(sj|si)

变成计算目标：

p_new(si_j, si-1_k) = p(prv_state=sk) \* P(sj|si) \* p(current_state=sj)

上述代码中`const double prob`就是联合概率矩阵:p_new(si_j, si-1_k)，最终取最大元素对应的(ii,jj)组，也就是(left，right)组，ii(left)就是当前时刻的隐状态：

```c++
for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {  // time sequence loop
  double max_prob = -DBL_MAX;
  std::size_t id = 0;
  for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {  // previous state loop
    const double prob = fused_sequence_probs_[i - 1](left) +     // each elem p[i,j] in union probability matrix
                        transition_matrix_(left, right) * s_alpha_ +
                        fused_oneshot_probs_[i](right);
    if (prob > max_prob) {   // find the previous state and current hidden state which have maximum value 
      max_prob = prob;
      id = left;
    }
  }
  fused_sequence_probs_[i](right) = max_prob; // update 2 state connected intersity
  state_back_trace_[i](right) = id; // 2 frame's connection
}
```

从上述代码和注释，可以很明显的看到求解的过程，最终可以得到一条由后往前连接的状态链。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/perception_obstacles/perception_obstacles_5_vertibi.png)

Apollo状态转移矩阵为

0.34 0.22 0.33 0.11
0.03 0.90 0.05 0.02
0.03 0.05 0.90 0.02
0.06 0.01 0.03 0.90
