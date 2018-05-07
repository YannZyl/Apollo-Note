# Apollo 2.0/2.5 预测模块代码层次结构说明

本文档将从代码层面讲解Apollo Prediction预测模块的工作，其实对比Preception模块与Prediction模块，后者更像是前者的一个子节点，换句话说：Prediction模块就只有一个线程工作。这个线程主要的任务就是接受Preception模块FusionSubnode发布的融合数据，其中包含了障碍物的位置、方向、尺寸、类别、速度与加速度等等信息，来预测障碍物未来时刻的运动轨迹。

我们先从`Prediction::Init`函数了解一下这个模块的输入输出，以及用到的管理器等信息，

```c++
Status Prediction::Init() {
  // Initialization of all managers
  AdapterManager::Init(adapter_conf_);
  ContainerManager::instance()->Init(adapter_conf_);
  EvaluatorManager::instance()->Init(prediction_conf_);
  PredictorManager::instance()->Init(prediction_conf_);


  // Set localization callback function
  AdapterManager::AddCallback(&Prediction::OnLocalization, this);
  // Set planning callback function
  AdapterManager::AddPlanningCallback(&Prediction::OnPlanning, this);
  // Set perception obstacle callback function
  AdapterManager::AddPerceptionObstaclesCallback(&Prediction::RunOnce, this);

  return Status::OK();
}
```

从代码中我们可以看到，Prediction确实很像Perception模块的子节点，通过ROS的topic订阅机制，来完成一些任务，这里主要可以得知两类类型的数据：

1. 第一类为管理器类，做的工作主要是负责一些数据或者api函数的管理，比如`AdapterManager`类的作用是管理各个topic对应的发布订阅函数、回调添加函数等。

2. 第二类为输入输出定义，这里可看到Prediction有三类输入，分别是：车辆位置信息(Localization topic)、高精地图与路径规划信息(Planning topic)，视觉感知障碍物信息(PerceptionObstacles topic)。