# Apollo 3.0 预测模块代码层次结构说明

## Prediction预测模块总体概述

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

注意：在上述三个回调函数中，第三个`Prediction::RunOnce`很容易理解，目的是从上游感知模块获取障碍物信息，并预测障碍物的未来时刻轨迹。第二个回调函数`Prediction::OnLocalization`在于向定位模块获取当前主车的位置，第一个回调函数`Prediction::OnPlanning`目的是获取上时刻主车的规划路径。**这两个函数对于有什么用处？答案是可以用来做一些基本的交互。 E.g. 障碍物的预测轨迹不太可能与无人车的规划路径重叠，通俗的说障碍物不会突然窜到无车人的前方，将在下文讲到一些简单地交互**


目前来说预测的核心思路是什么？基本可以归纳为4个点：

1. 车辆(Vehicle)大概率会沿着车道中心线行驶

2. 自行车(Bicycle)大概率会沿着车道中心线行驶

3. 未知物体(Unknown)暂时设定平行车道中心线移动

4. 行人(Pedestrain)自由移动

根据上面4个原则，预测模块基本围绕着车道选择在做。

-------------------

## Prediction预测流程介绍

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/prediction/prediction_arch.md)

从上图可以看到总体来说Prediction分为三个阶段，分别由三个Manager管理。

**a. 阶段1--候选车道集合生成**

这个阶段就需要根据当前障碍物的坐标，去查询高精地图，得到障碍物所在的车道以及邻近车道，这些候选车道就是障碍物在未来时刻可能运动的位置。

**b. 阶段2--车道概率评估**

这个阶段就是对1中得到的候选车道进行概率评估，评估的方法是根据障碍物的历史信息，以及候选车道的信息，共同判定未来障碍物在该车道的概率。目前使用的模型以简单的MLP为主。

**c. 阶段3--最终轨迹生成**

这个阶段工作内容是根据2中的概率以及一些基本的交互规则，过滤掉可能性小的车道，留下大概率车道。然后对这些车道进行采样、平滑就可以得到每个障碍物可能的运动轨迹。

可以看一下代码：

```c++
void Prediction::RunOnce(const PerceptionObstacles& perception_obstacles) {

  double start_timestamp = Clock::NowInSeconds();

  // Insert obstacle
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::PERCEPTION_OBSTACLES));
  obstacles_container->Insert(perception_obstacles);

  // Make evaluations
  EvaluatorManager::instance()->Run(perception_obstacles);

  // Make predictions
  PredictorManager::instance()->Run(perception_obstacles);

  auto prediction_obstacles = PredictorManager::instance()->prediction_obstacles();
  prediction_obstacles.set_start_timestamp(start_timestamp);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());

  Publish(&prediction_obstacles);
}
```

从上面的代码，`Prediction::RunOnce`是Predict模块收到Perception模块的ROS消息以后，启动的回调处理函数。上述的流程我们根据他的注释来解释。

1. 将Perception模块检测到的PerceptionObstacles插入到Obstacle的LRU缓存中，同时计算了该障碍物的候选车道集

2. 障碍物LaneSequence(候选车道上采样序列)的概率评估`EvaluatorManager::instance()->Run(perception_obstacles);`

3. LaneSequence的平滑预测轨迹生成`PredictorManager::instance()->Run(perception_obstacles);`

4. 消息发布，其中包括了障碍物的信息，运动状态信息以及每个障碍物对应的多条可能运动轨迹