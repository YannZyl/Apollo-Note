# 预测机制与流程

在介绍完前面ContainerManager管理器，EvaluatorManager管理器和PredictorManager管理器以后。我们就得到了一写障碍物的短时间内的运动轨迹。这一节我们就最后总结一下Prediction模块的预测机制和工作流程。

```c++
void Prediction::RunOnce(const PerceptionObstacles& perception_obstacles) {

  double start_timestamp = Clock::NowInSeconds();

  // Insert obstacle
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::PERCEPTION_OBSTACLES));
  obstacles_container->Insert(perception_obstacles);

  // Update ADC status
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  ADCTrajectoryContainer* adc_container = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::PLANNING_TRAJECTORY));

  PerceptionObstacle* adc = pose_container->ToPerceptionObstacle();
  if (adc != nullptr) {
    obstacles_container->InsertPerceptionObstacle(*adc, adc->timestamp());
    double x = adc->position().x();
    double y = adc->position().y();
    Vec2d adc_position(x, y);
    adc_container->SetPosition(adc_position);
  }

  // Make evaluations
  EvaluatorManager::instance()->Run(perception_obstacles);

  // No prediction for offline mode
  if (FLAGS_prediction_offline_mode) {
    return;
  }

  // Make predictions
  PredictorManager::instance()->Run(perception_obstacles);

  auto prediction_obstacles = PredictorManager::instance()->prediction_obstacles();
  prediction_obstacles.set_start_timestamp(start_timestamp);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());

  Publish(&prediction_obstacles);
}
```

从上面的代码，`Prediction::RunOnce`是Predict模块收到Perception模块的ROS消息以后，启动的回调处理函数。上述的流程我们根据他的注释来解释。

1. 将Perception模块检测到的PerceptionObstacles插入到Obstacle的LRU缓存中

2. 将当前车辆的位置信息保存到ADCTrajectoryContainer中，同时将当前车辆信息也插入到Obstacle的LRU中，意思是将主车也作为障碍物插入，但是后续不对主车进行归集预测，因为之车的轨迹是程序根据外界条件来具体生成的，不是预测出来的。

3. 障碍物LaneSequence的概率评估`EvaluatorManager::instance()->Run(perception_obstacles);`

4. LaneSequence的短时间轨迹生成`PredictorManager::instance()->Run(perception_obstacles);`

5. 消息发布，其中包括了障碍物的信息，运动状态信息以及每个障碍物对应的多条可能运动轨迹