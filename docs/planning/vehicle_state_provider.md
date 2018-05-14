# 车辆状态提供器: VehicleStateProvider

在这一小节，我们将介绍最简单的组件，车辆状态提供器。顾名思义，就是提供当前无人车的各类状态信息，车辆状态提供器的输入包括定位数据Localization以及底盘信息Chassis。

首先我们可以看一下VehicleStateProvider成员，其实很简单，只有两个成员变量:

1. 车辆状态信息`vehicle_state_`
2. 定位信息`original_localization_`

```c++
/// file in apollo/modules/common/vehicle_state/vehicle_state_provider.h
class VehicleStateProvider {
  ...
private:
  common::VehicleState vehicle_state_;
  localization::LocalizationEstimate original_localization_;
}
```

这两个成员对应的类型都是以protobuf的形式定义，`original_localization_`其实并不是主要成员，`vehicle_state_`才是完整保存车辆状态的载体，下面我们可以看一下`VehicleState`的定义：

```protobuf
import "modules/canbus/proto/chassis.proto";
import "modules/localization/proto/pose.proto";

message VehicleState {
  optional double x = 1 [default =0.0];            // 车辆世界ENU坐标系x坐标
  optional double y = 2 [default =0.0];            // 车辆世界ENU坐标系y坐标
  optional double z = 3 [default =0.0];            // 车辆世界ENU坐标系z坐标
  optional double timestamp = 4 [default =0.0];    // 时间戳信息
  optional double roll = 5 [default =0.0];         // 车辆姿态相对于世界坐标系x轴旋转角度
  optional double pitch = 6 [default =0.0];        // 车辆姿态相对于世界坐标系y轴旋转角度
  optional double yaw = 7 [default =0.0];          // 车辆姿态相对于世界坐标系z轴旋转角度
  optional double heading = 8 [default =0.0];      // 车辆速度方向
  optional double kappa = 9 [default =0.0];        // 车辆半径倒数1/R
  optional double linear_velocity = 10 [default =0.0];      // 车辆线速度
  optional double angular_velocity = 11 [default =0.0];     // 车辆角速度
  optional double linear_acceleration = 12 [default =0.0];  // 车辆线加速度
  optional apollo.canbus.Chassis.GearPosition gear = 13;    // 车辆齿轮状态，包含前进、倒车。停车、低速等状态
  optional apollo.canbus.Chassis.DrivingMode driving_mode = 14;  // 驾驶状态，包含手动驾驶、自动驾驶、转向、刹车与油门等状态
  optional apollo.localization.Pose pose = 15;     // 车辆姿态，包含坐标，局部到世界坐标系变换矩阵，线速度(矢量)，线加速度(矢量)等信息。
}
```


从上述可以看到，`VehicleState`基本包含了车辆的定位、姿态与底盘信息。`LocalizationEstimate`也是一样，信息基本被包括在`VehicleState`内。由`VehicleStateProvider::Update`函数可以看到`VehicleState`的更新过程，需要定位信息与底盘信息。

```c++
/// file in apollo/modules/common/vehicle_state/vehicle_state_provider.cc
Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate &localization,
    const canbus::Chassis &chassis)

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const
```

车辆状态提供器有两个比较重要的函数或者功能，一个是上述的车辆状态更新函数`VehicleStateProvider::Update`；另一个则是根据当前车辆状态，给定一个时间t，来预测t时间后的车辆状态`VehicleStateProvider::EstimateFuturePosition`，注意，这个时间t必须是短时间内的预测，如果t过大，预测将不会准确(因为预测前提是假定这个时间段内，车辆的状态不变)。

## 1.车辆状态更新函数( `VehicleStateProvider::Update`)

1. 设置车辆姿态信息

在函数`VehicleStateProvider::ConstructExceptLinearVelocity`中实现，即将`localization`中的信息抽取出来填充到`vehicle_state_`中，设置的内容包含:

- 车辆坐标(x,y,z)
- 车辆速度方向heading
- 车辆xy平面角速度angular_velocity
- 车辆半径倒数kappa
- 车辆xyz方向旋转角roll, pitch, yaw

2. 设置车辆速度与时间戳信息

- 车辆速度linear_velocity
- 时间戳timestamp

3. 设置车辆底盘信息

- 齿轮信息gear
- 驾驶模式driving_mode

## 2.车辆未来时刻预测函数( `VehicleStateProvider::EstimateFuturePosition`)

预测未来短时间内车辆的位置与姿态信息，其实这是对车辆状态的一种修正。主要是在使用`VehicleStateProvider::Update`更新状态以后，当需要使用`vehicle_state_`时，经过了一定的时间差t，外加车辆运动就造成车辆状态的偏差，所以可以使用该函数来对车辆状态进行一次修正，但是修正必须要满足一个条件：当前时间与update时间(`vehicle_state_.timestamp`)与之间的时间差必须小于一个阈值，代码中设置为20ms，可以对这个时间差进行车辆姿态矫正。

以下是一个矫正的例子：

```c++
if (FLAGS_estimate_current_vehicle_state && start_timestamp - vehicle_state.timestamp() < 0.020) {
    auto future_xy = VehicleStateProvider::instance()->EstimateFuturePosition(
        start_timestamp - vehicle_state.timestamp());
    vehicle_state.set_x(future_xy.x());
    vehicle_state.set_y(future_xy.y());
    vehicle_state.set_timestamp(start_timestamp);
  }
```

从上述代码可以看到，矫正主要是对他的xy平面(路面)坐标和时间戳进行矫正。下面我们看一下矫正的过程

1. 速度设置

```c++
/// file in apollo/modules/common/vehicle_state/vehicle_state_provider.cc
math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  // Step 1. set vehicle velocity
  double v = vehicle_state_.linear_velocity();
  if (vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
    v = -vehicle_state_.linear_velocity();
  }
}
```

查看齿轮状态，如果如果是倒车，速度就设置为负。

2. 计算t时刻以后车辆的状态(世界西坐标)

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/future_estimation.png)

```c++
/// file in apollo/modules/common/vehicle_state/vehicle_state_provider.cc
math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  // Step 1. set vehicle velocity
  ...
  // Step 2. Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity() *                 // x_new
                      (1.0 - std::cos(vehicle_state_.angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v /    // y_new
                      vehicle_state_.angular_velocity();
  }

  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    const auto &orientation = vehicle_state_.pose().orientation();        
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                            vehicle_state_.z());
    auto future_pos_3d = quaternion.toRotationMatrix() * vec_distance + pos_vec;  // (x', y')
    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  }
```

从上述代码和图可以不难理解新坐标的计算方式，注意一点: `vehicle_state_.angular_velocity()`是带有正负性的，逆时针为正；顺时针为负，这样也就可以理解上述公式的符号。
