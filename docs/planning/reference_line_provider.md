#参考线提供器: ReferenceLineProvider

本小节主要介绍参考线提供器的功能，参考线提供器主要完成的工作是计算车辆在规划路径上的短期可行路径。在控制规划地图Pnc Map中，有一个功能是路径段RouteSegments生成最终路径Path，这个Path就是车辆在规划路径上的可行路径，但是路径是以`std::vector<common::math::LineSegment2d> segments_`和`std::vector<MapPathPoint> path_points_`的离散形式存在的，如果还不了解，可以参考[控制规划地图Pnc Map](https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/pnc_map.md)进行了解。而本节参考线提供器就是对上述的Path进行样条函数Spline插值，得到平滑的路径曲线。

对path_points_中的离散点进行样条函数插值，得到连续平滑的路径函数，主要的步骤包含有:

1. 路径点采样与轨迹点矫正

2. knots分段与二次规划

## 路径点采样与轨迹点矫正

控制规划地图Pnc Map根据当前车辆状态与Routing模块规划路径响应可以得到当前情况下，车辆的可行驶方案Path的集合(每个RouteSegments生成对应的一个Path)。在Path中路径以`std::vector<common::math::LineSegment2d> segments_`和`std::vector<MapPathPoint> path_points_`存在，前者是段的形式，而后者是原始离散点的形式。那么这个Path其实就是路径的离散形式表示，在本节中，我们需要行驶路径(参考线)的连续表示法，也就是根据这些离散点拟合一个合理的连续函数。

路径点采样与轨迹点矫正阶段工作就是对路径做一个离散点采样，为什么不用知道的path_points_或者sample_points_，因为该阶段需要的条件不同，所以其实就是重新采样的过程，与sample_points_采样其实没太大区别，仅仅是采样的间隔不同。

首先，已知路径的长度length_，只需要给出采样的间距interval，就能完成采样。

```c++
/// file in apollo/modules/planning/reference_line/reference_line_provider.cc
void ReferenceLineProvider::GetAnchorPoints(const ReferenceLine &reference_line,
    						std::vector<AnchorPoint> *anchor_points) const {
  // interval为采样间隔，默认max_constraint_interval=5.0，即路径累积距离每5m采样一个点。
  const double interval = smoother_config_.max_constraint_interval();
  // 路径采样点数量计算
  int num_of_anchors = std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  // uniform_slice函数就是对[0.0, reference_line.Length()]区间等间隔采样，每两个点之间距离为(length_-0.0)/(num_of_anchors - 1)
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1, &anchor_s);
  // 根据每个采样点的累积距离s，以及Path的lane_segments_to_next_point_进行平滑插值，得到累积距离为s的采样点的坐标(x,y)，并进行轨迹点矫正
  for (const double s : anchor_s) {
    anchor_points->emplace_back(GetAnchorPoint(reference_line, s));
  }
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}
```

在上述过程中能够，其实可以很清楚的看到路径点采样与轨迹点矫正的整个流程，其余代码都比较简单，我们这里分析`GetAnchorPoint`函数是如何完成采样点的坐标计算与轨迹点坐标矫正的。

```c++
auto ref_point = reference_line.GetReferencePoint(s);
```

首先采样点坐标计算与Pnc Map中`Path::InitWidth`采样点计算一致，虽然这里多了一层`ReferenceLine::GetReferencePoint`的函数包装，但是计算过程是一模一样的，可以参考[Pnc Map道路采样点生成](https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/pnc_map.md/#sample)

当完成采样点的计算以后，下一步就是采样点(也就是轨迹点)的坐标矫正。为什么需要坐标矫正？因为采样点坐标是在道路的中心线上，但当道路比较宽时，车辆不能一味的在中间行驶，需要考虑到其他车辆超车情况。在这种情况下，车辆需要靠右行驶(当然不同区域的模式不一样，部分地区是靠左形式)，所以道路过宽时，需要将轨迹点向右或者向左矫正一段距离。

矫正的步骤如下：

1. 计算车辆宽度adc_width和半宽adc_half_width

```c++
const double adc_width = VehicleConfigHelper::GetConfig().vehicle_param().width();
const double adc_half_width = adc_width / 2.0;
```

2. 计算车道距左边界距离left_width和距右边界距离right_width 

```c++
double left_width = 0.0;
double right_width = 0.0;
waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);  // Hd Map中查询得到
double total_width = left_width + right_width; // 当前位置，车道总宽度
```

3. 计算车辆应该右移或者左移(矫正)的距离

```c++
//如果车道宽度大于车辆宽度的wide_lane_threshold_factor倍，默认为2，则需要靠边行驶，因为存在其他车辆超车的可能性
// shift to left (or right) on wide lanes
if (!(waypoint.lane->lane().left_boundary().virtual_() ||
      waypoint.lane->lane().right_boundary().virtual_()) &&
      total_width > adc_width * smoother_config_.wide_lane_threshold_factor()) { 
  // 靠右行驶模式
  if (smoother_config_.driving_side() == ReferenceLineSmootherConfig::RIGHT) {
    shifted_left_width = adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor();
  } else {
  	// 靠左形式模式
    shifted_left_width = std::fmax(
        adc_half_width,
        total_width - (adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor()));
  }
}
```

这部分代码可以从下面两张图理解

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/correct_right.png)

靠右行驶时，shifted_left_width的计算和后期矫正的过程

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/correct_left.png)

靠左行驶时，shifted_left_width的计算和后期矫正的过程