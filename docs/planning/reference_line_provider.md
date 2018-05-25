# 参考线提供器: ReferenceLineProvider

本小节主要介绍参考线提供器的功能，参考线提供器主要完成的工作是计算车辆在规划路径上的短期可行路径。在控制规划地图Pnc Map中，有一个功能是路径段RouteSegments生成最终路径Path，这个Path就是车辆在规划路径上的可行路径，但是路径是以`std::vector<common::math::LineSegment2d> segments_`和`std::vector<MapPathPoint> path_points_`的离散形式存在的，如果还不了解，可以参考[控制规划地图Pnc Map](https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/pnc_map.md)进行了解。而本节参考线提供器就是对上述的Path进行样条函数Spline插值，得到平滑的路径曲线。

对path_points_中的离散点进行样条函数插值，得到连续平滑的路径函数，主要的步骤包含有:

1. 路径点采样与轨迹点矫正

2. knots分段与二次规划进行参考线平滑

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

这部分代码可以从下面图中理解

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/correct_1.png)

从代码可以看到，如果官方代码没有错误，那么：

- 当driving_side为RIGHT时

`m = adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor()`计算矫正后车辆与左边界距离

- 当driving_side为LEFT时

`m = adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor()`计算矫正后车辆与右边界距离

这样的方式在车道宽度差异比较大的时候，车联靠边行驶，距离边界的距离(total_width - m)，这个距离不好控制。更不如使用一种更好的方法，同样是计算m，但是m的意义是距离最近边界线的距离，即靠边行驶与边界的距离，这个距离相对来说就比较好控制。只需要做一个更简单的改动，将上述代码的if-else判断条件交换一下即可，wide_lane_shift_remain_factor可以设置小于0.5的值。保证车辆边界与车到边界有一定距离(间隙)。

第二部分矫正是根据左右边界线是否是道路边界，如果车道左边界是道路边界线，那么shifted_left_width需要加上一个边界的缓冲距离curb_shift，默认0.2m，反之就是减去缓冲边界距离。

```c++
// shift away from curb boundary
auto left_type = hdmap::LeftBoundaryType(waypoint);
if (left_type == hdmap::LaneBoundaryType::CURB) {
  shifted_left_width += smoother_config_.curb_shift();
}
auto right_type = hdmap::RightBoundaryType(waypoint);
if (right_type == hdmap::LaneBoundaryType::CURB) {
  shifted_left_width -= smoother_config_.curb_shift();
}
```

最后矫正得到的平移距离就是shifted_left_width，如何根据这个平移距离求出车辆在**世界坐标系**中的矫正位置?

转换方法如下图，一目了然。

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/correct_2.png)

```c++
ref_point += left_vec * (left_width - shifted_left_width);
auto shifted_right_width = total_width - shifted_left_width;
anchor.path_point = ref_point.ToPathPoint(s);
double effective_width = std::min(shifted_left_width, shifted_right_width) -
                           adc_half_width - FLAGS_reference_line_lateral_buffer;
anchor.lateral_bound = std::max(smoother_config_.lateral_boundary_bound(), effective_width);
anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
```

最后当使用二次规划来拟合轨迹点时，需要设置该点的约束，`lateral_bound`指的是预测的x值需要在ref_point的L轴的lateral_bound左右领域内，`longitudinal_bound`是预测的y值需要在ref_point的F轴的longitudinal_bound前后领域内


## knots分段与二次规划进行参考线平滑

通过第一阶段路径点采样与轨迹点矫正，可以得到这条路径的anchor_point集合，里面是若干矫正过后的轨迹点，但还是离散形式。这个阶段我们需要对上述离散轨迹点进行多项式拟合。这部分内容也可以参考[Apollo参考线平滑器](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/reference_line_smoother.md)。官方文档介绍的偏简单，在这里将从代码入手介绍一下参考线平滑过程。

这个过程目的是得到一条平滑的参考线，也就是运行轨迹，具体的做法是使用knots分段与QP二次规划。主要的思路是，将anchor_point分成n组，每组用一个多项式函数去拟合，可以得到n个多项式函数。

函数的输入和输出又是什么？我们现在只anchor_point中每个点的车道累积距离差s，以及世界系坐标(x,y)。想要拟合出轨迹曲线，只能是累积距离s作为自变量，世界系坐标作为应变量。计算函数为：

$ x = f_i(s) = a_{i0} + a_{i1}s + a_{i2}s^2 +a_{i3}s^3 + a_{i4}s^4 + a_{i5}s^5 $

$ y = g_i(s) = b_{i0} + b_{i1}s + b_{i2}s^2 +b_{i3}s^3 + b_{i4}s^4 + b_{i5}s^5 $

在这里是分别对x和y用多项式函数拟合，函数的参数a和b的下标i表示哪一个段(两个knots之间的anchor point)

### A. 预处理：如何划分段，或者说设置knots？

简单，anchor point是对原始Path进行采样，采样间隔为`smoother_config_.max_constraint_interval()`，默认5m一个点。knots的采样其实也是相似的，采样间隔为`config_.qp_spline().max_spline_length()`，默认25m：

```c++
uint32_t num_spline = std::max(1u, static_cast<uint32_t>(length / config_.qp_spline().max_spline_length() + 0.5));
for (std::uint32_t i = 0; i <= num_spline; ++i) {
  t_knots_.push_back(i * 1.0);
}
```

最后得到的knots节点有num_spline+1个。得到了所有的knots，也就意味着可到了所有的段，很明显这里就需要拟合num_spline个段，每个段有x和y两个多项式函数。

此外，还需要对anchor_point的自变量s做处理，本来s是从0到length_递增，现进行如下处理:

```c++
const double scale = (anchor_points_.back().path_point.s() -
                        anchor_points_.front().path_point.s()) /
                       (t_knots_.back() - t_knots_.front());
std::vector<double> evaluated_t;
for (const auto& point : anchor_points_) {
  evaluated_t.emplace_back(point.path_point.s() / scale);
}
```

不难理解，就是将自变量s从[0,length_]区间按比例映射到[0,num_spline]区间，这样每个段内anchor point的s都属于[a,a+1]内，如果在减去knots[a]那么所有自变量的取值范围就是[0,1]，事实上代码中也是这样做的。

同时还需要对应变量(x,y)做处理，处理方法如下

```c++
for (const auto& point : anchor_points_) {
  const auto& path_point = point.path_point;
  headings.push_back(path_point.theta());
  longitudinal_bound.push_back(point.longitudinal_bound);
  lateral_bound.push_back(point.lateral_bound);
  xy_points.emplace_back(path_point.x() - ref_x_, path_point.y() - ref_y_);
}
```

可以看到x和y都需要减去Path第一个点的世界坐标系坐标，说白了2n个(2\*num_spline)函数的坐标原点是Path的第一个点。

### B. 如何设置约束条件？

在上一步预处理阶段，已经知道：

1. 需要拟合的多项式函数数量为2\*num_spline个，每两个knots之间会拟合x和y两个多项式

2. 多项式最高阶数为5(qp_spline.spline_order: 5)，所以每个多项式共6个参数，参数总和：(spline_order+1)\*2\*num_spline

3. 使用每个段内的anchor point去拟合多项式函数，自变量范围[0,1]，应变量相对于第一个anchor point的相对坐标。所以最后拟合出来的函数f和g的结果是相对于第一个anchor point的相对坐标。

那么在拟合过程中还需要满足一些约束，包括等式约束和不等式约束，例如：

- 预测的x'和y'需要保证在真实x和y的L轴lateral_bound、F轴longitudinal_bound领域内
- 第一个anchor point的heading和函数的一阶导方向需要一致，大小可以不一致，但是方向必需一致！
- x和y的n段函数之间，两两接壤部分应该是平滑的，两个函数值、一阶导、二阶导必须一致。

**边界约束**

```
/// file in apollo/modules/planning/reference_line/qp_spline_reference_line_smoother.cc
bool QpSplineReferenceLineSmoother::AddConstraint() {
// all points (x, y) should not deviate anchor points by a bounding box
  if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points, 
  										longitudinal_bound, lateral_bound)) {
    AERROR << "Add 2d boundary constraint failed.";
    return false;
  }
}
```

每个anchor point相对第一个点的相对参考系坐标为(x,y)，方向为heading。而该点坐在的段拟合出来的相对参考系坐标为(x',y')，坐标的计算方式为:

$ x' = f_i(s) = a_{i0} + a_{i1}s + a_{i2}s^2 +a_{i3}s^3 + a_{i4}s^4 + a_{i5}s^5 $

$ y' = g_i(s) = b_{i0} + b_{i1}s + b_{i2}s^2 +b_{i3}s^3 + b_{i4}s^4 + b_{i5}s^5 $

其中i是anchor point所在的knots段，i=1,2,...,n(n=num_spline)

![img](https://github.com/YannZyl/Apollo-Note/blob/master/images/planning/boundary_constraint.png)

1. 真实点(x,y)F和L轴投影计算

如上图，实际情况下需要满足拟合坐标(x',y')在真实坐标(x,y)的领域内，真实点的投影计算方法比较简单，首先坐标在侧方L轴上的投影(天蓝色星星)，投影点到原点的距离，也就是侧方距离计算方式为：

$ x_{p,later} = (cos(\theta+\pi/2), sin(\theta+\pi/2))·(x, y) $

注意上述公式·为内积操作。这部分对应的代码为：

```c++
const double d_lateral = SignDistance(ref_point[i], angle[i]);

double Spline2dConstraint::SignDistance(const Vec2d& xy_point, const double angle) const {
  return common::math::InnerProd(xy_point.x(), xy_point.y(), -std::sin(angle), std::cos(angle));
}
```

`ref_point[i]`是第i个anchor point的相对参考系坐标，`angle[i]`为该点的方向heading，也是公式中的theta。

注意一点，代码中的方向向量是(-sin(angle), cos(angle))，其实也可以等价为(cos(angle+pi/2), sin(angle+pi/2))，很明显，代码中的方向向量是在L轴的，所以在计算L轴上的投影距离是，直接将heading传入即可，不需要额外加上一个pi/2。最终代码的计算方式与公式是一致的。

真实点坐标在前方F轴上的投影(大红色星星)，投影点到原点的距离，也就是前方距离计算方式为：

$ y_{p,longi} = (cos(\theta), sin(\theta))·(x, y) $

注意上述公式·为内积操作。对应的代码为:

```c++
const double d_longitudinal = SignDistance(ref_point[i], angle[i] - M_PI / 2.0);
```

从L轴到F轴的方向向量，需要减去一个pi/2。

2. 函数预测点(x',y')F和L轴投影计算

根据多项式函数，可以简化出函数预测点(x',y')的计算方式为：

$ x = SA $

$ y = SB $

其中:

$ S = [1, s, s^2, s^3, s^4, s^5] $

$ A = [a_{i0} ,a_{i1}, a_{i2}, a_{i3}, a_{i4}, a_{i5}]^T $

$ B = [b_{i0} ,b_{i1}, b_{i2}, b_{i3}, b_{i4}, b_{i5}]^T $

接下去我们从代码中验证一下正确性。

```c++
const uint32_t index = FindIndex(t_coord[i]);
const double rel_t = t_coord[i] - t_knots_[index];
const uint32_t index_offset = 2 * index * (spline_order_ + 1);
```

上述过程index是计算公式中的i，也就是计算n个拟合段中anchor point所属的段。rel_t是anchor point累积距离s相对于下界knots累积距离s的相对差，说白了就是自变量归一化到[0,1]之间。

index_offset是该段拟合函数对应的参数位置，我们可以知道n段拟合多项式函数的参数总和为 2\*(spline_order+1)\*n。所以第i个拟合函数的参数偏移位置为2\*(spline_order+1)\*i。

- [2\*(spline_order+1)\*i， 2\*(spline_order+1)\*i+(spline_order+1)]是x多项式函数的参数，共(spline_order+1)个，即向量A；
- [2\*(spline_order+1)\*i+(spline_order+1)， 2\*(spline_order+1)\*(i+1)]是y多项式函数的参数，共(spline_order+1)个，即向量B

```
std::vector<double> longi_coef = AffineCoef(angle[i], rel_t);
std::vector<double> longitudinal_coef = AffineCoef(angle[i] - M_PI / 2, rel_t);

std::vector<double> Spline2dConstraint::AffineCoef(const double angle, const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  for (uint32_t i = 0; i < num_params; ++i) {
    result[i] = x_coef;
    result[i + num_params] = y_coef;
    x_coef *= t;
    y_coef *= t;
  }
  return result;
}
```

这部分`longi_coef`和`longitudinal_coef`也比较简单，一句话描述：

$ longicoef = [-sin(\theta)S, cos(\theta)S] = [cos(\theta+\pi/2)S, sin(\theta+\pi/2)S] $

$ longitudinalcoef = [-sin(\theta-\pi/2)S, cos(\theta-\pi/2)S] = [cos(\theta)S, sin(\theta)S] $

两个系数分别是在L轴和F轴上的投影系数。但是longi_coef名字可能改成lateral_coef更合适。最后可以根据这两个值求解在F和L轴上的投影。

$ x_{q,later} = (cos(\theta+\pi/2), sin(\theta+\pi/2))·(x', y') = (cos(\theta+\pi/2), sin(\theta+\pi/2))·(SA, SB) $

即$ x_{q,later} = [-sin(\theta)S, cos(\theta)S]·(A, B) =  longicoef · (A, B)$

$ y_{q,longi} = (cos(\theta), sin(\theta))·(x', y') = (cos(\theta), sin(\theta))·(SA, SB) $

即$ y_{q,longi} = [-sin(\theta-\pi/2)S, cos(\theta-\pi/2)S]·(A, B) =  longitudinalcoef · (A, B)$

3. 约束条件设置

现在可以计算真实点和拟合点在F轴L轴的投影，那么就有约束条件：

|d_lateral - longi_coef·(A, B)| <= lateral_bound 

|d_longitudinal - longitudinal_coef(A, B)| <= longitudinal_bound 

最后得到四个约束不等式：

- L轴上界不等式

d_lateral - longi_coef·(A, B) <= lateral_bound 

整理得到：**longi_coef·(A, B) >= d_lateral - lateral_bound**

- L轴下界不等式

d_lateral - longi_coef·(A, B) >= -lateral_bound 

整理得到： **-longi_coef·(A, B) >= -d_lateral - lateral_bound**

- F轴上界不等式

d_longitudinal - longitudinal_coef·(A, B) <= longitudinal_bound 

整理得到：**longitudinal_coef·(A, B) >= d_longitudinal - longitudinal_bound**

- F轴下界不等式

d_longitudinal - longitudinal_coef·(A, B) >= -longitudinal_bound

整理得到：**-longitudinal_coef·(A, B) >= -d_longitudinal - longitudinal_bound**

```c++
for (uint32_t j = 0; j < 2 * (spline_order_ + 1); ++j) {
  // upper longi，设置L轴上界不等式系数
  affine_inequality(4 * i, index_offset + j) = longi_coef[j];
  // lower longi，设置L轴下界不等式系数
  affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
  // upper longitudinal，设置F轴上界不等式系数
  affine_inequality(4 * i + 2, index_offset + j) = longitudinal_coef[j];
  // lower longitudinal，设置F轴下界不等式系数
  affine_inequality(4 * i + 3, index_offset + j) = -longitudinal_coef[j];
}

affine_boundary(4 * i, 0) = d_lateral - lateral_bound[i];       //设置L轴上界不等式的边界
affine_boundary(4 * i + 1, 0) = -d_lateral - lateral_bound[i];  //设置L轴下界不等式的边界
affine_boundary(4 * i + 2, 0) = d_longitudinal - longitudinal_bound[i];   //设置F轴上界不等式的边界
affine_boundary(4 * i + 3, 0) = -d_longitudinal - longitudinal_bound[i];  //设置L轴下界不等式的边界
```

配合代码和上述的公式可以不难看出不等式系数的设置和边界设置。经过上述赋值:

`affine_inequality`等同于: [longi_coef, -longi_coef, longitudinal_coef, -longitudinal_coef]

`affine_boundary`等同于: [d_lateral-lateral_bound, -d_lateral-lateral_bound, d_longitudinal-longitudinal_bound, -d_longitudinal-longitudinal_bound]

最后不等式约束：

`affine_inequality * [A1,B1,A2,B2,..An,Bn] >= affine_boundary`

**方向约束**

```
/// file in apollo/modules/planning/reference_line/qp_spline_reference_line_smoother.cc
bool QpSplineReferenceLineSmoother::AddConstraint() {
  // the heading of the first point should be identical to the anchor point.
  if (!spline_constraint->AddPointAngleConstraint(evaluated_t.front(),
                                                  headings.front())) {
    AERROR << "Add 2d point angle constraint failed.";
    return false;
  }
}
```

第一个anchor point的heading应该和第一段的多项式函数f1和g1的偏导数方向一致，大小可以不一致。也就是： heading = argtan(g1'(s), f1'(s))。从上述代码可以看到，参入的参数是第一个点。

```c++
/// file in apollo/modules/planning/math/smoothing_spline/spline_2d_constraint.cc
bool Spline2dConstraint::AddPointAngleConstraint(const double t,
                                                 const double angle) {
  const uint32_t index = FindIndex(t);
  const uint32_t num_params = spline_order_ + 1;
  const uint32_t index_offset = index * 2 * num_params;
  const double rel_t = t - t_knots_[index];
}
```

第一步还是计算第一个anchor point所属的函数索引index，index_offset是该段函数参数(Ai,Bi)所在的位置偏移，rel_t是第一个点的自变量，归一化到[0,1]。为了确保方向一致，代码中使用两个约束来保证这个方向一致的约束问题：

- L轴分量为0，保证方向相同或者相反
- 验证同向性

1. L轴分量为0，保证方向相同或者相反

```c++
bool Spline2dConstraint::AddPointAngleConstraint(const double t,
                                                 const double angle) {
  // add equality constraint
  Eigen::MatrixXd affine_equality = Eigen::MatrixXd::Zero(1, total_param_);
  Eigen::MatrixXd affine_boundary = Eigen::MatrixXd::Zero(1, 1);
  std::vector<double> line_derivative_coef = AffineDerivativeCoef(angle, rel_t);
  for (uint32_t i = 0; i < line_derivative_coef.size(); ++i) {
    affine_equality(0, i + index_offset) = line_derivative_coef[i];
  }
}

std::vector<double> Spline2dConstraint::AffineDerivativeCoef(
    const double angle, const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 1; i < num_params; ++i) {
    result[i] = x_coef * power_t[i - 1] * i;
    result[i + num_params] = y_coef * power_t[i - 1] * i;
  }
  return result;
}
```

等式约束还是那句话，第一个点的方向heading和多项式曲线在该点的斜率(一阶导)方向必须一致，大小可以不一致。**方向一致等价于：斜率在L轴方向上的分量为0**

代码中通过`Spline2dConstraint::AffineDerivativeCoef`函数计算得到的系数矩阵`line_derivative_coef`为：

微分矩阵 $ D = [0, 1, 2s, 3s^2, 4s^3, 5s^4] $

line_derivative_coef = [-sin(theta)D, cos(theta)D]

可以得到L轴方向分量的计算方式为 line_derivative_coef · (A, B) = 0

从代码我们可以看到一个问题：只是限制了L轴分量为零，但是不保证同向性。

2. 验证同向性

真实点的方向为heading，拟合多项式在该点的一阶导数为 (D·A, D·B)。代码中对heading做一个处理，规则化到[0, 2\*pi]

计算heading的方向向量sgn = [x_sign, y_sign]，计算方法为：

- 如果正则化heading在[0, pi/2]: sgn = [1, 1]
- 如果正则化heading在[pi/2, pi]: sgn = [-1, 1]
- 如果正则化heading在[pi,3\*pi/2]: sgn = [-1, -1]
- 如果正则化heading在[3\*pi/2, 2\*pi]: sgn = [1, -1]

只需要最后的内积 sgn·(D·A, D·B)>0表明方向一致。

其实有更加简单地方式，heading方向对应的单位向量为(cos(heading), sin(heading))，所以需要要如下代码就可以：

```c++
x_sign = std::cos(angle)
y_sign = std::sin(angle)
```

也不需要去计算normalized_angle这些，代码量明显减少了。但是代码这样的写法有一个明显的优势：系数为1或者-1，优化变得简单。

**各函数接壤处平滑约束**

边界约束和方向约束是对每个多项式拟合函数的约束，而相邻多项式函数之间也需要进行约束，需要保证函数间是连续可微的。具体包括：

- 两个函数接壤部分函数值相同，保证连续。(位置一致)

- 两个函数接壤部分一阶和二阶导相同，保证可微。(速度，加速度一致)

所以从上述可以得知，平滑约束共6个不等式。

$ f_i(knots[i+1].s-knots[i].s) - f_{i+1}(0) = 0 $

$ g_i(knots[i+1].s-knots[i].s) - g_{i+1}(0) = 0 $

$ f^{(1)}\_i(knots[i+1].s-knots[i].s) - f^{(1)}\_{i+1}(0) = 0 $

$ f^{(2)}\_i(knots[i+1].s-knots[i].s) - f^{(2)}\_{i+1}(0) = 0 $

$ g^{(1)}\_i(knots[i+1].s-knots[i].s) - g^{(1)}\_{i+1}(0) = 0 $

$ g^{(2)}\_i(knots[i+1].s-knots[i].s) - g^{(2)}\_{i+1}(0) = 0 $

以x的多项式拟合函数f为例，函数的一阶导和二阶导分别为

$ x' = f^{(1)}\_i(s) = 0 + a_{i1} + 2a_{i2}s + 3a_{i3}s^2 + 4a_{i4}s^3 + 5a_{i5}s^4 $

$ x'' = f^{(2)}\_i(s) = 0 + 0 + 2a_{i2} + 6a_{i3}s + 12a_{i4}s^2 + 20a_{i5}s^3 $

函数值系数: $ S = [1, s, s^2, s^3, s^4, s^5] $ 

一阶导系数: $ Ds = [0, 1, 2s, 3s^2, 4s^3, 5s^4] $ 

二阶导系数: $ DDs = [0, 0, 2, 6s, 12s^2, 20s^3] $ 

最终简化后的6个等式约束为：

$ SA_i - [1,0,0,0,0,0]A_{i+1} = 0 $ 

$ DsA_i - [0,1,0,0,0,0]A_{i+1} = 0 $ 

$ DDsA_i - [0,0,2,0,0,0]A_{i+1} = 0 $ 

$ SB_i - [1,0,0,0,0,0]B_{i+1} = 0 $ 

$ DsB_i - [0,1,0,0,0,0]B_{i+1} = 0 $ 

$ DDsB_i - [0,0,2,0,0,0]B_{i+1} = 0 $ 

代码如下

```c++
// guarantee upto second order derivative are joint
bool Spline2dConstraint::AddSecondDerivativeSmoothConstraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  // 6个等式，affine_equality是系数，affine_boundary是值。约束函数数量：6 * (n-1), n=t_knots_.size()-1
  Eigen::MatrixXd affine_equality = Eigen::MatrixXd::Zero(6 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary = Eigen::MatrixXd::Zero(6 * (t_knots_.size() - 2), 1);
  // 相邻两个knots对之间的多项式拟合函数进行约束
  for (uint32_t i = 0; i + 2 < t_knots_.size(); ++i) {
  	// 计算第一个曲线的自变量：t_knots[i+1].s-t_knots[i].s
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const uint32_t num_params = spline_order_ + 1;
    const uint32_t index_offset = 2 * i * num_params;
    // 函数值系数: [1, s, s^2, s^3, s^4, s^5]
    std::vector<double> power_t = PolyCoef(rel_t);
    // 一阶导系数: [0, 1, 2s, 3s^2, 4s^3, 5s^4]
    std::vector<double> derivative_t = DerivativeCoef(rel_t);
    // 二阶导系数: [0, 0, 2, 6s, 12s^2,20s^3]
    std::vector<double> second_derivative_t = SecondDerivativeCoef(rel_t);
    for (uint32_t j = 0; j < num_params; ++j) {
      affine_equality(6 * i, j + index_offset) = power_t[j];                             // 第一个多项式x曲线终点函数值
      affine_equality(6 * i + 1, j + index_offset) = derivative_t[j];                    // 第一个多项式x曲线终点一阶导
      affine_equality(6 * i + 2, j + index_offset) = second_derivative_t[j];             // 第一个多项式x曲线终点二阶导
      affine_equality(6 * i + 3, j + index_offset + num_params) = power_t[j];            // 第二个多项式y曲线终点函数值
      affine_equality(6 * i + 4, j + index_offset + num_params) = derivative_t[j];       // 第二个多项式y曲线终点一阶导
      affine_equality(6 * i + 5, j + index_offset + num_params) = second_derivative_t[j];// 第二个多项式y曲线终点二阶导
    }
    affine_equality(6 * i, index_offset + 2 * num_params) = -1.0;          // 第一个多项式x曲线终点函数值 - 第二个多项式x曲线起点函数值
    affine_equality(6 * i + 1, index_offset + 2 * num_params + 1) = -1.0;  // 第一个多项式x曲线终点一阶导 - 第二个多项式x曲线起点一阶导(速度一致)
    affine_equality(6 * i + 2, index_offset + 2 * num_params + 2) = -2.0;  // 第一个多项式x曲线终点二阶导 - 第二个多项式x曲线起点二阶导(加速度一致)
    affine_equality(6 * i + 3, index_offset + 3 * num_params) = -1.0;      // 第一个多项式y曲线终点函数值 - 第二个多项式y曲线起点函数值
    affine_equality(6 * i + 4, index_offset + 3 * num_params + 1) = -1.0;  // 第一个多项式y曲线终点一阶导 - 第二个多项式y曲线起点一阶导(速度一致)
    affine_equality(6 * i + 5, index_offset + 3 * num_params + 2) = -2.0;  // 第一个多项式y曲线终点二阶导 - 第二个多项式y曲线起点二阶导(加速度一致)
  }
  return AddEqualityConstraint(affine_equality, affine_boundary);
}
```

### C. 如何设置cost函数？

由[Apollo参考线平滑器](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/reference_line_smoother.md)可以看到Apollo使用的cost函数:

<p>
$$
cost = 
\sum_{i=1}^{n} 
\Big(
\int\limits_{0}^{t_i} (f_i''')^2(t) dt 
+ \int\limits_{0}^{t_i} (g_i''')^2(t) dt 
\Big)
$$
</p>