# PB Omni PID Pursuit Controller 完整解析报告

## 1. 报告范围

本文解析 `pb_omni_pid_pursuit_controller` 功能包及其在 `sentry_nav2_bringup` 中的接入方式，覆盖插件结构、控制流程、路径处理、PID 算法、速度约束、碰撞检测、参数配置、运行时接口、已知风险与调试建议。分析基于当前工作区源码和配置，不代表上游仓库后续版本。

涉及的主要文件：

- `src/pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp`
- `src/pb_omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp`
- `src/pb_omni_pid_pursuit_controller/src/pid.cpp`
- `src/pb_omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/pid.hpp`
- `src/pb_omni_pid_pursuit_controller/pb_omni_pid_pursuit_controller.xml`
- `src/pb_omni_pid_pursuit_controller/package.xml`
- `src/pb_omni_pid_pursuit_controller/CMakeLists.txt`
- `src/sentry_nav2_bringup/config/nav2_params.yaml`
- `src/sentry_nav2_bringup/package.xml`

## 2. 总体结论

该功能包实现了一个 `nav2_core::Controller` 插件，将路径跟踪拆成两个控制量：

1. 平移 PID 根据机器人坐标系原点到前视点的距离生成合成线速度。
2. 旋转 PID 根据目标姿态角生成角速度。
3. 合成线速度再按照前视点方向分解为 `linear.x` 和 `linear.y`，因此适用于可横移的全向底盘。
4. 输出前依次执行曲率降速、终点接近降速和离散路径点碰撞检查。

当前接入只替换 Nav2 `controller_server` 的 `FollowPath` 插件，定位、全局规划、代价地图、速度平滑与碰撞监控等其余链路不变。功能包已经通过插件 XML、构建导出和 bringup 运行依赖接入 Nav2。

不过，源码中存在若干值得优先处理的问题：动态参数更新没有同步重建 PID；`min_max_sum_error` 实际未用于 PID；速度限制接口未实现；碰撞检查只检查十个离散点且越界时直接判定无碰撞；曲率计算和状态初始化存在边界风险；旋转对齐逻辑使用局部路径末端姿态而非最终全局目标姿态；配置中的 `min_y_velocity_threshold: 0.5` 可能吞掉较小的横向里程计速度。

## 3. 软件结构与插件接入

### 3.1 类与接口

核心类 `pb_omni_pid_pursuit_controller::OmniPidPursuitController` 继承 `nav2_core::Controller`，实现以下生命周期和控制接口：

- `configure()`：声明并读取参数，保存 TF 与代价地图对象，创建发布器和 PID 实例。
- `activate()`：激活生命周期发布器并注册动态参数回调。
- `deactivate()`：停用发布器并注销动态参数回调。
- `cleanup()`：释放发布器。
- `setPlan()`：保存全局路径。
- `computeVelocityCommands()`：计算一次速度指令。
- `setSpeedLimit()`：当前仅输出警告，没有实际限速。

插件由 `PLUGINLIB_EXPORT_CLASS` 导出。插件描述文件声明其基类为 `nav2_core::Controller`，类名与配置中的插件字符串一致：

```yaml
plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
```

### 3.2 构建与依赖

功能包使用 C++17 和 `ament_cmake_auto`，构建一个共享库，源码包括控制器与 PID 两部分。`pluginlib_export_plugin_description_file` 导出插件描述。依赖包含 Nav2 Core、Nav2 Util、Nav2 Costmap、TF2、消息类型与 pluginlib。

`sentry_nav2_bringup/package.xml` 已添加：

```xml
<exec_depend>pb_omni_pid_pursuit_controller</exec_depend>
```

这保证由 bringup 功能包启动时，运行环境能够解析控制器插件依赖。

## 4. 单周期控制流程

`computeVelocityCommands()` 的实际数据流如下：

```text
全局路径 + 当前位姿 + 当前速度
          |
          v
路径裁剪并转换到 base_link
          |
          v
按速度计算前视距离并选取前视点
          |
          v
平移距离 PID + 目标姿态角 PID
          |
          v
曲率降速 -> 终点接近降速
          |
          v
线速度按前视点方位分解为 vx、vy
          |
          v
十点离散碰撞检查 -> TwistStamped
```

控制函数在进入时同时锁住控制器参数互斥量和 Costmap 互斥量。这样可阻止动态参数回调和代价地图更新与当前计算并发，但也意味着随后执行的 TF、路径转换、曲率计算和发布过程均占用 Costmap 锁，实时负载较高时可能增加等待时间。

## 5. 路径转换与裁剪

### 5.1 最近点搜索

控制器先把机器人位姿转换到全局路径坐标系，然后使用 `max_robot_pose_search_dist` 对最近点搜索范围设上限，避免在自交或回折路径上误选后续路径段。默认值由局部代价地图最大边长的一半得到。

### 5.2 局部路径截取

从最近路径点开始，提取距离机器人不超过代价地图最大范围的路径点，并逐点变换到机器人底盘坐标系。生成的局部路径发布到 `local_plan`。

处理完以后，已经经过的全局路径前缀会从 `global_plan_` 中删除，这是典型的路径剪枝。若原始路径为空、位姿转换失败或转换后路径为空，控制器抛出 `nav2_core::PlannerException`。

### 5.3 坐标系含义

局部路径位于 `base_link`，机器人自身位于原点。因此：

- 前视点距离是 `hypot(x, y)`。
- 前视点方位是 `atan2(y, x)`。
- 输出线速度可直接按该方位分解到机器人坐标系的 X、Y 轴。

## 6. 前视距离与前视点

### 6.1 前视距离

固定前视模式直接使用 `lookahead_dist`。速度缩放模式使用：

```text
lookahead = clamp(hypot(vx, vy) * lookahead_time,
                  min_lookahead_dist,
                  max_lookahead_dist)
```

当前配置启用速度缩放，范围为 0.35 至 0.8 m，预测时间为 1.0 s。低速时前视距离不会小于 0.35 m，高速时不会超过 0.8 m。

### 6.2 前视点选择

控制器选择局部路径上第一个到机器人原点的欧氏距离不小于前视距离的路径点。若整条局部路径均未达到该距离，则使用路径末点。

当前配置启用了插值。当前后两个路径点分别位于前视圆内部和外部时，控制器计算线段与以前视距离为半径、机器人原点为圆心的圆交点，从而减少路径离散密度导致的前视点跳变。

需要注意，交点公式未显式处理零长度线段、负判别式和极小数值误差。如果路径包含重复点或浮点误差导致根号项为负，可能产生 NaN。

## 7. 平移与旋转控制

### 7.1 平移 PID

平移误差为机器人到前视点的距离：

```text
e_translation = hypot(carrot.x, carrot.y)
```

PID 输出一个有符号的合成线速度 `lin_vel`，然后按前视点方向分解：

```text
vx = lin_vel * cos(theta_dist)
vy = lin_vel * sin(theta_dist)
```

这种结构允许底盘沿任意平面方向运动，而不要求车体先转向路径切线，符合全向底盘的运动能力。

### 7.2 旋转 PID

旋转控制由 `enable_rotation` 决定。开启时，旋转 PID 根据目标姿态角计算 `angular.z`。当前还启用了 `use_rotate_to_heading`：若局部路径末端姿态角绝对值大于阈值 0.1 rad，则将平移距离置零，机器人只旋转。

这里有两个语义风险：

1. 代码变量名 `angle_to_goal` 在普通模式取前视点姿态，在旋转对齐模式取局部转换路径的末端姿态。它未必是最终全局目标姿态。
2. 只要角度超过阈值，平移误差就被置零，可能在路径过程中频繁停车旋转，削弱全向底盘本可实现的边走边调姿能力。

如果目标是“只在最终目标附近对准朝向”，建议结合 GoalChecker 状态或剩余路径距离触发，而不是对每个局部路径窗口都执行该判断。

## 8. PID 实现细节

PID 计算公式为：

```text
e(k) = set_point - pv
P = kp * e(k)
integral += e(k) * dt
I = ki * integral
D = kd * (e(k) - e(k-1)) / dt
output = clamp(P + I + D, min, max)
```

积分状态被固定限制在 `[-1, 1]`。这里存在实现顺序问题：代码先计算 `i_out`，再裁剪 `integral_`，因此当前周期的积分输出仍可能使用超限值，裁剪要到下一周期才生效。

此外：

- 参数 `min_max_sum_error` 被声明、读取并支持动态更新，但没有传入 PID，也没有调用 `setSumError()`，因此当前不生效。
- 动态更新 `translation_kp/ki/kd` 或 `rotation_kp/ki/kd` 只修改控制器成员变量，已经创建的 PID 对象仍持有旧值。
- 动态更新速度上下限也不会更新 PID 内部的 `max_` 与 `min_`。
- PID 没有重置接口，切换路径、重新激活控制器或从碰撞异常恢复后，历史积分与微分状态仍可能保留。
- `dt` 来自 `1 / controller_frequency`，没有检查频率是否为零或负数。

因此，目前除 `transform_tolerance`、前视距离、曲率等直接读取成员变量的参数外，PID 增益和 PID 输出上下限不是真正实时可调。

## 9. 曲率降速

### 9.1 曲率估计

控制器计算局部路径累计弧长，并以前视点到机器人原点的欧氏距离近似其累计位置，然后分别在该位置前后按 `curvature_backward_dist` 和 `curvature_forward_dist` 取点。三点拟合圆，曲率为圆半径的倒数。

当前配置：

- `curvature_backward_dist: 0.3` m
- `curvature_forward_dist: 0.7` m
- `curvature_min: 0.4` 1/m
- `curvature_max: 0.5` 1/m
- `reduction_ratio_at_high_curvature: 0.7`

曲率低于 0.4 时不主动降速；0.4 到 0.5 之间线性插值；高于 0.5 后目标速度约为原速度的 70%。

### 9.2 变化率限制

目标降速值不会直接应用，而是通过 `max_velocity_scaling_factor_rate * control_duration` 限制单周期变化量，试图减少速度突变。

但成员名 `last_velocity_scaling_factor_` 实际保存的是上一次线速度，不是缩放因子，并且在头文件中没有初始化。首次调用时读取未初始化值属于未定义行为，应在构造或 `configure()` 中明确赋值。

另外，函数最后强制：

```text
scaled_linear_vel >= 2 * min_approach_linear_velocity
```

当前参数使曲率限制后的速度下界为 0.1 m/s。该下界与终点接近降速的 0.05 m/s 下界语义不同，可能导致两套限速逻辑难以直观调参。

### 9.3 数值边界

三点共线时圆心公式分母趋近零。代码在计算半径后检查 NaN、Inf 和极小半径，并返回一个极大半径，但分母为零的浮点运算仍会先发生。建议先检测三点叉积绝对值，再决定返回零曲率。

`findPoseAtDistance()` 在相邻累计距离相同的情况下会出现插值分母为零，重复路径点需要提前过滤或专门处理。

## 10. 终点接近降速

控制器先计算局部路径积分长度。当剩余路径长度小于 `approach_velocity_scaling_dist` 时，根据机器人到局部路径末点的欧氏距离得到缩放比例：

```text
scale = distance_to_last_pose / approach_velocity_scaling_dist
```

当前配置从 0.8 m 开始降速，最低线速度为 0.05 m/s。

需要注意，函数通过 `std::min(linear_vel, approach_vel)` 合并速度。该写法隐含线速度非负的假设。若未来允许倒车或 PID 产生负速度，以数值大小直接取最小值可能使负值绝对值反而更大，应按速度绝对值或运动方向分别处理。

## 11. 碰撞检测

控制器从变换后的局部路径中均匀抽取十个索引，将这些路径点转换到 Costmap 全局坐标系，再逐点查询栅格代价。如果任一点代价不低于 `INSCRIBED_INFLATED_OBSTACLE`，则抛出规划异常并停止输出运动指令。

该实现的安全边界较弱：

- 仅检查路径中心线上的十个离散点，没有使用机器人 footprint。
- 点间障碍物可能被漏检，长路径或高曲率路径更明显。
- 不检查从当前位姿按实际速度指令形成的时间参数化轨迹。
- `transformPose()` 的返回值被忽略，转换失败时仍可能压入默认位姿。
- 任一点位于 Costmap 外部时函数直接返回 `false`，把越界解释为“没有碰撞”，且后续点不再检查。
- 抽样使用整数索引，路径短于十点时会重复检查相同点。

Nav2 外层的 collision monitor 能提供额外保护，但不能替代控制器内部对速度指令和机器人 footprint 的可靠碰撞预测。建议使用 Costmap footprint 碰撞检查器，并按空间分辨率或预计运动时间对整段轨迹采样。

## 12. 当前 Nav2 配置解析

### 12.1 控制器服务器

当前配置的关键项：

```yaml
controller_frequency: 20.0
odom_topic: /fastlio2/lio_odom
controller_plugins: ["FollowPath"]
```

控制周期为 0.05 s。速度反馈来自 Fast-LIO2 里程计。`FollowPath` 已由 DWB 替换为 Omni PID Pursuit。

### 12.2 速度边界

```yaml
v_linear_min: -0.52
v_linear_max: 0.52
v_angular_min: -1.0
v_angular_max: 1.0
```

平移 PID 输出是合成线速度，分解后的 X、Y 分量绝对值不会超过合成速度上限。`v_linear_min` 允许负值，但正常距离误差非负，典型情况下不会主动生成倒车速度。

### 12.3 里程计速度阈值

`controller_server` 当前配置：

```yaml
min_x_velocity_threshold: 0.001
min_y_velocity_threshold: 0.5
min_theta_velocity_threshold: 0.001
```

横向阈值 0.5 m/s 非常接近最大合成线速度 0.52 m/s。这意味着绝大多数横向速度反馈可能被过滤为零，进而影响速度缩放前视距离。对于全向底盘，建议根据噪声实测把 Y 阈值设置到与 X 同数量级，例如 0.001 至 0.02 m/s，而不是 0.5 m/s。

### 12.4 目标检查

当前使用 `StoppedGoalChecker`：

- XY 容差 0.25 m
- 航向容差 0.25 rad
- 停止线速度阈值 0.05 m/s
- 停止角速度阈值 0.15 rad/s

控制器的最小接近线速度同为 0.05 m/s。若控制命令持续维持在恰好 0.05 m/s，受测量误差影响，GoalChecker 可能需要较长时间才能确认“已停止”。应通过实测确认停止判定使用严格小于还是小于等于，并给控制器最低速度和停止阈值留出裕量。

### 12.5 TF 容忍时间

控制器和局部代价地图均使用 2.0 s 的 TF 容忍时间，以覆盖当前慢仿真中 `map -> odom` 更新延迟。它提高了可用性，但也允许控制器使用更陈旧的变换。后续优化 localizer 周期与阻塞后，应重新缩小该值并记录最大、P95 和 P99 TF 延迟。

## 13. 发布话题与可观测性

控制器创建三个生命周期发布器：

- `local_plan`：裁剪并转换到机器人坐标系的局部路径。
- `lookahead_point`：当前前视点。
- `curvature_points_marker_array`：曲率计算使用的前后采样点，近点为绿色，远点为红色。

建议在 RViz 同时显示局部路径、前视点、曲率采样点、局部 Costmap、机器人 footprint 与速度向量。结合 `cmd_vel`、里程计速度、路径误差与 PID 各分项的时间曲线，可显著提高调参效率。当前代码没有发布 P/I/D 分量、曲率值、降速比例和碰撞采样结果，建议增加可选调试消息或 diagnostics。

## 14. 动态参数支持现状

动态回调支持多数 double 与部分 bool 参数，但存在不完整之处：

- `enable_rotation` 在 configure 中读取，但动态回调没有处理。
- `max_robot_pose_search_dist` 在 configure 中读取，但动态回调没有处理。
- PID 增益虽被回调写入成员变量，却不会更新 PID 实例。
- PID 输出上下限同样不会更新 PID 实例。
- 参数没有范围校验，例如 `curvature_max <= curvature_min` 会造成除零。
- 没有校验前视距离最小值是否大于最大值、速度下界是否小于上界、控制频率是否合法。

更稳妥的实现方式是在回调中先验证整组参数，验证成功后原子更新，并为 PID 提供 `setGains()`、`setLimits()`、`reset()` 等接口。

## 15. 风险分级与整改建议

### P0：运行安全与未定义行为

1. 初始化 `last_velocity_scaling_factor_`，并明确其单位和语义。
2. 碰撞检查变换失败时应安全失败，Costmap 越界不应直接视为无碰撞。
3. 使用 footprint 进行连续或足够密集的轨迹碰撞检查。
4. 为圆线交点、重复路径点和三点共线增加数值保护。

### P1：控制正确性

1. 修复动态 PID 参数更新，使新增益与新限幅真正生效。
2. 让 `min_max_sum_error` 实际控制积分限幅，并在计算 I 项前裁剪积分。
3. 在新路径、激活和异常恢复时重置 PID 状态。
4. 实现 `setSpeedLimit()`，正确响应 Nav2 速度限制区域或外部限速请求。
5. 重新设计旋转对齐触发条件，使其面向最终目标，而不是每个局部路径末端。
6. 修正负速度场景下的曲率和接近目标速度合并方式。

### P2：配置与可维护性

1. 将 `min_y_velocity_threshold` 调整到符合全向底盘低速反馈的范围。
2. 修正参数名拼写 `treshold`。为了兼容已有配置，可先同时接受旧名和新名 `threshold`，再逐步弃用旧名。
3. 增加参数描述文件、范围约束和自动化单元测试。
4. 清理日志中遗留的 `regulated_pure_pursuit_controller` 类型名称。
5. 检查 README、默认值与当前实现的一致性。

## 16. 建议测试矩阵

### 16.1 单元测试

- PID：P、I、D 单项输出、饱和、积分抗饱和、重置、动态增益。
- 前视点：空路径、单点路径、重复点、直线、折线、圆交点边界。
- 曲率：直线、已知半径圆弧、三点重合、共线和极小路径段。
- 接近降速：阈值内外、零距离、正负速度。
- 碰撞检测：致命栅格、膨胀栅格、越界、TF 失败和 footprint 贴边。

### 16.2 仿真场景

- 直线路径：检查稳态误差和速度上升时间。
- 90 度与 S 弯：检查曲率降速和横向跟踪。
- 原地横移：验证 Y 速度反馈阈值和全向分解。
- 窄通道与贴障路径：验证 footprint 碰撞安全。
- 终点不同航向：验证接近降速和最终旋转对齐。
- TF 延迟与丢帧：验证 2.0 s 容忍时间下的行为。
- 动态改参：确认增益、限幅、前视距离与开关是否立即生效。
- 外部限速：确认 `setSpeedLimit()` 接入后的绝对值与百分比语义。

### 16.3 建议记录指标

- 横向路径误差 RMS、P95 和最大值。
- 航向误差 RMS 与最大值。
- 到达时间、超调和停止确认时间。
- `cmd_vel` 的速度、加速度与 jerk。
- 控制周期耗时、TF 查询失败率和 Costmap 锁等待时间。
- 曲率降速触发比例与碰撞检查异常次数。

## 17. 调参顺序

建议按以下顺序调试，避免多组机制互相掩盖：

1. 先关闭旋转 PID、曲率降速和速度缩放前视，固定低速验证坐标系、X/Y 方向与路径转换。
2. 仅调平移 PID，先使用 P，再逐步添加 D，最后按需添加很小的 I。
3. 启用速度缩放前视，观察低速振荡和高速切弯。
4. 启用曲率降速，先验证曲率估计，再调整阈值和降速比例。
5. 启用终点接近降速，确保最低命令速度与 GoalChecker 停止阈值有裕量。
6. 最后启用旋转 PID与最终朝向对齐。
7. 完成碰撞、TF 延迟、外部限速和异常恢复测试后再提高速度上限。

## 18. 最终评价

该控制器的总体思路清晰，使用前视点距离控制合成线速度，再按前视方向分解 X/Y 速度，是面向全向底盘的直接实现。路径裁剪、速度自适应前视、曲率降速、终点降速、生命周期管理和 RViz 标记也已经形成完整框架。

当前版本更适合作为可运行的实验性控制器，而不是已经完成安全收敛的生产控制器。优先修复未初始化状态、碰撞检查、PID 动态更新与数值边界后，再围绕全向底盘的姿态策略、速度反馈阈值和最终目标对齐开展系统调参，能够显著提升稳定性、可预测性与安全性。
