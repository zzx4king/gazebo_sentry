# HERO 2026 Sentry 局部规划器移植说明

## 1. 移植目标

将 HERO_2026_Sentry_NAV 中的局部导航链路移植到本项目的 `rm_control_26` 工作空间，包括：

- `pb_minco_smoother`：Nav2 MINCO 路径平滑器；
- `hero_mpc_controller`：以 MINCO 多项式轨迹为参考输入的 Nav2 MPC 控制器；
- `interfaces/msg/MincoTrajectory.msg`：MINCO 平滑器与 MPC 控制器之间的轨迹消息；
- Nav2 行为树、控制器、平滑器及启动配置。

## 2. 已移植组件

### 2.1 MINCO 平滑器

源码位于：

`rm_control_26/src/pb_minco_smoother`

该包以 `nav2_core::Smoother` 插件形式接入 Nav2，插件类型为：

`pb_minco/MincoSmoother`

实现使用 gcopter 的 `MINCO_S3NU`，生成二维五次多项式轨迹，并包含平滑、时间、速度、加速度和基于代价地图 ESDF 的避障代价。

行为树 `sentry_nav2_bringup/behavior_trees/navigate_to_pose_w_collision_recovery.xml` 已通过 `SmoothPath` 节点调用 `minco_smoother`。

### 2.2 MPC 控制器

源码位于：

`rm_control_26/src/hero_mpc_controller`

该包以 `nav2_core::Controller` 插件形式接入 Nav2，接收 `MincoTrajectory` 多项式轨迹，并通过 acados 生成的全向机器人动态模型求解速度指令。

### 2.3 组件间接口

轨迹消息定义位于：

`rm_control_26/src/interfaces/msg/MincoTrajectory.msg`

MINCO 平滑器发布多项式轨迹，MPC 控制器订阅同一消息，由此保持平滑器输出和控制器参考轨迹的一致性。

## 3. 针对本项目所做的兼容性修改

### 3.1 acados 运行库随插件安装

原构建方式的安装产物依赖源码树中的 acados 动态库路径，且 `DT_RUNPATH` 无法可靠解析 `libacados.so` 的 qpOASES、HPIPM 和 BLASFEO 间接依赖。

已修改 `hero_mpc_controller/CMakeLists.txt`：

- 将插件安装 RPATH 设置为 `$ORIGIN`；
- 使用 `--disable-new-dtags` 生成传统 `DT_RPATH`；
- 将 `libacados.so`、`libhpipm.so`、`libblasfeo.so` 和 `libqpOASES_e.so` 一并安装到插件的 `lib` 目录。

这样安装后的 Nav2 控制器不再依赖手工设置 `LD_LIBRARY_PATH` 或源码目录的绝对路径。

### 3.2 acados 模板与运行时版本兼容

移植的求解器由较新的 `acados_template` 生成，其中包含当前随包运行时不支持的灵敏度选项：

- `with_solution_sens_wrt_params`；
- `with_value_sens_wrt_params`；
- `solution_sens_qp_t_lam_min`。

这些灵敏度在当前控制器中未启用。已移除对应的选项设置，保留运行时默认值，避免 MPC 控制器配置阶段因未知选项中止。

### 3.3 仿真激光雷达 TF 兜底

仿真中观察到 `robot_state_publisher` 进程存在但 DDS 节点和 TF 发布消失的半失效状态。此时点云存在，但 `livox_to_laserscan` 会持续等待 `base_link -> livox_lidar`。

`scripts/start_navigation.sh` 现在会在导航启动前检查该 TF；若缺失，则使用与 fastlio2 外参一致的固定变换启动兜底发布器，确保 `/scan` 能正常生成。

## 4. Nav2 调用链

端到端调用顺序如下：

1. Nav2 接收 NavigateToPose Goal；
2. 全局规划器生成路径；
3. 行为树调用 `minco_smoother` 平滑路径；
4. MINCO 平滑器发布多项式参考轨迹；
5. `hero_mpc_controller` 使用 acados MPC 求解速度指令；
6. 底盘执行 `cmd_vel`，Nav2 持续闭环直至目标完成。

## 5. 构建与测试结论

移植后的包已完成构建和安装，并使用项目导航启动链进行了端到端仿真测试。

测试日志 `logs/navigation/nav2.log` 中的关键证据：

`[FollowPath] 正在配置 MPC 控制器...`

说明 Nav2 Controller Server 实际加载并配置了移植后的 MPC 插件，而不是原有追踪控制器。

在发布 Nav2 Goal 后，日志最终记录：

`Goal succeeded`

说明 NavigateToPose 行为树已执行完成，机器人通过新的 MINCO + MPC 局部导航链路到达目标点。

## 6. 复现方法

在项目根目录执行项目现有的仿真和导航启动脚本，待定位、TF、代价地图以及 Nav2 生命周期节点进入活动状态后，通过 RViz 的 Nav2 Goal 工具或 NavigateToPose Action 发布目标。

验证时建议检查：

- Controller Server 日志中出现 MPC 控制器配置消息；
- Smoother Server 成功加载 `pb_minco/MincoSmoother`；
- 无 acados 未知选项或动态库加载错误；
- `/scan`、TF、里程计持续发布；
- `bt_navigator` 最终输出 `Goal succeeded`。

## 7. 当前工作区说明

为解决最终端到端测试中发现的运行时兼容问题，以下文件保留了未提交修改：

- `rm_control_26/src/hero_mpc_controller/CMakeLists.txt`；
- `rm_control_26/src/hero_mpc_controller/model/c_generated_code/acados_solver_omnidirectional_robot_dynamic.c`；
- `scripts/start_navigation.sh`；
- 本文档。

这些修改均属于本次移植或端到端运行所需的适配，未覆盖或清理工作区中的其他用户改动。
