# 项目记忆 — rm_gazebo_26

> 本文件是项目级记忆，供 agent 会话与协作者快速了解项目结构、分工与关键接口。改动项目结构或接口后请同步更新。

## 项目概览

26 赛季哨兵（sentry）仿真与控制。目录下含两个独立的 ROS 2 工作空间：

| 工作空间 | 职责 | 状态 |
|---|---|---|
| `rm_sim_26/` | **仿真**：Gazebo Sim Harmonic + ROS 2 Jazzy，哨兵全向轮底盘 + 三轴云台，26 场地 | 可运行，已有实例启动 |
| `rm_control_26/` | **控制**：算法/导航/云台控制侧 | 已有 `car_control` 功能包（Python，速度/方向控制 → `/cmd_vel`） |

## rm_sim_26（仿真工作空间）

- 环境：Ubuntu 24.04 + Gazebo Sim Harmonic (gz-sim 8) + ROS 2 Jazzy，包名 `rm_sim_26`。
- 目标：视觉组哨兵导航算法仿真（全向轮底盘真车构型 + RMUC/RMUL 场地）。
- 机器人模型：`sentry`（全向轮 + 三轴云台，几何/质量/惯量取自 SolidWorks 总装，非手工估算）。
- TF 树：`base_link → gimbal_link(大yaw) → small_yaw_link → pitch_link`；
  `livox_lidar` 挂在 `gimbal_link`（**随大 yaw 回转**），`gimbal_imu` 挂在 `pitch_link`。
- 启动：`source install/setup.bash && ros2 launch rm_sim_26 rmuc_2025_sim.launch.py`（launch 内已硬编码场地为 `worlds/rmul_2026_world.world`）。
- 雷达模式：`config/sim_config.yaml` 的 `lidar_mode: auto`，无 NVIDIA 显卡时回退 `gpu_lidar`（进程用 `_no_rgl_` 临时 world）。
- 切换模型：`config/sim_config.yaml` 的 `robot_model`（sentry / mecanum_car）。
- 场地分区着色：`models/rmul_2026` 的 visual 已按结构拆分到 `meshes/parts/`（floor / platform_top / mid_top / walls / structure_faces / under_platform / underside / bumpers，各挂独立材质；该场地 STL 无斜面，全垂直立面）；collision 与 visual 共用同一几何，**物理行为随 mesh 同步**。调色改 `model.sdf` 各 visual 的 `<material>`；重新拆分：`python3 scripts/split_field_mesh.py`（含逐顶点一致性校验）。
- **logo 凸起已移除（2026-09-01）**：原三处毫米级凸起标识（中央低地内 1.2×1.05m 图案高 1mm、西北/东南角 1.5×2.0m 图案高 3+2mm，180° 对称）已压平到承载面下方 0.5mm 并删除其立面（4490→4298 tri），原网格备份 `meshes/rmul_2026.stl.orig`；改为颜色标识贴片+描边框：`mark_center.stl` 黑、`mark_nw.stl` 红 / `mark_se.stl` 蓝（贴片抬升 0.5mm，框再叠 0.5mm）。重新生成：`python3 scripts/flatten_logo_marks.py`（幂等，含写回复核）→ `split_field_mesh.py` → `colcon build`。
- **地面已铲平（2026-09-01）**：可行驶地面统一到 z=0，仅保留 200/400mm 平台、围墙、立柱等场地结构。处理（`python3 scripts/flatten_ground_plane.py`，dry-run 支持，含覆盖/零面积/写回复核）：① 中央 12mm 岛（2.4×2.4m，含顶面+裙边+底面）压平/删除；② 四角 20mm 墩台顶面压到 0、口袋墙底部 20→0；③ 三处 0.5mm 凹坑抬到 0（上次压平 logo 遗留）与 5 面 3mm 边界立面删除（上次按质心判定漏删）；④ 三处颜色贴片全部重新贴到地板面 z=0。4298→4277 tri，备份 `meshes/rmul_2026.stl.pre_flatten`。注意：**地板面在 logo/岛/墩台区域下方原本挖空，此类特征必须"压到 0"补地板而不能直接删除（会留洞）**；`/model/sentry_bot/pose` 话题实测发布的是 livox_lidar 相对位姿而非 base_link 世界位姿，查真值用 gz 侧 `/world/default/pose/info`。
- **改动 `models/`、`worlds/`、`launch/` 后必须重新 `colcon build --packages-select rm_sim_26`**：launch 将 `GZ_SIM_RESOURCE_PATH` 指向 install 空间（`install/rm_sim_26/share/rm_sim_26/models`），源码树的新增 mesh/改动不会自动生效，否则报 `uri could not be resolved` 导致 gz server 起不来（2026-09-01 拆分着色后踩过）。

## 话题接口（正在运行，ros2 topic list 实测）

### 指令类（ROS → Gazebo，控制侧写入）
| 话题 | 类型 | 作用 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度 `(vx, vy, wz)`，由 `omni_drive` 节点订阅并解算 |
| `/wheel_fl|rl|rr|fr/cmd_vel` | `std_msgs/msg/Float64` | 四轮关节角速度指令（`omni_drive.py` 逆运动学输出），供 JointController |
| `/gimbal/big_yaw/cmd_pos` | `Float64` | 大 yaw 位置指令（连续无限位，DM4310） |
| `/gimbal/small_yaw/cmd_pos` | `Float64` | 小 yaw 位置指令（GM6020，限位 ±1.57 rad） |
| `/gimbal/pitch/cmd_pos` | `Float64` | pitch 位置指令（连杆，限位 -0.35~0.55 rad） |

### 传感器类（Gazebo → ROS，算法订阅）
| 话题 | 类型 | 作用 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Mid360 点云，gpu_lidar 模式 32×1875，量程 0.1–40m，10Hz（实测 ~6.6Hz），斜装 60° 随大 yaw 回转 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Mid360 内置 IMU，100Hz，frame 在 livox_lidar |
| `/gimbal/imu` | `sensor_msgs/msg/Imu` | 自瞄 IMU（DM-IMU-L1），pitch 体上，200Hz |

### 状态/反馈类
| 话题 | 类型 | 作用 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 7 关节（4 轮 + 大/小 yaw + pitch）位置/速度/力矩 |
| `/model/sentry_bot/pose` | `geometry_msgs/msg/PoseStamped` | 注意：实测发布的是 livox_lidar 相对 model 的位姿（随云台变化），非 base_link 世界位姿；模型世界真值查 gz 侧 `/world/default/pose/info` |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟（use_sim_time=true） |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | TF 变换（robot_state_publisher + lidar 静态变换） |

### 已桥接但无数据源（空转，注意）
| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera` | `sensor_msgs/msg/Image` | 自瞄相机；`model.sdf` 无相机传感器，仅 URDF 有 `camera_mount` 安装点。不影响导航 |
| `/chassis/imu` | `sensor_msgs/msg/Imu` | 底盘 IMU；SDF 中无此传感器。不影响导航（LIO 用 `/livox/lidar` + `/livox/imu`） |

## rm_control_26（控制工作空间）

- 包 `car_control`（Python，ament_python 构建类型，位于 `src/car_control`），职责：小车运动控制 → 发布 `/cmd_vel`。
- 节点：
  - `car_controller`：核心控制节点。订阅指令/速度 → 限速 + 看门狗自动停车 → 发布 `/cmd_vel`；提供急停服务。
  - `cmd_publisher`：命令行演示/调试节点（向指令话题发布字符串指令）。
  - `keyboard_teleop`：键盘控制 UI 节点（WASD/方向键实时控制，`+/-`/`[]` 调速，`e` 急停，`q` 退出；仅用标准库 termios/tty/select，无第三方依赖，需交互式 TTY）。
  - `qt_control_panel`：PyQt5 图形控制面板（方向按钮 + 速度滑块 + 急停 + 状态显示；QTimer 驱动 `spin_once` 单线程集成 ROS，`rclpy.ok()` 守护 + 自定义 SIGINT/SIGTERM 处理器实现优雅退出；依赖 `python3-pyqt5`）。
- 接口：
  | 接口 | 类型 | 方向 | 作用 |
  |---|---|---|---|
  | `/car_control/cmd` | `std_msgs/msg/String` | 输入 | 字符串指令 `forward/backward/left/right/stop [速度]`，另有全向轮平移 `strafe_left/strafe_right` |
  | `/car_control/cmd_vel` | `geometry_msgs/msg/Twist` | 输入 | 上层算法节点直接下发速度 `(linear.x/y, angular.z)` |
  | `/cmd_vel` | `geometry_msgs/msg/Twist` | 输出 | 底盘速度，对接仿真 `omni_drive` |
  | `/car_control/emergency_stop` | `std_srvs/srv/Trigger` | 服务 | 紧急停车 |
- 参数：`config/car_control.yaml`（默认线速 0.5 m/s、角速 0.8 rad/s、限速、看门狗 timeout 0.5s）。
- 启动：`colcon build --packages-select car_control && source install/setup.bash && ros2 launch car_control car_control.launch.py`。

## 导航栈（fastlio2 + localizer + livox_to_laserscan + nav2，2026-09-09 修复后全链可用）

- 启动：`./scripts/start_navigation.sh`（前提：仿真已起、**车在 spawn 点 (-5,0,0.3) yaw=0**——车被开走后初始定位会对不上，可用 `gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: "sentry_bot", position: {x: -5, y: 0, z: 0.3}, orientation: {w: 1}'` 归位后重跑）。组件日志落盘在 `logs/navigation/*.log`（fastlio2/localizer/livox_to_laserscan/nav2），排障先看这里。
- **fastlio2 已修复三处**（`src/fastlio2`）：
  1. `lio_node.cpp`：config 的 `r_il` 经 SVD 投影到严格正交旋转阵——9/7 填入的 4 位小数斜装外参正交误差 ~1e-4，超过 Sophus `SO3(Matrix)` 断言，导致每次进 `IESKF::update()` 必 SIGABRT（exit -6），此前 `/fastlio2/lio_odom` 无发布者。
  2. `lio_node.cpp` `imuCB`：livox 内置 IMU 与雷达同源平行，IMU 测量先乘 `r_il` 转到 base_link 系再积分——否则 LIO 状态是斜装 IMU 体姿态（含 60° roll），`odom→base_link` TF 会带 60° 斜装角，nav2 costmap 方向全错。
  3. `lidar_processor.cpp`：选点分母 `sqrt(norm())`（四次方根）改为 `norm()`。
- **`r_il` 的安装角除 60° 斜装外还含约 -77.4° yaw**（ZYX 分解 atan2(-0.9762, 0.2169)），world1 旧地图（08:48 建）odom 系 x 轴沿雷达朝向（非车头）；修复后 LIO 的 odom 系 x 轴沿车头，两者差 77.4° yaw + `t_il` 平移。**脚本默认初始位姿 (x=-0.043, y=0.211, z=-0.336, yaw=1.35) 即该变换**，仅对"车在 spawn 点 + 旧地图"有效；**用修复后 LIO 重建地图后应改回全 0**。
- `livox_to_laserscan`（`pointcloud_to_scan.py`）：TF 查询用 `Time(0)`（最新）而非点云时刻——慢仿真 RTF<1 下点云 stamp 恒超前 robot_state_publisher TF 0.2~0.7s，按点云时刻查会持续 "extrapolation into the future" 丢帧（/scan 仅 0.5Hz）；**scan 的 stamp 必须保持点云时刻**（lio_odom/map→odom TF 均以点云时刻为基准），改成 joint-TF 时刻会让 controller 报 "Transform data too old" 直接 Goal failed。
- `sentry_nav2_bringup` `nav2_params.yaml`：local costmap voxel 层 `origin_z: 0.0 → -0.2`（传感器原点 z≈0 恰在体素下边界，被拒后无法 raytrace，每帧刷 "out of map bounds" 警告）。
- localizer 的 `relocalize` 服务返回 success 只代表已受理，配准是否收敛要查 `/localizer/relocalize_check`（`interface/srv/IsValid`）；脚本在启动 nav2 前轮询该服务 45s，未收敛会 fail 退出（否则 nav2 激活时 `odom→base_link`/`map→odom` 缺失，controller 激活超时导致整栈 inactive）。速度链路：controller → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → `/cmd_vel`。
- **DWB 插件有自己的 transform_tolerance（2026-09-09 修复单点导航必失败）**：nav2_params.yaml 里 `controller_server.FollowPath.transform_tolerance`（DWB `transformGlobalPlan` 把 robot pose 从 odom 变到 map 时经 `nav_2d_utils/tf_help` 校验 map→odom stamp 年龄）默认 0.1，曾设 0.2 仍不够——慢仿真下 localizer 1Hz ICP 阻塞使 map→odom stamp 滞后 odom 链 0.19s(峰值 0.7s)，超限即报 "Transform data too old / Unable to transform robot pose into global plan's frame" → Goal failed。已放宽到 2.0（costmap/behavior_server/collision_monitor 同名参数此前已各自放宽，DWB 是当时漏掉的一处；map→odom 两次配准间数值不变，放宽无精度损失）。**凡 TF 时间戳容差问题，需逐节点查每个插件自身的 transform_tolerance**。
- localizer RViz（显示 `/localizer/map_cloud` + `/fastlio2/body_cloud`，fixed frame=map）已由 `start_navigation.sh` 随栈启动，受 `--no-rviz` 控制。
- **仿真中途重启会卡死 robot_state_publisher（2026-09-09 实测）**：11:07 用 `start_sim_teleop.sh` 重启仿真（其 cleanup 杀旧 gz/rsp/桥接）时，新 rsp 进程存活且日志打印 "Robot initialized"、URDF 参数完好，但 DDS 参与者僵死——不在 `ros2 node list`、不订阅 `/joint_states`、不发布任何 TF（SIGTERM 也杀不掉，需 kill -9）。症状链：`/tf_static` 无 gimbal 链 → cloud_to_scan 报 "two or more unconnected trees" → `/scan` 无数据（话题存在但 0 发布）→ costmap 无障碍物。修复：`kill -9 <旧rsp_pid>` 后用原 launch 的两个 `/tmp/launch_params_*` 文件重启 rsp 进程即可（`--params-file` 一个含 `use_sim_time: true`，一个含 `robot_description`），TF 树与 `/scan`（~9.8Hz）立即恢复，导航栈其余部分无需重启。**排障口诀：话题存在≠有数据**，`start_navigation.sh` 的 `wait_for_topic` 已改为 `ros2 topic echo --once` 数据级校验，能当场拦住此类故障。
- **重启仿真的正确顺序**：先停导航栈（`pkill -f start_navigation.sh` 让其 cleanup 带走全栈）→ 再跑 `start_sim_teleop.sh` → 把车归位 spawn → 最后 `start_navigation.sh`。导航栈运行期间重启仿真 = 必踩 rsp 卡死坑。
- **start_navigation.sh 自带单例清场（2026-09-09）**：脚本启动前 `kill_stale_nav` 自动终止残留导航进程——旧 start_navigation.sh 自身（TERM 触发其 cleanup）、fastlio2 lio_node、localizer_node、livox_to_laserscan/pointcloud_to_scan、`ros2 launch sentry_nav2_bringup` 与全部 `lib/nav2_*`/`opennav_docking` 组件、两个导航侧 rviz（localizer.rviz / nav2_red_scan.rviz），TERM 后 2s 再 KILL 兜底；**不碰仿真侧**（gz/rsp/桥接/omni_drive）。背景：13:0x 用户在 12:20 栈未退的情况下又跑了一份脚本，两套 lio/localizer/nav2 并存——`/fastlio2/lio_odom` 出现 2 个发布者、两份 map→odom 交替广播，localizer 定位持续漂移。**定位莫名漂移先查 `ros2 topic info /fastlio2/lio_odom -v` 的 Publisher count 是否为 1、`pgrep -cf 'lib/nav2_'` 是否单套**。

## 关键注意点

- **导航**（LIO 定位/建图/规划）依赖 `/livox/lidar` + `/livox/imu`，两者刚体固连，是 FAST-LIO / LIO-SAM 的标准输入。
- **Mid360 随大 yaw 回转**：导航时若云台自转，点云坐标系相对底盘旋转；需云台锁定或正确接入 TF 链 `base_link → gimbal_link → livox_lidar`。
- `scripts/omni_drive.py` 的 `SIGN=-1.0`（车动方向反则取反）；安装半径 `R=0.245m`、轮半径 `r=0.076m`。
- 模型改动后核对惯量：`python3 scripts/check_inertia.py models/sentry/model.sdf`。
- **场地中央/东侧结构是垂直台阶，本车构型不可攀爬**（2026-09 无头复现实测）：`rmul_2026` 为整块 STL 网格碰撞体，中央 0.2m 平台（x≈-3.5 立面）、二级 0.4m 台（x≈-1.65 立面）与 x≈-1.5 的 0.388m 垂直悬崖均为垂直面，无坡道倒角；`base_collision` 圆柱（r=0.30，底隙仅 6.6cm，前伸超轮球 5.1cm）会先于轮子撞上 >6.6cm 的垂直面，产生剧烈 yaw 冲击（实测 >1 rad/s），表现为"中间路段不按速度指令左右偏摆"，甚至顶死卡住（JointController 无看门狗，会保持最后轮速指令空转）。全向轮 ODE 摩擦 hack（mu=0.05/mu2=2.0/fdir1=轮轴）仅在法向近垂直的平面接触下方向正确，立面/棱边接触时高摩擦方向指向竖直或斜向，进一步放大偏转。根因在仿真侧（场地网格 + 底盘碰撞体），控制节点为开环无反馈、与此无关。
