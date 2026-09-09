# gazebo_sentry

面向 RoboMaster 2026 赛季哨兵机器人的 ROS 2 Jazzy 与 Gazebo Sim Harmonic 仿真、建图、定位、导航和控制项目。项目包含仿真工作空间 `rm_sim_26` 与控制算法工作空间 `rm_control_26`。

## 项目完成情况

当前已完成主体仿真、LIO 建图定位与 Nav2 单点导航全链路，项目处于可运行、持续联调阶段。

### 已完成

- [x] 搭建 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim Harmonic 仿真环境
- [x] 建立哨兵全向轮底盘与三轴云台模型
- [x] 根据 CAD 数据配置主要几何、质量、惯量与传感器位姿
- [x] 建立完整 TF 链并发布关节状态
- [x] 接入 Livox Mid360 点云与 IMU 仿真
- [x] 支持 RGL 与 Gazebo `gpu_lidar` 雷达模式及自动回退
- [x] 加入并处理 2026 赛季比赛场地，完成区域拆分、着色与地面平整
- [x] 实现全向轮底盘速度控制与云台三轴位置控制接口
- [x] 建立控制工作空间，提供命令行、键盘及 PyQt 控制入口
- [x] 迁入 FAST-LIO2、HBA、PGO、定位与 PCD 转栅格地图相关功能包
- [x] 打通点云、IMU、TF 和底盘速度控制等主要接口
- [x] 修复 FAST-LIO2 斜装外参正交化与 IMU 体坐标系变换，里程计稳定输出
- [x] 打通 FAST-LIO2 + ICP localizer + livox_to_laserscan + Nav2 单点导航全链路
- [x] 提供 `scripts/start_navigation.sh` 一键启动导航栈（含单例清场与双 RViz）
- [x] 移植 KISS-Matcher 全局点云配准（ROS 2 Humble → Jazzy 适配），实测配准收敛

### 进行中

- [ ] 使用实车测量值校准质量、惯量、关节限位和云台 PID
- [ ] 优化底盘及云台碰撞体，改善台阶和网格边缘处的接触表现
- [ ] 切换回 RMUL 2026 真实场地重建地图，验证导航鲁棒性
- [ ] 完善导航规划、自动控制与整车闭环验证

## 目录结构

- `rm_sim_26/`：Gazebo 仿真、机器人与场地模型、传感器和运动插件
- `rm_control_26/`：FAST-LIO2、ICP 定位、点云转激光扫描、Nav2 导航、控制及地图转换功能包
- `rm_control_26/maps/`：已保存的点云/栅格地图（`world1` 为当前导航用图）
- `scripts/`：项目级辅助脚本（仿真、建图、导航一键启动与清场）

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim Harmonic
- colcon

## 构建

```bash
source /opt/ros/jazzy/setup.bash

cd rm_sim_26
colcon build --symlink-install

cd ../rm_control_26
colcon build --symlink-install
```

## 启动仿真

```bash
cd rm_sim_26
source install/setup.bash
ros2 launch rm_sim_26 rmuc_2025_sim.launch.py
```

仿真配置位于 `rm_sim_26/config/sim_config.yaml`。可通过 `robot_model` 切换机器人模型，通过 `lidar_mode` 选择 `auto`、`rgl` 或 `gpu_lidar`。

当前 `rmul_2026_world.world` 使用 20 m × 15 m 平整封闭场地（导航链路联调用），哨兵于 (-5, 0) 出生；RMUL 2026 真实场地模型保留在 `rm_sim_26/models/rmul_2026`，切换回真实场地后需重建地图。

## 启动导航

一键启动定位与 Nav2 单点导航（Fast-LIO2 里程计 → ICP 重定位 → /scan → Nav2）：

```bash
# 前提：仿真已启动，且哨兵位于出生点 (-5, 0) yaw=0。
# 车被开走后可先归位再重跑：
gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --req 'name: "sentry_bot", position: {x: -5, y: 0, z: 0.3}, orientation: {w: 1}'

./scripts/start_navigation.sh
```

脚本自动完成：

- 清理残留导航进程（单例保证，不影响仿真侧）
- 锁定大 yaw 为 0，保证雷达外参恒定
- 启动 Fast-LIO2、ICP localizer（加载 `maps/world1/pcd/map.pcd`）、livox_to_laserscan、Nav2
- 启动 Nav2 RViz 与 localizer RViz（`--no-rviz` 关闭）
- 轮询 `relocalize_check`，定位收敛后才拉起 Nav2

组件日志落盘 `logs/navigation/`。启动后在 RViz 用 2D Goal Pose 发布单点导航目标。

## 主要接口

### 底盘与云台

- `/cmd_vel`：底盘速度指令
- `/livox/lidar`：Livox Mid360 点云
- `/livox/imu`：Livox IMU
- `/gimbal/imu`：云台 IMU
- `/joint_states`：底盘和云台关节状态
- `/gimbal/big_yaw/cmd_pos`：大 yaw 位置指令
- `/gimbal/small_yaw/cmd_pos`：小 yaw 位置指令
- `/gimbal/pitch/cmd_pos`：pitch 位置指令

### 导航栈

- `/fastlio2/lio_odom`：LIO 里程计（odom → base_link）
- `/fastlio2/body_cloud`：车体系点云
- `/scan`：点云转出的 2D 激光扫描（Nav2 costmap 输入）
- `/localizer/map_cloud`：点云地图（localizer RViz 显示）
- `/localizer/relocalize`、`/localizer/relocalize_check`：重定位请求与收敛检查服务

## 点云全局配准（KISS-Matcher）

KISS-Matcher（RA-L 2025）已从 ROS 2 Humble 移植到 Jazzy（vendored 于 `rm_control_26/src/KISS-Matcher/`），提供无需初值的全局配准：

```bash
# 构建（首次配置会从 GitHub 拉 ROBIN 依赖，需网络）
cd rm_control_26
source /opt/ros/jazzy/setup.bash
colcon build --packages-select kiss_matcher_ros

# CLI 配准两个 PCD，输出 4x4 变换与内点判定
./install/kiss_matcher_ros/lib/kiss_matcher_ros/run_kiss_matcher \
    <src.pcd> <tgt.pcd> 0.2 nogui quatro
# 可选 flag: nogui(跳过弹窗) quatro(yaw-only GNC, 车辆场景推荐) noratio(提速)
# resolution 建议 0.2~0.3; 本场地 0.5 因几何特征弱而不收敛

# ROS 节点(读两 PCD 配准并在 RViz 动画演示)
ros2 launch kiss_matcher_ros visualizer_launch.py
```

实测：合成测试恢复已知 60° 旋转误差 <0.2°；80s 驾驶累积点云对 world1 旧地图配准，精确恢复 map→odom 变换（yaw 误差 0.2°、平移 <4cm、0.13s）。

## 注意事项

- 修改 `rm_sim_26/models`、`worlds` 或 `launch` 后，需要重新构建 `rm_sim_26`。
- Mid360 安装在大 yaw 云台上。导航脚本启动时会自动锁定大 yaw 为 0；云台自转会导致雷达外参变化、定位漂移。
- `start_navigation.sh` 自带单例清场；定位莫名漂移时先查 `ros2 topic info /fastlio2/lio_odom -v` 的 Publisher count 是否为 1。
- 慢仿真（RTF < 1）下 TF 时间戳存在滞后，Nav2 各组件的 `transform_tolerance` 已按需放宽，请勿随意调回默认值。
- 重启仿真前必须先停导航栈（`pkill -f start_navigation.sh`），顺序：停导航 → 重启仿真 → 车归位 → 重跑导航脚本。
- `world1` 地图由修复前雷达体系语义构建，导航脚本默认初始位姿为非零补偿值；用修复后 LIO 重建地图后应将 `--x/--y/--z/--yaw` 归零。
- `build`、`install`、`log` 和运行日志不纳入版本控制，需要在本机重新构建生成。

## 致谢

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)：激光惯性里程计
- [MIT-SPARK/KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)：快速全局点云配准（本仓库完成 ROS 2 Jazzy 移植适配）
- [is-buiquocdoanh/livox_to_laserscan](https://github.com/is-buiquocdoanh/livox_to_laserscan)：点云转激光扫描（本仓库在其基础上适配了慢仿真下的 TF 查询与时间戳处理）
- [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)：Nav2 导航框架
