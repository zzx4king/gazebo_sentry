# gazebo_sentry

面向 RoboMaster 2026 赛季哨兵机器人的 ROS 2 Jazzy 与 Gazebo Sim Harmonic 仿真、建图、定位和控制项目。项目包含仿真工作空间 `rm_sim_26` 与控制算法工作空间 `rm_control_26`。

## 项目完成情况

当前已完成主体仿真与基础算法链路，项目处于可运行、持续联调阶段。

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

### 进行中

- [ ] 完成 FAST-LIO2 建图、重定位与控制工作空间的稳定联调
- [ ] 使用实车测量值校准质量、惯量、关节限位和云台 PID
- [ ] 优化底盘及云台碰撞体，改善台阶和网格边缘处的接触表现
- [ ] 修正并保存统一的 Gazebo 启动视角
- [ ] 完善导航规划、自动控制与整车闭环验证

## 目录结构

- `rm_sim_26/`：Gazebo 仿真、机器人与场地模型、传感器和运动插件
- `rm_control_26/`：FAST-LIO2、定位、建图、控制及地图转换功能包
- `scripts/`：项目级辅助脚本

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

## 主要接口

- `/cmd_vel`：底盘速度指令
- `/livox/lidar`：Livox Mid360 点云
- `/livox/imu`：Livox IMU
- `/gimbal/imu`：云台 IMU
- `/joint_states`：底盘和云台关节状态
- `/gimbal/big_yaw/cmd_pos`：大 yaw 位置指令
- `/gimbal/small_yaw/cmd_pos`：小 yaw 位置指令
- `/gimbal/pitch/cmd_pos`：pitch 位置指令

## 注意事项

- 修改 `rm_sim_26/models`、`worlds` 或 `launch` 后，需要重新构建 `rm_sim_26`。
- Mid360 安装在大 yaw 云台上。导航联调时应锁定云台，或确保动态 TF 被算法正确使用。
- `build`、`install`、`log` 和运行日志不纳入版本控制，需要在本机重新构建生成。

