# RM_SIM

## 项目简介

本项目是为了26赛季的视觉组算法仿真而创建，当前聚焦哨兵导航仿真：全向轮底盘（真车构型）与 RMUC 2025 场地。机械臂仿真已移出范围（26赛季无工程机器人）。

哨兵模型 `models/sentry/` 的几何、质量、惯量与传感器位姿提取自 SolidWorks 总装图纸（烧饼备份11.14），非手工估算。

## 环境要求

- Ubuntu 24.04
- Gazebo Sim Harmonic
- ROS 2 Jazzy

## 雷达仿真模式

本项目支持两种雷达仿真方案，可通过配置文件 `config/sim_config.yaml` 切换：

### 1. RGL模式 (需要NVIDIA显卡)

使用 [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) 进行高精度 Livox Mid360 仿真。

```yaml
lidar_mode: "rgl"
```

**要求**：
- NVIDIA显卡 + 驱动
- 编译安装RGLGazeboPlugin

### 2. gpu_lidar模式 (通用)

使用Gazebo内置的 `gpu_lidar` 传感器进行仿真，无需NVIDIA显卡。

```yaml
lidar_mode: "gpu_lidar"
```

### 3. 自动检测 (默认)

自动检测系统是否有NVIDIA显卡，有则使用RGL，否则使用gpu_lidar。

```yaml
lidar_mode: "auto"
```

## 哨兵模型 (models/sentry)

从 SolidWorks 总装提取的实测参数：

| 项 | 值 | 来源 |
|---|---|---|
| 全向轮 | 直径 152 mm、宽 39 mm，4 轮 X 布局 | 网格包围盒 |
| 轮安装半径 | 245 mm（轮心距回转轴心） | 装配变换矩阵 |
| 底盘离地 | 96 mm（base_link 原点在轮心上方 20 mm） | 装配变换矩阵 |
| 整机质量 | 35.44 kg（底盘 13.56 + 云台 12.03 + 其余） | SW 质量属性 |
| 大 yaw | DM4310 + 同步带 + 滑环，无限位 | BOM |
| 小 yaw | GM6020 直驱，轴心前置 58.5 mm、高 121 mm | 装配变换矩阵 |
| pitch | DM4310 + 连杆，铰点在小 yaw 上方 169 mm | 轴承位姿 |
| Mid360 | 斜装 60°，**随大 yaw 回转**（非固定在底盘） | 装配变换矩阵 |
| 自瞄相机/IMU | MV-CA016 与 DM-IMU-L1 均在 pitch 体上 | 装配变换矩阵 |

TF 树：`base_link → gimbal_link(大yaw) → small_yaw_link → pitch_link`，`livox_lidar` 挂在 `gimbal_link` 下。

网格为 GLB 格式（Onshape 导出即 glTF，GLB 是其二进制封装），保留零件层级与材质。

外观由 `scripts/cad/recolor.py` 生成，两层规则：

1. **按名字覆盖**（优先）：裁判系统模块（装甲、灯条、测速、主控、场地交互、定位）为白色塑料，MV-CA016 相机为银灰铝，Livox Mid360 为深灰金属，全向轮辊子为白色聚氨酯，C 板为墨绿 PCB。
2. **按 CAD 材料密度分类**（其余结构件）：玻纤板与打印件哑光黑，铝件与钢件保持金属本色。

原始外观中高饱和度的零件（镜头、灯条）保留其色相。

改配色只需改 `scripts/cad/recolor.py` 的 `OVERRIDES` 与 `PALETTE_SRGB`（CAD 流水线脚本的数据依赖见 `scripts/cad/README.md`）。注意 glTF 的 `baseColorFactor` 是**线性空间**值，渲染时转 sRGB 显示，直接填 0.145 会显示成 RGB≈107 的灰色，脚本已做 sRGB→线性换算。

### 话题接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | Twist | 底盘速度，经 `omni_drive` 节点解算为四轮速度 |
| `/gimbal/big_yaw/cmd_pos`、`/gimbal/small_yaw/cmd_pos`、`/gimbal/pitch/cmd_pos` | Float64 | 云台三轴位置指令 |
| `/livox/lidar`、`/livox/imu` | PointCloud2、Imu | Mid360 |
| `/gimbal/imu` | Imu | 自瞄 IMU（pitch 体） |
| `/joint_states` | JointState | 7 个关节 |

### 云台惯量与参数辨识

SDF 的 `<inertia>` 填的是**关于连杆质心、沿连杆坐标轴**的张量。辨识 `τ = J·θ̈ + b·θ̇ + f_c·sgn(θ̇)` 用的是**关于转轴**的标量 J，两者差一个平行轴项：

下表为 `scripts/check_inertia.py` 直接从 `model.sdf` 解析算出的值，即仿真里实际生效的量（非手算）：

| 轴 | J (kg·m²) | 下游质量 kg | 来源与可信度 |
|---|---|---|---|
| 大 yaw | **0.2368** | 12.736 | CAD 精确值 + 外购件质量修正。质心距轴 8 mm，平行轴项占 0.4% |
| 小 yaw | **0.0185** | 3.446 | CAD 精确值。质心离轴 43.7 mm，平行轴项占 30.4% |
| pitch | **0.0149** | 1.200 | 估计值。CAD 无独立子装配 |
| 单轮 | 0.0032 | 1.343 | 绕自转轴 |

改动模型后重新核对：`python3 scripts/check_inertia.py models/sentry/model.sdf`

注意小 yaw 一项：GM6020 质量从 468 g 补到 960 g 后 J **完全不变**，因为电机正装在小 yaw 转轴上，到轴的垂直距离为零。这是个有用的自检——补质量若让该值变了，说明位置或坐标系算错了。

pitch 体的质量由物理约束定界：小 yaw 总成 2.954 kg 是 CAD 精确值，减去 pitch 估计后若出现负惯量即不可能，据此得 **m_pitch ≤ 1.65 kg**；模型取 1.2 kg。pitch 重力力矩峰值约 0.75 N·m（力臂 64.3 mm）。

#### CAD 材料核查（密度 = 质量 ÷ 体积 反推）

板材是对的，误差在外购件。结构板件在 CAD 中均为 ρ=2000 kg/m³，属玻纤/G10 范围，与实车黑色玻纤板一致（云台底板、弹仓板、悬挂板、防护件、装甲板固定、3508 安装板均为此值）。典型 G10 实测 1850~1950，CAD 取 2000 偏高约 3~8%，可忽略。

真正的误差源是**外购件未赋材料**——共 79 个零件密度≈1000 kg/m³，正是 SolidWorks 未设材料时的默认水密度：

| 零件 | CAD | 实物 | 比值 |
|---|---|---|---|
| M3508+减速箱 | 60.7 g | ~365 g | 0.17× |
| C620 电调 | 8.3 g | ~83 g | 0.10× |
| Livox Mid360 | 52.6 g | ~265 g | 0.20× |
| GM6020 | 468 g | ~960 g | 0.49× |
| MV-CA016 相机 | 110 g | ~55 g | 2.0× |

这些修正已应用到 `model.sdf`，每个 link 的注释标明了来源档次。

**整机质量不能直接用 CAD 根装配的 35.44 kg**——该装配含"干涉/范围"幻影件（做间隙检查用的非实体几何，`定位模块干涉范围` 单件就报 100.7 kg，`装甲板干涉` 报 376.8 kg），其中若干为隐藏件仍计入总质量。真实结构应取两个实体子装配之和：底盘 13.56 + 云台 12.03 = **25.59 kg**。本模型总质量 27.51 kg，差额即上表的外购件补足量，自洽。

实车称重是最终判据。

**用于辨识时必须注意：**

- CAD 惯量是**先验初值**，不是真值。实测 J 通常大于 CAD 值，因为电机转子惯量按传动比平方折算后计入负载：`J_eff = J_load + N²·J_rotor`。
- 大 yaw 的同步带经 CAD 实测为 **1:1**（主动轮与从动轮均为 63.51 mm），因此 N 只来自 DM4310 自身减速比，带传动不额外放大。
- CAD 完全不含摩擦、带弹性、线缆拖拽、齿隙，这些必须靠辨识获得。
- 惯量积在模型中写为 0（实际量级：云台 max|P|=0.033，约为最小主惯量的 15%；小 yaw 为 43%）。这会让惯量主轴与几何轴不重合，影响轴承反力与耦合项，但**不影响绕该轴的标量 J**，故对单轴辨识无影响。
- 质量值依赖 SolidWorks 材料设置，电机、NUC、电池等外购件模型密度常失真。整机 CAD 值 35.44 kg 应与实车称重核对。

建议流程：以上表 J 为初值做正弦扫频或阶跃辨识，若结果与 CAD 值相差在 1.5 倍以内属正常（转子折算 + 摩擦），相差数倍则应回头核查质量设置。

### 待实测校准的量

以下取自 CAD 或估算，实车测得后应替换：

- pitch 与小 yaw 的关节限位（当前为估计值）
- pitch 体质量与惯量（CAD 无独立子装配，按 1.0 kg 拆分）
- 云台三轴 PID 增益（当前为初值，云台参数辨识后校准）
- 各质量属性依赖 SW 材料设置，外购件模型常失真，建议整机称重校准
- Mid360 光学中心相对 CAD 装配原点的偏移（估计 <50 mm）
- `scripts/omni_drive.py` 的 `SIGN` 常量（若车运动方向相反则取反）

## 切换机器人模型

`config/sim_config.yaml`：

```yaml
robot_model: "sentry"       # 26赛季哨兵（默认）
# robot_model: "mecanum_car"  # 旧麦轮测试车
```

## 快速开始

```bash
# 构建
colcon build

# 运行仿真
source install/setup.bash
ros2 launch rm_sim_26 rmuc_2025_sim.launch.py
```

当前仅维护 `rmuc_2025_sim.launch.py` 这一启动入口。

## 仓库结构

- `config/`: 仿真模式与桥接配置
- `launch/`: Gazebo / ROS 2 启动文件
- `models/`: 机器人与场地模型资源
- `plugin/`: Livox Mid360 雷达pattern文件
- `worlds/`: Gazebo world文件

## RGL模式准备

如需启用 `lidar_mode: "rgl"`，先初始化并编译RGL子模块：

```bash
git submodule update --init --recursive
```

## TODO

- ✅ 给机器人加入云台结构
- ✅ 机器人能够发布TF
- ✅ 加入比赛场地模型
- ✅ 实现Mid360的仿真
- ✅ 支持无NVIDIA显卡的仿真方案
- ✅ 车身模型对齐真车：全向轮底盘 + 三轴云台，几何取自 CAD 总装
- [ ] 修正每次启动的视角
- [ ] 用实测值替换上文「待实测校准的量」
- [ ] 碰撞体细化（当前底盘/云台为圆柱包络，轮为球）

## 致谢

致敬[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)开发者的贡献
