#!/usr/bin/env bash
# 一键启动定位、激光转换、TF 与 Nav2 单点导航，不启动任何离线建图功能。
# 前提：Gazebo 仿真及 /livox/lidar、/livox/imu 已经启动。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
CTRL_WS="${PROJECT_ROOT}/rm_control_26"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
MAP_YAML="${CTRL_WS}/maps/rmul26/pgm/map1.yaml"
MAP_PCD="${CTRL_WS}/maps/rmul26/pcd/map.pcd"
RVIZ=true
# 自动初始位姿: 启动时采集点云, 经 KISS-Matcher/几何搜索双引擎配准出
# T_map_body 喂给 relocalize(车可在场内任意位置); --manual-pose 跳过,
# 使用下方硬编码值(仅对"车在 rmul26 建图起点"有效)。
AUTO_POSE=true
# rmul26 地图为 LIO 修复后所建(odom 原点=建图起点车体位姿, x 轴沿车头),
# 车回到建图起点时 T_map_odom 为单位变换, 故默认值全 0。
# 若换回旧"雷达体语义"地图(如 world1), 需改为对应变换:
# 平移约 -Rot_z(77.4°)*t_il, yaw +77.4°(1.35 rad)。
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_Z=0.0
INITIAL_YAW=0.0
INITIAL_PITCH=0.0
INITIAL_ROLL=0.0
PIDS=()

log() { echo "[$(date +%H:%M:%S)] $*"; }
fail() { echo "[错误] $*" >&2; exit 1; }

usage() {
    echo "用法: bash scripts/start_navigation.sh [--no-rviz] [--manual-pose] [--x 米] [--y 米] [--z 米] [--yaw 弧度] [--pitch 弧度] [--roll 弧度]"
    echo "  默认自动估计初始位姿(KISS-Matcher/几何搜索, 车可在场内任意位置)"
    echo "  --manual-pose: 跳过自动估计, 使用硬编码默认值(全 0, 仅对车在 rmul26 建图起点有效)"
    echo "  车不在建图起点时用 --manual-pose, 请自行传 --x/--y/--z/--yaw"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --no-rviz) RVIZ=false; shift ;;
        --manual-pose) AUTO_POSE=false; shift ;;
        --x) INITIAL_X="$2"; shift 2 ;;
        --y) INITIAL_Y="$2"; shift 2 ;;
        --z) INITIAL_Z="$2"; shift 2 ;;
        --yaw) INITIAL_YAW="$2"; shift 2 ;;
        --pitch) INITIAL_PITCH="$2"; shift 2 ;;
        --roll) INITIAL_ROLL="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) usage; fail "未知参数: $1" ;;
    esac
done

# 组件日志落盘, 便于排障(曾因输出全部丢弃, 排查 lio_node SIGABRT 只能靠 gdb 复现)
NAV_LOG_DIR="${PROJECT_ROOT}/logs/navigation"
mkdir -p "${NAV_LOG_DIR}"
start_component() {
    local name="$1"; shift
    setsid "$@" >"${NAV_LOG_DIR}/${name}.log" 2>&1 &
    PIDS+=("$!")
}

cleanup() {
    trap - INT TERM EXIT
    log "关闭导航相关进程..."
    for pid in "${PIDS[@]}"; do
        kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
    done
}
trap cleanup INT TERM EXIT

# 单例清场: 旧栈残留时两套 lio/localizer/nav2 并存, 互相抢话题(如两个 /fastlio2/lio_odom
# 发布者)与 TF, 导致定位漂移(2026-09-09 实测)。启动前统一终止导航侧进程, 不碰仿真
# (gz/robot_state_publisher/桥接/omni_drive)。
kill_stale_nav() {
    # 1) 旧的 start_navigation.sh 先发 TERM, 让其 cleanup 优雅收尾(排除自身)
    local p
    for p in $(pgrep -f "scripts/start_navigation\.sh" 2>/dev/null); do
        [ "${p}" != "$$" ] && kill -TERM "${p}" 2>/dev/null
    done
    sleep 1
    # 2) 按命令行兜底清理各组件(旧脚本异常退出时其子进程可能残留)
    local pats=(
        "ros2 launch fastlio2"
        "fastlio2/lio_node"
        "localizer_node"
        "ros2 launch livox_to_laserscan"
        "pointcloud_to_scan"
        "ros2 launch sentry_nav2_bringup"
        "lib/nav2_"
        "opennav_docking"
        "rviz2 .*localizer\.rviz"
        "rviz2 .*nav2_red_scan\.rviz"
    )
    local pat
    for pat in "${pats[@]}"; do
        pkill -TERM -f "${pat}" 2>/dev/null
    done
    sleep 2
    # 3) 仍残留者强杀(残留的 lifecycle/SIGTERM 不可杀节点会继续抢话题)
    for pat in "${pats[@]}"; do
        pkill -KILL -f "${pat}" 2>/dev/null
    done
    sleep 1
}

wait_for_topic() {
    local topic="$1" timeout="${2:-60}" elapsed=0
    while [ "${elapsed}" -lt "${timeout}" ]; do
        # 话题存在且确有数据(存在但无数据=发布节点空转, 2026-09-09 rsp 卡死时 /scan 即此状态)
        if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
            timeout 5 ros2 topic echo --once "${topic}" >/dev/null 2>&1 && return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

wait_for_service() {
    local service="$1" timeout="${2:-60}" elapsed=0
    while [ "${elapsed}" -lt "${timeout}" ]; do
        ros2 service list 2>/dev/null | grep -qx "${service}" && return 0
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

[ -f "${ROS_SETUP}" ] || fail "找不到 ${ROS_SETUP}"
[ -f "${CTRL_WS}/install/setup.bash" ] || fail "rm_control_26 尚未构建"
[ -f "${MAP_YAML}" ] || fail "找不到栅格地图 ${MAP_YAML}"
[ -f "${MAP_PCD}" ] || fail "找不到点云地图 ${MAP_PCD}"

source "${ROS_SETUP}"
source "${CTRL_WS}/install/setup.bash"
export ROS_LOG_DIR="${NAV_LOG_DIR}"

log "清场: 终止残留导航进程(仅导航侧, 不碰仿真)..."
kill_stale_nav

# /clock 必须有数据: 时钟桥僵死时(进程存活但 DDS 参与者已死)话题存在却 0 发布者,
# 所有 use_sim_time 节点时钟冻结, costmap 永不发布而导航表面正常(2026-09-09 实测)。
# 排查: pgrep -af 'parameter_bridge /clock'; 桥僵死 SIGTERM 杀不掉, 需 kill -9 后重启仿真。
wait_for_topic /clock 30 || fail "未检测到 /clock 数据，仿真时钟桥可能僵死，请重启仿真"
wait_for_topic /livox/lidar 30 || fail "未检测到 /livox/lidar，请先启动仿真"
wait_for_topic /livox/imu 30 || fail "未检测到 /livox/imu，请先启动仿真"

log "锁定大 yaw 为 0，保证导航期间雷达外参恒定..."
ros2 topic pub --once /gimbal/big_yaw/cmd_pos std_msgs/msg/Float64 "{data: 0.0}" >/dev/null 2>&1 || true

log "启动 Fast-LIO2 定位里程计（不启动其 RViz，不执行离线建图流程）..."
start_component fastlio2 ros2 launch fastlio2 lio_launch.py use_sim_time:=true rviz:=false
wait_for_topic /fastlio2/lio_odom 60 || { tail -20 "${NAV_LOG_DIR}/fastlio2.log" >&2; fail "Fast-LIO2 未发布里程计"; }

# ===== 自动初始位姿: 静止采集 -> 双引擎配准 -> 失败则驾驶采集重试 -> 仍失败回退硬编码 =====
KM_HELPER="${SCRIPT_DIR}/km_initial_pose.py"
KM_BIN="${CTRL_WS}/install/kiss_matcher_ros/lib/kiss_matcher_ros/run_kiss_matcher"
if [ "${AUTO_POSE}" = "true" ]; then
    if [ ! -f "${KM_HELPER}" ] || [ ! -x "${KM_BIN}" ]; then
        log "警告: km_initial_pose.py 或 kiss_matcher_ros 未构建, 跳过自动位姿, 用默认值"
    else
        log "自动估计初始位姿: 静止采集 12s + KISS-Matcher/几何搜索双引擎..."
        KM_RESULT=$(timeout 150 python3 "${KM_HELPER}" \
            --km-bin "${KM_BIN}" --map-pcd "${MAP_PCD}" \
            --collect-sec 12 --resolution 0.2 2>>"${NAV_LOG_DIR}/km_pose.log")
        KM_RC=$?
        if [ ${KM_RC} -ne 0 ] && [ ${KM_RC} -ne 2 ]; then
            log "静止采集未收敛(退出码 ${KM_RC}), 驾驶采集重试(约 40s)..."
            KM_RESULT=$(timeout 180 python3 "${KM_HELPER}" \
                --km-bin "${KM_BIN}" --map-pcd "${MAP_PCD}" \
                --collect-sec 40 --resolution 0.2 --drive 2>>"${NAV_LOG_DIR}/km_pose.log")
            KM_RC=$?
        fi
        if [ ${KM_RC} -eq 0 ] && [ -n "${KM_RESULT}" ]; then
            read -r INITIAL_X INITIAL_Y INITIAL_Z INITIAL_YAW INITIAL_PITCH INITIAL_ROLL <<< "${KM_RESULT}"
            log "初始位姿估计成功: x=${INITIAL_X} y=${INITIAL_Y} z=${INITIAL_Z} yaw=${INITIAL_YAW}"
        else
            log "警告: 自动位姿估计失败(退出码 ${KM_RC}), 回退默认值(全 0, 仅对车在 rmul26 建图起点有效)"
            tail -8 "${NAV_LOG_DIR}/km_pose.log" >&2 || true
        fi
    fi
fi

# localizer_launch.py 会再次启动 Fast-LIO2，造成两个建图/里程计实例。
# 此处只启动定位节点，复用上面唯一的 Fast-LIO2 实例。
log "启动纯定位节点，定位成功后发布 map -> odom..."
start_component localizer ros2 run localizer localizer_node --ros-args \
    -r __ns:=/localizer \
    -p use_sim_time:=true \
    -p config_path:="${CTRL_WS}/install/localizer/share/localizer/config/localizer.yaml"
wait_for_service /localizer/relocalize 60 || { tail -20 "${NAV_LOG_DIR}/localizer.log" >&2; fail "定位服务未就绪"; }

# localizer 自带 RViz(显示 map_cloud / body_cloud / TF), 与 nav2 RViz 一样受 --no-rviz 控制
if [ "${RVIZ}" = "true" ]; then
    log "启动 localizer RViz (地图点云 / 里程计点云)..."
    start_component localizer_rviz rviz2 -d "${CTRL_WS}/install/localizer/share/localizer/rviz/localizer.rviz" \
        --ros-args -p use_sim_time:=true
fi

log "加载已有 rmul26 点云地图进行定位并发布初始定位信息..."
ros2 service call /localizer/relocalize interface/srv/Relocalize "{pcd_path: '${MAP_PCD}', x: ${INITIAL_X}, y: ${INITIAL_Y}, z: ${INITIAL_Z}, yaw: ${INITIAL_YAW}, pitch: ${INITIAL_PITCH}, roll: ${INITIAL_ROLL}}" >/dev/null || fail "初始定位请求失败"

# relocalize 服务返回成功仅代表已受理, 还需等配准收敛(发布 map->odom), 否则 Nav2 激活会超时失败。
log "等待定位配准收敛 (relocalize_check)..."
RELOC_OK=false
for _ in $(seq 1 45); do
    if ros2 service call /localizer/relocalize_check interface/srv/IsValid 2>/dev/null | grep -q "valid=True"; then
        RELOC_OK=true
        break
    fi
    sleep 1
done
if [ "${RELOC_OK}" != "true" ]; then
    tail -20 "${NAV_LOG_DIR}/localizer.log" >&2 || true
    echo "[错误] 定位配准未收敛 (45s)。请检查初始位姿参数或地图，日志: ${NAV_LOG_DIR}/localizer.log" >&2
    exit 1
fi
log "定位收敛，map -> odom 持续发布中。"

log "启动 livox_to_laserscan，发布 /scan..."
start_component livox_to_laserscan ros2 launch livox_to_laserscan livox_scan.launch.py
wait_for_topic /scan 30 || { tail -20 "${NAV_LOG_DIR}/livox_to_laserscan.log" >&2; fail "未检测到 /scan"; }

log "启动 Nav2，加载 rmul26 栅格地图..."
start_component nav2 ros2 launch sentry_nav2_bringup single_point_navigation.launch.py map:="${MAP_YAML}" rviz:="${RVIZ}" use_sim_time:=true

log "TF 链应为 map -> odom -> base_link；传感器固定 TF 由仿真 robot_state_publisher 提供。"
log "组件日志位于 ${NAV_LOG_DIR}/，可在 RViz2 使用 2D Goal Pose 发布单点导航目标。按 Ctrl+C 退出。"
wait "${PIDS[-1]}"
