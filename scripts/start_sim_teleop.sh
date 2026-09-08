#!/usr/bin/env bash
# =============================================================================
# start_sim_teleop.sh — 一键启动 Gazebo 仿真环境 + 键盘控制小车运动
#
# 功能流程：
#   1. 清理残留的 Gazebo / ROS 进程（gz sim、omni_drive、桥接、控制节点等）
#   2. 校验并加载 ROS 2 + 两个工作空间（rm_sim_26 / rm_control_26）环境
#   3. 后台启动仿真：ros2 launch rm_sim_26 rmuc_2025_sim.launch.py
#      （Gazebo + 哨兵模型 + 各话题桥接 + omni_drive 轮系逆解算）
#   4. 等待机器人姿态话题出现（仿真与 spawn 就绪）
#   5. 后台启动控制节点：ros2 launch car_control car_control.launch.py
#   6. 等待 car_controller 节点就绪
#   7. 启动控制 UI：默认启动 Qt 图形面板（qt_control_panel，关闭窗口退出）
#   8. 控制 UI 退出后，自动关闭所有后台子进程并清理残留，不留僵尸
#
# 用法：
#   bash scripts/start_sim_teleop.sh                 # 标准启动（Qt 图形面板）
#   bash scripts/start_sim_teleop.sh --keyboard      # 改用键盘控制（keyboard_teleop）
#   bash scripts/start_sim_teleop.sh --no-cleanup    # 跳过启动前残留清理
#   bash scripts/start_sim_teleop.sh -h              # 显示帮助
#
# 依赖：
#   - 已安装 ROS 2 Jazzy（/opt/ros/jazzy）
#   - rm_sim_26 与 rm_control_26 两个工作区均已 colcon build 完成
#   - 图形桌面环境（Gazebo GUI 需要 DISPLAY；无显示环境请改用 -s 服务器模式）
# =============================================================================
# 注意：不使用 `set -u`（nounset）。ROS/colcon 的 setup.bash 会引用未定义变量
# （如 AMENT_TRACE_SETUP_FILES），nounset 会导致 source 失败；改用显式错误检查。
set -o pipefail

# -----------------------------------------------------------------------------
# 路径与常量配置（如需调整，改这里）
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SIM_WS="${PROJECT_ROOT}/rm_sim_26"
CTRL_WS="${PROJECT_ROOT}/rm_control_26"
SIM_LAUNCH="rmuc_2025_sim.launch.py"      # 仿真启动文件（rm_sim_26 包内）
CTRL_LAUNCH="car_control.launch.py"       # 控制节点启动文件（car_control 包内）
ROBOT_NAME="sentry_bot"                   # 机器人模型名（sentry → sentry_bot）
ROBOT_POSE_TOPIC="/model/${ROBOT_NAME}/pose"  # 机器人姿态话题（spawn 完成标志）
LOG_DIR="${PROJECT_ROOT}/logs"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="${LOG_DIR}/sim_${TIMESTAMP}.log"
CTRL_LOG="${LOG_DIR}/controller_${TIMESTAMP}.log"

# -----------------------------------------------------------------------------
# 全局状态
# -----------------------------------------------------------------------------
DO_CLEANUP=1            # 是否在启动前清理残留进程
UI_MODE="qt"            # 控制方式：qt（图形面板，默认）| keyboard（键盘）
BACKGROUND_PIDS=()      # 记录后台启动的进程 PID，退出时统一关闭
CLEANED_UP=0            # 防止清理函数被重复调用

# -----------------------------------------------------------------------------
# 工具函数
# -----------------------------------------------------------------------------

# 打印普通日志（带时间戳）
log() {
    echo "[$(date +%H:%M:%S)] $*"
}

# 打印错误日志
log_err() {
    echo "[$(date +%H:%M:%S)][错误] $*" >&2
}

# 打印错误并退出（退出时由 trap 触发清理）
error_exit() {
    log_err "$*"
    exit 1
}

# 按命令行模式杀进程（跳过脚本自身与父进程，避免误杀）
# 参数：$1=匹配模式  $2=信号（默认 TERM）
kill_by_pattern() {
    local pattern="$1"
    local signal="${2:-TERM}"
    local pids
    pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
    [ -z "$pids" ] && return 0
    for pid in $pids; do
        # 绝不杀脚本自身或它的父进程
        if [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; then
            continue
        fi
        kill -"$signal" "$pid" 2>/dev/null || true
    done
}

# 清理残留的 Gazebo / ROS 进程（启动前调用，避免端口/资源冲突）
cleanup_residual() {
    log "清理残留的 Gazebo / ROS 进程..."
    # 需要清理的进程命令行特征（按本项目实际运行的节点整理）
    local patterns=(
        "gz sim"                 # Gazebo（gz sim / gz sim server / gz sim gui）
        "gzserver"               # 旧版 Gazebo 服务端（兼容）
        "gzclient"               # 旧版 Gazebo 客户端（兼容）
        "omni_drive"             # 轮系逆运动学节点
        "robot_state_publisher"  # TF 发布节点
        "parameter_bridge"       # ros_gz_bridge 桥接节点
        "ros_gz_sim"             # spawn 机器人等 ros_gz_sim 工具
        "car_controller"         # 控制核心节点
        "keyboard_teleop"        # 键盘控制节点
        "cmd_publisher"          # 命令行演示节点
        "qt_control_panel"       # Qt 图形面板节点
        "${SIM_LAUNCH}"          # 仿真 launch 进程
        "${CTRL_LAUNCH}"         # 控制 launch 进程
    )
    local p
    # 第一遍：优雅终止（SIGTERM），给各进程留出退出清理时间
    for p in "${patterns[@]}"; do
        kill_by_pattern "$p" TERM
    done
    sleep 2
    # 第二遍：强杀仍未退出的顽固进程（SIGKILL）
    for p in "${patterns[@]}"; do
        kill_by_pattern "$p" KILL
    done
    log "残留进程清理完成"
}

# 等待某个 ROS 话题出现（用于判断仿真/节点是否就绪）
# 参数：$1=话题名  $2=超时秒数（默认 90）
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-90}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout" ]; do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

# 等待某个 ROS 节点出现
# 参数：$1=节点名（不带斜杠）  $2=超时秒数（默认 30）
wait_for_node() {
    local node="$1"
    local timeout="${2:-30}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout" ]; do
        if ros2 node list 2>/dev/null | grep -qx "/${node}"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

# 关闭所有由本脚本启动的后台子进程
kill_background() {
    local pid
    for pid in "${BACKGROUND_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            # 用 setsid 启动的进程自成进程组，负 PID 可整组关闭（连同子节点）
            kill -- "-${pid}" 2>/dev/null || kill "$pid" 2>/dev/null || true
        fi
    done
    BACKGROUND_PIDS=()
}

# 总清理：关闭后台子进程 + 兜底扫除残留（幂等，退出时由 trap 调用）
cleanup() {
    [ "$CLEANED_UP" = "1" ] && return
    CLEANED_UP=1
    echo ""
    log "正在关闭所有子进程并清理..."
    kill_background
    sleep 1
    cleanup_residual
    log "清理完成，退出。"
}

# 显示帮助
usage() {
    cat <<'EOF'
用法: bash scripts/start_sim_teleop.sh [选项]

一键启动 Gazebo 仿真环境 + 键盘控制小车运动。

选项:
  -h, --help         显示本帮助
      --keyboard     改用键盘控制（keyboard_teleop），默认启动 Qt 图形面板
      --no-cleanup   跳过启动前的残留进程清理

控制说明（keyboard_teleop 键盘控制）:
  w / ↑  前进        x / ↓  后退
  a / ←  左转        d / →  右转
  s / 空格  停止
  + / -  线速度加减   ] / [  角速度加减
  r      重置速度     e      紧急停车
  q / Ctrl+C  退出（退出前自动停车）
EOF
}

# -----------------------------------------------------------------------------
# 参数解析
# -----------------------------------------------------------------------------
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --keyboard)
            UI_MODE="keyboard"
            ;;
        --no-cleanup)
            DO_CLEANUP=0
            ;;
        *)
            log_err "未知参数: $1"
            usage
            exit 2
            ;;
    esac
    shift
done

# 注册退出清理：无论正常退出、Ctrl+C、SIGTERM，都触发 cleanup
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# -----------------------------------------------------------------------------
# 主流程
# -----------------------------------------------------------------------------
log "=== 启动仿真 + 控制界面 ==="

# 1. 启动前清理残留进程（避免端口/资源冲突）
if [ "$DO_CLEANUP" = "1" ]; then
    cleanup_residual
else
    log "已跳过残留进程清理（--no-cleanup）"
fi

# 2. 校验并加载 ROS 环境
[ -f "$ROS_SETUP" ] || error_exit "未找到 ROS 环境: $ROS_SETUP"
[ -f "$SIM_WS/install/setup.bash" ] || error_exit "rm_sim_26 未构建，请先 cd $SIM_WS && colcon build"
[ -f "$CTRL_WS/install/setup.bash" ] || error_exit "rm_control_26 未构建，请先 cd $CTRL_WS && colcon build"
# shellcheck disable=SC1090,SC1091
source "$ROS_SETUP"   || error_exit "加载 ROS 环境失败"
source "$SIM_WS/install/setup.bash"   || error_exit "加载 rm_sim_26 环境失败"
source "$CTRL_WS/install/setup.bash"  || error_exit "加载 rm_control_26 环境失败"
command -v ros2 >/dev/null 2>&1 || error_exit "找不到 ros2 命令"
log "ROS 环境加载完成"

# 3. 后台启动仿真
mkdir -p "$LOG_DIR"
log "后台启动仿真（日志: $SIM_LOG）..."
setsid ros2 launch rm_sim_26 "$SIM_LAUNCH" >"$SIM_LOG" 2>&1 &
SIM_PID=$!
BACKGROUND_PIDS+=("$SIM_PID")

# 快速失败检测：仿真进程若 3 秒内立即退出，说明启动失败（如包缺失、配置错误）
sleep 3
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    log_err "仿真启动进程立即退出，最近日志如下:"
    tail -n 30 "$SIM_LOG" >&2 || true
    error_exit "仿真启动失败"
fi

# 4. 等待机器人姿态话题出现（仿真 + spawn 就绪，超时 120s）
log "等待仿真就绪（话题 $ROBOT_POSE_TOPIC，最长 120s）..."
if ! wait_for_topic "$ROBOT_POSE_TOPIC" 120; then
    log_err "等待仿真就绪超时，最近日志如下:"
    tail -n 30 "$SIM_LOG" >&2 || true
    error_exit "仿真启动失败或超时"
fi
log "仿真已就绪"

# 5. 后台启动控制节点
log "后台启动控制节点（日志: $CTRL_LOG）..."
setsid ros2 launch car_control "$CTRL_LAUNCH" >"$CTRL_LOG" 2>&1 &
CTRL_PID=$!
BACKGROUND_PIDS+=("$CTRL_PID")

# 6. 等待 car_controller 节点就绪
log "等待控制节点就绪（最长 30s）..."
if ! wait_for_node "car_controller" 30; then
    log_err "控制节点启动失败，日志如下:"
    tail -n 30 "$CTRL_LOG" >&2 || true
    error_exit "car_controller 启动失败或超时"
fi
log "控制节点已就绪"

# 7. 启动控制 UI（默认 Qt 图形面板，阻塞直到用户关闭窗口）
if [ "$UI_MODE" = "keyboard" ]; then
    log "启动键盘控制（q 或 Ctrl+C 退出）..."
    ros2 run car_control keyboard_teleop
else
    log "启动 Qt 图形控制面板（关闭窗口退出）..."
    ros2 run car_control qt_control_panel
fi

# 8. 控制 UI 退出后，脚本结束，由 EXIT trap 触发清理
log "控制界面已退出，正在关闭仿真与所有子进程..."
# （此处无需显式清理，脚本结束即触发 trap cleanup）
