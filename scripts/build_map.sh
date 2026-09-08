#!/usr/bin/env bash
# Gazebo 哨兵 FASTLIO2 建图脚本。Gazebo 需提前启动并发布 /clock、/livox/lidar、/livox/imu。
# 用法: ./build_map.sh <map_name> <duration_seconds>
set -euo pipefail

PROJECT_DIR="/home/robomaster/project/rm_gazebo_26"
WORKSPACE_DIR="${PROJECT_DIR}/rm_control_26"
MAPS_ROOT_DIR="${WORKSPACE_DIR}/maps"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
LIO_NODE_EXEC="${WORKSPACE_DIR}/install/fastlio2/lib/fastlio2/lio_node"
PGO_NODE_EXEC="${WORKSPACE_DIR}/install/pgo/lib/pgo/pgo_node"
LIO_CONFIG="${WORKSPACE_DIR}/install/fastlio2/share/fastlio2/config/lio.yaml"
PGO_CONFIG="${WORKSPACE_DIR}/install/pgo/share/pgo/config/pgo.yaml"
SAVE_MAP_SRV="/pgo/save_maps"
SAVE_MAP_SRV_TYPE="interface/srv/SaveMaps"
LIDAR_TOPIC="/livox/lidar"
IMU_TOPIC="/livox/imu"
ODOM_TOPIC="/fastlio2/lio_odom"
READY_TIMEOUT=30
SAVE_TIMEOUT=120

info() { printf '[INFO] %s\n' "$*"; }
ok() { printf '[ OK ] %s\n' "$*"; }
error() { printf '[ERR ] %s\n' "$*" >&2; }
usage() { printf '用法: %s <map_name> <duration_seconds>\n' "$(basename "$0")"; }

if [[ $# -ne 2 ]]; then usage; exit 1; fi
MAP_NAME="$1"
DURATION="$2"
if [[ ! ${MAP_NAME} =~ ^[A-Za-z0-9_-]+$ ]]; then error "地图名称只能包含字母、数字、下划线和连字符"; exit 1; fi
if [[ ! ${DURATION} =~ ^[0-9]+$ ]] || (( DURATION <= 0 )); then error "建图时长必须是正整数秒"; exit 1; fi

for file in "${ROS_SETUP}" "${WORKSPACE_SETUP}" "${LIO_CONFIG}" "${PGO_CONFIG}"; do
    [[ -f ${file} ]] || { error "缺少文件: ${file}"; exit 1; }
done
for executable in "${LIO_NODE_EXEC}" "${PGO_NODE_EXEC}"; do
    [[ -x ${executable} ]] || { error "缺少可执行文件: ${executable}，请先编译 rm_control_26"; exit 1; }
done

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"
set -u
ros2 interface show "${SAVE_MAP_SRV_TYPE}" >/dev/null 2>&1 || { error "服务类型不可用: ${SAVE_MAP_SRV_TYPE}"; exit 1; }

MAP_DIR="${MAPS_ROOT_DIR}/${MAP_NAME}"
PCD_DIR="${MAP_DIR}/pcd"
PGM_DIR="${MAP_DIR}/pgm"
PCD_FILE="${PCD_DIR}/map.pcd"
mkdir -p "${PCD_DIR}" "${PGM_DIR}"

ROS_LOG_DIR="$(mktemp -d)"
LIO_LOG="$(mktemp --suffix=.log)"
PGO_LOG="$(mktemp --suffix=.log)"
SAVE_LOG="$(mktemp --suffix=.log)"
export ROS_LOG_DIR RCUTILS_LOGGING_USE_STDOUT=1
LIO_PID=''
PGO_PID=''
INTERRUPTED=0

stop_process() {
    local pid="$1"
    if [[ -n ${pid} ]] && kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    fi
}
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_process "${PGO_PID}"
    stop_process "${LIO_PID}"
    rm -f "${LIO_LOG}" "${PGO_LOG}" "${SAVE_LOG}" 2>/dev/null || true
    rm -rf "${ROS_LOG_DIR}" 2>/dev/null || true
    exit "${status}"
}
trap cleanup EXIT
trap 'INTERRUPTED=1' INT TERM

check_nodes() {
    if [[ -n ${LIO_PID} ]] && ! kill -0 "${LIO_PID}" 2>/dev/null; then error 'lio_node 异常退出'; tail -n 30 "${LIO_LOG}" >&2 || true; exit 1; fi
    if [[ -n ${PGO_PID} ]] && ! kill -0 "${PGO_PID}" 2>/dev/null; then error 'pgo_node 异常退出'; tail -n 30 "${PGO_LOG}" >&2 || true; exit 1; fi
}
wait_for_topic() {
    local topic="$1" elapsed=0
    while (( elapsed < READY_TIMEOUT )); do
        check_nodes
        if ros2 topic list 2>/dev/null | grep -Fxq "${topic}" && timeout 5 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then return 0; fi
        sleep 1
        ((elapsed += 1))
    done
    return 1
}

info '检查 Gazebo 仿真话题'
for topic in /clock "${LIDAR_TOPIC}" "${IMU_TOPIC}"; do
    wait_for_topic "${topic}" || { error "未收到话题数据: ${topic}，请先启动 Gazebo 哨兵仿真"; exit 1; }
    ok "话题正常: ${topic}"
done
if ros2 node list 2>/dev/null | grep -Eq '^/fastlio2/lio_node$|^/pgo_node$'; then error '已有 FASTLIO2 或 PGO 节点正在运行'; exit 1; fi

info '启动 FASTLIO2 与 PGO'
"${LIO_NODE_EXEC}" --ros-args -r __ns:=/fastlio2 -p config_path:="${LIO_CONFIG}" -p use_sim_time:=true >"${LIO_LOG}" 2>&1 &
LIO_PID=$!
"${PGO_NODE_EXEC}" --ros-args -p config_path:="${PGO_CONFIG}" -p use_sim_time:=true >"${PGO_LOG}" 2>&1 &
PGO_PID=$!
sleep 2
check_nodes

elapsed=0
until ros2 service list 2>/dev/null | grep -Fxq "${SAVE_MAP_SRV}"; do
    check_nodes
    (( elapsed++ >= READY_TIMEOUT )) && { error "保存服务未就绪: ${SAVE_MAP_SRV}"; exit 1; }
    sleep 1
done
wait_for_topic "${ODOM_TOPIC}" || { error "未收到里程计: ${ODOM_TOPIC}"; tail -n 30 "${LIO_LOG}" >&2 || true; exit 1; }

ok "开始建图 ${DURATION}s，按 Ctrl+C 可提前保存"
remaining=${DURATION}
while (( remaining > 0 && INTERRUPTED == 0 )); do
    check_nodes
    printf '\r建图中，剩余 %4ds' "${remaining}"
    sleep 1 || true
    remaining=$((remaining - 1))
done
printf '\n'

info "保存地图: ${PCD_FILE}"
trap '' INT TERM
if ! timeout "${SAVE_TIMEOUT}" ros2 service call "${SAVE_MAP_SRV}" "${SAVE_MAP_SRV_TYPE}" "{file_path: '${PCD_DIR}', save_patches: false}" >"${SAVE_LOG}" 2>&1; then
    error '保存地图服务调用失败'
    cat "${SAVE_LOG}" >&2 || true
    exit 1
fi
[[ -s ${PCD_FILE} ]] || { error "地图文件未生成或为空: ${PCD_FILE}"; cat "${SAVE_LOG}" >&2 || true; exit 1; }
ok "建图完成: ${PCD_FILE} ($(stat -c%s "${PCD_FILE}") bytes)"
