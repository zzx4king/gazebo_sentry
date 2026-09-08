#!/usr/bin/env bash
#
# pcd2pgm.sh
#
# 将指定 PCD 点云文件转换为 PGM 栅格地图。
# 使用 sentry_ws 工作空间中的 pcd2pgm 节点处理点云并在 /map 话题发布
# OccupancyGrid，再用 nav2_map_server 的 map_saver_cli 保存为 PGM。
#
# 输出目录规则: 与 PCD 文件父级目录同级的 pgm 目录。
#   例如 PCD 位于 .../maps/test/pcd/map.pcd
#        则输出到  .../maps/test/pgm/map.pgm
#
# 用法:
#   ./pcd2pgm.sh <pcd_file> [output_name]
#
# 示例:
#   ./pcd2pgm.sh /home/robomaster/project/sentry_ws/maps/test/pcd/map.pcd
#   ./pcd2pgm.sh /path/to/scan.pcd my_map
#

set -euo pipefail

# ==================== 基础配置 ====================
WORKSPACE_DIR="/home/robomaster/project/rm_gazebo_26/rm_control_26"
PCD2PGM_PKG_DIR="${WORKSPACE_DIR}/src/pcd2pgm"
DEFAULT_CONFIG="${PCD2PGM_PKG_DIR}/config/pcd2pgm.yaml"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
NODE_EXEC="${WORKSPACE_DIR}/install/pcd2pgm/lib/pcd2pgm/pcd2pgm_node"

# 节点处理点云的最大等待时间（秒）
NODE_PROCESS_TIMEOUT=60
# map_saver_cli 超时时间（秒）
MAP_SAVER_TIMEOUT=30

# ==================== 颜色输出 ====================
if [[ -t 1 ]]; then
    C_RED='\033[0;31m'
    C_GREEN='\033[0;32m'
    C_YELLOW='\033[0;33m'
    C_BLUE='\033[0;34m'
    C_RESET='\033[0m'
else
    C_RED=''; C_GREEN=''; C_YELLOW=''; C_BLUE=''; C_RESET=''
fi

log_info()  { echo -e "${C_BLUE}[INFO]${C_RESET} $*"; }
log_ok()    { echo -e "${C_GREEN}[ OK ]${C_RESET} $*"; }
log_warn()  { echo -e "${C_YELLOW}[WARN]${C_RESET} $*"; }
log_error() { echo -e "${C_RED}[ERR ]${C_RESET} $*" >&2; }

# ==================== 运行时状态变量 ====================
NODE_PID=""
TMP_PARAMS=""
NODE_LOG=""
SAVER_LOG=""
ROS_LOG_DIR=""

# ==================== 清理函数 ====================
cleanup() {
    local exit_code=$?
    # 停止 pcd2pgm 节点
    if [[ -n "${NODE_PID}" ]] && kill -0 "${NODE_PID}" 2>/dev/null; then
        log_info "停止 pcd2pgm 节点 (PID: ${NODE_PID}) ..."
        kill "${NODE_PID}" 2>/dev/null || true
        wait "${NODE_PID}" 2>/dev/null || true
    fi
    # 清理临时文件
    [[ -n "${TMP_PARAMS}" && -f "${TMP_PARAMS}" ]] && rm -f "${TMP_PARAMS}"
    [[ -n "${NODE_LOG}" && -f "${NODE_LOG}" ]] && rm -f "${NODE_LOG}"
    [[ -n "${SAVER_LOG}" && -f "${SAVER_LOG}" ]] && rm -f "${SAVER_LOG}"
    [[ -n "${ROS_LOG_DIR}" && -d "${ROS_LOG_DIR}" ]] && rm -rf "${ROS_LOG_DIR}"
    exit "${exit_code}"
}
trap cleanup EXIT INT TERM

# ==================== 帮助信息 ====================
usage() {
    cat <<EOF
用法: $(basename "$0") <pcd_file> [output_name]

参数:
  pcd_file      输入的 PCD 点云文件路径（必填）
  output_name   输出 PGM 文件名（不含扩展名，可选，默认与 PCD 同名）

选项:
  -h, --help    显示此帮助信息

示例:
  $(basename "$0") /home/robomaster/project/sentry_ws/maps/test/pcd/map.pcd
  $(basename "$0") /path/to/scan.pcd my_map
EOF
}

# ==================== 参数解析 ====================
if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

PCD_FILE="$1"
OUTPUT_NAME="${2:-}"

if [[ "${PCD_FILE}" == "-h" || "${PCD_FILE}" == "--help" ]]; then
    usage
    exit 0
fi

# ==================== 文件路径验证 ====================
log_info "验证输入文件路径 ..."

# 转换为绝对路径
if [[ "${PCD_FILE}" != /* ]]; then
    PCD_FILE="$(cd "$(dirname "${PCD_FILE}")" 2>/dev/null && pwd)/$(basename "${PCD_FILE}")"
fi

if [[ ! -f "${PCD_FILE}" ]]; then
    log_error "PCD 文件不存在: ${PCD_FILE}"
    exit 1
fi

if [[ ! -s "${PCD_FILE}" ]]; then
    log_error "PCD 文件为空: ${PCD_FILE}"
    exit 1
fi

# 校验文件扩展名
if [[ "${PCD_FILE,,}" != *.pcd ]]; then
    log_warn "文件扩展名不是 .pcd，仍尝试处理: ${PCD_FILE}"
fi

# 默认输出名 = PCD 文件名（去扩展名）
if [[ -z "${OUTPUT_NAME}" ]]; then
    OUTPUT_NAME="$(basename "${PCD_FILE}" .pcd)"
fi

log_ok "输入文件: ${PCD_FILE}"
log_ok "输出名称: ${OUTPUT_NAME}"

# ==================== 输出目录处理 ====================
# 输出目录 = PCD 文件父级目录的同级 pgm 目录
PCD_PARENT_DIR="$(dirname "${PCD_FILE}")"
OUTPUT_DIR="$(dirname "${PCD_PARENT_DIR}")/pgm"

log_info "输出目录: ${OUTPUT_DIR}"
if ! mkdir -p "${OUTPUT_DIR}"; then
    log_error "无法创建输出目录: ${OUTPUT_DIR}"
    exit 1
fi

OUTPUT_PGM="${OUTPUT_DIR}/${OUTPUT_NAME}.pgm"
OUTPUT_YAML="${OUTPUT_DIR}/${OUTPUT_NAME}.yaml"

# ==================== 环境源化与依赖检查 ====================
log_info "源化 ROS 2 环境 ..."
if [[ ! -f "${ROS_SETUP}" ]]; then
    log_error "未找到 ROS 2 setup.bash: ${ROS_SETUP}"
    exit 1
fi
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    log_error "未找到工作空间 install/setup.bash: ${WORKSPACE_SETUP}"
    log_error "请先构建 pcd2pgm 包: colcon build --packages-select pcd2pgm"
    exit 1
fi

# ROS setup 脚本会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），临时关闭 -u
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
    log_error "ros2 命令不可用，请检查 ROS 2 环境"
    exit 1
fi

# 检查 pcd2pgm_node 可执行文件
if [[ ! -x "${NODE_EXEC}" ]]; then
    log_error "pcd2pgm_node 可执行文件不存在: ${NODE_EXEC}"
    log_error "请先构建 pcd2pgm 包: colcon build --packages-select pcd2pgm"
    exit 1
fi

# 检查 nav2_map_server（map_saver_cli）是否可用
if ! ros2 pkg list 2>/dev/null | grep -q "^nav2_map_server$"; then
    log_error "未找到 nav2_map_server 包，map_saver_cli 不可用"
    log_error "请安装: sudo apt install ros-\${ROS_DISTRO}-nav2-map-server"
    exit 1
fi

log_ok "环境与依赖检查通过"

# 将 ROS 日志重定向到可写临时目录，避免 ~/.ros/log 权限问题
ROS_LOG_DIR="$(mktemp -d)"
export ROS_LOG_DIR
export RCUTILS_LOGGING_USE_STDOUT=1

# ==================== 生成临时参数文件 ====================
log_info "生成临时参数文件 (pcd_file -> ${PCD_FILE}) ..."
TMP_PARAMS="$(mktemp --suffix=.yaml)"
# 用 sed 替换 pcd_file 行，保留其余参数
sed -E "s|^[[:space:]]*pcd_file:.*|    pcd_file: ${PCD_FILE}|" "${DEFAULT_CONFIG}" > "${TMP_PARAMS}"

if ! grep -q "pcd_file: ${PCD_FILE}" "${TMP_PARAMS}"; then
    log_error "参数文件生成失败，无法替换 pcd_file"
    exit 1
fi
log_ok "临时参数文件已生成"

# ==================== 启动 pcd2pgm 节点 ====================
# 清理残留的 pcd2pgm 节点进程，避免旧地图 (transient_local latch) 干扰 map_saver
if pgrep -f '[p]cd2pgm_node' >/dev/null 2>&1; then
    log_warn "发现残留 pcd2pgm_node 进程，正在清理 ..."
    pkill -f '[p]cd2pgm_node' 2>/dev/null || true
    sleep 1
fi

NODE_LOG="$(mktemp --suffix=.log)"
log_info "启动 pcd2pgm 节点处理点云 ..."
log_info "节点日志: ${NODE_LOG}"

# 直接执行节点二进制（而非 ros2 run），确保 NODE_PID 就是节点进程本身，
# cleanup 时 kill 才能真正终止节点，避免孤儿进程持续发布旧地图
"${NODE_EXEC}" --ros-args --params-file "${TMP_PARAMS}" \
    > "${NODE_LOG}" 2>&1 &
NODE_PID=$!

log_info "节点进程 PID: ${NODE_PID}"

# ==================== 等待节点完成点云处理 ====================
log_info "等待节点加载并处理点云 ..."
# 节点构造函数中处理完成后会输出 "Map data size" 或错误信息
elapsed=0
processed=0
while [[ ${elapsed} -lt ${NODE_PROCESS_TIMEOUT} ]]; do
    # 节点异常退出检查
    if ! kill -0 "${NODE_PID}" 2>/dev/null; then
        log_error "pcd2pgm 节点异常退出"
        log_error "节点日志末尾:"
        tail -n 20 "${NODE_LOG}" >&2 || true
        exit 1
    fi

    # 处理完成标志
    if grep -q "Map data size" "${NODE_LOG}" 2>/dev/null; then
        processed=1
        break
    fi
    # 点云为空
    if grep -q "Point cloud is empty" "${NODE_LOG}" 2>/dev/null; then
        log_error "点云经滤波后为空，无法生成地图"
        tail -n 20 "${NODE_LOG}" >&2 || true
        exit 1
    fi
    # 读取失败
    if grep -q "Couldn't read file" "${NODE_LOG}" 2>/dev/null; then
        log_error "节点无法读取 PCD 文件"
        tail -n 20 "${NODE_LOG}" >&2 || true
        exit 1
    fi

    printf "  ... 处理中 (%ds)\r" "${elapsed}"
    sleep 1
    elapsed=$((elapsed + 1))
done
echo ""

if [[ ${processed} -ne 1 ]]; then
    log_error "节点在 ${NODE_PROCESS_TIMEOUT}s 内未完成点云处理"
    tail -n 20 "${NODE_LOG}" >&2 || true
    exit 1
fi

log_ok "点云处理完成"
# 显示节点处理摘要
log_info "节点处理摘要:"
grep -E "(Initial point cloud|After PassThrough|After RadiusOutlier|Map data size)" "${NODE_LOG}" \
    | sed 's/^/    /' || true

# 给发布定时器一点时间首次发布地图
sleep 2

# ==================== 保存地图为 PGM ====================
SAVER_LOG="$(mktemp --suffix=.log)"
log_info "使用 map_saver_cli 保存地图到: ${OUTPUT_PGM}"
log_info "保存中 ..."

if timeout "${MAP_SAVER_TIMEOUT}" ros2 run nav2_map_server map_saver_cli \
    -f "${OUTPUT_DIR}/${OUTPUT_NAME}" \
    > "${SAVER_LOG}" 2>&1; then
    log_ok "map_saver_cli 执行完成"
else
    rc=$?
    log_error "map_saver_cli 执行失败 (退出码 ${rc})"
    log_error "map_saver 日志:"
    cat "${SAVER_LOG}" >&2 || true
    exit ${rc}
fi

# ==================== 验证输出文件 ====================
log_info "验证输出文件 ..."

if [[ ! -f "${OUTPUT_PGM}" ]]; then
    log_error "未生成 PGM 文件: ${OUTPUT_PGM}"
    exit 1
fi

if [[ ! -s "${OUTPUT_PGM}" ]]; then
    log_error "PGM 文件为空: ${OUTPUT_PGM}"
    exit 1
fi

PGM_SIZE=$(stat -c%s "${OUTPUT_PGM}")

# ==================== 完成 ====================
log_ok "转换成功!"
echo ""
echo -e "${C_GREEN}========================================${C_RESET}"
echo -e "${C_GREEN} PCD -> PGM 转换完成${C_RESET}"
echo -e "${C_GREEN}========================================${C_RESET}"
echo -e " 输入 PCD : ${PCD_FILE}"
echo -e " 输出 PGM : ${OUTPUT_PGM} (${PGM_SIZE} bytes)"
if [[ -f "${OUTPUT_YAML}" ]]; then
    echo -e " 输出 YAML: ${OUTPUT_YAML}"
fi
echo -e "${C_GREEN}========================================${C_RESET}"

exit 0
