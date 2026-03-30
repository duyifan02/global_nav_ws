#!/bin/bash
# 自动化 Benchmark 验证脚本
# 用法: bash run_auto_benchmark.sh [--trials N] [--planner JPS_3D|TopoNav] [--force]
#
# 前提:
#   Terminal 1: ros2 run jps_planner_3d jps_planner_node
#   Terminal 2: ros2 run topo_nav topo_nav_node   (若需要 TopoNav)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
CSV_PATH="${WS_ROOT}/benchmark.csv"
PYTHON_SCRIPT="${SCRIPT_DIR}/auto_benchmark.py"

# ── 参数默认值 ─────────────────────────────────────────────────────────────────
TRIALS=3
PLANNER=""
FORCE=""
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --trials)    TRIALS="$2"; shift 2 ;;
        --planner)   PLANNER="$2"; shift 2 ;;
        --force)     FORCE="--force"; shift ;;
        *)           EXTRA_ARGS="$EXTRA_ARGS $1"; shift ;;
    esac
done

# ── 环境 ───────────────────────────────────────────────────────────────────────
echo "============================================================"
echo "  自动化 Benchmark"
echo "============================================================"
echo "  工作空间: ${WS_ROOT}"
echo "  CSV:      ${CSV_PATH}"
echo "  trials:   ${TRIALS}"
echo "  planner:  ${PLANNER:-ALL}"
echo "  force:    ${FORCE:-false}"
echo "============================================================"

# Source ROS2 环境
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
    echo "[ENV] 已加载 ${WS_ROOT}/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    echo "[ENV] 已加载 /opt/ros/humble/setup.bash"
else
    echo "[WARN] 未找到 ROS2 setup.bash，继续尝试..."
fi

# ── 检查 CSV ───────────────────────────────────────────────────────────────────
if [[ ! -f "${CSV_PATH}" ]]; then
    echo "[ERROR] CSV 文件不存在: ${CSV_PATH}"
    exit 1
fi
echo "[INFO] CSV 行数: $(wc -l < "${CSV_PATH}")"

# ── 构建 python3 命令 ──────────────────────────────────────────────────────────
CMD="python3 ${PYTHON_SCRIPT} \
    --csv ${CSV_PATH} \
    --trials ${TRIALS} \
    ${FORCE} \
    ${EXTRA_ARGS}"

if [[ -n "${PLANNER}" ]]; then
    CMD="${CMD} --planner ${PLANNER}"
fi

echo ""
echo "[RUN] ${CMD}"
echo "============================================================"
eval ${CMD}
