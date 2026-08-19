#!/usr/bin/env bash
# walk_kick_framework 终端2: ROS2 运控 (soccer_control_node.py)
# 冷启动完成后按 LT+RT+A 进入足球状态(走路找球/踢球/跌倒起立), LT+RT+B 退出
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"

echo "=========================================="
echo "  walk_kick_framework 终端2: ROS2 运控"
echo "=========================================="
echo ""

# 1. 加载 bashrc
echo "[1/3] 加载 bashrc..."
source ~/.bashrc

# 2. 设置 ROS2 环境
echo "[2/3] 设置 ROS2 环境..."
if command -v rosenv &> /dev/null; then
    rosenv foxy
else
    source /opt/ros/foxy/setup.bash
fi
source "$PROJECT_ROOT/ros2/src/instinct_onboard_soccer/instinct_onboard/scripts/setup_soccer_ros2_env.sh"

# 3. 启动 Route C 控制 (脚本内部自行 source 环境)
echo "[3/3] 启动 soccer_control_node.py ..."
echo "    等待 cold start 完成后按 LT+RT+A 进入, LT+RT+B 退出"
echo "=========================================="
echo ""

"$PROJECT_ROOT/ros2/src/instinct_onboard_soccer/instinct_onboard/scripts/start_route_c_ros2_control.sh"
