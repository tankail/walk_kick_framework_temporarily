#!/usr/bin/env bash
# walk_kick_framework 终端3: ROS1 视觉 + 行为树
# 启动 UDP 桥 ROS1 侧 + roslaunch dlaunch piplus.launch (dvision/dbehavior/dplanner/dancer-io)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"

echo "=========================================="
echo "  walk_kick_framework 终端3: ROS1 视觉 + 行为树"
echo "=========================================="
echo ""

"$PROJECT_ROOT/ros2/src/instinct_onboard_soccer/instinct_onboard/scripts/start_route_c_ros1_decision.sh"
