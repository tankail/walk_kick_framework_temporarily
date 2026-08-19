#!/usr/bin/env bash
# walk_kick_framework 一键录制 rosbag (ROS1 + ROS2), 供复盘"踢球不稳定"等现场数据。
# 用法: ./start_rosbag.sh [run_name]
#   可选: WITH_IMAGE=1 ./start_rosbag.sh xxx      # 额外录相机压缩图
#         WITH_RAW_IMAGE=1 ./start_rosbag.sh xxx   # 录原始相机图 (很大)
#         BAG_DIR=/path ./start_rosbag.sh xxx       # 换输出根目录 (默认 ~/route_c_bags)
# 录完 Ctrl+C, bag 在 BAG_DIR/<run_name>/ 下 (ros1*.bag + ros2/ + metadata.txt)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"
SCRIPTS_DIR="$PROJECT_ROOT/ros2/src/instinct_onboard_soccer/instinct_onboard/scripts"
RECORDER="$SCRIPTS_DIR/rosbag_route_c.sh"

if [[ ! -f "$RECORDER" ]]; then
  echo "[ERROR] 找不到录制脚本: $RECORDER" >&2
  exit 1
fi

echo "=========================================="
echo "  walk_kick_framework 一键 rosbag 录制"
echo "=========================================="
echo "  recorder : $RECORDER"
echo "  输出目录 : ${BAG_DIR:-$HOME/route_c_bags}/<run_name>/"
echo "  按 Ctrl+C 结束录制"
echo ""
echo "  可选: WITH_IMAGE=1 (录压缩相机图) / WITH_RAW_IMAGE=1 (录原始图, 大) / BAG_DIR=/path"
echo "=========================================="

exec "$RECORDER" "$@"
