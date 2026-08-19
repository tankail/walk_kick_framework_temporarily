#!/usr/bin/env bash
# 停止 dvision 视觉单独测试 (bringup + head 反馈 + dvision + overlay [+ 本次 roscore])
# 全部用 pkill -9 + [x] 括号技巧防自匹配。
set -uo pipefail

echo "[stop] 停止 head 反馈 / MotionInfo 发布..."
pkill -9 -f "[r]os2_head_state_udp_sender" 2>/dev/null || true
pkill -9 -f "[r]os1_head_state_udp_receiver" 2>/dev/null || true
pkill -9 -f "[p]ublish_motion_info.py" 2>/dev/null || true

echo "[stop] 停止 ROS2 bringup / midware..."
pkill -9 -f "[r]os2 launch hightorque_bringup" 2>/dev/null || true
pkill -9 -f "[h]ightorque_midware_node" 2>/dev/null || true
pkill -9 -f "[m]ove_to_zero" 2>/dev/null || true

echo "[stop] 停止 vision_web_overlay..."
pkill -9 -f "[v]ision_web_overlay.py" 2>/dev/null || true

echo "[stop] 停止 dvision roslaunch / dvision_node..."
pkill -9 -f "[r]oslaunch.*dvision_vision_only" 2>/dev/null || true
pkill -9 -f "[d]vision_node" 2>/dev/null || true

if [[ -f /tmp/vision_roscore_pid ]]; then
  echo "[stop] 停止本次启动的 roscore (PID=$(cat /tmp/vision_roscore_pid))..."
  kill "$(cat /tmp/vision_roscore_pid)" 2>/dev/null || true
  rm -f /tmp/vision_roscore_pid
fi

sleep 1
echo "[stop] 剩余相关进程:"
pgrep -af "[d]vision_node|[v]ision_web_overlay|[h]ightorque_midware_node|[r]os1_head_state_udp_receiver|[p]ublish_motion_info|[r]os2_head_state_udp_sender" || echo "  无"
echo "[stop] 完成。"
