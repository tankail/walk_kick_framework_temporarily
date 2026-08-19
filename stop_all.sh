#!/usr/bin/env bash
# walk_kick_framework + sim2real 一键停止
# 停掉本框架所有进程, 也停掉同机的 sim2real (两个系统共用电机, 不能同时跑)。
set -u

echo "=========================================="
echo "  walk_kick_framework + sim2real 一键停止"
echo "=========================================="
echo ""

# 1. ROS2 侧: soccer + 各 UDP 桥 + 接收端
pkill -f "soccer_control_node.py" && echo "  ✓ soccer" || true
pkill -f "ros2_imu_udp_receiver" && echo "  ✓ imu receiver" || true
pkill -f "ros1_imu_udp_sender" && echo "  ✓ imu sender" || true
pkill -f "ros2_cmd_vel_udp_receiver" && echo "  ✓ cmd_vel bridge" || true
pkill -f "ros2_head_joint_udp_receiver" && echo "  ✓ head bridge" || true
pkill -f "ros2_head_state_udp_sender" && echo "  ✓ head state bridge" || true
pkill -f "ros1_head_joint_udp_sender" && echo "  ✓ head sender" || true
pkill -f "ros1_head_state_udp_receiver" && echo "  ✓ head state receiver" || true
pkill -f "ros1_ball_position_udp_sender" && echo "  ✓ ball sender" || true
pkill -f "ros1_goal_yaw_udp_sender" && echo "  ✓ goal_yaw sender" || true

# 2. midware / bringup / joy / power
pkill -f "hightorque_midware_node" && echo "  ✓ midware" || true
pkill -f "joy_linux_node" && echo "  ✓ joy_linux" || true
pkill -f "joy_mapper_node" && echo "  ✓ joy_mapper" || true
pkill -f "hightorque_bringup" && echo "  ✓ bringup launch" || true
pkill -f "hightorque_power/lib/hightorque_power/power_node" && echo "  ✓ power_node (walk_kick)" || true
pkill -f "hightorque_oled" && echo "  ✓ oled" || true
pkill -f "yesense_imu" && echo "  ✓ yesense_imu" || true
pkill -f "move_to_zero" && echo "  ✓ move_to_zero" || true
pkill -f "_ros2_daemon" && echo "  ✓ ros2 daemon" || true

# 3. ROS1 侧: 视觉 + 行为树 + dancer
pkill -f "roslaunch dlaunch piplus.launch" && echo "  ✓ roslaunch piplus" || true
pkill -f "dvision_node" && echo "  ✓ dvision" || true
pkill -f "dbehavior.*main.py" && echo "  ✓ dbehavior" || true
pkill -f "motion_hub_communicator" && echo "  ✓ dplanner" || true
pkill -f "dancer-io" && echo "  ✓ dancer-io" || true

# 4. sim2real (同机另一套系统, 共用电机)
echo "--- 停止 sim2real ---"
if tmux has-session -t sim2real 2>/dev/null; then
    tmux kill-session -t sim2real
    echo "  ✓ sim2real tmux 会话"
else
    echo "  - 无 sim2real tmux 会话"
fi
pkill -f "sim2real/install" && echo "  ✓ sim2real 节点" || echo "  - 无 sim2real 节点"
pkill -f "livelybot_logger" && echo "  ✓ livelybot_logger" || true
pkill -f "livelybot_power" && echo "  ✓ livelybot_power" || true
pkill -f "livelybot_oled" && echo "  ✓ livelybot_oled" || true
pkill -f "livelybot_bringup" && echo "  ✓ livelybot_bringup" || true
pkill -f "lr_control" && echo "  ✓ lr_control" || true

sleep 2
echo ""
echo "剩余相关进程(应为空):"
ps aux | grep -E "soccer_control|imu_udp|head_joint|head_state|cmd_vel_udp|ball_position_udp|goal_yaw_udp|hightorque_midware|joy_linux|joy_mapper|power_node|hightorque_oled|yesense_imu|dvision_node|dbehavior|motion_hub|dancer-io|sim2real|livelybot|lr_control" | grep -v grep || echo "  (干净)"
echo "完成。"
