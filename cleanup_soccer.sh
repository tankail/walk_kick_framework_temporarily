#!/usr/bin/env bash
# 清理Soccer所有进程和ROS2 daemon

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"

echo "=========================================="
echo "  清理 Soccer 所有进程"
echo "=========================================="
echo ""

# 1. 停止所有ROS1进程 (roscore/yesense/dvision/dbehavior)
echo "[1/7] 停止ROS1进程..."
pkill -f "roslaunch\|roscore\|rosmaster\|dvision_node\|dbehavior\|motion_hub_communicator" || true
pkill -f "yesense_imu_node" || true
sudo killall rosmaster 2>/dev/null || true

# 2. 停止 IMU 桥 (ros1_bridge)
echo "[2/7] 停止 IMU 桥 (ros1_bridge)..."
pkill -f "dynamic_bridge\|parameter_bridge\|ros1_bridge" || true

# 3. 停止所有UDP桥接节点
echo "[3/7] 停止UDP桥接节点..."
pkill -f "ros1_.*_udp_.*\.py\|ros2_.*_udp_.*\.py" || true

# 4. 停止控制节点
echo "[4/7] 停止控制节点..."
pkill -f "soccer_control_node\.py" || true

# 5. 停止手柄节点
echo "[5/7] 停止手柄节点..."
pkill -f "joy_linux_node" || true

# 6. 停止ROS2 bringup（最后停止，因为其他节点可能依赖它）
echo "[6/7] 停止ROS2 bringup..."
pkill -f "hightorque_midware_node\|hightorque_oled_node\|yesense_imu_node\|power_node\|joy_mapper_node" || true
pkill -f "ros2 launch hightorque_bringup" || true

# 7. 强杀孤儿进程
echo "[7/7] 强杀残留孤儿进程..."
pkill -9 -f "$PROJECT_ROOT/ros2/install/.*/hightorque_midware_node" || true
pkill -9 -f "ros2/install/.*/joy_mapper_node.py" || true

# 等待进程结束
sleep 2

# Some ROS2 nodes can be orphaned by launch shutdown and keep DDS participant
# indices occupied. Keep this scoped to the Route C workspace processes.
pkill -9 -f "$PROJECT_ROOT/ros2/install/.*/hightorque_midware_node" || true
pkill -9 -f "$PROJECT_ROOT/ros2/install/.*/yesense_imu_node" || true
pkill -9 -f "$PROJECT_ROOT/ros2/install/.*/hightorque_oled_node" || true
pkill -9 -f "$PROJECT_ROOT/ros2/install/.*/power_node" || true
pkill -9 -f "$PROJECT_ROOT/ros2/install/.*/joy_mapper_node.py" || true

# 清理ROS2 daemon
echo ""
echo "清理ROS2 daemon..."
export ROS_DOMAIN_ID=56
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PROJECT_ROOT/ros2/src/instinct_onboard_soccer/instinct_onboard/scripts/cyclonedds_route_c_local.xml"
source /opt/ros/foxy/setup.bash 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 1

# 显示剩余进程
echo ""
echo "=========================================="
REMAINING=$(ps aux | grep -E "(ros2|roslaunch|udp|soccer)" | grep -v grep | grep -v cleanup_soccer | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "  ✅ 所有进程已清理"
else
    echo "  ⚠️  还有 $REMAINING 个进程残留："
    ps aux | grep -E "(ros2|roslaunch|udp|soccer)" | grep -v grep | grep -v cleanup_soccer | awk '{print "  - PID " $2 ": " $11}'
    echo ""
    echo "如需强制清理，运行: pkill -9 -f ros2"
fi
echo "=========================================="
echo ""
echo "清理完成！现在可以按顺序重新启动："
echo "  1. ./start_terminal1_bringup.sh   (bringup + 归零)"
echo "  2. ./start_terminal2_control_route_c.sh  (ROS2 运控, 冷启动完成后按 LT+RT+A 进入)"
echo "  3. ./start_terminal3_ros1_decision.sh   (ROS1 视觉 + 行为树)"
