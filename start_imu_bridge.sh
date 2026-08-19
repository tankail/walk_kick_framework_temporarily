#!/usr/bin/env bash
# walk_kick_framework IMU 桥: ROS1 yesense (ttyUSB0) -> UDP -> ROS2 /imu/data
# 用 UDP 转发绕开 ros1_bridge dynamic_bridge(全话题桥会污染 /cmd_vel 和
# /pi_plus_absolute, 导致命令被覆盖)。需要先跑本脚本, 再启动终端1/2/3。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"
ROS2_WS="$PROJECT_ROOT/ros2"
YESENSE_WS="${YESENSE_WS:-$HOME/sim2real}"  # 83: 用 sim2real 的 yesense_imu_node (支持 _yesense_port)
# IMU = yesense 模块, 接在 /dev/ttyUSB0 (Silicon Labs CP2102N), 460800 baud, 帧头 0x59 0x53。
# 注意: 电机板是 /dev/ttyACM0-6 (Livelybot Communication Board), 不要混淆。
IMU_PORT="${IMU_PORT:-/dev/ttyUSB0}"
IMU_BAUD="${IMU_BAUD:-460800}"

echo "=========================================="
echo "  walk_kick_framework IMU 桥"
echo "  ROS1 yesense($IMU_PORT) -> ros1_bridge -> ROS2 /imu"
echo "=========================================="
echo ""

# 0. 清理旧的 UDP 桥进程, 避免重复 sender/receiver 累积
#    (重复 receiver 会导致 UDP 包分发不定, /imu/data 时断时续)
echo "[0/5] 清理旧的 IMU UDP 桥进程 ..."
pkill -f "ros1_imu_udp_sender" 2>/dev/null && echo "  已杀旧 sender" || true
pkill -f "ros2_imu_udp_receiver" 2>/dev/null && echo "  已杀旧 receiver" || true
sleep 1

# 1. 启动 ROS1 roscore
echo "[1/4] 启动 ROS1 roscore ..."
source /opt/ros/noetic/setup.bash
if ! pgrep -x roscore >/dev/null 2>&1; then
    roscore &
    sleep 3
else
    echo "  roscore 已在运行"
fi

# 2. 启动 ROS1 yesense 驱动 (ttyTHS0 = IMU)
echo "[2/4] 启动 yesense IMU 驱动 ($IMU_PORT) ..."
source "$YESENSE_WS/install/setup.bash" 2>/dev/null || true
if pgrep -af "yesense_imu_node.*$IMU_PORT" >/dev/null 2>&1; then
    echo "  yesense_imu_node 已在运行"
else
    rosrun yesense_imu yesense_imu_node _yesense_port:=$IMU_PORT _yesense_baudrate:=$IMU_BAUD &
    sleep 4
fi

# 3. 验证 ROS1 /imu/data
echo "[3/4] 验证 ROS1 /imu/data ..."
timeout 3 rostopic echo -n1 /imu/data 2>/dev/null | head -5 | grep -q orientation && echo "  ✓ ROS1 IMU 数据正常" || echo "  ⚠️ 未读到 IMU 数据, 检查串口"

# 4. 启动 UDP 桥 (ROS1 /imu/data -> UDP -> ROS2 /imu/data)
echo "[4/4] 启动 UDP IMU 桥 (只桥 /imu/data, 不污染其他话题) ..."
SCRIPTS_DIR="$PROJECT_ROOT/ros2/src/instinct_onboard_soccer/instinct_onboard/scripts"
IMU_UDP_PORT="${IMU_UDP_PORT:-15008}"

# ROS1 侧发送端 (订阅 /imu/data -> UDP)
# 注意: 必须用系统 python3 (/usr/bin/python3), 因为 rospy/rospkg 装在系统 python 里。
# 若用 conda instinct_venv 的 python 会报 "Run this script in a ROS1 environment"。
source /opt/ros/noetic/setup.bash
source "$YESENSE_WS/install/setup.bash" 2>/dev/null || true
/usr/bin/python3 "$SCRIPTS_DIR/ros1_imu_udp_sender.py" \
  --topic /imu/data \
  --host 127.0.0.1 \
  --port "$IMU_UDP_PORT" &
SENDER_PID=$!
sleep 2

# ROS2 侧接收端 (UDP -> /imu/data)
source /opt/ros/foxy/setup.bash
source "$ROS2_WS/install/setup.bash"
source "$SCRIPTS_DIR/setup_soccer_ros2_env.sh"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate instinct_venv
export PYTHONPATH="$SCRIPTS_DIR:$HOME/ros2_numpy:/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}"
python "$SCRIPTS_DIR/ros2_imu_udp_receiver.py" \
  --topic /imu/data \
  --host 127.0.0.1 \
  --port "$IMU_UDP_PORT" &
RECEIVER_PID=$!
sleep 3
echo "  UDP IMU 桥已启动 (sender=$SENDER_PID receiver=$RECEIVER_PID)"
echo "  ROS2 topic: /imu/data (UDP $IMU_UDP_PORT)"

wait "$SENDER_PID" "$RECEIVER_PID"
