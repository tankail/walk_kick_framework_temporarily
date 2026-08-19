#!/usr/bin/env bash
# walk_kick_framework 终端1: ROS2 bringup + 归零
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"
ROS2_WS="$PROJECT_ROOT/ros2"
SCRIPTS_DIR="$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard/scripts"

echo "=========================================="
echo "  walk_kick_framework 终端1: Bringup + 归零"
echo "=========================================="
echo ""

# 1. 加载 bashrc
echo "[1/6] 加载 bashrc..."
source ~/.bashrc

# 2. 设置 ROS2 环境 (Foxy + cyclonedds)
echo "[2/6] 设置 ROS2 环境..."
if command -v rosenv &> /dev/null; then
    rosenv foxy
else
    source /opt/ros/foxy/setup.bash
fi
source "$SCRIPTS_DIR/setup_soccer_ros2_env.sh"

# 3. conda base (py3.13) 会破坏 rclpy, 切到 python3.8 的 instinct_venv
echo "[3/6] 激活 instinct_venv (python3.8)..."
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate instinct_venv
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

# 4. 进入 ROS2 工作区并 source
echo "[4/6] 加载 ROS2 工作区..."
cd "$ROS2_WS"
if [[ ! -f install/setup.bash ]]; then
    echo "[ERROR] Missing $ROS2_WS/install/setup.bash. Build first:" >&2
    echo "        cd $ROS2_WS && source /opt/ros/foxy/setup.bash && colcon build --symlink-install" >&2
    exit 1
fi
source install/setup.bash
export PYTHONPATH="$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard:$HOME/ros2_numpy:/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}"

# 5. 启动 IMU 桥 (yesense ttyUSB0 -> ROS2 /imu/data) 后台
echo "[5/7] 启动 IMU 桥..."
"$PROJECT_ROOT/start_imu_bridge.sh" > /tmp/imu_bridge_run.log 2>&1 &
IMU_PID=$!
sleep 5

# 6. 启动 bringup
echo "[6/7] 启动 hightorque_bringup..."
ros2 launch hightorque_bringup pi_plus_orin.launch.py &
BRINGUP_PID=$!

# 7. 等待 request_control 服务就绪后归零
echo "[7/7] 等待电机服务并归零..."
# 清掉 ros2 daemon + 残留 CycloneDDS 共享内存段: 多次启动累积的 participant
# index 会让新节点无法创建 DDS (归零/手柄失败)。daemon 只在 ros2 CLI 用,
# 归零节点直接 python 运行, 不需要 daemon。
ros2 daemon stop > /dev/null 2>&1 || true
pkill -9 -f _ros2_daemon 2>/dev/null || true
# 删残留 shm (dest 状态的 CycloneDDS 段; 被占用时删除失败无害)
for shid in $(ipcs -m 2>/dev/null | awk '$6=="dest" || $2!="key" {print $2}' | grep -E '^[0-9]+$'); do
    ipcrm -m "$shid" 2>/dev/null || true
done

MOVE_TO_ZERO="$ROS2_WS/install/hightorque_midware/lib/hightorque_midware/move_to_zero.py"

# 用 python 直接等 request_control 服务 (rclpy client.wait_for_service)
for i in $(seq 1 60); do
    if timeout 2 python3 - "$i" << 'PYEOF'
import sys
import rclpy
from hightorque_msgs.srv import RequestControl
rclpy.init()
node = rclpy.create_node("_svc_check_" + str(sys.argv[1]))
client = node.create_client(RequestControl, "request_control")
ok = client.wait_for_service(timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF
    then
        echo "  request_control 服务就绪 (${i}s)"
        break
    fi
    sleep 1
done

# 直接 python 执行归零 (绕过 ros2 run, 避免再次拉起 daemon)
python3 "$MOVE_TO_ZERO"

wait "$BRINGUP_PID"
