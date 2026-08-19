#!/usr/bin/env bash
# walk_kick_framework 一键启动: 干净重启 3 个终端 + IMU 桥
# 用法: ./start_all.sh
#   - 自动先跑 stop_all.sh 清理所有旧进程(解决重复桥问题)
#   - 终端1: IMU桥 + bringup + 归零
#   - 终端2: 3个UDP桥 + soccer_control_node
#   - 终端3: ROS1 视觉 + 行为树
# 日志: /tmp/t1.log /tmp/t2.log /tmp/t3.log
# 注意: 启动完成后, 需按任意手柄键让 soccer 完成启动检查, 再按 LT+RT+A 进入。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$SCRIPT_DIR}"
ROS2_WS="$PROJECT_ROOT/ros2"
SCRIPTS_DIR="$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard/scripts"

echo "=========================================="
echo "  walk_kick_framework 一键启动"
echo "=========================================="
echo ""

# 0. 清理旧进程
echo "[0/4] 清理所有旧进程..."
"$PROJECT_ROOT/stop_all.sh"
sleep 2
# 0b. 清理 CycloneDDS 残留: 杀 daemon + 删共享内存段。
# 多次启动累积的 domain 56 participant index 会让 move_to_zero/soccer
# 无法创建 DDS 节点 (不归零、手柄收不到)。进程已停, 此时删 shm 才有效。
pkill -9 -f "_ros2_daemon" 2>/dev/null || true
for shid in $(ipcs -m 2>/dev/null | awk '$6=="dest" {print $2}' | grep -E '^[0-9]+$'); do
    ipcrm -m "$shid" 2>/dev/null || true
done

# 1. 终端1: IMU桥 + bringup + 归零
echo "[1/4] 启动终端1 (IMU桥 + bringup + 归零) ..."
nohup "$PROJECT_ROOT/start_terminal1_bringup.sh" > /tmp/t1.log 2>&1 &
echo "      终端1 PID=$! -> /tmp/t1.log"

# 等 midware 的 request_control 服务就绪 (最多 60s)
# 用 python rclpy 检测 (避免 ros2 CLI 拉起 daemon 又累积 participant)
echo "      等待电机服务 (request_control) 就绪..."
source /opt/ros/foxy/setup.bash
source "$SCRIPTS_DIR/setup_soccer_ros2_env.sh"
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  conda activate instinct_venv 2>/dev/null || true
fi
if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
fi
READY=0
for i in $(seq 1 60); do
  if timeout 2 python3 - "$i" << 'PYEOF'
import sys
import rclpy
from hightorque_msgs.srv import RequestControl
rclpy.init()
node = rclpy.create_node("_ready_check_" + str(sys.argv[1]))
client = node.create_client(RequestControl, "request_control")
ok = client.wait_for_service(timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF
  then
    READY=1
    echo "      电机服务就绪 (${i}s)"
    break
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "[ERROR] 60s 内未等到 request_control, 检查 /tmp/t1.log" >&2
  exit 1
fi

# 2. 终端2: 3个UDP桥 + soccer
echo "[2/4] 启动终端2 (3个UDP桥 + soccer) ..."
nohup "$PROJECT_ROOT/start_terminal2_control_route_c.sh" > /tmp/t2.log 2>&1 &
echo "      终端2 PID=$! -> /tmp/t2.log"
sleep 8

# 3. 终端3: ROS1 视觉 + 行为树
echo "[3/4] 启动终端3 (ROS1 视觉 + 行为树) ..."
nohup "$PROJECT_ROOT/start_terminal3_ros1_decision.sh" > /tmp/t3.log 2>&1 &
echo "      终端3 PID=$! -> /tmp/t3.log"

# 4. 验证
echo "[4/4] 等待各环节就绪并验证..."
sleep 15
IMU_OK=$(timeout 3 rostopic echo -n1 /imu/data 2>/dev/null | grep -c orientation || true)
SOCCER_UP=$(ps aux | grep -c "[s]occer_control_node.py" || true)
echo ""
echo "=========================================="
echo "  启动完成!"
echo "  IMU(ROS1 /imu/data): $([ "$IMU_OK" -gt 0 ] && echo 'OK' || echo '检查 /tmp/t1.log')"
echo "  soccer 进程: $([ "$SOCCER_UP" -gt 0 ] && echo '运行中' || echo '未启动, 查 /tmp/t2.log')"
echo "=========================================="
echo ""
echo "下一步:"
echo "  1. 按一下手柄任意键 (让 soccer 完成启动检查)"
echo "  2. 按 LT+RT+A 进入足球状态 (走路找球/踢球/跌倒起立)"
echo "  3. LT+RT+B 退出 (保持站立)"
echo "  4. LT+RT+START 急停 (注意: 扳机静止位问题, 单独按 START 也会触发, 已知隐患)"
echo ""
echo "日志:"
echo "  终端1: tail -f /tmp/t1.log"
echo "  终端2: tail -f /tmp/t2.log"
echo "  终端3: tail -f /tmp/t3.log"
