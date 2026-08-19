#!/usr/bin/env bash
# ============================================================
# walk_kick_framework —— dvision 视觉单独测试 (带 ROS2 bringup, 真实头部角度)
#   - 启动 ROS2 bringup(hightorque midware) -> /joint_states
#     -> ros2_head_state_udp_sender -> UDP 15002
#     -> ros1_head_state_udp_receiver -> MotorState 话题
#     -> publish_motion_info.py -> /dmotion_X/MotionInfo (dvision/overlay 用)
#   - dvision 以 VISION_ONLY + OfflineRecord 模式启动 (发布 cam_image)
#   - vision_web_overlay.py 在 http://<robot-ip>:8090 提供
#     实时画面 + 球标记 + eye(相机)/base_link 坐标 (FK 含真实头部角度)
# 用法: ./start_vision_test.sh
#   环境变量:
#     VISION_START_BRINGUP=0   不启动 bringup(纯视觉, 头部角度=0)
#     VISION_MOVE_TO_ZERO=1    启动后 move_to_zero(默认 0=不归零, 读取当前头部姿态)
# 停止: ./stop_vision_test.sh
# ============================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIPLUS_AMP="${PIPLUS_AMP:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ROS1_WS="${ROS1_WS:-$PIPLUS_AMP/RoboCup_Workspace}"
ROS2_WS="$PIPLUS_AMP/ros2"
SCRIPTS_DIR="$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard/scripts"
LAUNCH_FILE="$SCRIPT_DIR/dvision_vision_only.launch"
OVERLAY_PY="$SCRIPT_DIR/vision_web_overlay.py"
MOTION_PUB_PY="$SCRIPT_DIR/publish_motion_info.py"
WEB_PORT="${WEB_PORT:-8090}"
ROBOT_ID="${ZJUDANCER_ROBOTID:-1}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
VISION_START_BRINGUP="${VISION_START_BRINGUP:-1}"
VISION_MOVE_TO_ZERO="${VISION_MOVE_TO_ZERO:-0}"

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

echo "=========================================="
echo "  dvision 视觉单独测试 (bringup=$VISION_START_BRINGUP, move_to_zero=$VISION_MOVE_TO_ZERO)"
echo "=========================================="

# 0) 清理旧进程 (dvision 独占 ZED; 只留一个 bringup)
echo "[0/9] 清理旧进程..."
pkill -9 -f "[r]os2_head_state_udp_sender" 2>/dev/null || true
pkill -9 -f "[r]os1_head_state_udp_receiver" 2>/dev/null || true
pkill -9 -f "[p]ublish_motion_info.py" 2>/dev/null || true
pkill -9 -f "[r]os2 launch hightorque_bringup" 2>/dev/null || true
pkill -9 -f "[h]ightorque_midware_node" 2>/dev/null || true
pkill -9 -f "[m]ove_to_zero" 2>/dev/null || true
pkill -9 -f "[d]vision_node" 2>/dev/null || true
pkill -9 -f "[v]ision_web_overlay.py" 2>/dev/null || true
pkill -9 -f "[r]oslaunch.*dvision_vision_only" 2>/dev/null || true
sleep 1

# 1) 本地 IP
ROBOT_IP="${ROBOT_IP:-}"
if [[ -z "$ROBOT_IP" ]]; then
  ROBOT_IP="$(ip route get 1 2>/dev/null | awk '{print $7; exit}')"
fi
if [[ -z "$ROBOT_IP" || "$ROBOT_IP" == "172."* ]]; then
  ROBOT_IP="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^192\.168\.' | head -1)"
fi
if [[ -z "$ROBOT_IP" ]]; then
  ROBOT_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
echo "[1/9] 机器人 IP: ${ROBOT_IP:-<unknown>}  (可用 ROBOT_IP=xx.xx.xx.xx 覆盖)"

# 2) 预检查
if [[ ! -f "$LAUNCH_FILE" ]]; then echo "[ERROR] missing $LAUNCH_FILE" >&2; exit 1; fi
if [[ ! -x "$ROS1_WS/core/devel/lib/dvision/dvision_node" ]]; then
  echo "[ERROR] dvision 未构建: $ROS1_WS/core/devel/lib/dvision/dvision_node" >&2
  exit 1
fi

# 3) ROS1 环境 (conda python 会破坏 rospy)
echo "[2/9] 设置 ROS1 环境..."
source_setup /opt/ros/noetic/setup.bash
if [[ ! -d "$ROS1_WS/lib/devel" || ! -d "$ROS1_WS/core/devel" ]]; then
  echo "[ERROR] 未找到 ROS1 工作空间 devel: $ROS1_WS" >&2
  exit 1
fi
source_setup "$ROS1_WS/lib/devel/setup.bash"
source_setup "$ROS1_WS/core/devel/setup.bash"
export LD_LIBRARY_PATH="$ROS1_WS/lib/devel/lib:$ROS1_WS/core/devel/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export ZJUDANCER_ROBOTID="$ROBOT_ID"
export ZJUDANCER_GPU="${ZJUDANCER_GPU:-1}"
export PATH="$(printf '%s' "${PATH:-}" | tr ':' '\n' | grep -v 'miniconda3' | paste -sd:)"

# 4) roscore
echo "[3/9] 检查 roscore..."
if ! pgrep -f "rosmaster" >/dev/null 2>&1; then
  nohup roscore > /tmp/vision_roscore.log 2>&1 &
  echo "$!" > /tmp/vision_roscore_pid
  echo "      启动 roscore PID=$(cat /tmp/vision_roscore_pid)"
  for i in $(seq 1 20); do
    if rostopic list >/dev/null 2>&1; then break; fi
    sleep 1
  done
else
  rm -f /tmp/vision_roscore_pid
  echo "      roscore 已在运行"
fi

# 5) ROS1 head 反馈接收 + MotionInfo 发布 (真实头部角度给 dvision/overlay)
if [[ "$VISION_START_BRINGUP" == "1" ]]; then
  echo "[4/9] 启动 ros1 head feedback 接收 + MotionInfo 发布..."
  "$PYTHON_BIN" "$SCRIPTS_DIR/ros1_head_state_udp_receiver.py" \
    --host 127.0.0.1 --port 15002 > /tmp/vision_head_recv.log 2>&1 &
  echo "      head receiver PID=$!"
  "$PYTHON_BIN" "$MOTION_PUB_PY" --robot-id "$ROBOT_ID" > /tmp/vision_motion_pub.log 2>&1 &
  echo "      motion pub PID=$!"
fi

# 6) ROS2 bringup + head state UDP sender (读电机编码器 -> /joint_states)
#    整个 ROS2 环境放在子 shell 里, 避免 foxy 的 PYTHONPATH 污染主脚本的 ROS1 环境。
if [[ "$VISION_START_BRINGUP" == "1" ]]; then
  (
    echo "[5/9] 设置 ROS2 环境 (instinct_venv)..."
    unset ROS_DISTRO ROS_PACKAGE_PATH
    if command -v rosenv &> /dev/null; then
      rosenv foxy
    else
      source_setup /opt/ros/foxy/setup.bash
    fi
    source_setup "$SCRIPTS_DIR/setup_soccer_ros2_env.sh"
    if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
      source "$HOME/miniconda3/etc/profile.d/conda.sh"
      conda activate instinct_venv 2>/dev/null || true
    fi
    export LD_LIBRARY_PATH="${CONDA_PREFIX:-$HOME/miniconda3/envs/instinct_venv}/lib:${LD_LIBRARY_PATH:-}"
    if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
      cd "$ROS2_WS" && source_setup install/setup.bash
    fi
    export PYTHONPATH="$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard:$HOME/ros2_numpy:/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}"

    echo "[6/9] 启动 hightorque bringup -> /tmp/vision_bringup.log"
    nohup ros2 launch hightorque_bringup pi_plus_orin.launch.py > /tmp/vision_bringup.log 2>&1 &
    echo "      bringup PID=$!"
    READY=0
    for i in $(seq 1 60); do
      if ros2 service list 2>/dev/null | grep -q request_control; then
        READY=1
        echo "      电机服务就绪 (${i}s)"
        break
      fi
      sleep 1
    done
    if [[ "$READY" -ne 1 ]]; then
      echo "[WARN] 60s 未等到 request_control, 看 /tmp/vision_bringup.log; 头部角度可能不可用" >&2
    fi

    echo "[7/9] 启动 head state UDP sender (/joint_states -> 15002)..."
    nohup python "$SCRIPTS_DIR/ros2_head_state_udp_sender.py" \
      --topic /joint_states --host 127.0.0.1 --yaw-index 20 --pitch-index 21 \
      > /tmp/vision_head_sender.log 2>&1 &
    echo "      head sender PID=$!"

    if [[ "$VISION_MOVE_TO_ZERO" == "1" ]]; then
      echo "[8/9] move_to_zero..."
      ros2 run hightorque_midware move_to_zero.py || true
    fi
  )
else
  echo "[5/9] 跳过 bringup (VISION_START_BRINGUP=0, 头部角度将恒为0)"
fi

# 7) 启动 dvision (VISION_ONLY + OfflineRecord)
echo "[8/9] 启动 dvision (VISION_ONLY) -> /tmp/vision_dvision.log"
roslaunch "$LAUNCH_FILE" --screen > /tmp/vision_dvision.log 2>&1 &
ROSLAUNCH_PID=$!
echo "      roslaunch PID=$ROSLAUNCH_PID"

echo "      等待 /dvision_${ROBOT_ID}/VisionInfo 出现 (首次跑 TRT 引擎可能较久)..."
VISION_UP=0
for i in $(seq 1 60); do
  if rosnode list 2>/dev/null | grep -q "dvision_${ROBOT_ID}"; then
    if rostopic info "/dvision_${ROBOT_ID}/VisionInfo" >/dev/null 2>&1; then
      VISION_UP=1
      echo "      dvision 就绪 (${i}s)"
      break
    fi
  fi
  if ! kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
    echo "[ERROR] dvision 启动失败, 看 /tmp/vision_dvision.log" >&2
    tail -20 /tmp/vision_dvision.log >&2 || true
    exit 1
  fi
  sleep 1
done
if [[ "$VISION_UP" -ne 1 ]]; then
  echo "[ERROR] 60s 内未等到 dvision VisionInfo, 看 /tmp/vision_dvision.log" >&2
  exit 1
fi
grep -E "VISION_ONLY|REALWORLD" /tmp/vision_dvision.log | tail -2

# 8) 启动 web 叠加
echo "[9/9] 启动 web 叠加 -> /tmp/vision_web_overlay.log"
nohup "$PYTHON_BIN" "$OVERLAY_PY" --robot-id "$ROBOT_ID" --port "$WEB_PORT" \
  > /tmp/vision_web_overlay.log 2>&1 &
echo "      overlay PID=$!"

sleep 2
echo ""
echo "=========================================="
echo "  视觉单独测试 已启动!"
echo "=========================================="
echo "  浏览器打开: http://${ROBOT_IP}:${WEB_PORT}"
echo "    实时画面 + 球标记 + eye(相机)/base_link 坐标"
echo "  状态文本:   curl http://${ROBOT_IP}:${WEB_PORT}/status"
echo "  head 角度:  $(pgrep -f '[h]ightorque_midware_node' >/dev/null && echo 'bringup 运行中, 真实头部角度经 /dmotion_1/MotionInfo' || echo 'bringup 未运行(头部=0)')"
echo "  终端检查:"
echo "    rostopic echo /dvision_${ROBOT_ID}/VisionInfo/see_ball"
echo "    rostopic echo /dmotion_${ROBOT_ID}/MotionInfo/curPlat"
echo "  日志:"
echo "    dvision:   tail -f /tmp/vision_dvision.log"
echo "    overlay:   tail -f /tmp/vision_web_overlay.log"
echo "    bringup:   tail -f /tmp/vision_bringup.log"
echo "    head 反馈: tail -f /tmp/vision_head_recv.log /tmp/vision_motion_pub.log"
echo "  停止: ./stop_vision_test.sh"
echo "=========================================="
