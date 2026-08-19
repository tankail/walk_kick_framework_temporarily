#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIPLUS_AMP="${PIPLUS_AMP:-$(cd "$SCRIPT_DIR/../../../../.." && pwd)}"
if [[ -z "${ROS1_WS:-}" ]]; then
  if [[ -d "$PIPLUS_AMP/RoboCup_Workspace/core/src" ]]; then
    ROS1_WS="$PIPLUS_AMP/RoboCup_Workspace"
  elif [[ -d "$HOME/RoboCup_Workspace/core/src" ]]; then
    ROS1_WS="$HOME/RoboCup_Workspace"
  else
    echo "[ERROR] Cannot find RoboCup_Workspace. Set ROS1_WS explicitly." >&2
    exit 1
  fi
fi
ROUTE_C_ROS1_LAUNCH="${ROUTE_C_ROS1_LAUNCH:-dlaunch piplus.launch}"
BALL_FIELD_SCALE="${BALL_FIELD_SCALE:-1.0}"

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

PIDS=()
cleanup() {
  if ((${#PIDS[@]})); then
    kill "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

source_setup /opt/ros/noetic/setup.bash
source_setup "$ROS1_WS/lib/devel/setup.bash"
source_setup "$ROS1_WS/core/devel/setup.bash"
export LD_LIBRARY_PATH="$ROS1_WS/lib/devel/lib:$ROS1_WS/core/devel/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROS1_WS/core/src/dbehavior/src:${PYTHONPATH:-}"
export ZJUDANCER_ROBOTID="${ZJUDANCER_ROBOTID:-1}"
export ZJUDANCER_GPU="${ZJUDANCER_GPU:-1}"

# ROS1 python 桥必须用系统 python3 (有 rospy/dmsgs/livelybot_msg)。
# conda 的 python (base/instinct_venv) 没有这些模块, 会报
# "Run this script in a ROS1 environment" 然后全部启动失败。
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
# 去掉 conda base/venv 的 PATH, 保证 roslaunch 拉起的 dbehavior (python3 main.py)
# 也用系统 python3 (有 rospkg/rospy), 否则 dbehavior 崩溃报 "No module named rospkg"。
export PATH="$(printf '%s' "${PATH:-}" | tr ':' '\n' | grep -v 'miniconda3' | paste -sd:)"

echo "[INFO] Route C ROS1 workspace: $ROS1_WS"
echo "[INFO] Route C ROS1 launch: $ROUTE_C_ROS1_LAUNCH"
echo "[INFO] Route C ball_field UDP scale: $BALL_FIELD_SCALE"
echo "[INFO] Route C uses legacy ROS1 dvision and requires exclusive access to the ZED camera."
if [[ "$ROS1_WS" == "$PIPLUS_AMP/RoboCup_Workspace" ]]; then
  echo "[INFO] Using Route C repo copy of RoboCup_Workspace"
else
  echo "[INFO] Using custom/robot-local RoboCup_Workspace"
fi

if [[ "$ROUTE_C_ROS1_LAUNCH" == "dlaunch piplus.launch" ]]; then
  if [[ ! -f "$ROS1_WS/core/src/dlaunch/launch/piplus.launch" ]]; then
    echo "[ERROR] Missing Route C ROS1 launch file:" >&2
    echo "        $ROS1_WS/core/src/dlaunch/launch/piplus.launch" >&2
    exit 1
  fi
  if [[ ! -x "$ROS1_WS/core/devel/lib/dvision/dvision_node" ]]; then
    echo "[ERROR] Missing built legacy dvision node:" >&2
    echo "        $ROS1_WS/core/devel/lib/dvision/dvision_node" >&2
    echo "[ERROR] Build it on the robot with:" >&2
    echo "        cd $ROS1_WS/core" >&2
    echo "        source /opt/ros/noetic/setup.bash" >&2
    echo "        source ../lib/devel/setup.bash" >&2
    echo "        ZJUDANCER_GPU=1 catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dvision --pkg dvision" >&2
    echo "        catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES='dconfig;dlaunch;dbehavior;dplanner'" >&2
    exit 1
  fi
  if [[ ! -x "$ROS1_WS/core/devel/lib/dancer-io/dancer-io" ]]; then
    echo "[ERROR] Missing built dancer-io node required for /dmotion_${ZJUDANCER_ROBOTID}/MotionInfo:" >&2
    echo "        $ROS1_WS/core/devel/lib/dancer-io/dancer-io" >&2
    echo "[ERROR] Build the Route C ROS1 feedback packages on the robot with:" >&2
    echo "        cd $ROS1_WS/core" >&2
    echo "        source /opt/ros/noetic/setup.bash" >&2
    echo "        source ../lib/devel/setup.bash" >&2
    echo "        catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES='serial;livelybot_msg;livelybot_serial;dancer-io;dlaunch'" >&2
    exit 1
  fi
  if ! "$PYTHON_BIN" -c "from livelybot_msg.msg import MotorState" >/dev/null 2>&1; then
    echo "[ERROR] Missing generated livelybot_msg Python module required by ros1_head_state_udp_receiver.py." >&2
    echo "[ERROR] Build livelybot_msg/livelybot_serial/dancer-io and source $ROS1_WS/core/devel/setup.bash before Route C ROS1 startup." >&2
    exit 1
  fi
  for model_file in "$ROS1_WS/core/src/dvision/model/best26.engine" "$ROS1_WS/core/src/dvision/model/hum.names"; do
    if [[ ! -f "$model_file" ]]; then
      echo "[ERROR] Missing Route C dvision model file:" >&2
      echo "        $model_file" >&2
      exit 1
    fi
  done
fi

if pgrep -af "[s]occer_perception_ros|[r]un_ros2_backend.sh|[z]ed_ball_viser.py" >/dev/null; then
  echo "[ERROR] Another ZED vision process appears to be running." >&2
  echo "[ERROR] Route C legacy dvision needs exclusive access to the ZED camera." >&2
  pgrep -af "[s]occer_perception_ros|[r]un_ros2_backend.sh|[z]ed_ball_viser.py" >&2 || true
  exit 1
fi

$PYTHON_BIN "$SCRIPT_DIR/ros1_cmd_vel_udp_sender.py" \
  --topic /cmd_vel \
  --host 127.0.0.1 \
  --port 15000 &
PIDS+=("$!")

$PYTHON_BIN "$SCRIPT_DIR/ros1_head_joint_udp_sender.py" \
  --topic /pi_plus_absolute \
  --host 127.0.0.1 \
  --port 15001 &
PIDS+=("$!")

$PYTHON_BIN "$SCRIPT_DIR/ros1_head_state_udp_receiver.py" \
  --host 127.0.0.1 \
  --port 15002 \
  --timeout 0.5 &
PIDS+=("$!")

$PYTHON_BIN "$SCRIPT_DIR/ros1_ball_position_udp_sender.py" \
  --topic "/dvision_${ZJUDANCER_ROBOTID}/VisionInfo" \
  --host 127.0.0.1 \
  --port 15003 \
  --scale "$BALL_FIELD_SCALE" &
PIDS+=("$!")

$PYTHON_BIN "$SCRIPT_DIR/ros1_goal_yaw_udp_sender.py" \
  --action-topic "/dbehavior_${ZJUDANCER_ROBOTID}/ActionCommand" \
  --vision-topic "/dvision_${ZJUDANCER_ROBOTID}/VisionInfo" \
  --host 127.0.0.1 \
  --port 15004 \
  --timeout 0.5 &
PIDS+=("$!")

roslaunch $ROUTE_C_ROS1_LAUNCH
