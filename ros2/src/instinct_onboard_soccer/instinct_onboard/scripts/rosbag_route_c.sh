#!/usr/bin/env bash
# One-command Route C recorder.
#
# It records ROS1 and ROS2 bags at the same time, then stores them under one
# timestamped directory. It does not start Route C and does not publish commands.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$SCRIPT_DIR/../../../../.." && pwd)}"
ROS1_WS="${ROS1_WS:-$PROJECT_ROOT/RoboCup_Workspace}"
ROS2_WS="${ROS2_WS:-$PROJECT_ROOT/ros2}"
ROS2_ENV_SETUP="${ROS2_ENV_SETUP:-$SCRIPT_DIR/setup_soccer_ros2_env.sh}"

BAG_ROOT="${BAG_DIR:-$HOME/route_c_bags}"
RUN_NAME="${1:-route_c_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$BAG_ROOT/$RUN_NAME"

WAIT_TIMEOUT="${WAIT_TIMEOUT:-120}"
ROS1_READY_TOPIC="${ROS1_READY_TOPIC:-/dvision_1/VisionInfo}"
ROS2_READY_TOPIC="${ROS2_READY_TOPIC:-/joint_states}"
WITH_IMAGE="${WITH_IMAGE:-0}"
WITH_RAW_IMAGE="${WITH_RAW_IMAGE:-0}"
SPLIT_MB="${SPLIT_MB:-2048}"
MIN_FREE="${MIN_FREE:-3G}"
ROS2_STORAGE="${ROS2_STORAGE:-sqlite3}"
ROS2_MAX_BAG_SIZE="${ROS2_MAX_BAG_SIZE:-$((SPLIT_MB * 1024 * 1024))}"

mkdir -p "$OUT_DIR"

ROS1_TOPICS=(
  /dvision_1/VisionInfo
  /dbehavior_1/ActionCommand
  /dbehavior_1/BehaviorInfo
  /cmd_vel
  /pi_plus_absolute
  /dmotion_1/MotionInfo
  /livelybot_real_real/head_pitch_joint_controller/state
  /livelybot_real_real/head_yaw_joint_controller/state
  /livelybot_real_real/Head_Pitch_controller/state
  /livelybot_real_real/Head_Yaw_controller/state
  /rosout
  /tf
)

if [[ "$WITH_IMAGE" == "1" ]]; then
  ROS1_TOPICS+=(
    /dvision_1/cam_image/compressed
    /dvision_1/cam_image/theora
  )
fi

if [[ "$WITH_RAW_IMAGE" == "1" ]]; then
  ROS1_TOPICS+=(
    /dvision_1/cam_image
  )
fi

ROS2_TOPICS=(
  /ros1_cmd_vel
  /cmd_vel
  /ros1_ball_position
  /ros1_goal_yaw_b
  /pi_plus_absolute
  /soccer/ball_base_xy_velocity
  /soccer/decision_debug
  /control_command
  /raw_actions
  /joint_states
  /error_joint_states
  /imu
  /rpy
  /quat
  /gyro
  /acc
  /fsm_state
  /session_status
  /hightorque_joy
  /joy
  /debug_msg
  /power_switch_state
  /battery_voltage
  /battery_current
  /battery_level
  /battery_temperature
  /battery_bms_temperature
  /tf
  /rosout
)

if [[ -n "${ROS1_EXTRA_TOPICS:-}" ]]; then
  read -r -a EXTRA <<< "$ROS1_EXTRA_TOPICS"
  ROS1_TOPICS+=("${EXTRA[@]}")
fi

if [[ -n "${ROS2_EXTRA_TOPICS:-}" ]]; then
  read -r -a EXTRA <<< "$ROS2_EXTRA_TOPICS"
  ROS2_TOPICS+=("${EXTRA[@]}")
fi

source_setup() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "[ERROR] setup file not found: $path" >&2
    exit 1
  fi
  set +u
  # shellcheck source=/dev/null
  source "$path"
  set -u
}

strip_noetic() {
  printf '%s' "$1" | tr ':' '\n' | grep -v '/opt/ros/noetic' | paste -sd:
}

source_ros1() {
  source_setup /opt/ros/noetic/setup.bash
  source_setup "$ROS1_WS/lib/devel/setup.bash"
  source_setup "$ROS1_WS/core/devel/setup.bash"
  export ZJUDANCER_ROBOTID="${ZJUDANCER_ROBOTID:-1}"
}

source_ros2() {
  set +u
  source "$HOME/.bashrc" >/dev/null 2>&1 || true
  if declare -F rosenv >/dev/null 2>&1; then
    rosenv foxy
  else
    source /opt/ros/foxy/setup.bash
  fi
  set -u
  source_setup "$ROS2_ENV_SETUP"
  cd "$ROS2_WS"
  source_setup install/setup.bash
  source_setup "$ROS2_ENV_SETUP"

  export PATH="$(strip_noetic "${PATH:-}")"
  export PYTHONPATH="$(strip_noetic "${PYTHONPATH:-}")"
  export LD_LIBRARY_PATH="$(strip_noetic "${LD_LIBRARY_PATH:-}")"
  export CMAKE_PREFIX_PATH="$(strip_noetic "${CMAKE_PREFIX_PATH:-}")"
  unset ROS_MASTER_URI ROS_IP ROS_ROOT ROS_PACKAGE_PATH ROS_ETC_DIR 2>/dev/null || true

  if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
    source_setup "$HOME/miniconda3/etc/profile.d/conda.sh"
    if conda env list | awk '{print $1}' | grep -Fx instinct_venv >/dev/null 2>&1; then
      conda activate instinct_venv
      export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
    fi
  fi
}

wait_until() {
  local label="$1"
  local command="$2"
  local start
  start="$(date +%s)"
  while true; do
    if bash -lc "$command" >/dev/null 2>&1; then
      echo "[$label] ready"
      return 0
    fi
    local now
    now="$(date +%s)"
    if (( now - start >= WAIT_TIMEOUT )); then
      echo "[$label] not ready after ${WAIT_TIMEOUT}s, recording anyway" >&2
      return 0
    fi
    sleep 1
  done
}

write_metadata() {
  {
    echo "run_name=$RUN_NAME"
    echo "created_at=$(date --iso-8601=seconds)"
    echo "host=$(hostname)"
    echo "out_dir=$OUT_DIR"
    echo "project_root=$PROJECT_ROOT"
    echo "ros1_ws=$ROS1_WS"
    echo "ros2_ws=$ROS2_WS"
    echo "ros2_storage=$ROS2_STORAGE"
    echo "with_image=$WITH_IMAGE"
    echo "with_raw_image=$WITH_RAW_IMAGE"
    echo "split_mb=$SPLIT_MB"
    echo "min_free=$MIN_FREE"
    echo ""
    echo "[ros1_topics]"
    printf '%s\n' "${ROS1_TOPICS[@]}"
    echo ""
    echo "[ros2_topics]"
    printf '%s\n' "${ROS2_TOPICS[@]}"
  } > "$OUT_DIR/metadata.txt"
}

PIDS=()
STOPPING=0

cleanup() {
  if [[ "$STOPPING" == "1" ]]; then
    return
  fi
  STOPPING=1

  echo ""
  echo "[route_c_bag] stopping recorders ..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done

  local deadline
  deadline=$((SECONDS + 25))
  while (( SECONDS < deadline )); do
    local alive=0
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=1
      fi
    done
    (( alive == 0 )) && break
    sleep 1
  done

  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true

  echo "[route_c_bag] done: $OUT_DIR"
  echo "[route_c_bag] logs: $OUT_DIR/ros1_record.log  $OUT_DIR/ros2_record.log"
  find "$OUT_DIR" -maxdepth 2 -type f -printf '%p %s bytes\n' 2>/dev/null | sort || true
}

trap cleanup INT TERM EXIT

write_metadata

(
  source_ros1
  wait_until "ROS1 master" "source /opt/ros/noetic/setup.bash && rostopic list"
  wait_until "ROS1 topic $ROS1_READY_TOPIC" "source /opt/ros/noetic/setup.bash && rostopic list | grep -Fx '$ROS1_READY_TOPIC'"
  exec rosbag record \
    --split \
    --size="$SPLIT_MB" \
    --min-space="$MIN_FREE" \
    -O "$OUT_DIR/ros1" \
    "${ROS1_TOPICS[@]}"
) > "$OUT_DIR/ros1_record.log" 2>&1 &
PIDS+=("$!")

(
  source_ros2
  wait_until "ROS2 topic $ROS2_READY_TOPIC" "source /opt/ros/foxy/setup.bash && source '$ROS2_ENV_SETUP' && cd '$ROS2_WS' && source install/setup.bash && source '$ROS2_ENV_SETUP' && ros2 topic list | grep -Fx '$ROS2_READY_TOPIC'"
  exec ros2 bag record \
    -s "$ROS2_STORAGE" \
    --max-bag-size "$ROS2_MAX_BAG_SIZE" \
    -o "$OUT_DIR/ros2" \
    "${ROS2_TOPICS[@]}"
) > "$OUT_DIR/ros2_record.log" 2>&1 &
PIDS+=("$!")

echo "=========================================="
echo " Route C rosbag recording started"
echo " Output dir : $OUT_DIR"
echo " ROS1 bag   : $OUT_DIR/ros1*.bag"
echo " ROS2 bag   : $OUT_DIR/ros2/"
echo " Logs       : $OUT_DIR/ros1_record.log"
echo "              $OUT_DIR/ros2_record.log"
echo " Stop       : Ctrl+C"
echo ""
echo " Optional:"
echo "   WITH_IMAGE=1      record compressed/theora camera topics"
echo "   WITH_RAW_IMAGE=1  record raw camera image; very large"
echo "   BAG_DIR=/path     change output root"
echo "=========================================="

wait "${PIDS[@]}"
