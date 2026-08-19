#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$SCRIPT_DIR/../../../../.." && pwd)}"
ROS2_WS="${ROS2_WS:-$PROJECT_ROOT/ros2}"
WALK_CONFIG="${WALK_CONFIG:-$PROJECT_ROOT/config/route_c_walk.yaml}"
SOCCER_POLICY="${SOCCER_POLICY:-$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard/instinct_onboard/resources/kick_policy/20260730_125511_from20260728_105738-8-1}"
FALL_RECOVERY_ROOT="${FALL_RECOVERY_ROOT:-$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard/instinct_onboard/resources/fall_recovery/fall_recovery_06281302}"
FACEUP_GET_UP_POLICY="${FACEUP_GET_UP_POLICY:-$FALL_RECOVERY_ROOT/faceup/20260625_131016_H0W_FaceUpDirect_preview10_robustV3_reset06_rp08_yaw015_jp06_vel02_gain085115_mass0911__GPU5_from20260623_101049}"
FACEUP_GET_UP_MOTION="${FACEUP_GET_UP_MOTION:-$FALL_RECOVERY_ROOT/faceup/motion}"
FACEDOWN_GET_UP_POLICY="${FACEDOWN_GET_UP_POLICY:-$FALL_RECOVERY_ROOT/facedown/20260625_095417_H0W_FallRecovery_facedown_quick_robustV1_from20260624_173850_87000}"
FACEDOWN_GET_UP_MOTION="${FACEDOWN_GET_UP_MOTION:-$FALL_RECOVERY_ROOT/facedown/motion}"

# Keep ROS2 DDS local to this robot unless the caller explicitly overrides it.
source "$SCRIPT_DIR/setup_soccer_ros2_env.sh"

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

set +u
source "$HOME/.bashrc" >/dev/null 2>&1 || true
set -u
if declare -F rosenv >/dev/null 2>&1; then
  set +u
  rosenv foxy
  set -u
else
  source_setup /opt/ros/foxy/setup.bash
fi
source "$SCRIPT_DIR/setup_soccer_ros2_env.sh"

cd "$ROS2_WS"
source_setup install/setup.bash
source "$SCRIPT_DIR/setup_soccer_ros2_env.sh"
export PYTHONPATH="$ROS2_WS/src/instinct_onboard_soccer/instinct_onboard:$HOME/ros2_numpy:/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}"

# Strip ROS1 Noetic leftovers inherited from the parent shell. rosenv only cleans
# PATH, so Noetic otherwise shadows Foxy on PYTHONPATH (its geometry_msgs/std_msgs/
# tf2_ros break ROS2 type support and pull in rospkg) and on LD_LIBRARY_PATH (its
# libtf2.so makes Foxy's tf2_py crash with an undefined-symbol error).
strip_noetic() { printf '%s' "$1" | tr ':' '\n' | grep -v '/opt/ros/noetic' | paste -sd:; }
export PATH="$(strip_noetic "${PATH:-}")"
export PYTHONPATH="$(strip_noetic "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(strip_noetic "${LD_LIBRARY_PATH:-}")"
export CMAKE_PREFIX_PATH="$(strip_noetic "${CMAKE_PREFIX_PATH:-}")"
unset ROS_MASTER_URI ROS_IP ROS_ROOT ROS_PACKAGE_PATH ROS_ETC_DIR 2>/dev/null || true

# Foxy is built for python3.8, but ~/.bashrc auto-activates conda base (python
# 3.13), which cannot load Foxy's compiled message bindings. Run every ROS2 node
# (UDP receivers included) under the 3.8 venv.
source_setup "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate instinct_venv
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

if [[ -f "$WALK_CONFIG" ]]; then
  eval "$(python - "$WALK_CONFIG" "$PROJECT_ROOT" <<'PY'
import os
import shlex
import sys
import yaml

cfg_path, project_root = sys.argv[1], sys.argv[2]
with open(cfg_path) as f:
    data = yaml.safe_load(f) or {}
walk = data.get("walk", data)
valid_modes = {"h0w_nowaist", "h1w_locked_waist"}
mode = os.environ.get("WALK_MODE") or str(walk.get("mode", "h0w_nowaist"))
if mode not in valid_modes:
    raise SystemExit(f"Unsupported walk.mode '{mode}', expected one of: {', '.join(sorted(valid_modes))}")

default_policies = {
    "h0w_nowaist": "policy/piplus_walk_20260612_164526",
    "h1w_locked_waist": "policy/piplus_walk_20260612_164526",
}
default_robot_classes = {
    "h0w_nowaist": "PiPlus_S_12L8A0G2H0W",
    "h1w_locked_waist": "PiPlus_S_12L8A0G2H1W",
}

policy = os.environ.get("WALK_POLICY") or (walk.get("policies") or {}).get(mode) or default_policies[mode]
if not os.path.isabs(policy):
    policy = os.path.join(project_root, policy)
robot_class = os.environ.get("WALK_ROBOT_CLASS") or (walk.get("robot_classes") or {}).get(mode) or default_robot_classes[mode]

for key, value in {
    "WALK_MODE": mode,
    "WALK_POLICY": policy,
    "WALK_ROBOT_CLASS": robot_class,
}.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"
else
  WALK_MODE="${WALK_MODE:-h1w_locked_waist}"
  WALK_POLICY="${WALK_POLICY:-$PROJECT_ROOT/policy/piplus_walk_20260612_164526}"
  WALK_ROBOT_CLASS="${WALK_ROBOT_CLASS:-PiPlus_S_12L8A0G2H1W}"
fi

echo "[INFO] Route C ROS2 workspace: $ROS2_WS"
echo "[INFO] Route C walk config: config=$WALK_CONFIG mode=$WALK_MODE robot_class=$WALK_ROBOT_CLASS"
echo "[INFO] Route C policies: walk=$WALK_POLICY soccer=$SOCCER_POLICY"
echo "[INFO] Route C fall recovery: faceup=$FACEUP_GET_UP_POLICY facedown=$FACEDOWN_GET_UP_POLICY"
echo "[INFO] Route C vision source: legacy ROS1 /dvision_${ZJUDANCER_ROBOTID:-1}/VisionInfo via UDP 15003"
echo "[INFO] Route C DDS guard: ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}, RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"

python "$SCRIPT_DIR/ros2_cmd_vel_udp_receiver.py" \
  --topic /ros1_cmd_vel \
  --host 127.0.0.1 \
  --port 15000 \
  --timeout 0.5 &
PIDS+=("$!")

python "$SCRIPT_DIR/ros2_head_joint_udp_receiver.py" \
  --topic /pi_plus_absolute \
  --host 127.0.0.1 \
  --port 15001 \
  --timeout 0.5 &
PIDS+=("$!")

python "$SCRIPT_DIR/ros2_head_state_udp_sender.py" \
  --topic /joint_states \
  --host 127.0.0.1 \
  --yaw-index 20 \
  --pitch-index 21 \
  --port 15002 &
PIDS+=("$!")

python "$SCRIPT_DIR/ros2_ball_position_udp_receiver.py" \
  --topic /ros1_ball_position \
  --host 127.0.0.1 \
  --port 15003 \
  --timeout 0.5 \
  --frame-id base_link &
PIDS+=("$!")

python "$SCRIPT_DIR/ros2_goal_yaw_udp_receiver.py" \
  --topic /ros1_goal_yaw_b \
  --host 127.0.0.1 \
  --port 15004 \
  --timeout 0.5 &
PIDS+=("$!")

python "$SCRIPT_DIR/soccer_control_node.py" \
  --walk_logdir "$WALK_POLICY" \
  --walk_mode "$WALK_MODE" \
  --walk_robot_class "$WALK_ROBOT_CLASS" \
  --soccer_logdir "$SOCCER_POLICY" \
  --cmd_vel_topic /ros1_cmd_vel \
  --cmd_vel_timeout 0.5 \
  --max_cmd_vel 0.5 0.5 1.0 \
  --head_joint_topic /pi_plus_absolute \
  --head_joint_timeout 0.5 \
  --ball_base_topic /ros1_ball_position \
  --ball_base_timeout 0.5 \
  --goal_yaw_topic /ros1_goal_yaw_b \
  --goal_yaw_timeout 0.5 \
  --enable_fall_detection \
  --fall_gravity_z_threshold 0.55 \
  --fall_confirm_frames 15 \
  --fall_face_axis 0 \
  --fall_faceup_sign -1.0 \
  --fall_face_threshold 0.65 \
  --recovery_confirm_frames 10 \
  --recovery_start_delay 1.0 \
  --classify_ang_vel_max 2.0 \
  --faceup_get_up_logdir "$FACEUP_GET_UP_POLICY" \
  --faceup_get_up_motion_dir "$FACEUP_GET_UP_MOTION" \
  --facedown_get_up_logdir "$FACEDOWN_GET_UP_POLICY" \
  --facedown_get_up_motion_dir "$FACEDOWN_GET_UP_MOTION" \
  --get_up_prepare_steps 60 \
  --get_up_prepare_kpkd_factor 0.1 \
  --kick_speed 3.5 \
  --kick_latch_duration 1.0 \
  --close_to_ball 1.0 \
  --close_to_ball_exit 1.2 \
  --kick_distance 0.55 \
  --kick_exit_distance 0.85 \
  --kick_intent_distance 0.55 \
  --kick_intent_face_tolerance_deg 45.0 \
  --face_ball_tolerance_deg 50.0 \
  --face_ball_exit_tolerance_deg 70.0 \
  --nodryrun
