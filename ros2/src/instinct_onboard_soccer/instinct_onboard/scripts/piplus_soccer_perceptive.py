from __future__ import annotations

import math
import os
import queue
import time
from typing import Optional, Tuple

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent, OnboardAgent
from instinct_onboard.agents.ort_utils import get_policy_ort_providers
from instinct_onboard.normalizer import Normalizer
from instinct_onboard.ros_nodes.base import RealNode
from instinct_onboard.ros_nodes.hightorque import HightorqueNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500
MAIN_LOOP_DT = 0.02  # 50 Hz

# The ball topic carries the ball center in the camera FLU frame
# (+X forward, +Y left, +Z up). This node runs the FULL transform chain to
# reach the training frame:
#   camera FLU --head FK (head_yaw, head_pitch)--> base_link (torso frame)
#              --IMU de-tilt (drop torso roll/pitch)--> yaw-base frame
# so (ball_x_b, ball_y_b) match the training SoccerCommand frame exactly
# (gravity-aligned, follows heading only).
DEFAULT_BALL_POSITION_CAMERA_TOPIC = "/soccer/ball_position_camera"

# --------------------------------------------------------------------------- #
# Camera extrinsics / head FK constants (ported verbatim from piplus_soccer.py)
# --------------------------------------------------------------------------- #
# NOTE ON BASELINE: these constants assume a ZED *Mini* (63 mm baseline). The
# detector launch in this workspace defaults to camera_model:=zed2i, whose
# baseline is 120 mm. If you are actually running a ZED 2i (or any other model),
# UPDATE ZED_MINI_BASELINE_M and re-verify CAMERA_LINK_POS_HEAD_PITCH /
# LEFT_EYE_POS_CAMERA_LINK against that camera's real left-eye offset. The error
# is at the cm level for a ball ~0.3 m away, but it is a systematic bias in the
# ball's base-frame xy and worth fixing before relying on it.
ZED_MINI_BASELINE_M = 0.063
CAMERA_LINK_POS_HEAD_PITCH = np.array([0.048925, 0.0053519, 0.058472], dtype=np.float64)
LEFT_EYE_POS_CAMERA_LINK = np.array([0.0, 0.5 * ZED_MINI_BASELINE_M, 0.0], dtype=np.float64)
HEAD_YAW_POS_BASE = np.array([0.014, -0.00026667, 0.1981], dtype=np.float64)
HEAD_PITCH_POS_HEAD_YAW = np.array([0.0, 0.0, 0.0275], dtype=np.float64)

# Input ball frame -> camera_link (robot axes FLU: +X fwd, +Y left, +Z up).
# The topic is published in camera FLU (+X forward, +Y left, +Z up) -- the SAME
# axis convention as camera_link -- so this rotation is IDENTITY. The head FK
# translation + head_yaw/pitch rotation + IMU de-tilt below still apply.
R_CAMERA_LINK_BALL_INPUT = np.eye(3, dtype=np.float64)
# Reference (NOT used; current input is FLU -> identity):
#   optical  (+Z fwd, +X right, +Y down): np.array([[0,0,1],[-1,0,0],[0,-1,0]])
#   camera LFU (+X left, +Y fwd, +Z up) : np.array([[0,1,0],[1,0,0],[0,0,1]])

"""
PiPlus Soccer Node  (NO-WAIST, 22-DOF)  --  ENCODER/DECODER variant + CAMERA
============================================================================

Deployment node for PiPlus_S_12L8A0G2H0W (22-DOF: 12 legs + 2 head + 8 arms,
NO waist) running the **encoder/decoder** soccer policy (BallEstimatorActorCritic
+ EstimatorWasabiPPO), trained with pi_plus_soccer_encdec_env_cfg.py.

Training env : HT_lab/tasks/pi_plus_soccer/.../pi_plus_soccer_encdec_env_cfg.py
Robot class  : PiPlus_S_12L8A0G2H0W   (22 actions, full body, no waist)
Node base    : HightorqueNode          (/imu, /joy, hightorque_midware motors)

VISION BALL (this file vs the plain encdec node)
------------------------------------------------
This version adds the SAME camera ball-position path that piplus_soccer.py has:
while KICKING (A held), the ball offset fed to the policy is taken from the
reserved topic

    /soccer/ball_position_camera   (geometry_msgs/PointStamped)

which carries the ball center xyz in the camera FLU frame
(+X forward, +Y left, +Z up). The agent expresses it in the training frame in
TWO steps:

  (1) head FK: camera FLU --Rz(head_yaw)*Ry(head_pitch)--> base_link
      (torso frame). Uses the live head joint angles from joint_pos_. This
      accounts for HEAD rotation but the result still tilts with the torso.
  (2) IMU de-tilt: base_link --drop torso roll/pitch--> yaw-base frame.
      Uses the IMU quat (q) and its yaw-only part (q_yaw); the residual
      rotation t = q_yaw^-1 * q is the torso roll/pitch, applied to the ball
      so the output is gravity-aligned and follows heading only.

Training uses yaw-base (quat_apply_inverse(yaw_quat(root_quat_w), ...)), so BOTH
steps are needed: step (1) alone gives base_link, which only equals yaw-base
when the torso is upright. During the kick lunge the torso pitches forward and
the two differ by several cm at a ~0.3 m ball; step (2) removes that bias.

  *** Kick mode only (deliberate). *** In WALK/IDLE the ball is still masked to
  0, exactly as before: training dumped the walk-role ball ~8 m away so the
  policy / encoder saw 0 during walking. Feeding a live vision ball during walk
  would be off-distribution for the encoder. If you DO want walk-toward-ball
  behavior, that is a training-side change, not just a deploy toggle.

How the encdec graph consumes the ball (unchanged)
--------------------------------------------------
The exported actor.onnx for the encdec model **contains the encoder**:
    full policy obs -> encode ball_pos_b_noisy (H*2) -> z (latent_dim)
    -> drop ball_pos_b_noisy, concat z -> actor MLP -> action
So inference (step) is unchanged: feed the FULL policy obs, get an action; the
encoder runs inside the ONNX graph. The vision ball only changes the [3:5]
columns of the 7-D soccer command BEFORE it is pushed into the shared ball
history, which then feeds BOTH obs terms:

  (A) soccer_command (soccer_command_async) with drop_ball_pos -> the ball
      columns [3:5] are DELETED (5 cols/frame remain). The actor never sees the
      ball through soccer_command.
  (B) ball_pos_b_noisy (ball_pos_b_noisy_hist) -> the ball columns of the SAME
      downsampled history, history_samples=10 frames of (ball_x_b, ball_y_b) =
      10*2 = 20, appended LAST. This is what the encoder compresses to z.

PolicyCfg obs order (must match env.yaml exactly; base.py iterates config order):
    base_ang_vel  (imu_ang_vel,           hist 8, scale 0.25)   3*8 = 24
    projected_gravity (imu_projected_gravity, hist 8)           3*8 = 24
    soccer_command (soccer_command_async, hist 1)  -> 50  (ball cols deleted)
    joint_pos     (joint_pos_rel,         hist 8)               22*8 = 176
    joint_vel     (joint_vel_rel, sc 0.05,hist 8)               22*8 = 176
    actions       (last_action,           hist 8)               22*8 = 176
    ball_pos_b_noisy (ball_pos_b_noisy_hist, hist 1) -> 10*2 = 20

SoccerCommand layout (7-D, ALL in robot yaw-base frame):
    [0] vx_b   [1] vy_b   [2] wz_b
    [3] ball_x_b   [4] ball_y_b   <- from VISION while kicking (else 0)
    [5] kick_dir_yaw_b   [6] kick_speed_target

Control mapping, state machine, joystick controls: see piplus_soccer.py.
Usage:
    python piplus_soccer_encdec.py \\
        --logdir /path/to/encdec_policy \\
        --ball_position_camera_topic /soccer/ball_position_camera \\
        --nodryrun
"""


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def resolve_logdir_and_actor(path: str) -> Tuple[str, Optional[str]]:
    """Accept either a logdir or a direct .onnx path."""
    path = os.path.abspath(path)
    if path.endswith(".onnx"):
        actor_path = path
        logdir = os.path.dirname(os.path.dirname(path))  # exported/'s parent
        return logdir, actor_path
    return path, None


def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat_wxyz(q: np.ndarray) -> float:
    """Extract yaw (rotation about world z) from a wxyz quaternion."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


# --- quaternion helpers (wxyz) for the IMU de-tilt (base_link -> yaw frame) --- #
def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def _quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q (wxyz)."""
    w = float(q[0])
    u = np.array([q[1], q[2], q[3]], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64).reshape(3)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


# --------------------------------------------------------------------------- #
# Soccer agent (encoder/decoder unified walk+kick policy, no-waist) + camera
# --------------------------------------------------------------------------- #
class SoccerEncDecAgent(OnboardAgent):
    """ONNX inference agent for the encdec SoccerCommand policy (no-waist).

    Ball from the camera topic (camera FLU) is transformed FLU -> base_link
    (head FK) -> yaw-base (IMU de-tilt) before being used as (ball_x_b, ball_y_b).
    Maintains the async ball-position history and exposes TWO obs terms:
        _get_soccer_command_async_obs   -> 50-dim, ball columns [3:5] DELETED
        _get_ball_pos_b_noisy_hist_obs  -> history_samples*2 ball history (clean)
    The exported actor.onnx contains the encoder, so step() is unchanged.
    """

    LSTICK_DEADZONE: float = 0.15
    _LSTICK_DEADZONE_SQ: float = LSTICK_DEADZONE * LSTICK_DEADZONE

    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
        actor_path: Optional[str] = None,
        kick_speed: float = 1.0,
        x_vel_scale: float = 0.5,
        y_vel_scale: float = 0.5,
        yaw_vel_scale: float = 1.0,
        # async soccer-command / ball history — MUST match the env cfg
        # (BALL_PUB_PERIOD and history_samples in pi_plus_soccer_env_cfg.py;
        #  ball_pos_b_noisy history_samples MUST equal soccer_command's).
        soccer_pub_period: int = 5,
        soccer_history_samples: int = 10,
        # camera ball topic (vision-driven ball position while kicking)
        ball_position_camera_topic: str = DEFAULT_BALL_POSITION_CAMERA_TOPIC,
        ball_topic_timeout: float = 0.2,
        # head lock: override the head joints to a fixed look direction every step
        head_lock: bool = False,
        head_pitch_rad: float = math.radians(45.0),
        head_yaw_rad: float = 0.0,
    ):
        super().__init__(logdir, ros_node)
        self._actor_path_override = actor_path

        # --- command parameters ---
        self._kick_speed_const: float = float(kick_speed)

        self.x_vel_scale = float(x_vel_scale)
        self.y_vel_scale = float(y_vel_scale)
        self.yaw_vel_scale = float(yaw_vel_scale)

        # --- async ball history state (shared by soccer_command + ball_pos_b_noisy) ---
        self._pub_period: int = max(1, int(soccer_pub_period))
        self._history_samples: int = max(1, int(soccer_history_samples))
        self._soccer_hist: Optional[np.ndarray] = None   # (H, 7), newest at -1
        self._pub_counter: int = 0
        # per-step refresh guard so BOTH obs terms read one consistent buffer
        self._step_id: int = 0
        self._ball_hist_step_id: int = -1
        self._last_cmd7: Optional[np.ndarray] = None

        # --- world-frame ("anchor frame") kick-direction state ---
        # Only the kick DIRECTION is world-anchored now; the ball comes straight
        # from vision (or is masked to 0), so there is no anchored ball position.
        self._kick_was_active: bool = False
        self._robot_yaw_w_at_anchor: float = 0.0
        self._kick_dir_yaw_w: float = 0.0

        # --- camera ball topic ---
        self._ball_position_camera_topic = str(ball_position_camera_topic)
        self._ball_topic_timeout = float(ball_topic_timeout)
        self._head_yaw_joint_idx = self.ros_node.sim_joint_names.index("head_yaw_joint")
        self._head_pitch_joint_idx = self.ros_node.sim_joint_names.index("head_pitch_joint")

        # --- head lock (override head joints to a fixed look direction) ---
        # Override happens in action space, AFTER inference, BEFORE _send:
        #   joint_target = action*action_scale + action_offset   (base.py)
        # so to force head joint j to a physical angle theta:
        #   action[j] = (theta - action_offset[j]) / action_scale[j]
        # The real head then physically tracks `theta`, so joint_pos obs reads
        # the locked angle naturally, and last_action obs (fed the commanded
        # action) is consistent too -- no need to fake any observation.
        self._head_lock = bool(head_lock)
        self._head_pitch_target_rad = float(head_pitch_rad)
        self._head_yaw_target_rad = float(head_yaw_rad)
        # Will be filled in after action config is parsed (need scale/offset).
        self._head_lock_action_yaw: Optional[float] = None
        self._head_lock_action_pitch: Optional[float] = None

        self.ort_sessions: dict = dict()
        self._parse_obs_config()
        self._parse_action_config()
        self._load_models()

        if self._head_lock:
            self._precompute_head_lock_actions()

    # ------------- model loading ------------- #
    def _load_models(self) -> None:
        ort_execution_providers = get_policy_ort_providers()
        actor_path = self._actor_path_override or os.path.join(
            self.logdir, "exported", "actor.onnx"
        )
        self.ort_sessions["actor"] = ort.InferenceSession(
            actor_path, providers=ort_execution_providers
        )
        print(f"[SoccerEncDecAgent] Loaded ONNX actor (encoder+actor) from {actor_path}")

        normalizer_path = os.path.join(self.logdir, "exported", "policy_normalizer.npz")
        if os.path.exists(normalizer_path):
            self.normalizer = Normalizer(load_path=normalizer_path)
            print(f"[SoccerEncDecAgent] Loaded normalizer from {normalizer_path}")
        else:
            self.normalizer = None
            print("[SoccerEncDecAgent] No policy_normalizer.npz, running un-normalized")

    # ------------- lifecycle ------------- #
    def reset(self) -> None:
        self._kick_was_active = False
        self._soccer_hist = None
        self._pub_counter = 0
        self._step_id = 0
        self._ball_hist_step_id = -1
        self._last_cmd7 = None
        super().reset()
        self.ros_node.get_logger().info(
            f"[SoccerEncDecAgent] reset: "
            f"kick_speed={self._kick_speed_const:.2f} m/s, "
            f"vel_scale=(x={self.x_vel_scale:+.2f}, y={self.y_vel_scale:+.2f}, "
            f"yaw={self.yaw_vel_scale:+.2f}), "
            f"ball_hist={self._history_samples}x2 @ pub_period={self._pub_period} "
            f"(soccer_command ball cols deleted; ball surfaced via ball_pos_b_noisy), "
            f"ball_topic={self._ball_position_camera_topic} (timeout={self._ball_topic_timeout:.2f}s); "
            f"ball = vision while kicking, else 0 (no virtual fallback)"
        )

    def _precompute_head_lock_actions(self) -> None:
        """Invert joint_target = action*scale + offset for the two head joints,
        so that commanding these action values pins the head to the requested
        look direction. Guards against a zero action_scale (head not actually
        action-driven through this mapping)."""
        scale = np.asarray(self.action_scale, dtype=np.float64)
        offset = np.asarray(self.action_offset, dtype=np.float64)
        eps = 1e-6

        def invert(idx: int, theta: float, name: str) -> Optional[float]:
            s = float(scale[idx])
            if abs(s) < eps:
                self.ros_node.get_logger().warn(
                    f"[head_lock] action_scale[{name}]={s:.3g} ~ 0; cannot map a "
                    f"target angle through action for this joint. Head lock on "
                    f"{name} will be SKIPPED."
                )
                return None
            return (theta - float(offset[idx])) / s

        self._head_lock_action_yaw = invert(
            self._head_yaw_joint_idx, self._head_yaw_target_rad, "head_yaw_joint"
        )
        self._head_lock_action_pitch = invert(
            self._head_pitch_joint_idx, self._head_pitch_target_rad, "head_pitch_joint"
        )
        self.ros_node.get_logger().info(
            f"[head_lock] ENABLED: target head_yaw={math.degrees(self._head_yaw_target_rad):+.1f} deg, "
            f"head_pitch={math.degrees(self._head_pitch_target_rad):+.1f} deg "
            f"(+pitch sign follows the joint's own convention; if the head tilts "
            f"the WRONG way, negate --head_pitch_deg). "
            f"action_yaw={self._head_lock_action_yaw}, action_pitch={self._head_lock_action_pitch}"
        )

    def _apply_head_lock(self, action: np.ndarray) -> np.ndarray:
        """Overwrite the head action dims so the head holds the fixed look dir."""
        if not self._head_lock:
            return action
        if self._head_lock_action_yaw is not None:
            action[self._head_yaw_joint_idx] = self._head_lock_action_yaw
        if self._head_lock_action_pitch is not None:
            action[self._head_pitch_joint_idx] = self._head_lock_action_pitch
        return action

    # ------------- one inference step ------------- #
    def step(self) -> Tuple[np.ndarray, bool]:
        # advance the per-step id so the ball history refreshes exactly once,
        # regardless of obs-term ordering in env.yaml.
        self._step_id += 1
        obs = self._get_observation()
        if self.normalizer is not None:
            normalized_obs = self.normalizer.normalize(obs).astype(np.float32)[None, :]
        else:
            normalized_obs = obs.astype(np.float32)[None, :]

        actor_in = self.ort_sessions["actor"].get_inputs()[0].name
        action = self.ort_sessions["actor"].run(None, {actor_in: normalized_obs})[0]
        action = action.reshape(-1)
        # Pin the head to the configured look direction (no-op unless head_lock).
        action = self._apply_head_lock(action)
        done = False
        return action, done

    # ------------- anchoring helpers (virtual-ball fallback) ------------- #
    def _current_robot_yaw_rel(self) -> float:
        quat_wxyz = self.ros_node._get_quat_w_obs()
        return _wrap_to_pi(_yaw_from_quat_wxyz(quat_wxyz) - self._robot_yaw_w_at_anchor)

    def _anchor_kick(self) -> None:
        """Snapshot the world frame on the A rising edge: anchor yaw = current
        IMU yaw, kick direction straight ahead. The ball is NOT anchored here
        (it comes from vision, or is 0 when unseen)."""
        quat_wxyz = self.ros_node._get_quat_w_obs()
        self._robot_yaw_w_at_anchor = _yaw_from_quat_wxyz(quat_wxyz)
        self._kick_dir_yaw_w = 0.0
        self.ros_node.get_logger().info(
            f"[SoccerEncDecAgent] kick anchored: yaw_w={self._robot_yaw_w_at_anchor:+.3f} rad "
            f"(ball from vision / 0)"
        )

    # ------------- camera ball -> yaw-base frame ------------- #
    def _latest_ball_xyz_camera(self) -> Optional[np.ndarray]:
        """Latest ball xyz in the camera FLU frame, or None if stale/absent."""
        getter = getattr(self.ros_node, "get_latest_ball_position_camera", None)
        if getter is None:
            return None
        return getter(max_age_s=self._ball_topic_timeout)

    def _ball_xyz_base_from_camera(self, ball_xyz_camera: np.ndarray) -> np.ndarray:
        """Step (1): camera-FLU ball xyz -> base_link (torso) frame via head FK.

        Chain: base -> head_yaw(Rz) -> head_pitch(Ry) -> camera_link ->
        left eye -> input(R_CAMERA_LINK_BALL_INPUT = identity for FLU input).
        Reads the live head joint angles from joint_pos_. The ball vector is in
        camera FLU (+X fwd, +Y left, +Z up), same axes as camera_link, so only
        the head rotation + extrinsic translation apply. Output STILL tilts with
        the torso (de-tilt happens in step 2).
        """
        joint_pos = self.ros_node.joint_pos_
        head_yaw = float(joint_pos[self._head_yaw_joint_idx])
        head_pitch = float(joint_pos[self._head_pitch_joint_idx])

        r_base_head_yaw = _rot_z(head_yaw)
        r_base_head_pitch = r_base_head_yaw @ _rot_y(head_pitch)
        p_base_head_pitch = HEAD_YAW_POS_BASE + r_base_head_yaw @ HEAD_PITCH_POS_HEAD_YAW
        p_base_camera_link = p_base_head_pitch + r_base_head_pitch @ CAMERA_LINK_POS_HEAD_PITCH
        p_base_left_eye = p_base_camera_link + r_base_head_pitch @ LEFT_EYE_POS_CAMERA_LINK
        r_base_left_eye_input = r_base_head_pitch @ R_CAMERA_LINK_BALL_INPUT

        ball_xyz_camera = np.asarray(ball_xyz_camera, dtype=np.float64).reshape(3)
        return p_base_left_eye + r_base_left_eye_input @ ball_xyz_camera

    def _detilt_base_to_yaw(self, ball_xyz_base: np.ndarray) -> np.ndarray:
        """Step (2): base_link (torso) frame -> yaw-base (gravity-aligned) frame.

        Uses the IMU quat q (world<-base) and its yaw-only part q_yaw
        (world<-yaw). The residual t = q_yaw^-1 * q is the torso roll/pitch;
        applying it to a base-frame vector rotates it into the yaw frame, exactly
        matching training's quat_apply_inverse(yaw_quat(root_quat_w), ...).
        """
        q = np.asarray(self.ros_node._get_quat_w_obs(), dtype=np.float64).reshape(4)  # wxyz
        yaw = _yaw_from_quat_wxyz(q)
        q_yaw = np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)], dtype=np.float64)
        t = _quat_mul(_quat_conj(q_yaw), q)   # yaw <- base residual (roll/pitch only)
        return _quat_apply(t, ball_xyz_base)

    # ------------- 7-D soccer command (single snapshot) ------------- #
    def _build_soccer_cmd(self) -> np.ndarray:
        """Build the current 7-D soccer command in robot yaw-base frame.

        KICK mode (A held): vel forced 0. Ball offset (cols [3:5]) comes from the
        camera topic when fresh, transformed FLU -> base_link (head FK) ->
        yaw-base (IMU de-tilt). When stale/absent the ball is masked to 0 -- NO
        fallback, NO hold. Kick dir steered by the left stick; kick speed = const
        only past the stick deadzone.

        WALK / IDLE (A not held): vel from the sticks; ball masked to 0; kick
        dir/speed forced 0.

        In the encdec variant these [3:5] are later DELETED from the
        soccer_command obs and surfaced instead through ball_pos_b_noisy -> z.
        """
        lx = float(self.ros_node.joy_stick_data.lx or 0.0)
        ly = float(self.ros_node.joy_stick_data.ly or 0.0)
        rx = float(self.ros_node.joy_stick_data.rx or 0.0)
        a_pressed = bool(getattr(self.ros_node.joy_stick_data, "A", False))

        if a_pressed:
            # ---------------- KICK mode ----------------
            if not self._kick_was_active:
                self._anchor_kick()
            self._kick_was_active = True

            vx, vy, wz = 0.0, 0.0, 0.0
            yaw_rel = self._current_robot_yaw_rel()

            # Ball: ONLY from vision. camera FLU -> base_link (head FK) -> yaw-base
            # (IMU de-tilt). Stale/absent -> mask to 0. No fallback/hold.
            ball_xyz_camera = self._latest_ball_xyz_camera()
            if ball_xyz_camera is not None:
                ball_xyz_base = self._ball_xyz_base_from_camera(ball_xyz_camera)  # (1) -> base_link
                ball_xyz_yaw = self._detilt_base_to_yaw(ball_xyz_base)            # (2) -> yaw frame
                ball_x_b = float(ball_xyz_yaw[0])   # +X forward (gravity-aligned)
                ball_y_b = float(ball_xyz_yaw[1])   # +Y left
            else:
                ball_x_b = 0.0
                ball_y_b = 0.0

            if lx * lx + ly * ly >= self._LSTICK_DEADZONE_SQ:
                vx_intent = ly * self.x_vel_scale
                vy_intent = -lx * self.y_vel_scale
                stick_dir_base = math.atan2(vy_intent, vx_intent)
                self._kick_dir_yaw_w = _wrap_to_pi(yaw_rel + stick_dir_base)
                kick_speed = self._kick_speed_const
            else:
                kick_speed = 0.0

            kick_dir_b = _wrap_to_pi(self._kick_dir_yaw_w - yaw_rel)
        else:
            # ---------------- WALK / IDLE mode ----------------
            self._kick_was_active = False
            vx = ly * self.x_vel_scale
            vy = -lx * self.y_vel_scale
            wz = -rx * self.yaw_vel_scale
            # far-ball mask -> 0 (matches SoccerCommand walk-role; ball ~8 m off).
            # Vision is intentionally NOT used here (off-distribution for walk).
            ball_x_b = 0.0
            ball_y_b = 0.0
            kick_dir_b = 0.0
            kick_speed = 0.0

        return np.array(
            [vx, vy, wz, ball_x_b, ball_y_b, kick_dir_b, kick_speed],
            dtype=np.float32,
        )

    # ------------- shared async ball-history refresh ------------- #
    def _ensure_ball_history_fresh(self) -> None:
        """Refresh the (H, 7) ball history at most ONCE per control step.

        Both _get_soccer_command_async_obs and _get_ball_pos_b_noisy_hist_obs
        call this; the per-step guard makes them read one consistent buffer
        regardless of which runs first. Mirrors soccer_command_async:
          - ball xy (cols 3:5) is the only async, downsampled-at-pub_period part
          - vel (0:3) and kick (5:7) are live and not part of the history here
        """
        if self._ball_hist_step_id == self._step_id and self._soccer_hist is not None:
            return  # already refreshed this step

        cmd7 = self._build_soccer_cmd()
        self._last_cmd7 = cmd7

        if self._soccer_hist is None:
            # seed the whole ring with the live command
            self._soccer_hist = np.tile(cmd7, (self._history_samples, 1)).astype(np.float32)
            self._pub_counter = 0

        # push a fresh ball snapshot every pub_period steps (10 Hz on 50 Hz loop),
        # hold in between. newest at index -1.
        if self._pub_counter % self._pub_period == 0:
            self._soccer_hist[:-1] = self._soccer_hist[1:]
            self._soccer_hist[-1] = cmd7
        self._pub_counter += 1

        self._ball_hist_step_id = self._step_id

    # ------------- soccer_command obs (async, BALL COLUMNS DELETED) ------------- #
    # env: soccer_obs.soccer_command_async with drop_ball_pos=True (encdec).
    # base.py resolves this by func name "soccer_command_async".
    #
    # *** VERIFIED against the exported actor.onnx graph ***
    # The onnx soccer_command slice is 50 dims = history_samples*5, i.e.
    # drop_ball_pos here DELETES the ball columns [3:5] (does NOT zero them),
    # leaving 5 cols/frame: [vx, vy, wz, kick_dir, kick_speed]. The ball reaches
    # the actor only through ball_pos_b_noisy -> encoder -> z.
    def _get_soccer_command_async_obs(self) -> np.ndarray:
        self._ensure_ball_history_fresh()
        out = self._soccer_hist.copy()                  # (H, 7)
        # live vel/kick broadcast into every frame (NOT downsampled):
        out[:, 0:3] = self._last_cmd7[0:3]
        out[:, 5:7] = self._last_cmd7[5:7]
        # drop_ball_pos=True -> DELETE ball cols 3:5 (5 cols/frame remain):
        out = np.delete(out, [3, 4], axis=1)            # (H, 5)
        return out.reshape(-1)                          # (history_samples*5,)

    # ------------- ball_pos_b_noisy obs (fed to the encoder) ------------- #
    # env: soccer_obs.ball_pos_b_noisy_hist (history_samples=10, buf_prefix="").
    # base.py resolves this by func name "ball_pos_b_noisy_hist".
    # Clean at deploy (encoder denoises toward the true ball; clean is the easy
    # end of the training noise distribution).
    def _get_ball_pos_b_noisy_hist_obs(self) -> np.ndarray:
        self._ensure_ball_history_fresh()
        # ball columns of the same downsampled history, oldest-first / newest-last
        # ([f0_x, f0_y, ... f9_x, f9_y]) -- same buffer layout as soccer_command.
        return self._soccer_hist[:, 3:5].reshape(-1).copy()  # (history_samples*2,)

    # ------------- alias for env.yaml "imu_*" obs func names ------------- #
    def _get_imu_ang_vel_obs(self) -> np.ndarray:
        return self.ros_node._get_base_ang_vel_obs()

    def _get_imu_projected_gravity_obs(self) -> np.ndarray:
        return self.ros_node._get_projected_gravity_obs()


# --------------------------------------------------------------------------- #
# Agent factory (strips the `concatenate_dim` key, same workaround as the
# walk / kick / soccer deployment scripts)
# --------------------------------------------------------------------------- #
def _create_soccer_encdec_agent(
    logdir: str,
    actor_path: Optional[str],
    ros_node: RealNode,
    **kwargs,
) -> SoccerEncDecAgent:
    from instinct_onboard.agents import base as _base

    _orig_parse = _base.OnboardAgent._parse_obs_config

    def _patched_parse(self):
        self.cfg["observations"]["policy"].pop("concatenate_dim", None)
        return _orig_parse(self)

    _base.OnboardAgent._parse_obs_config = _patched_parse
    try:
        agent = SoccerEncDecAgent(
            logdir=logdir,
            ros_node=ros_node,
            actor_path=actor_path,
            **kwargs,
        )
    finally:
        _base.OnboardAgent._parse_obs_config = _orig_parse
    return agent


# --------------------------------------------------------------------------- #
# ROS node
# --------------------------------------------------------------------------- #
class PiPlusSoccerEncDecNode(HightorqueNode):
    """State machine:
        cold_start --[R1, when done]--> soccer
        soccer     --[R1]--> re-reset soccer (clears obs history)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents: dict = dict()
        self.current_agent_name: Optional[str] = None
        # camera ball cache
        self._ball_position_camera: Optional[np.ndarray] = None
        self._ball_position_camera_stamp: Optional[float] = None
        self._ball_position_camera_topic: str = DEFAULT_BALL_POSITION_CAMERA_TOPIC

    def _get_imu_ang_vel_obs(self):
        return self._get_base_ang_vel_obs()

    def _get_imu_orientation_obs(self):
        return self._get_quat_w_obs()

    def _get_imu_projected_gravity_obs(self):
        return self._get_projected_gravity_obs()

    def register_agent(self, name: str, agent) -> None:
        self.available_agents[name] = agent

    # ------------- camera ball topic ------------- #
    def set_ball_position_camera_topic(self, topic: str) -> None:
        self._ball_position_camera_topic = str(topic)

    def _ball_position_camera_callback(self, msg: PointStamped) -> None:
        # msg.point.{x,y,z} = ball center in camera FLU (+X fwd, +Y left, +Z up).
        self._ball_position_camera = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=np.float32
        )
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self._ball_position_camera_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        else:
            self._ball_position_camera_stamp = self.get_clock().now().nanoseconds * 1e-9

    def get_latest_ball_position_camera(self, max_age_s: float = 0.2) -> Optional[np.ndarray]:
        if self._ball_position_camera is None or self._ball_position_camera_stamp is None:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._ball_position_camera_stamp > max_age_s:
            return None
        return self._ball_position_camera.copy()

    def start_ros_handlers(self) -> None:
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ball_position_camera_subscriber = self.create_subscription(
            PointStamped,
            self._ball_position_camera_topic,
            self._ball_position_camera_callback,
            10,
        )
        self.get_logger().info(
            f"Ball position subscriber: {self._ball_position_camera_topic} "
            "(geometry_msgs/PointStamped, camera FLU: +X forward, +Y left, +Z up; "
            "node transforms FLU -> base_link (head FK) -> yaw frame (IMU de-tilt))"
        )
        self.get_logger().info(f"Starting main loop with duration: {MAIN_LOOP_DT} seconds.")
        self.main_loop_timer = self.create_timer(MAIN_LOOP_DT, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(
                maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL
            )

    def main_loop_callback(self) -> None:
        t0 = time.time()

        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        agent = self.available_agents[self.current_agent_name]

        if self.current_agent_name == "cold_start":
            action, done = agent.step()
            if done:
                self.get_logger().info(
                    "ColdStartAgent done, press 'R1' to switch to soccer agent.",
                    throttle_duration_sec=10.0,
                )
            self._send(agent, action)
            if done and self.joy_stick_data.R1:
                self.get_logger().info("R1 pressed, switching to soccer agent.")
                self._switch_to("soccer")

        elif self.current_agent_name == "soccer":
            action, done = agent.step()
            self._send(agent, action)
            if self.joy_stick_data.R1:
                self.get_logger().info("R1 pressed, re-resetting soccer agent.")
                agent.reset()

        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_callback_time_consumptions.put(time.time() - t0)
            self.main_loop_timer_counter += 1
            if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL == 0:
                times = [
                    self.main_loop_callback_time_consumptions.get()
                    for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
                ]
                self.get_logger().info(
                    f"Actual main loop frequency: "
                    f"{(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time)):.2f} Hz. "
                    f"Mean time consumption: {np.mean(times):.4f} s."
                )
                self.main_loop_timer_counter = 0
                self.main_loop_timer_counter_time = time.time()

    def _switch_to(self, name: str) -> None:
        self.current_agent_name = name
        self.available_agents[name].reset()

    def _send(self, agent, action: np.ndarray) -> None:
        self.send_action(
            action,
            agent.action_offset,
            agent.action_scale,
            agent.p_gains,
            agent.d_gains,
        )


# --------------------------------------------------------------------------- #
# Entrypoint
# --------------------------------------------------------------------------- #
def main(args) -> None:
    rclpy.init()

    logdir, soccer_actor_path = resolve_logdir_and_actor(args.logdir)

    # PiPlus_S_12L8A0G2H0W: 22-DOF, no waist (12 legs + 2 head + 8 arms)
    node = PiPlusSoccerEncDecNode(
        robot_class_name="PiPlus_S_12L8A0G2H0W",
        joint_pos_protect_ratio=2.0,
        dryrun=not args.nodryrun,
        computer_clip_torque=False,
    )
    node.set_ball_position_camera_topic(args.ball_position_camera_topic)

    soccer_agent = _create_soccer_encdec_agent(
        logdir=logdir,
        actor_path=soccer_actor_path,
        ros_node=node,
        kick_speed=args.kick_speed,
        x_vel_scale=args.x_vel_scale,
        y_vel_scale=args.y_vel_scale,
        yaw_vel_scale=args.yaw_vel_scale,
        soccer_pub_period=args.soccer_pub_period,
        soccer_history_samples=args.soccer_history_samples,
        ball_position_camera_topic=args.ball_position_camera_topic,
        ball_topic_timeout=args.ball_topic_timeout,
        head_lock=args.head_lock,
        head_pitch_rad=math.radians(args.head_pitch_deg),
        head_yaw_rad=math.radians(args.head_yaw_deg),
    )
    node.register_agent("soccer", soccer_agent)

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=node,
        joint_target_pos=soccer_agent.default_joint_pos,
        action_scale=soccer_agent.action_scale,
        action_offset=soccer_agent.action_offset,
        p_gains=soccer_agent.p_gains * args.kpkd_factor,
        d_gains=soccer_agent.d_gains * args.kpkd_factor,
    )
    node.register_agent("cold_start", cold_start_agent)

    print(f"logdir (encdec): {logdir}  |  actor: {soccer_actor_path or '(default exported/actor.onnx)'}")
    print(f"kick_speed      : {args.kick_speed:.2f} m/s  (only when A held + stick pushed)")
    print(f"ball topic      : {args.ball_position_camera_topic}  "
          f"(PointStamped xyz in camera FLU frame; timeout={args.ball_topic_timeout:.2f}s)")
    print(f"ball transform  : FLU -> base_link (head FK) -> yaw frame (IMU de-tilt)")
    print(f"ball (kick)     : vision when seen, else 0  (NO virtual fallback)")
    print(f"ball walk/idle  : (0.00, 0.00) m  (always masked to 0 when A not held)")
    print(f"vel scales      : x={args.x_vel_scale:+.2f}  y={args.y_vel_scale:+.2f}  yaw={args.yaw_vel_scale:+.2f}")
    print(f"ball history    : {args.soccer_history_samples} samples @ pub_period={args.soccer_pub_period}")
    print(f"  -> soccer_command : {args.soccer_history_samples * 5}-dim (ball cols [3:5] DELETED, drop_ball_pos -> 5/frame)")
    print(f"  -> ball_pos_b_noisy: {args.soccer_history_samples * 2}-dim (fed to encoder; vision while kicking, else 0)")
    print(f"  -> expected onnx input total: 24+24+{args.soccer_history_samples*5}+176+176+176+{args.soccer_history_samples*2} "
          f"= {24+24+args.soccer_history_samples*5+176+176+176+args.soccer_history_samples*2}")

    node.start_ros_handlers()
    node.get_logger().info("PiPlusSoccerEncDecNode is ready.")

    if args.skip_cold_start:
        node.current_agent_name = "soccer"
        node.available_agents["soccer"].reset()
        node.get_logger().info("Skipping cold start, starting with soccer agent directly.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="PiPlus Soccer Node (22-DOF, no waist, encoder/decoder unified walk + kick, vision ball)"
    )
    parser.add_argument(
        "--logdir",
        type=str,
        required=True,
        help=(
            "encdec soccer policy path. Either:\n"
            "  directory: /path/to/model (auto-loads exported/actor.onnx)\n"
            "  .onnx file: /path/to/model/run_xxx/exported/actor.onnx\n"
            "NOTE: actor.onnx must be the encdec export (encoder baked in)."
        ),
    )
    parser.add_argument(
        "--kick_speed", type=float, default=2.0,
        help="Desired kick speed magnitude in m/s (held constant). Default: 2.0",
    )
    parser.add_argument(
        "--ball_position_camera_topic",
        type=str,
        default=DEFAULT_BALL_POSITION_CAMERA_TOPIC,
        help=(
            "Ball position topic. Expects geometry_msgs/PointStamped xyz in the "
            "camera FLU frame (+X forward, +Y left, +Z up). The node transforms "
            "it FLU -> base_link (head FK) -> yaw frame (IMU de-tilt). Used while "
            "KICKING; stale/absent -> masked to 0."
        ),
    )
    parser.add_argument(
        "--ball_topic_timeout",
        type=float,
        default=0.2,
        help="Max age in seconds for using the subscribed ball position before "
             "masking the ball to 0. Default: 0.2",
    )
    parser.add_argument(
        "--soccer_pub_period", type=int, default=5,
        help="Push a fresh BALL-position snapshot into the history every N control "
             "steps on the 50Hz loop. 5 -> 10Hz (matches BALL_PUB_PERIOD in training). "
             "Default: 5",
    )
    parser.add_argument(
        "--soccer_history_samples", type=int, default=10,
        help="Number of stacked ball-history snapshots. MUST match history_samples "
             "in the env cfg for BOTH soccer_command AND ball_pos_b_noisy. Default: 10",
    )
    parser.add_argument(
        "--startup_step_size", type=float, default=0.2,
        help="Cold start step size (default: 0.2)",
    )
    parser.add_argument("--head_lock", action="store_true", default=False,
                        help="Override the head joints every step to a fixed look "
                             "direction (the policy's head action is ignored). "
                             "Legs/arms are unaffected.")
    parser.add_argument("--head_pitch_deg", type=float, default=45.0,
                        help="Head pitch target in degrees when --head_lock is set. "
                             "Sign follows the joint's own convention; if the head "
                             "tilts the WRONG way, negate this. Default: 45 (look down).")
    parser.add_argument("--head_yaw_deg", type=float, default=0.0,
                        help="Head yaw target in degrees when --head_lock is set. "
                             "Default: 0 (straight ahead).")
    parser.add_argument("--kpkd_factor", type=float, default=1.0,
                        help="KP/KD gain multiplier for cold start")
    parser.add_argument("--x_vel_scale", type=float, default=1.0,
                        help="Forward/backward velocity scale (default: 1.0)")
    parser.add_argument("--y_vel_scale", type=float, default=-0.5,
                        help="Lateral velocity scale (default: -0.5)")
    parser.add_argument("--yaw_vel_scale", type=float, default=-1.2,
                        help="Yaw velocity scale (default: -1.2)")
    parser.add_argument("--nodryrun", action="store_true", default=False,
                        help="Actually send commands to robot (default: dryrun)")
    parser.add_argument("--skip_cold_start", action="store_true", default=False,
                        help="Skip cold start agent")
    parser.add_argument("--debug", action="store_true", default=False,
                        help="Enable debugpy on port 6789")

    args = parser.parse_args()

    if args.debug:
        import debugpy

        ip_address = ("0.0.0.0", 6789)
        print("Waiting for debugger at %s:%d" % ip_address, flush=True)
        debugpy.listen(ip_address)
        debugpy.wait_for_client()
        debugpy.breakpoint()

    main(args)
