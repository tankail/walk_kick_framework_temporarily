#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path
import queue
import time
import traceback
from typing import Optional
from copy import copy

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.ros_nodes.hightorque import HightorqueNode
from piplus_soccer_decision import PiPlusSoccerDecision
from piplus_h0w_fall_recovery import FallAndGetUpAgent, FallRecoveryAgent
import piplus_h0w_locked_waist_walk as locked_walk
from soccer_state_machine import SoccerMode, SoccerStateContext, SoccerStateMachine
import piplus_soccer_perceptive as soccer
import piplus_soccer_ballvel_perceptive as ballvel_perceptive
from play_walk_nowaist import WalkAgent

MAIN_LOOP_DT = 0.02
MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500
FALL_RECOVERY_RESOURCE_DIR = Path(__file__).resolve().parents[1] / "instinct_onboard" / "resources" / "fall_recovery"
DEFAULT_FALL_RECOVERY_ROOT = FALL_RECOVERY_RESOURCE_DIR / "fall_recovery_06281302"
DEFAULT_FACEUP_GET_UP_LOGDIR = (
    DEFAULT_FALL_RECOVERY_ROOT
    / "faceup"
    / "20260625_131016_H0W_FaceUpDirect_preview10_robustV3_reset06_rp08_yaw015_jp06_vel02_gain085115_mass0911__GPU5_from20260623_101049"
)
DEFAULT_FACEUP_GET_UP_MOTION_DIR = DEFAULT_FALL_RECOVERY_ROOT / "faceup" / "motion"
DEFAULT_FACEDOWN_GET_UP_LOGDIR = (
    DEFAULT_FALL_RECOVERY_ROOT
    / "facedown"
    / "20260625_095417_H0W_FallRecovery_facedown_quick_robustV1_from20260624_173850_87000"
)
DEFAULT_FACEDOWN_GET_UP_MOTION_DIR = DEFAULT_FALL_RECOVERY_ROOT / "facedown" / "motion"
DEFAULT_KICK_LOGDIR = Path(__file__).resolve().parents[1] / "instinct_onboard" / "resources" / "kick_policy" / "20260730_125511_from20260728_105738-8-1"

# TEMP TEST: set to False, or remove the block in _run_walk_policy(), to restore real walk actions.
ZERO_WALK_ACTION_FOR_TEST = False


class SoccerKickAgent(ballvel_perceptive.WalkKickBallVelHeadFreeEncDecAgent):
    """Head-free + ball-velocity soccer policy used only for kick frames.

    决策层自主驱动 (LT+RT+A 进入足球状态): 覆盖 _build_soccer_cmd, 把新 agent 默认的
    手柄那套 (A=踢球) 换成用 SoccerDecision。球位置历史全程喂 (含走路/找球),
    供 encoder 球速度有限差分; kick 时加 kick_dir/kick_speed。
    """

    def _build_soccer_cmd(self) -> np.ndarray:
        decision = self.ros_node.get_soccer_decision()
        if decision is not None and decision.sees_ball:
            bx = float(decision.ball_x_b)
            by = float(decision.ball_y_b)
            self._ball_active = math.hypot(bx, by) <= self.BALL_FAR_MASK_DIST
        else:
            bx, by = 0.0, 0.0
            self._ball_active = False
        if decision is not None and decision.kick:
            if not self._kick_was_active:
                self._anchor_kick()
            self._kick_was_active = True
            kick_cmd = np.array(
                [
                    0.0,
                    0.0,
                    0.0,
                    bx,
                    by,
                    decision.kick_dir_yaw_b,
                    decision.kick_speed,
                ],
                dtype=np.float32,
            )
        else:
            self._kick_was_active = False
            kick_cmd = np.array([0.0, 0.0, 0.0, bx, by, 0.0, 0.0], dtype=np.float32)
        # 门控: 踢球指令生效 (kick_speed != 0) 时打开, 策略才能看到球的位置/速度。
        # 基类 _build_soccer_cmd 原本在 kick_speed!=0 分支更新 _ball_gate_is_open;
        # 本 override 完全替换了基类方法, 必须在这里复刻同样的门控逻辑,
        # 否则 _ball_gate_is_open 恒为 False -> 球位置/球速 obs 全 0 -> 踢球盲踢。
        kick_speed = float(kick_cmd[6])
        if kick_speed != 0.0:
            self._ball_gate_release_t = None
            gate = True
        else:
            if self._ball_gate_is_open and self._ball_gate_release_t is None:
                self._ball_gate_release_t = time.time()
            gate = (
                self._ball_gate_release_t is not None
                and (time.time() - self._ball_gate_release_t) < self.BALL_GATE_RELEASE_HOLD_S
            )
            if not gate:
                self._ball_gate_release_t = None
        self._ball_gate_is_open = bool(gate)
        return kick_cmd


class SoccerKickShadowNode(soccer.PiPlusSoccerEncDecNode):
    """A non-commanding 22-DOF view used to build soccer-policy observations."""

    def __init__(self, source_node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.source_node = source_node
        self.name_to_shadow_idx = {name: i for i, name in enumerate(self.sim_joint_names)}
        # Head-free ballvel 策略的 last_action obs 是 20 维 (无 head), 每帧由
        # _run_kick_policy 从 agent._raw_policy_action 更新。宽度按 agent 动作列数。
        self._last_action_for_obs = np.zeros(20, dtype=np.float32)

    def sync_from_source(self) -> None:
        src = self.source_node
        src_name_to_idx = {name: i for i, name in enumerate(src.sim_joint_names)}
        for name, shadow_idx in self.name_to_shadow_idx.items():
            src_idx = src_name_to_idx.get(name)
            if src_idx is None:
                continue
            self.joint_pos_[shadow_idx] = src.joint_pos_[src_idx]
            self.joint_vel_[shadow_idx] = src.joint_vel_[src_idx]
        self._joy_stick_data = copy(src.joy_stick_data)
        if hasattr(src, "torso_imu_buffer"):
            self.torso_imu_buffer = src.torso_imu_buffer

    def get_soccer_decision(self):
        return self.source_node.get_soccer_decision()

    def get_latest_ball_base(self, *args, **kwargs):
        """转发到 source node (head-free ballvel agent 的 _latest_ball_base 会用到)。
        源方法无参数 (球新鲜度由源节点管理), 忽略 agent 传的 max_age_s。"""
        getter = getattr(self.source_node, "get_latest_ball_base", None)
        if getter is None:
            return None
        return getter()

    def _get_last_action_obs(self):
        """Head-free: 返回 20 维 (无 head), 与训练 mdp.last_action 宽度一致。"""
        return self._last_action_for_obs

    def sync_raw_policy_action(self, agent) -> None:
        """把 kick agent 这一帧的 20 维原始动作拷给 last_action obs。"""
        raw = getattr(agent, "_raw_policy_action", None)
        if raw is not None:
            arr = np.asarray(raw, dtype=np.float32).reshape(-1)
            if self._last_action_for_obs.shape[0] != arr.shape[0]:
                self._last_action_for_obs = np.zeros(arr.shape[0], dtype=np.float32)
            self._last_action_for_obs[:] = arr


class SoccerH0WShadowNode(SoccerKickShadowNode):
    """Non-commanding H0W node used by 22-DOF fall-recovery policies."""

    def _get_last_action_obs(self):
        """Fall-recovery H0W policy is 22-DOF (含 head): last_action 用 22 维的
        self.action 缓冲 (由 _send_h0w_shadow_action 每帧更新), 而不是
        SoccerKickShadowNode 硬编码的 20 维 head-free 缓冲。否则 obs 600 维 vs
        策略 normalizer/actor 602 维, normalize 直接 ValueError 崩溃。"""
        return self.action


class SoccerWalkShadowMixin:
    """Shared non-commanding walk shadow that feeds Route C cmd_vel to WalkAgent."""

    def __init__(self, source_node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.source_node = source_node
        self.name_to_shadow_idx = {name: i for i, name in enumerate(self.sim_joint_names)}
        self.x_vel_scale = 0.5
        self.y_vel_scale = 0.5
        self.yaw_vel_scale = 1.0

    def configure_cmd_vel_mapping(self, x_vel_scale: float, y_vel_scale: float, yaw_vel_scale: float) -> None:
        self.x_vel_scale = float(x_vel_scale)
        self.y_vel_scale = float(y_vel_scale)
        self.yaw_vel_scale = float(yaw_vel_scale)

    def sync_from_source(self) -> None:
        src = self.source_node
        src_name_to_idx = {name: i for i, name in enumerate(src.sim_joint_names)}
        for name, shadow_idx in self.name_to_shadow_idx.items():
            src_idx = src_name_to_idx.get(name)
            if src_idx is None:
                continue
            self.joint_pos_[shadow_idx] = src.joint_pos_[src_idx]
            self.joint_vel_[shadow_idx] = src.joint_vel_[src_idx]
        self._joy_stick_data = src.joy_stick_data
        self.apply_cmd_vel_as_joystick()
        if hasattr(src, "torso_imu_buffer"):
            self.torso_imu_buffer = src.torso_imu_buffer

    def apply_cmd_vel_as_joystick(self) -> None:
        cmd_vel = self.source_node.get_external_cmd_vel()
        if cmd_vel is None:
            return
        vx, vy, wz = [float(v) for v in np.asarray(cmd_vel, dtype=np.float32).reshape(3)]
        self._joy_stick_data.ly = self._safe_axis(vx, self.x_vel_scale)
        self._joy_stick_data.lx = self._safe_axis(-vy, self.y_vel_scale)
        self._joy_stick_data.rx = self._safe_axis(-wz, self.yaw_vel_scale)
        if self._joy_stick_data.ry is None:
            self._joy_stick_data.ry = 0.0

    @staticmethod
    def _safe_axis(value: float, scale: float) -> float:
        if abs(scale) < 1e-6:
            return 0.0
        return float(np.clip(value / scale, -1.0, 1.0))


class SoccerWalkShadowNode(SoccerWalkShadowMixin, HightorqueNode):
    """Non-commanding 22-DOF H0W walk shadow for the current no-waist policy."""


class SoccerLockedWaistWalkShadowNode(SoccerWalkShadowMixin, locked_walk.H0WLockedWaistHightorqueNode):
    """Non-commanding 23-DOF H1W walk shadow; waist commands are ignored by the H0W sender."""


def create_walk_agent(logdir: str, ros_node, **kwargs):
    x_vel_scale = kwargs.pop("x_vel_scale", 0.5)
    y_vel_scale = kwargs.pop("y_vel_scale", 0.5)
    yaw_vel_scale = kwargs.pop("yaw_vel_scale", 1.0)
    ros_node.configure_cmd_vel_mapping(
        x_vel_scale=x_vel_scale,
        y_vel_scale=y_vel_scale,
        yaw_vel_scale=yaw_vel_scale,
    )
    for unused_key in (
        "max_cmd_vel",
        "vy_limit",
        "wz_limit",
        "vy_sign",
        "wz_sign",
        "cmd_lowpass",
        "force_zero_cmd_vel",
        "no_joystick_fallback",
        "log_period_s",
    ):
        kwargs.pop(unused_key, None)
    return WalkAgent(
        logdir=logdir,
        ros_node=ros_node,
        x_vel_scale=x_vel_scale,
        y_vel_scale=y_vel_scale,
        yaw_vel_scale=yaw_vel_scale,
        **kwargs,
    )


def create_soccer_kick_agent(logdir: str, actor_path: Optional[str], ros_node, **kwargs) -> SoccerKickAgent:
    from instinct_onboard.agents import base as _base

    original_parse = _base.OnboardAgent._parse_obs_config

    def patched_parse(self):
        self.cfg["observations"]["policy"].pop("concatenate_dim", None)
        return original_parse(self)

    _base.OnboardAgent._parse_obs_config = patched_parse
    try:
        # head-free + ballvel agent 部署默认值 (调用方可覆盖)
        kwargs.setdefault("head_mode", "zero_gain")
        kwargs.setdefault("ball_frame", "base_link")
        kwargs.setdefault("ball_vel_source", "measured")
        kwargs.setdefault("posonly", False)
        return SoccerKickAgent(logdir=logdir, ros_node=ros_node, actor_path=actor_path, **kwargs)
    finally:
        _base.OnboardAgent._parse_obs_config = original_parse


def create_fall_recovery_agent(logdir: str, motion_dir: str, ros_node, **kwargs) -> FallRecoveryAgent:
    from instinct_onboard.agents import base as _base

    original_parse = _base.OnboardAgent._parse_obs_config

    def patched_parse(self):
        self.cfg["observations"]["policy"].pop("concatenate_dim", None)
        return original_parse(self)

    _base.OnboardAgent._parse_obs_config = patched_parse
    try:
        return FallRecoveryAgent(logdir=logdir, motion_file_dir=motion_dir, ros_node=ros_node, **kwargs)
    finally:
        _base.OnboardAgent._parse_obs_config = original_parse


class SoccerPolicyControlNode(HightorqueNode):
    def __init__(
        self,
        *args,
        cmd_vel_topic: Optional[str] = None,
        cmd_vel_timeout: float = 0.5,
        max_cmd_vel=None,
        head_joint_topic: Optional[str] = None,
        head_joint_timeout: float = 0.5,
        head_yaw_limit: float = 1.5,
        head_pitch_limit: float = 1.0,
        head_kp: float = 30.0,
        head_kd: float = 1.0,
        ball_base_topic: Optional[str] = None,
        ball_base_timeout: float = 0.5,
        ball_base_vel_topic: Optional[str] = None,
        ball_base_vel_timeout: float = 0.5,
        goal_yaw_topic: Optional[str] = None,
        goal_yaw_timeout: float = 0.5,
        behavior_intent_topic: Optional[str] = None,
        behavior_intent_timeout: float = 0.5,
        fall_topic: Optional[str] = None,
        fall_topic_timeout: float = 0.5,
        hold_cmd_vel=None,
        decision: Optional[PiPlusSoccerDecision] = None,
        scan_phase_sec: float = 4.0,
        turn_phase_sec: float = 2.0,
        scan_turn_wz: float = 0.5,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.available_agents = {}
        self.current_agent_name = None

        self.cmd_vel_topic = cmd_vel_topic
        self.cmd_vel_timeout = float(cmd_vel_timeout)
        self.max_cmd_vel = np.asarray(max_cmd_vel or [0.5, 0.5, 1.0], dtype=np.float32)
        self.cmd_vel_command = np.zeros(3, dtype=np.float32)
        self.cmd_vel_time = None

        self.head_joint_topic = head_joint_topic
        self.head_joint_timeout = float(head_joint_timeout)
        self.head_joint_time = None
        self.head_joint_targets = {}
        self.head_sim_indices = {
            "head_yaw_joint": self.sim_joint_names.index("head_yaw_joint"),
            "head_pitch_joint": self.sim_joint_names.index("head_pitch_joint"),
        }
        self.head_joint_limits = {
            "head_yaw_joint": abs(float(head_yaw_limit)),
            "head_pitch_joint": abs(float(head_pitch_limit)),
        }
        self.head_kp = float(head_kp)
        self.head_kd = float(head_kd)

        self.ball_base_topic = ball_base_topic
        self.ball_base_timeout = float(ball_base_timeout)
        self.ball_base_position: Optional[np.ndarray] = None
        self.ball_base_velocity: Optional[np.ndarray] = None
        self.ball_base_time = None
        self.ball_base_vel_topic = ball_base_vel_topic
        self.ball_base_vel_timeout = float(ball_base_vel_timeout)
        self.goal_yaw_topic = goal_yaw_topic
        self.goal_yaw_timeout = float(goal_yaw_timeout)
        self.goal_yaw_b = None
        self.goal_yaw_time = None
        self.behavior_intent_topic = behavior_intent_topic
        self.behavior_intent_timeout = float(behavior_intent_timeout)
        self.behavior_intent = None
        self.behavior_intent_time = None
        self.prev_behavior_action_type = None
        self.fall_topic = fall_topic
        self.fall_topic_timeout = float(fall_topic_timeout)
        self.fall_direction = "NONE"
        self.fall_direction_time = None
        self.recovery_type = "none"
        self.mode_cmd_vel_override = None
        self.hold_cmd_vel = np.asarray(hold_cmd_vel or [0.0, 0.0, 0.0], dtype=np.float32)
        self.get_up_done = False
        self.decision = decision or PiPlusSoccerDecision(ball_timeout=ball_base_timeout)
        self.latest_decision = None
        self.prev_decision_state = None
        self.prev_soccer_mode = None
        self.prev_enter_combo = False  # LT+RT+A 上升沿
        self.prev_exit_combo = False   # LT+RT+B 上升沿
        # 找不到球时的交替搜索节奏: 扫头(站立) -> 转身 -> 扫头 -> ...
        self.scan_phase_sec = float(scan_phase_sec)
        self.turn_phase_sec = float(turn_phase_sec)
        self.scan_turn_wz = float(scan_turn_wz)
        self._scan_turn_phase = None
        self._scan_turn_phase_start = None
        self.last_soccer_status_log_time = 0.0
        self.soccer_state_machine = SoccerStateMachine()

        self.soccer_shadow_node: Optional[SoccerKickShadowNode] = None
        self.fall_recovery_shadow_node: Optional[SoccerH0WShadowNode] = None
        self.fall_and_getup_agent: Optional[FallAndGetUpAgent] = None
        self.walk_shadow_node: Optional[SoccerWalkShadowMixin] = None
        self.name_to_idx = {name: i for i, name in enumerate(self.sim_joint_names)}

        # 50Hz 决策调试话题: 供 rosbag 录制/复盘"踢球不稳定"
        self.decision_debug_publisher = None
        self._last_decision_ang_vel_z = 0.0

    def register_agent(self, name: str, agent) -> None:
        self.available_agents[name] = agent

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_command[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.cmd_vel_time = time.time()

    def _head_joint_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.head_sim_indices:
                limit = self.head_joint_limits[name]
                self.head_joint_targets[name] = float(np.clip(position, -limit, limit))
        if self.head_joint_targets:
            self.head_joint_time = time.time()

    def _ball_base_callback(self, msg: PointStamped) -> None:
        self.ball_base_position = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self.ball_base_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        else:
            self.ball_base_time = self.get_clock().now().nanoseconds * 1e-9

    def _ball_base_vel_callback(self, msg) -> None:
        """Odometry: 位置 + 线速度 (来自 /soccer/ball_base_xy_velocity, 桥转发 dvision 球速)。"""
        self.ball_base_position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=np.float32,
        )
        self.ball_base_velocity = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, 0.0],
            dtype=np.float32,
        )
        self.ball_base_time = self.get_clock().now().nanoseconds * 1e-9

    def _goal_yaw_callback(self, msg: Float32) -> None:
        self.goal_yaw_b = float(msg.data)
        self.goal_yaw_time = time.time()

    def _fall_callback(self, msg: String) -> None:
        direction = str(msg.data or "NONE").upper()
        self.fall_direction = direction
        self.fall_direction_time = time.time()

    def _behavior_intent_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn("Ignoring malformed ROS1 tactical intent JSON.", throttle_duration_sec=2.0)
            return
        action_type = str(payload.get("action_type", "") or "").upper()
        if action_type and action_type not in ("DRIBBLE", "SHOOT", "PASS", "CLEAR", "HOLD", "POSITION"):
            self.get_logger().warn(f"Ignoring unknown ROS1 tactical action_type '{action_type}'.")
            return
        payload["action_type"] = action_type
        self.behavior_intent = payload
        self.behavior_intent_time = time.time()
        if action_type != self.prev_behavior_action_type:
            self.get_logger().info(
                f"ROS1 tactical intent: action={action_type or '(none)'}, "
                f"tactic={payload.get('accepted_tactic', '')}, "
                f"role={payload.get('dynamic_role', '')}"
            )
            self.prev_behavior_action_type = action_type

    def _suppress_external_soccer_inputs(self) -> bool:
        return (
            self.current_agent_name == "soccer"
            and (
                self.prev_soccer_mode == SoccerMode.FALL_AND_GETUP
                or self.soccer_state_machine.fallen_count > 0
            )
        )

    def get_external_cmd_vel(self):
        if self._suppress_external_soccer_inputs():
            return np.zeros(3, dtype=np.float32)
        if self.mode_cmd_vel_override is not None:
            return np.clip(self.mode_cmd_vel_override, -self.max_cmd_vel, self.max_cmd_vel)
        if not self.cmd_vel_topic or self.cmd_vel_time is None:
            return None
        if time.time() - self.cmd_vel_time > self.cmd_vel_timeout:
            return np.zeros(3, dtype=np.float32)
        return np.clip(self.cmd_vel_command, -self.max_cmd_vel, self.max_cmd_vel)

    def get_latest_ball_base(self):
        if self.ball_base_position is None or self.ball_base_time is None:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self.ball_base_time > self.ball_base_timeout:
            return None
        # [x, y, z, vx, vy, vz] — 前 3 位置, 后 3 速度 (供 ballvel 踢球策略 measured 源)
        if self.ball_base_velocity is not None:
            return np.concatenate([self.ball_base_position, self.ball_base_velocity]).astype(np.float32)
        return np.concatenate([self.ball_base_position, np.zeros(3, dtype=np.float32)]).astype(np.float32)

    def get_latest_goal_yaw_b(self):
        if not self.goal_yaw_topic or self.goal_yaw_b is None or self.goal_yaw_time is None:
            return None
        if time.time() - self.goal_yaw_time > self.goal_yaw_timeout:
            return None
        return float(self.goal_yaw_b)

    def get_latest_behavior_intent(self):
        if not self.behavior_intent_topic or self.behavior_intent is None or self.behavior_intent_time is None:
            return None
        timeout = self.behavior_intent_timeout
        ttl_ms = int(self.behavior_intent.get("command_ttl_ms") or 0)
        if ttl_ms > 0:
            timeout = min(timeout, float(ttl_ms) / 1000.0)
        if time.time() - self.behavior_intent_time > timeout:
            return None
        return self.behavior_intent

    def get_soccer_decision(self):
        return self.latest_decision

    def apply_external_head_targets(self, target_joint_pos, p_gains=None, d_gains=None):
        if not self.head_joint_topic or self.head_joint_time is None:
            return target_joint_pos, p_gains, d_gains
        if time.time() - self.head_joint_time > self.head_joint_timeout:
            return target_joint_pos, p_gains, d_gains
        for name, target in self.head_joint_targets.items():
            idx = self.head_sim_indices[name]
            target_joint_pos[idx] = target
            if p_gains is not None:
                p_gains[idx] = max(float(p_gains[idx]), self.head_kp)
            if d_gains is not None:
                d_gains[idx] = max(float(d_gains[idx]), self.head_kd)
        return target_joint_pos, p_gains, d_gains

    def start_ros_handlers(self) -> None:
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        # 50Hz 决策调试 (JSON), 复盘"踢球不稳定"用; 由 _update_soccer_decision 每帧填充
        self.decision_debug_publisher = self.create_publisher(String, "soccer/decision_debug", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.cmd_vel_topic:
            self.cmd_vel_subscriber = self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
            self.get_logger().info(
                f"Using soccer velocity topic '{self.cmd_vel_topic}' timeout={self.cmd_vel_timeout:.2f}s."
            )
        if self.head_joint_topic:
            self.head_joint_subscriber = self.create_subscription(
                JointState, self.head_joint_topic, self._head_joint_callback, 10
            )
            self.get_logger().info(
                f"Using soccer head topic '{self.head_joint_topic}' timeout={self.head_joint_timeout:.2f}s."
            )
        if self.ball_base_topic:
            self.ball_base_subscriber = self.create_subscription(
                PointStamped, self.ball_base_topic, self._ball_base_callback, 10
            )
            self.get_logger().info(
                f"Using soccer ball topic '{self.ball_base_topic}' timeout={self.ball_base_timeout:.2f}s."
            )
        if self.ball_base_vel_topic:
            self.ball_base_vel_subscriber = self.create_subscription(
                Odometry, self.ball_base_vel_topic, self._ball_base_vel_callback, 10
            )
            self.get_logger().info(
                f"Using soccer ball velocity topic '{self.ball_base_vel_topic}' timeout={self.ball_base_vel_timeout:.2f}s."
            )
        if self.goal_yaw_topic:
            self.goal_yaw_subscriber = self.create_subscription(
                Float32, self.goal_yaw_topic, self._goal_yaw_callback, 10
            )
            self.get_logger().info(
                f"Using soccer target yaw topic '{self.goal_yaw_topic}' timeout={self.goal_yaw_timeout:.2f}s."
            )
        if self.behavior_intent_topic:
            self.behavior_intent_subscriber = self.create_subscription(
                String, self.behavior_intent_topic, self._behavior_intent_callback, 10
            )
            self.get_logger().info(
                f"Using soccer tactical intent topic '{self.behavior_intent_topic}' "
                f"timeout={self.behavior_intent_timeout:.2f}s."
            )
        if self.fall_topic:
            self.fall_subscriber = self.create_subscription(String, self.fall_topic, self._fall_callback, 10)
            self.get_logger().info(f"Using soccer fall topic '{self.fall_topic}'.")
        self.main_loop_timer = self.create_timer(MAIN_LOOP_DT, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
        self.get_logger().info("soccer dual-policy control node is ready.")

    def _update_soccer_decision(self) -> None:
        ball_base = self.get_latest_ball_base()
        goal_yaw_b = self.get_latest_goal_yaw_b()
        head_pitch = float(self.joint_pos_[self.head_sim_indices["head_pitch_joint"]])
        head_yaw = float(self.joint_pos_[self.head_sim_indices["head_yaw_joint"]])
        ang_vel_z = 0.0
        if hasattr(self, "torso_imu_buffer"):
            ang_vel_z = float(self.torso_imu_buffer.angular_velocity.z)
        self._last_decision_ang_vel_z = ang_vel_z
        self.latest_decision = self.decision.update(
            ball_xyz_camera=ball_base,
            ball_xyz_base=ball_base,
            head_pitch=head_pitch,
            head_yaw=head_yaw,
            goal_yaw_b=goal_yaw_b,
            ang_vel_z=ang_vel_z,
        )
        if self.latest_decision.state != self.prev_decision_state:
            self.get_logger().info(
                f"Soccer decision: state={self.latest_decision.state}, "
                f"ball=({self.latest_decision.ball_x_b:+.2f}, {self.latest_decision.ball_y_b:+.2f}), "
                f"goal_yaw_b={goal_yaw_b if goal_yaw_b is not None else 0.0:+.2f}, "
                f"kick={self.latest_decision.kick}, dir={self.latest_decision.kick_dir_yaw_b:+.2f}, "
                f"speed={self.latest_decision.kick_speed:.2f}"
            )
            self.prev_decision_state = self.latest_decision.state

    def _publish_decision_debug(self, soccer_mode: str = "", fsm_reason: str = "") -> None:
        """每帧(50Hz)发布决策内部状态 JSON 到 soccer/decision_debug。

        soccer_mode / fsm_reason 由 main_loop 在状态机更新后回填。
        """
        if self.decision_debug_publisher is None:
            return
        debug = self.decision.debug_dict(
            self.latest_decision,
            ang_vel_z=getattr(self, "_last_decision_ang_vel_z", 0.0),
            soccer_mode=soccer_mode,
            fsm_reason=fsm_reason,
        )
        msg = String()
        msg.data = json.dumps(debug)
        self.decision_debug_publisher.publish(msg)

    def _send_walk_action(self, agent, action: np.ndarray) -> None:
        self.send_action(action, agent.action_offset, agent.action_scale, agent.p_gains, agent.d_gains)

    def _send_kick_action(self, agent: SoccerKickAgent, action: np.ndarray) -> None:
        assert self.soccer_shadow_node is not None
        self._send_h0w_shadow_action(self.soccer_shadow_node, agent, action)

    def _build_soccer_state_context(self) -> SoccerStateContext:
        projected_gravity = None
        angular_velocity = None
        if hasattr(self, "torso_imu_buffer"):
            projected_gravity = self._get_projected_gravity_obs()
            angular_velocity = self._get_imu_ang_vel_obs()
        fall_direction = "NONE"
        if self.fall_direction_time is not None and time.time() - self.fall_direction_time <= self.fall_topic_timeout:
            fall_direction = self.fall_direction
        behavior_intent = self.get_latest_behavior_intent()
        return SoccerStateContext(
            decision=self.latest_decision,
            fall_direction=fall_direction,
            projected_gravity=projected_gravity,
            angular_velocity=angular_velocity,
            behavior_action_type="" if behavior_intent is None else behavior_intent.get("action_type", ""),
            manual_mode=None,
            now=time.time(),
            active_mode=self.prev_soccer_mode,
            get_up_done=bool(self.fall_and_getup_agent.done) if self.fall_and_getup_agent is not None else False,
            recovery_type=(
                self.fall_and_getup_agent.recovery_type if self.fall_and_getup_agent is not None else self.recovery_type
            ),
        )

    def _run_walk_policy(self) -> None:
        agent = self.available_agents["walk"]
        if self.walk_shadow_node is not None:
            self.walk_shadow_node.sync_from_source()
        action, _done = agent.step()
        if ZERO_WALK_ACTION_FOR_TEST:
            if not getattr(self, "_zero_walk_action_logged", False):
                self.get_logger().warn(
                    "ZERO_WALK_ACTION_FOR_TEST is enabled: walk policy action is overridden to all zeros."
                )
                self._zero_walk_action_logged = True
            action = np.zeros_like(action, dtype=np.float32)
        if self.walk_shadow_node is not None:
            self._send_h0w_shadow_action(self.walk_shadow_node, agent, action)
            return
        self._send_walk_action(agent, action)

    def _run_kick_policy(self) -> None:
        assert self.soccer_shadow_node is not None
        self.soccer_shadow_node.sync_from_source()
        agent = self.available_agents["kick"]
        action, _done = agent.step()
        # Head-free: 20 维原始动作 -> last_action obs (下一帧 step 里用)
        sync = getattr(self.soccer_shadow_node, "sync_raw_policy_action", None)
        if sync is not None:
            sync(agent)
        self._send_kick_action(agent, action)

    def _send_h0w_shadow_action(self, shadow_node, agent, action: np.ndarray) -> None:
        shadow_node.action[:] = action

        shadow_target = action * agent.action_scale + agent.action_offset
        shadow_p = np.clip(agent.p_gains * self.kp_factor, 0.0, self.kp_clip)
        shadow_d = np.clip(agent.d_gains * self.kd_factor, 0.0, self.kd_clip)

        target = self.joint_pos_.copy()
        p_gains = np.zeros(self.NUM_JOINTS, dtype=np.float32)
        d_gains = np.zeros(self.NUM_JOINTS, dtype=np.float32)

        for shadow_idx, name in enumerate(shadow_node.sim_joint_names):
            target_idx = self.name_to_idx.get(name)
            if target_idx is None:
                continue
            target[target_idx] = shadow_target[shadow_idx]
            p_gains[target_idx] = shadow_p[shadow_idx]
            d_gains[target_idx] = shadow_d[shadow_idx]

        target, p_gains, d_gains = self.apply_external_head_targets(target, p_gains, d_gains)
        self._publish_motor_cmd(target, p_gains, d_gains)

    def _hold_current_h0w_pose(self, reason: str) -> None:
        reference_agent = self.available_agents.get("walk")
        target = self.joint_pos_.copy()
        if reference_agent is not None and reference_agent.p_gains.shape[0] == self.NUM_JOINTS:
            p_gains = reference_agent.p_gains
            d_gains = reference_agent.d_gains
        else:
            p_gains = np.full(self.NUM_JOINTS, 60.0, dtype=np.float32)
            d_gains = np.full(self.NUM_JOINTS, 2.0, dtype=np.float32)
        p = np.clip(p_gains * self.kp_factor, 0.0, self.kp_clip)
        d = np.clip(d_gains * self.kd_factor, 0.0, self.kd_clip)
        self.get_logger().warn(
            f"Soccer holding current H0W pose directly ({reason}); no walk/kick policy is running.",
            throttle_duration_sec=2.0,
        )
        self._publish_motor_cmd(target, p, d)

    def _run_fall_and_getup_agent(self) -> None:
        if self.fall_and_getup_agent is None:
            self._hold_current_h0w_pose("fall detected but fall-and-getup agent is not registered")
            return
        projected_gravity = self._get_projected_gravity_obs() if hasattr(self, "torso_imu_buffer") else None
        angular_velocity = self._get_imu_ang_vel_obs() if hasattr(self, "torso_imu_buffer") else None
        done = self.fall_and_getup_agent.step(projected_gravity, angular_velocity)
        self.recovery_type = self.fall_and_getup_agent.recovery_type
        self.get_up_done = bool(done)
        if done:
            self.fall_direction = "NONE"
            self.fall_direction_time = None

    def _run_soccer_mode(self, mode: SoccerMode) -> None:
        self.mode_cmd_vel_override = None
        if mode == SoccerMode.KICK:
            self._run_kick_policy()
            return

        if mode == SoccerMode.FALL_AND_GETUP:
            self._run_fall_and_getup_agent()
            return

        # 扫描找球: 没看到球且行为树也没在命令移动时, 自动缓慢转身找球。
        # 修复"扫头后不转身": Route C 行为树扫描极慢(25-45s)且转身角度小,
        # 依赖它的转身机器人几乎不动。这里在 soccer 层补一个低速持续转身。
        self._apply_scan_turn_override()
        self._run_walk_policy()

    def _apply_scan_turn_override(self) -> None:
        """找不到球时的交替搜索: 扫头(站立) -> 转身 -> 扫头 -> 循环。

        用户要求: "扫头找球, 没找到就转身体再去扫头找球" (交替式, 不是原地连续转圈)。
        节奏: scan_phase_sec 秒站立扫头(行为树扫头) -> turn_phase_sec 秒转身 -> 循环。
        扫头阶段不覆盖 cmd_vel, 机器人站立由行为树扫头;
        转身阶段若行为树已在自己移动则尊重它, 否则用 scan_turn_wz 转身。

        找到球(sees_ball=True)后: 用决策算好的 vx/vy/wz 面向并接近球。
        决策基于球相对 base_link 的位置(ball_x_b, ball_y_b)算转向角速度,
        这正是用户要的 "找到球后转到球的方向去"。
        """
        decision = self.latest_decision
        sees_ball = decision is not None and bool(getattr(decision, "sees_ball", False))
        now = time.time()
        if sees_ball:
            # 找到球: 应用决策的 vx/vy/wz (approach_ball/face_ball/final_approach)
            # 决策基于球相对 base_link 位置算转向, 即用户要的"转到球的方向去"。
            self._scan_turn_phase = None
            self._scan_turn_phase_start = None
            if decision is not None:
                self.mode_cmd_vel_override = np.array(
                    [
                        float(getattr(decision, "vx", 0.0)),
                        float(getattr(decision, "vy", 0.0)),
                        float(getattr(decision, "wz", 0.0)),
                    ],
                    dtype=np.float32,
                )
            return
        # 没找到球: 由行为树完成"完整扫头 -> 转身 -> 再扫头"(HeadScanThenBodyTurn)。
        # 这里不覆盖 cmd_vel, 避免在扫头中途强制转身打断扫描。
        return

    # ===== 手柄组合键进入/退出 (LT+RT+A 进入, LT+RT+B 退出) =====

    def _is_enter_combo(self) -> bool:
        combo = bool(self.joy_stick_data.L2 and self.joy_stick_data.R2 and self.joy_stick_data.A)
        self.get_logger().info(
            f"[joydbg] enter: L2={self.joy_stick_data.L2} R2={self.joy_stick_data.R2} "
            f"A={self.joy_stick_data.A} -> {combo}",
            throttle_duration_sec=1.0,
        )
        return combo

    def _is_exit_combo(self) -> bool:
        combo = bool(self.joy_stick_data.L2 and self.joy_stick_data.R2 and self.joy_stick_data.B)
        self.get_logger().info(
            f"[joydbg] exit: L2={self.joy_stick_data.L2} R2={self.joy_stick_data.R2} "
            f"B={self.joy_stick_data.B} -> {combo}",
            throttle_duration_sec=1.0,
        )
        return combo

    def _enter_soccer(self) -> None:
        self.get_logger().info("LT+RT+A pressed: entering soccer walk/kick/fall-recovery.")
        self.current_agent_name = "soccer"
        self.available_agents["walk"].reset()
        self.available_agents["kick"].reset()
        if "dribble" in self.available_agents:
            self.available_agents["dribble"].reset()
        self.soccer_state_machine.reset()
        self.prev_soccer_mode = None

    def _exit_soccer_to_hold(self) -> None:
        self.get_logger().info("LT+RT+B pressed: exiting soccer, holding current pose.")
        self.current_agent_name = "hold"
        self.mode_cmd_vel_override = None
        self.soccer_state_machine.reset()
        self.prev_soccer_mode = None
        # 不释放 control_uuid, 以便 LT+RT+A 可再次进入; 由 hold 分支持续下发位姿保持命令.

    def _soccer_status_source(self, reason: str) -> str:
        if str(reason).startswith("behavior_intent_"):
            return "BHuman"
        return "StateMachine"

    def _log_soccer_status(self, state, ctx: SoccerStateContext) -> None:
        now = time.time()
        if now - self.last_soccer_status_log_time < 1.0:
            return
        self.last_soccer_status_log_time = now

        decision = ctx.decision
        if decision is None:
            decision_state = "none"
            sees_ball = False
            ball_x = 0.0
            ball_y = 0.0
            kick = False
        else:
            decision_state = str(getattr(decision, "state", ""))
            sees_ball = bool(getattr(decision, "sees_ball", False))
            ball_x = float(getattr(decision, "ball_x_b", 0.0))
            ball_y = float(getattr(decision, "ball_y_b", 0.0))
            kick = bool(getattr(decision, "kick", False))

        action_type = str(getattr(ctx, "behavior_action_type", "") or "none")
        gravity_text = "none"
        if ctx.projected_gravity is not None:
            gravity = np.asarray(ctx.projected_gravity, dtype=np.float32)
            if gravity.shape[0] >= 3:
                gravity_text = "({:+.2f},{:+.2f},{:+.2f})".format(
                    float(gravity[0]),
                    float(gravity[1]),
                    float(gravity[2]),
                )
        gyro_norm = 0.0
        if ctx.angular_velocity is not None:
            angular_velocity = np.asarray(ctx.angular_velocity, dtype=np.float32)
            if angular_velocity.shape[0] >= 3:
                gyro_norm = float(np.linalg.norm(angular_velocity[:3]))
        self.get_logger().info(
            "Soccer status: mode={} source={} reason={} action_type={} "
            "decision={} sees_ball={} ball=({:+.2f},{:+.2f}) kick={} "
            "gravity={} gyro_norm={:.2f} fall_count={}/{}".format(
                state.mode.value,
                self._soccer_status_source(state.reason),
                state.reason,
                action_type,
                decision_state,
                sees_ball,
                ball_x,
                ball_y,
                kick,
                gravity_text,
                gyro_norm,
                self.soccer_state_machine.fallen_count,
                self.soccer_state_machine.fall_confirm_frames,
            )
        )

    def main_loop_callback(self) -> None:
        t0 = time.time()

        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        if self.current_agent_name == "cold_start":
            agent = self.available_agents["cold_start"]
            if self.walk_shadow_node is not None:
                self.walk_shadow_node.sync_from_source()
            action, done = agent.step()
            if done:
                self.get_logger().info(
                    "ColdStartAgent done, press LT+RT+A to switch to soccer control.",
                    throttle_duration_sec=10.0,
                )
            if self.walk_shadow_node is not None:
                self._send_h0w_shadow_action(self.walk_shadow_node, agent, action)
            else:
                self._send_walk_action(agent, action)
            enter_combo = self._is_enter_combo()
            enter_edge = enter_combo and not self.prev_enter_combo
            if done and enter_edge:
                self._enter_soccer()
            self.prev_enter_combo = enter_combo

        elif self.current_agent_name == "soccer":
            exit_combo = self._is_exit_combo()
            exit_edge = exit_combo and not self.prev_exit_combo
            self.prev_exit_combo = exit_combo
            if exit_edge:
                self._exit_soccer_to_hold()
            else:
                self._update_soccer_decision()
                ctx = self._build_soccer_state_context()
                state = self.soccer_state_machine.update(ctx)
                if state.mode != self.prev_soccer_mode:
                    self.get_logger().info(
                        f"Soccer mode -> {state.mode.value} ({state.reason}, recovery={state.recovery_type})"
                    )
                    if state.reset_agent and state.mode.value in self.available_agents:
                        self.available_agents[state.mode.value].reset()
                    if state.mode == SoccerMode.FALL_AND_GETUP:
                        self.recovery_type = state.recovery_type
                        self.get_up_done = False
                    else:
                        self.recovery_type = "none"
                        self.get_up_done = False
                    self.prev_soccer_mode = state.mode
                self._log_soccer_status(state, ctx)
                self._publish_decision_debug(
                    soccer_mode=state.mode.value,
                    fsm_reason=state.reason,
                )
                self._run_soccer_mode(state.mode)

        elif self.current_agent_name == "hold":
            self._hold_current_h0w_pose("manual exit; press LT+RT+A to re-enter soccer")
            enter_combo = self._is_enter_combo()
            enter_edge = enter_combo and not self.prev_enter_combo
            if enter_edge:
                self._enter_soccer()
            self.prev_enter_combo = enter_combo

        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_callback_time_consumptions.put(time.time() - t0)
            self.main_loop_timer_counter += 1
            if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL == 0:
                times = [self.main_loop_callback_time_consumptions.get() for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)]
                self.get_logger().info(
                    f"Actual main loop frequency: "
                    f"{MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time):.2f} Hz. "
                    f"Mean time consumption: {np.mean(times):.4f} s."
                )
                self.main_loop_timer_counter = 0
                self.main_loop_timer_counter_time = time.time()


def main(args) -> None:
    rclpy.init()
    soccer_logdir, soccer_actor_path = soccer.resolve_logdir_and_actor(args.soccer_logdir)

    decision = PiPlusSoccerDecision(
        ball_timeout=args.ball_base_timeout,
        close_to_ball=args.close_to_ball,
        close_to_ball_exit=args.close_to_ball_exit,
        kick_distance=args.kick_distance,
        kick_exit_distance=args.kick_exit_distance,
        face_ball_tolerance_deg=args.face_ball_tolerance_deg,
        face_ball_exit_tolerance_deg=args.face_ball_exit_tolerance_deg,
        approach_face_tolerance_deg=args.approach_face_tolerance_deg,
        face_goal_tolerance_deg=args.face_goal_tolerance_deg,
        kick_face_tolerance_deg=args.kick_face_tolerance_deg,
        kick_stable_ang_vel=args.kick_stable_ang_vel,
        kick_stable_frames=args.kick_stable_frames,
        kick_stable_ball_dist=args.kick_stable_ball_dist,
        kick_speed=args.kick_speed,
        kick_latch_duration=args.kick_latch_duration,
        goal_yaw_b=args.goal_yaw_b,
    )
    if args.walk_mode == "h1w_locked_waist":
        walk_shadow_cls = SoccerLockedWaistWalkShadowNode
        walk_robot_class = args.walk_robot_class or "PiPlus_S_12L8A0G2H1W"
    else:
        walk_shadow_cls = SoccerWalkShadowNode
        walk_robot_class = args.walk_robot_class or "PiPlus_S_12L8A0G2H0W"

    node = SoccerPolicyControlNode(
        robot_class_name="PiPlus_S_12L8A0G2H0W",
        joint_pos_protect_ratio=2.0,
        dryrun=not args.nodryrun,
        computer_clip_torque=False,
        cmd_vel_topic=args.cmd_vel_topic,
        cmd_vel_timeout=args.cmd_vel_timeout,
        max_cmd_vel=args.max_cmd_vel,
        head_joint_topic=args.head_joint_topic,
        head_joint_timeout=args.head_joint_timeout,
        head_yaw_limit=args.head_yaw_limit,
        head_pitch_limit=args.head_pitch_limit,
        head_kp=args.head_kp,
        head_kd=args.head_kd,
        ball_base_topic=args.ball_base_topic,
        ball_base_timeout=args.ball_base_timeout,
        ball_base_vel_topic=args.ball_base_vel_topic,
        ball_base_vel_timeout=args.ball_base_vel_timeout,
        goal_yaw_topic=args.goal_yaw_topic,
        goal_yaw_timeout=args.goal_yaw_timeout,
        behavior_intent_topic=args.behavior_intent_topic,
        behavior_intent_timeout=args.behavior_intent_timeout,
        fall_topic=args.fall_topic,
        fall_topic_timeout=args.fall_topic_timeout,
        decision=decision,
        scan_phase_sec=args.scan_phase_sec,
        turn_phase_sec=args.turn_phase_sec,
        scan_turn_wz=args.scan_turn_wz,
    )
    node.soccer_state_machine = SoccerStateMachine(
        enable_fall_detection=bool(args.enable_fall_detection or args.fall_topic),
        fall_gravity_z_threshold=args.fall_gravity_z_threshold,
        fall_confirm_frames=args.fall_confirm_frames,
        fall_face_axis=args.fall_face_axis,
        fall_face_threshold=args.fall_face_threshold,
        fall_faceup_sign=args.fall_faceup_sign,
        recovery_confirm_frames=args.recovery_confirm_frames,
        recovery_start_delay=args.recovery_start_delay,
        classify_ang_vel_max=args.classify_ang_vel_max,
        unknown_recovery_hold_time=args.unknown_recovery_hold_time,
        standing_gravity_z_threshold=args.standing_gravity_z_threshold,
        kick_intent_distance=args.kick_intent_distance,
        kick_intent_face_tolerance_deg=args.kick_intent_face_tolerance_deg,
    )

    walk_shadow = walk_shadow_cls(
        node,
        robot_class_name=walk_robot_class,
        joint_pos_protect_ratio=2.0,
        dryrun=True,
        computer_clip_torque=False,
        node_name="soccer_walk_shadow_node",
    )
    node.walk_shadow_node = walk_shadow

    walk_agent = create_walk_agent(
        logdir=args.walk_logdir,
        ros_node=walk_shadow,
        x_vel_scale=args.x_vel_scale,
        y_vel_scale=args.y_vel_scale,
        yaw_vel_scale=args.yaw_vel_scale,
    )
    node.register_agent("walk", walk_agent)

    kick_shadow = SoccerKickShadowNode(
        node,
        robot_class_name="PiPlus_S_12L8A0G2H0W",
        joint_pos_protect_ratio=2.0,
        dryrun=True,
        computer_clip_torque=False,
        node_name="soccer_kick_shadow_node",
    )
    kick_shadow.set_ball_position_camera_topic(args.ball_base_topic)
    node.soccer_shadow_node = kick_shadow

    kick_agent = create_soccer_kick_agent(
        logdir=soccer_logdir,
        actor_path=soccer_actor_path,
        ros_node=kick_shadow,
        kick_speed=args.kick_speed,
        x_vel_scale=args.x_vel_scale,
        y_vel_scale=args.y_vel_scale,
        yaw_vel_scale=args.yaw_vel_scale,
        soccer_pub_period=args.soccer_pub_period,
        soccer_history_samples=args.soccer_history_samples,
        ball_topic_timeout=args.ball_base_timeout,
        head_mode="zero_gain",      # 头部由框架 apply_external_head_targets 控制, agent 不抢头
        ball_frame="base_link",     # 球已在 base 系 (get_latest_ball_base 返回 [x,y,z,vx,vy,vz])
        ball_vel_source="measured", # 读 dvision 球速 (经桥 -> /soccer/ball_base_xy_velocity)
        posonly=False,              # 650 obs (含球速)
    )
    node.register_agent("kick", kick_agent)

    fall_recovery_shadow = None
    if args.get_up_logdir:
        args.faceup_get_up_logdir = args.get_up_logdir
    if args.get_up_motion_dir:
        args.faceup_get_up_motion_dir = args.get_up_motion_dir
    faceup_get_up_logdir = Path(args.faceup_get_up_logdir).expanduser()
    faceup_get_up_motion_dir = Path(args.faceup_get_up_motion_dir).expanduser()
    facedown_get_up_logdir = Path(args.facedown_get_up_logdir).expanduser()
    facedown_get_up_motion_dir = Path(args.facedown_get_up_motion_dir).expanduser()
    has_faceup_get_up = faceup_get_up_logdir.exists() and faceup_get_up_motion_dir.exists()
    has_facedown_get_up = facedown_get_up_logdir.exists() and facedown_get_up_motion_dir.exists()
    get_up_faceup_agent = None
    get_up_facedown_agent = None
    if has_faceup_get_up or has_facedown_get_up:
        fall_recovery_shadow = SoccerH0WShadowNode(
            node,
            robot_class_name="PiPlus_S_12L8A0G2H0W_FallRecovery",
            joint_pos_protect_ratio=2.0,
            dryrun=True,
            computer_clip_torque=False,
            node_name="soccer_fall_recovery_shadow_node",
        )
        node.fall_recovery_shadow_node = fall_recovery_shadow
        if has_faceup_get_up:
            get_up_faceup_agent = create_fall_recovery_agent(
                logdir=str(faceup_get_up_logdir),
                motion_dir=str(faceup_get_up_motion_dir),
                ros_node=fall_recovery_shadow,
            )
        if has_facedown_get_up:
            get_up_facedown_agent = create_fall_recovery_agent(
                logdir=str(facedown_get_up_logdir),
                motion_dir=str(facedown_get_up_motion_dir),
                ros_node=fall_recovery_shadow,
            )
        fall_and_getup_agent = FallAndGetUpAgent(
            shadow_node=fall_recovery_shadow,
            faceup_agent=get_up_faceup_agent,
            facedown_agent=get_up_facedown_agent,
            send_shadow_action=node._send_h0w_shadow_action,
            logger=node.get_logger(),
            fallen_abs_z_threshold=args.fall_gravity_z_threshold,
            fallen_confirm_frames=args.fall_confirm_frames,
            face_axis=args.fall_face_axis,
            faceup_sign=args.fall_faceup_sign,
            face_threshold=args.fall_face_threshold,
            posture_confirm_frames=args.recovery_confirm_frames,
            classify_ang_vel_max=args.classify_ang_vel_max,
            recovery_start_delay=args.recovery_start_delay,
            get_up_prepare_steps=args.get_up_prepare_steps,
            get_up_prepare_kpkd_factor=args.get_up_prepare_kpkd_factor,
        )
        node.fall_and_getup_agent = fall_and_getup_agent
        node.register_agent(SoccerMode.FALL_AND_GETUP.value, fall_and_getup_agent)
    else:
        print(f"faceup get-up policy missing: {faceup_get_up_logdir}")
        print(f"faceup get-up motion missing: {faceup_get_up_motion_dir}")
        print(f"facedown get-up policy missing: {facedown_get_up_logdir}")
        print(f"facedown get-up motion missing: {facedown_get_up_motion_dir}")

    # 头部关节由行为树/dancer-io 独立控制: cold_start 不检查也不命令头部。
    # 否则行为树扫头时 head_pitch(≈0.266rad)/head_yaw(扫动)会让 cold_start 永远无法 done,
    # LT+RT+A 因 done=False 进不了 soccer → 手柄控制不了 + 扫头后不转身。
    head_joints_mask = np.zeros(walk_shadow.NUM_JOINTS, dtype=bool)
    for _hj_idx, _hj_name in enumerate(walk_shadow.sim_joint_names):
        if "head" in _hj_name:
            head_joints_mask[_hj_idx] = True
    if not head_joints_mask.any():
        print("WARNING: no head joints in sim_joint_names, cold_start will include head joints.")

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=walk_shadow,
        joint_target_pos=walk_agent.default_joint_pos,
        action_scale=walk_agent.action_scale,
        action_offset=walk_agent.action_offset,
        p_gains=walk_agent.p_gains * args.kpkd_factor,
        d_gains=walk_agent.d_gains * args.kpkd_factor,
        excluded_joints=head_joints_mask,
    )
    node.register_agent("cold_start", cold_start_agent)

    print(f"walk mode             : {args.walk_mode} ({walk_shadow_cls.__name__})")
    print(f"walk robot class      : {walk_robot_class}")
    print(f"walk logdir           : {args.walk_logdir}")
    print(f"soccer/kick logdir    : {soccer_logdir}  |  actor: {soccer_actor_path or '(default exported/actor.onnx)'}")
    print(f"cmd_vel topic         : {args.cmd_vel_topic}")
    print(f"head topic            : {args.head_joint_topic}")
    print(f"ball base topic       : {args.ball_base_topic}")
    print(f"target yaw topic      : {args.goal_yaw_topic}")
    print(f"behavior intent topic : {args.behavior_intent_topic or '(disabled)'}")
    print(f"fall topic            : {args.fall_topic or '(disabled)'}")
    print(
        f"fall auto detection   : {'enabled' if args.enable_fall_detection else 'disabled'} "
        f"(gravity_z<{args.fall_gravity_z_threshold:.2f}, stand_z>={args.standing_gravity_z_threshold:.2f})"
    )
    print(f"fall hold before classify: {args.recovery_start_delay:.2f}s")
    print(f"faceup get-up policy  : {faceup_get_up_logdir if faceup_get_up_logdir.exists() else '(not registered)'}")
    print(f"faceup get-up motion  : {faceup_get_up_motion_dir if faceup_get_up_motion_dir.exists() else '(not registered)'}")
    print(f"facedown get-up policy: {facedown_get_up_logdir if facedown_get_up_logdir.exists() else '(not registered)'}")
    print(f"facedown get-up motion: {facedown_get_up_motion_dir if facedown_get_up_motion_dir.exists() else '(not registered)'}")
    print(
        f"kick condition        : enter ball_x <= {args.kick_distance:.2f}, "
        f"exit ball_x > {args.kick_exit_distance:.2f}, "
        f"distance enter/exit={args.close_to_ball:.2f}/{args.close_to_ball_exit:.2f}"
    )
    print(
        f"face ball tolerance   : enter {args.face_ball_tolerance_deg:.1f}deg, "
        f"exit {args.face_ball_exit_tolerance_deg:.1f}deg"
    )
    print(f"kick latch duration   : {args.kick_latch_duration:.2f}s")
    print(f"dryrun                : {not args.nodryrun}")

    node.start_ros_handlers()

    if args.skip_cold_start:
        node.current_agent_name = "soccer"
        node.available_agents["walk"].reset()
        node.available_agents["kick"].reset()
        if "dribble" in node.available_agents:
            node.available_agents["dribble"].reset()
        node.get_logger().info("Skipping cold start, starting with soccer walk/kick control.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        kick_shadow.destroy_node()
        if fall_recovery_shadow is not None:
            fall_recovery_shadow.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="soccer ROS1 decision + ROS2 dual-policy walk/kick control.")
    parser.add_argument("--walk_logdir", type=str, required=True)
    parser.add_argument("--walk_mode", type=str, choices=["h0w_nowaist", "h1w_locked_waist"], default="h0w_nowaist")
    parser.add_argument("--walk_robot_class", type=str, default=None)
    parser.add_argument("--soccer_logdir", type=str, default=str(DEFAULT_KICK_LOGDIR))
    parser.add_argument("--cmd_vel_topic", type=str, default="/ros1_cmd_vel")
    parser.add_argument("--cmd_vel_timeout", type=float, default=0.5)
    parser.add_argument("--max_cmd_vel", type=float, nargs=3, default=[0.5, 0.5, 1.0])
    parser.add_argument("--head_joint_topic", type=str, default="/pi_plus_absolute")
    parser.add_argument("--head_joint_timeout", type=float, default=0.5)
    parser.add_argument("--head_yaw_limit", type=float, default=1.5)
    parser.add_argument("--head_pitch_limit", type=float, default=1.0)
    parser.add_argument("--head_kp", type=float, default=30.0)
    parser.add_argument("--head_kd", type=float, default=1.0)
    parser.add_argument("--ball_base_topic", type=str, default="/ros1_ball_position")
    parser.add_argument("--ball_base_timeout", type=float, default=0.5)
    parser.add_argument("--ball_base_vel_topic", type=str, default="/soccer/ball_base_xy_velocity")
    parser.add_argument("--ball_base_vel_timeout", type=float, default=0.5)
    parser.add_argument("--goal_yaw_topic", type=str, default="/ros1_goal_yaw_b")
    parser.add_argument("--goal_yaw_timeout", type=float, default=0.5)
    parser.add_argument("--behavior_intent_topic", type=str, default=None)
    parser.add_argument("--behavior_intent_timeout", type=float, default=0.5)
    parser.add_argument("--fall_topic", type=str, default=None)
    parser.add_argument("--fall_topic_timeout", type=float, default=0.5)
    parser.add_argument("--enable_fall_detection", action="store_true", default=False)
    parser.add_argument("--fall_gravity_z_threshold", type=float, default=0.55)
    parser.add_argument("--fall_confirm_frames", type=int, default=15)
    parser.add_argument("--fall_face_axis", type=int, default=0)
    parser.add_argument("--fall_face_threshold", type=float, default=0.65)
    parser.add_argument("--fall_faceup_sign", type=float, default=-1.0)
    parser.add_argument("--recovery_confirm_frames", type=int, default=10)
    parser.add_argument("--recovery_start_delay", type=float, default=1.0)
    parser.add_argument("--classify_ang_vel_max", type=float, default=2.0)
    parser.add_argument("--unknown_recovery_hold_time", type=float, default=0.75)
    parser.add_argument("--standing_gravity_z_threshold", type=float, default=0.75)
    parser.add_argument("--faceup_get_up_logdir", type=str, default=str(DEFAULT_FACEUP_GET_UP_LOGDIR))
    parser.add_argument("--faceup_get_up_motion_dir", type=str, default=str(DEFAULT_FACEUP_GET_UP_MOTION_DIR))
    parser.add_argument("--facedown_get_up_logdir", type=str, default=str(DEFAULT_FACEDOWN_GET_UP_LOGDIR))
    parser.add_argument("--facedown_get_up_motion_dir", type=str, default=str(DEFAULT_FACEDOWN_GET_UP_MOTION_DIR))
    parser.add_argument(
        "--get_up_prepare_steps",
        type=int,
        default=60,
        help="跌倒恢复平滑过渡帧数(50Hz): 恢复启动时前 N 帧在'跌倒保持姿势'和'策略目标'间线性插值, 避免第一帧猛追参考(手臂外展±90°)导致的剧烈全身展开。0=关闭。",
    )
    parser.add_argument("--get_up_prepare_kpkd_factor", type=float, default=0.3)
    parser.add_argument("--get_up_logdir", type=str, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--get_up_motion_dir", type=str, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--kick_speed", type=float, default=3.5, help="踢球力度命令 (训练 hard_speed 2.0-5.0; 默认2.0卡在soft/hard边界, 改3.5明确硬踢)")
    parser.add_argument(
        "--kick_latch_duration",
        type=float,
        default=1.0,
        help="踢球锁定窗口(秒): 进入踢球后强制保持 KICK 跑完整动作, 不被打断; 窗口结束后若球还在脚边自动再踢, 球被踢远才切走路。",
    )
    parser.add_argument("--close_to_ball", type=float, default=1.0)
    parser.add_argument("--close_to_ball_exit", type=float, default=1.2)
    parser.add_argument("--kick_distance", type=float, default=0.55, help="踢球触发距离(球x), 0.50太近脚会撞球, 0.65更早踢")
    parser.add_argument("--kick_exit_distance", type=float, default=0.70)
    parser.add_argument("--kick_intent_distance", type=float, default=0.70)
    parser.add_argument("--kick_intent_face_tolerance_deg", type=float, default=70.0)
    parser.add_argument("--face_ball_tolerance_deg", type=float, default=50.0)
    parser.add_argument("--face_ball_exit_tolerance_deg", type=float, default=70.0)
    parser.add_argument("--approach_face_tolerance_deg", type=float, default=5.0, help="接近球时球角超过该值原地转身对正, 对正后才直线走(互斥)")
    parser.add_argument("--face_goal_tolerance_deg", type=float, default=10.0)
    parser.add_argument("--kick_face_tolerance_deg", type=float, default=5.0, help="踢球前朝向对准球的容差(度), 朝正后才踢 (8°视觉上偏斜, 收紧到5°)")
    parser.add_argument("--kick_stable_ang_vel", type=float, default=0.35, help="踢球前允许的最大转动角速度(rad/s)")
    parser.add_argument("--kick_stable_frames", type=int, default=10, help="对准且稳定后需要持续的帧数才踢球")
    parser.add_argument("--kick_stable_ball_dist", type=float, default=0.06, help="踢球前窗口内球位置最大漂移(米), 球在动则不踢")
    parser.add_argument("--goal_yaw_b", type=float, default=0.0)
    parser.add_argument("--soccer_pub_period", type=int, default=5)
    parser.add_argument("--soccer_history_samples", type=int, default=10)
    parser.add_argument("--startup_step_size", type=float, default=0.2)
    parser.add_argument("--kpkd_factor", type=float, default=1.0)
    parser.add_argument("--x_vel_scale", type=float, default=0.5)
    parser.add_argument("--y_vel_scale", type=float, default=-0.5)
    parser.add_argument("--yaw_vel_scale", type=float, default=-1.0)
    parser.add_argument("--nodryrun", action="store_true", default=False)
    parser.add_argument("--skip_cold_start", action="store_true", default=False)
    parser.add_argument("--scan_phase_sec", type=float, default=4.0, help="找不到球时扫头阶段时长(秒)")
    parser.add_argument("--turn_phase_sec", type=float, default=2.0, help="找不到球时转身阶段时长(秒)")
    parser.add_argument("--scan_turn_wz", type=float, default=0.5, help="找不到球时转身角速度(rad/s)")
    main(parser.parse_args())
