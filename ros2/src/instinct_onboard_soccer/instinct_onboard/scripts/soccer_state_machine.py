#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import numpy as np

from piplus_h0w_fall_recovery import create_fall_posture_detector


class SoccerMode(str, Enum):
    WALK = "walk"
    KICK = "kick"
    DRIBBLE = "dribble"
    FALL_AND_GETUP = "fall_and_getup"


@dataclass
class SoccerStateContext:
    decision: Optional[Any] = None
    fall_direction: str = "NONE"
    projected_gravity: Optional[np.ndarray] = None
    angular_velocity: Optional[np.ndarray] = None
    behavior_action_type: str = ""
    now: float = 0.0
    active_mode: Optional[SoccerMode] = None
    manual_mode: Optional[SoccerMode] = None
    get_up_done: bool = False
    recovery_type: str = "none"
    recovery_hold_elapsed: float = 0.0
    fallen_elapsed: float = 0.0


@dataclass
class SoccerStateOutput:
    mode: SoccerMode
    reason: str
    recovery_type: str = "none"
    reset_agent: bool = False


class SoccerStateMachine:
    """Selects the soccer execution mode from decision and robot status.

    Add new behavior by extending the mode enum, adding transition conditions
    here, then registering a matching agent in soccer_control_node.py.
    """

    def __init__(
        self,
        *,
        enable_fall_detection: bool = False,
        fall_gravity_z_threshold: float = 0.55,
        fall_confirm_frames: int = 15,
        fall_face_axis: int = 0,
        fall_face_threshold: float = 0.65,
        fall_faceup_sign: float = -1.0,
        recovery_confirm_frames: int = 10,
        recovery_start_delay: float = 1.0,
        classify_ang_vel_max: float = 2.0,
        unknown_recovery_hold_time: float = 0.75,
        standing_gravity_z_threshold: float = 0.75,
        dribble_enter_distance: float = 0.65,
        dribble_exit_distance: float = 0.85,
        dribble_goal_yaw_tolerance: float = 0.6,
        enable_dribble_mode: bool = True,
        kick_intent_distance: float = 0.70,
        kick_intent_face_tolerance_deg: float = 70.0,
    ):
        self.enable_fall_detection = bool(enable_fall_detection)
        self.fall_gravity_z_threshold = float(fall_gravity_z_threshold)
        self.fall_confirm_frames = max(1, int(fall_confirm_frames))
        self.fall_face_axis = int(np.clip(fall_face_axis, 0, 2))
        self.fall_face_threshold = abs(float(fall_face_threshold))
        self.fall_faceup_sign = -1.0 if float(fall_faceup_sign) < 0.0 else 1.0
        self.recovery_confirm_frames = max(1, int(recovery_confirm_frames))
        self.recovery_start_delay = max(0.0, float(recovery_start_delay))
        self.classify_ang_vel_max = float(classify_ang_vel_max)
        self.unknown_recovery_hold_time = max(0.0, float(unknown_recovery_hold_time))
        self.standing_gravity_z_threshold = abs(float(standing_gravity_z_threshold))
        self.dribble_enter_distance = float(dribble_enter_distance)
        self.dribble_exit_distance = max(float(dribble_exit_distance), self.dribble_enter_distance)
        self.dribble_goal_yaw_tolerance = abs(float(dribble_goal_yaw_tolerance))
        self.enable_dribble_mode = bool(enable_dribble_mode)
        self.kick_intent_distance = max(float(kick_intent_distance), 0.0)
        self.kick_intent_face_tolerance = np.deg2rad(abs(float(kick_intent_face_tolerance_deg)))
        self.active_mode: Optional[SoccerMode] = None
        self.fall_detector = create_fall_posture_detector(
            fallen_abs_z_threshold=self.fall_gravity_z_threshold,
            fallen_confirm_frames=self.fall_confirm_frames,
            face_axis=self.fall_face_axis,
            faceup_sign=self.fall_faceup_sign,
            face_threshold=self.fall_face_threshold,
            posture_confirm_frames=self.recovery_confirm_frames,
            classify_ang_vel_max=self.classify_ang_vel_max,
        )
        self.locked_recovery_type = "none"
        self.output_recovery_type = "none"

    @property
    def fallen_count(self) -> int:
        return self.fall_detector.fallen_count

    @property
    def fallen_confirmed(self) -> bool:
        return self.fall_detector.fallen_confirmed

    @property
    def faceup_count(self) -> int:
        return self.fall_detector.faceup_count

    @property
    def facedown_count(self) -> int:
        return self.fall_detector.facedown_count

    def reset(self) -> None:
        self.active_mode = None
        self.fall_detector.reset()
        self.locked_recovery_type = "none"
        self.output_recovery_type = "none"

    def update(self, ctx: SoccerStateContext) -> SoccerStateOutput:
        requested = self._select_mode(ctx)
        reset_agent = requested != self.active_mode
        self.active_mode = requested
        return SoccerStateOutput(
            mode=requested,
            reason=self._reason(requested, ctx),
            recovery_type=self.output_recovery_type if requested == SoccerMode.FALL_AND_GETUP else "none",
            reset_agent=reset_agent,
        )

    def _select_mode(self, ctx: SoccerStateContext) -> SoccerMode:
        self.output_recovery_type = "none"

        if ctx.active_mode == SoccerMode.FALL_AND_GETUP:
            if ctx.get_up_done:
                self._reset_fall_detector()
                self.output_recovery_type = "done"
                return self._normal_mode(ctx)
            self.output_recovery_type = ctx.recovery_type or "hold"
            return SoccerMode.FALL_AND_GETUP

        if self._update_fallen_confirmed(ctx):
            self.output_recovery_type = "hold"
            return SoccerMode.FALL_AND_GETUP

        if self.fallen_count > 0:
            return SoccerMode.WALK

        if ctx.manual_mode is not None:
            return ctx.manual_mode

        self.locked_recovery_type = "none"
        return self._normal_mode(ctx)

    def _normal_mode(self, ctx: SoccerStateContext) -> SoccerMode:
        decision = ctx.decision
        action_type = self._behavior_action_type(ctx)
        if action_type in ("SHOOT", "PASS", "CLEAR"):
            if self._safe_to_kick_intent(ctx, decision):
                return SoccerMode.KICK
            return SoccerMode.WALK

        if action_type in ("HOLD", "POSITION"):
            return SoccerMode.WALK

        if decision is not None and getattr(decision, "kick", False):
            return SoccerMode.KICK

        return SoccerMode.WALK

    def _behavior_action_type(self, ctx: SoccerStateContext) -> str:
        return str(getattr(ctx, "behavior_action_type", "") or "").upper()

    def _safe_to_kick_intent(self, ctx: SoccerStateContext, decision: Optional[Any]) -> bool:
        if decision is None or not getattr(decision, "sees_ball", False):
            return False
        ball_x = float(getattr(decision, "ball_x_b", 0.0))
        ball_y = float(getattr(decision, "ball_y_b", 0.0))
        if ball_x <= 0.0 or ball_x > self.kick_intent_distance:
            return False
        face_ball_error = abs(float(np.arctan2(ball_y, max(ball_x, 1e-3))))
        return face_ball_error <= self.kick_intent_face_tolerance

    def _safe_to_dribble(self, ctx: SoccerStateContext, decision: Optional[Any]) -> bool:
        if decision is None or not getattr(decision, "sees_ball", False):
            return False
        ball_x = float(getattr(decision, "ball_x_b", 0.0))
        ball_y = abs(float(getattr(decision, "ball_y_b", 0.0)))
        enter_distance = self.dribble_exit_distance if ctx.active_mode == SoccerMode.DRIBBLE else self.dribble_enter_distance
        return 0.0 < ball_x <= enter_distance and ball_y <= 0.45

    def _is_fallen(self, ctx: SoccerStateContext) -> bool:
        if not self.enable_fall_detection:
            return False
        if ctx.fall_direction and ctx.fall_direction.upper() != "NONE":
            return True
        return self.fall_detector.is_fallen(ctx.projected_gravity)

    def _update_fallen_confirmed(self, ctx: SoccerStateContext) -> bool:
        if not self.enable_fall_detection:
            return False
        if ctx.fall_direction and ctx.fall_direction.upper() != "NONE":
            self.fall_detector.force_fallen_confirmed()
            return True
        if self.fallen_confirmed:
            return True
        self.fall_detector.update(ctx.projected_gravity, ctx.angular_velocity)
        return self.fallen_confirmed

    def _reset_fall_detector(self) -> None:
        self.fall_detector.reset()

    def _should_dribble(self, ctx: SoccerStateContext, decision: Optional[Any]) -> bool:
        if decision is None or not getattr(decision, "sees_ball", False):
            return False
        state = str(getattr(decision, "state", ""))
        if state == "dribble":
            return True
        ball_x = float(getattr(decision, "ball_x_b", 0.0))
        ball_y = abs(float(getattr(decision, "ball_y_b", 0.0)))
        enter_distance = self.dribble_exit_distance if ctx.active_mode == SoccerMode.DRIBBLE else self.dribble_enter_distance
        goal_yaw = abs(float(getattr(decision, "kick_dir_yaw_b", 0.0)))
        return 0.0 < ball_x <= enter_distance and ball_y <= 0.35 and goal_yaw <= self.dribble_goal_yaw_tolerance

    def _reason(self, mode: SoccerMode, ctx: SoccerStateContext) -> str:
        if ctx.manual_mode is not None:
            return "manual_override"
        action_type = self._behavior_action_type(ctx)
        if mode == SoccerMode.KICK and action_type in ("SHOOT", "PASS", "CLEAR"):
            return "behavior_intent_{}".format(action_type.lower())
        if mode == SoccerMode.DRIBBLE and action_type == "DRIBBLE":
            return "behavior_intent_dribble"
        if mode == SoccerMode.WALK and action_type in ("HOLD", "POSITION"):
            return "behavior_intent_{}".format(action_type.lower())
        if mode == SoccerMode.FALL_AND_GETUP:
            return "fall_detected"
        if mode == SoccerMode.KICK:
            return "decision_kick"
        if mode == SoccerMode.DRIBBLE:
            return "controlled_dribble"
        return "default_walk"
