#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1-style soccer decision layer for PiPlus ROS2 deployment.

This module is intentionally ROS-free. It ports the practical parts of the
RoboCup ROS1 dbehavior/dplanner chain into a small pure-Python state machine:

- optional head search/track: ScanFieldDown + TrackBall style targets
- body command: approach ball, face ball, align to goal, kick intent

The caller owns ROS IO and maps this decision output into the deployed policy's
velocity/kick command. Route C normally lets the ROS1 behavior/dancer-io chain
own head targets through /pi_plus_absolute.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple

import numpy as np


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class SoccerDecision:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    kick: bool = False
    kick_dir_yaw_b: float = 0.0
    kick_speed: float = 0.0
    ball_x_b: float = 0.0
    ball_y_b: float = 0.0
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    sees_ball: bool = False
    state: str = "scan"


class PiPlusSoccerDecision:
    """Small decision layer mirroring the ROS1 soccer behavior/planner intent."""

    def __init__(
        self,
        *,
        ball_timeout: float = 0.5,
        close_to_ball: float = 1.0,
        close_to_ball_exit: float = 1.2,
        kick_distance: float = 0.55,
        kick_exit_distance: float = 0.85,
        face_ball_tolerance_deg: float = 50.0,
        face_ball_exit_tolerance_deg: float = 70.0,
        approach_face_tolerance_deg: float = 5.0,
        approach_vy_factor: float = 0.5,
        face_goal_tolerance_deg: float = 10.0,
        kick_face_tolerance_deg: float = 8.0,
        kick_stable_ang_vel: float = 0.35,
        kick_stable_frames: int = 10,
        kick_stable_ball_dist: float = 0.06,
        min_linear_speed: float = 0.4,
        max_linear_speed: float = 0.5,
        min_angular_speed: float = 0.5,
        max_angular_speed: float = 1.0,
        yaw_kp: float = 0.57,
        approach_kp: float = 0.5,
        rotate_vy: float = 0.5,
        kick_speed: float = 2.0,
        kick_latch_duration: float = 1.0,
        goal_yaw_b: float = 0.0,
        head_max_speed_deg_s: float = 60.0,
        head_yaw_sign: float = 1.0,
        head_pitch_sign: float = 1.0,
        dt: float = 0.02,
        enable_head_control: bool = False,
    ):
        self.ball_timeout = float(ball_timeout)
        self.close_to_ball = float(close_to_ball)
        self.close_to_ball_exit = max(float(close_to_ball_exit), self.close_to_ball)
        self.kick_distance = float(kick_distance)
        self.kick_exit_distance = max(float(kick_exit_distance), self.kick_distance)
        self.face_ball_tolerance = math.radians(face_ball_tolerance_deg)
        self.face_ball_exit_tolerance = max(math.radians(face_ball_exit_tolerance_deg), self.face_ball_tolerance)
        self.face_goal_tolerance = math.radians(face_goal_tolerance_deg)
        # 接近球时: 球角超过该容差只原地转身对正, 不横移/不走 (用户要求"先转身对正再直线走")
        self.approach_face_tolerance = max(math.radians(approach_face_tolerance_deg), 0.0)
        # 对准后接近阶段的横向速度衰减系数 (压低 vy, 让接近基本是直线)
        self.approach_vy_factor = float(approach_vy_factor)
        # 踢球稳定性: 只有朝向对准球(朝正) 且 转动平稳 且 球位置稳定 且 连续若干帧, 才允许踢球
        self.kick_face_tolerance = math.radians(kick_face_tolerance_deg)
        self.kick_stable_ang_vel = float(kick_stable_ang_vel)
        self.kick_stable_frames = int(kick_stable_frames)
        self.kick_stable_ball_dist = float(kick_stable_ball_dist)
        self.kick_face_frames = 0
        self._ball_pos_hist: list = []
        self.min_linear_speed = float(min_linear_speed)
        self.max_linear_speed = float(max_linear_speed)
        self.min_angular_speed = float(min_angular_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.yaw_kp = float(yaw_kp)
        self.approach_kp = float(approach_kp)
        self.rotate_vy = float(rotate_vy)
        self.kick_speed = float(kick_speed)
        self.kick_latch_duration = float(kick_latch_duration)
        self.goal_yaw_b = float(goal_yaw_b)

        self.enable_head_control = bool(enable_head_control)
        self.head_yaw_sign = float(head_yaw_sign)
        self.head_pitch_sign = float(head_pitch_sign)
        self.head_max_delta = abs(math.radians(head_max_speed_deg_s)) * float(dt)
        self.track_thresh = math.radians(3.0)
        self.head_arrive_thresh = math.radians(8.0)
        self.head_scan_hold_s = 0.3
        self.head_pitch_min = math.radians(-10.0)
        self.head_pitch_max = math.radians(45.0)
        self.head_yaw_min = math.radians(-75.0)
        self.head_yaw_max = math.radians(75.0)
        self.head_scan_targets = [
            (math.radians(10.0), math.radians(75.0)),
            (math.radians(10.0), math.radians(45.0)),
            (math.radians(10.0), math.radians(0.0)),
            (math.radians(10.0), math.radians(-45.0)),
            (math.radians(10.0), math.radians(-75.0)),
            (math.radians(33.0), math.radians(-75.0)),
            (math.radians(33.0), math.radians(-45.0)),
            (math.radians(33.0), math.radians(0.0)),
            (math.radians(33.0), math.radians(45.0)),
            (math.radians(33.0), math.radians(75.0)),
        ]
        self.reset()

    def reset(self) -> None:
        self.head_scan_index = 0
        self.head_scan_arrived_since: Optional[float] = None
        self.head_cmd_pitch: Optional[float] = None
        self.head_cmd_yaw: Optional[float] = None
        self.kick_latch_until: float = 0.0
        self.kick_latch_decision: Optional[SoccerDecision] = None
        self.kick_active = False
        self.kick_face_frames = 0
        self._ball_pos_hist: list = []
        self._last_goal_yaw_b_input: Optional[float] = None

    def debug_dict(
        self,
        decision: Optional[SoccerDecision] = None,
        *,
        ang_vel_z: float = 0.0,
        soccer_mode: str = "",
        fsm_reason: str = "",
    ) -> dict:
        """导出决策层内部状态, 供 50Hz /soccer/decision_debug 话题录制。

        用于复盘"踢球不稳定": 逐帧记录踢球进入条件相关的内部量,
        (朝向误差, 角速度, 稳定帧计数, 球位置窗口漂移, 各阈值, 状态机模式)。
        """
        ball_stable = self._ball_stable()
        face_ball_error = 0.0
        distance = 0.0
        ball_x = 0.0
        ball_y = 0.0
        sees_ball = False
        if decision is not None:
            ball_x = float(decision.ball_x_b)
            ball_y = float(decision.ball_y_b)
            sees_ball = bool(decision.sees_ball)
            distance = math.hypot(ball_x, ball_y)
            face_ball_error = wrap_pi(math.atan2(ball_y, ball_x)) if sees_ball else 0.0
        return {
            # 决策输出
            "t": time.time(),
            "state": decision.state if decision is not None else "",
            "kick": decision.kick if decision is not None else False,
            "kick_dir_yaw_b": decision.kick_dir_yaw_b if decision is not None else 0.0,
            "kick_speed": decision.kick_speed if decision is not None else 0.0,
            "vx": decision.vx if decision is not None else 0.0,
            "vy": decision.vy if decision is not None else 0.0,
            "wz": decision.wz if decision is not None else 0.0,
            "sees_ball": sees_ball,
            "ball_x_b": ball_x,
            "ball_y_b": ball_y,
            "distance": distance,
            "face_ball_error_deg": math.degrees(face_ball_error),
            # 上一次 update 实际收到的 goal_yaw_b (决定 kick_dir_yaw_b 的方向)。
            # self.goal_yaw_b 只是构造时的默认值 (未收到话题时用), 不代表实际输入。
            "goal_yaw_b_input": getattr(self, "_last_goal_yaw_b_input", None),
            # 踢球稳定判定
            "kick_face_frames": self.kick_face_frames,
            "ball_stable": ball_stable,
            "kick_active": self.kick_active,
            "kick_latch_until": self.kick_latch_until,
            "kick_latched": time.time() < self.kick_latch_until,
            "ang_vel_z": ang_vel_z,
            "ball_hist_len": len(self._ball_pos_hist),
            # 阈值 (记录基准值, 方便事后核对用的是哪套参数)
            "kick_face_tolerance_deg": math.degrees(self.kick_face_tolerance),
            "kick_stable_ang_vel": self.kick_stable_ang_vel,
            "kick_stable_frames": self.kick_stable_frames,
            "kick_stable_ball_dist": self.kick_stable_ball_dist,
            "kick_distance": self.kick_distance,
            "close_to_ball": self.close_to_ball,
            "close_to_ball_exit": self.close_to_ball_exit,
            "face_ball_tolerance_deg": math.degrees(self.face_ball_tolerance),
            # 状态机
            "soccer_mode": soccer_mode,
            "fsm_reason": fsm_reason,
        }

    def update(
        self,
        *,
        ball_xyz_camera: Optional[np.ndarray],
        ball_xyz_base: Optional[np.ndarray],
        head_pitch: float,
        head_yaw: float,
        goal_yaw_b: Optional[float] = None,
        ang_vel_z: float = 0.0,
    ) -> SoccerDecision:
        now = time.time()
        current_head_pitch = float(head_pitch)
        current_head_yaw = float(head_yaw)
        if self.kick_latch_decision is not None and now < self.kick_latch_until:
            # 锁定踢球 (用户要求"一旦进踢球就跑完固定时长, 不被打断"): latch 窗口
            # (kick_latch_duration, 默认1.0s)内无条件保持 kick, 让踢球策略跑完整整个动作,
            # 不被球抖动/身体转动打断 (否则0.2s就切回走路, 球踢不出去)。
            # 窗口结束后由 kick_active 分支重新评估:
            #   球被踢远(>close_to_ball_exit)  -> 切走路去找球
            #   球还在脚边(<=kick_exit_distance) -> 自动再踢
            #   球偏到侧面                       -> 转身对正后再踢
            if self.enable_head_control:
                pitch, yaw = self._slew_head_target(
                    self.kick_latch_decision.head_pitch,
                    self.kick_latch_decision.head_yaw,
                    current_head_pitch,
                    current_head_yaw,
                )
            else:
                pitch, yaw = current_head_pitch, current_head_yaw
            decision = SoccerDecision(**vars(self.kick_latch_decision))
            decision.head_pitch = pitch
            decision.head_yaw = yaw
            return decision

        self._last_goal_yaw_b_input = None if goal_yaw_b is None else float(goal_yaw_b)

        sees_ball = ball_xyz_camera is not None and ball_xyz_base is not None
        # 维护球位置历史, 用于踢球前的"球位置稳定"判断
        if sees_ball:
            self._ball_pos_hist.append((float(ball_xyz_base[0]), float(ball_xyz_base[1])))
            if len(self._ball_pos_hist) > max(self.kick_stable_frames, 10):
                self._ball_pos_hist.pop(0)
        else:
            self._ball_pos_hist.append((0.0, 0.0))
            if len(self._ball_pos_hist) > max(self.kick_stable_frames, 10):
                self._ball_pos_hist.pop(0)
        if not sees_ball:
            self._clear_kick_state()
            if self.enable_head_control:
                pitch, yaw = self._scan_head_target(current_head_pitch, current_head_yaw)
                pitch, yaw = self._slew_head_target(pitch, yaw, current_head_pitch, current_head_yaw)
            else:
                pitch, yaw = current_head_pitch, current_head_yaw
            return SoccerDecision(head_pitch=pitch, head_yaw=yaw, sees_ball=False, state="scan")

        ball_x = float(ball_xyz_base[0])
        ball_y = float(ball_xyz_base[1])
        distance = math.hypot(ball_x, ball_y)
        face_ball_error = wrap_pi(math.atan2(ball_y, ball_x))
        # face_goal_error 只用于被注释掉的 align_goal 逻辑; 踢球方向不再用它 (方案A: 朝球踢)。
        face_goal_error = wrap_pi(self.goal_yaw_b if goal_yaw_b is None else float(goal_yaw_b))

        if self.enable_head_control:
            pitch, yaw = self._track_head_target(ball_xyz_camera, ball_x, ball_y, current_head_pitch, current_head_yaw)
            pitch, yaw = self._slew_head_target(pitch, yaw, current_head_pitch, current_head_yaw)
        else:
            pitch, yaw = current_head_pitch, current_head_yaw

        decision = SoccerDecision(
            ball_x_b=ball_x,
            ball_y_b=ball_y,
            head_pitch=pitch,
            head_yaw=yaw,
            sees_ball=True,
        )

        if self.kick_active:
            if distance > self.close_to_ball_exit:
                # 用户要求: 先原地转身正对球, 再直线走到球前 (转身和前进互斥)。
                self._clear_kick_state()
                decision.state = "approach_ball"
                if abs(face_ball_error) > self.approach_face_tolerance:
                    # 未正对: 原地转身
                    decision.wz = self._yaw_speed(face_ball_error)
                else:
                    # 已正对: 直线前进 (vy 只做横向微调, 幅度压小, 避免斜走)
                    decision.vx = self._signed_speed(self.approach_kp * ball_x, self.min_linear_speed, self.max_linear_speed)
                    decision.vy = self._signed_speed(self.approach_kp * ball_y * self.approach_vy_factor, self.min_linear_speed * 0.15, self.max_linear_speed)
            elif abs(face_ball_error) > self.kick_face_tolerance:
                # kick_active 也必须先转身对正球才踢: 球偏 > kick_face_tolerance 就不踢,
                # 先转身。否则踢过一脚后球弹到斜对面(如50°), 机器人会直接再踢而不对正。
                self._clear_kick_state()
                decision.state = "face_ball"
                decision.wz = self._yaw_speed(face_ball_error)
                self.kick_face_frames = 0
            elif abs(ang_vel_z) > self.kick_stable_ang_vel:
                # 已对准但还在转: 停转等待稳定, 不踢
                self._clear_kick_state()
                decision.state = "face_ball"
                decision.wz = 0.0
                self.kick_face_frames = 0
            elif ball_x <= self.kick_exit_distance:
                # 对正且转动平稳: 连续稳定帧才踢 (不再要求球位置稳定)
                self.kick_face_frames += 1
                if self.kick_face_frames < self.kick_stable_frames:
                    decision.state = "face_ball"
                    decision.wz = 0.0
                else:
                    # 方案A: 踢球方向 = 球相对机器人的方向, 朝球踢 (不依赖 ROS1 goal_yaw_b)。
                    self._fill_kick_decision(decision, face_ball_error)
                    self._set_kick_latch(decision)
            else:
                self._clear_kick_state()
                decision.state = "final_approach"
                # 逼近减速: 球越近 vx 越小, 避免冲刺撞球, 给 kick 留缓冲
                decision.vx = self._approach_speed(ball_x)
        elif distance > self.close_to_ball:
            # 用户要求: 先原地转身正对球, 再直线走到球前 (转身和前进互斥)。
            # 球角 > approach_face_tolerance: 只转身(wz), 不走不横移;
            # 球角 <= approach_face_tolerance: 已正对, 直线前进(vx), 不再转(wz=0)。
            self._clear_kick_state()
            decision.state = "approach_ball"
            if abs(face_ball_error) > self.approach_face_tolerance:
                # 未正对: 原地转身
                decision.wz = self._yaw_speed(face_ball_error)
            else:
                # 已正对: 直线前进 (vy 只做横向微调, 幅度压小, 避免斜走)
                decision.vx = self._signed_speed(self.approach_kp * ball_x, self.min_linear_speed, self.max_linear_speed)
                decision.vy = self._signed_speed(self.approach_kp * ball_y * self.approach_vy_factor, self.min_linear_speed * 0.15, self.max_linear_speed)
        elif abs(face_ball_error) > self.face_ball_tolerance:
            # 距球近但朝向偏差大(>50°): 转身面向球
            self._clear_kick_state()
            decision.state = "face_ball"
            decision.wz = self._yaw_speed(face_ball_error)
            self.kick_face_frames = 0
        # Soccer control uses the ROS1 target yaw as kick_dir_b, but does not block kick
        # on goal-yaw alignment. This avoids circling around the ball indefinitely
        # when ROS1 asks for a large angled shot.
        # elif abs(face_goal_error) > self.face_goal_tolerance:
        #     decision.state = "align_goal"
        #     direction = -1.0 if face_goal_error > 0.0 else 1.0
        #     decision.vy = direction * self.rotate_vy
        #     decision.wz = -decision.vy / max(0.1, self.close_to_ball * 0.75)
        elif abs(face_ball_error) > self.kick_face_tolerance:
            # 朝向偏差在 50°~12° 之间: 继续转身对准球 (朝正后才允许踢)
            self._clear_kick_state()
            decision.state = "face_ball"
            decision.wz = self._yaw_speed(face_ball_error)
            self.kick_face_frames = 0
        elif abs(ang_vel_z) > self.kick_stable_ang_vel:
            # 已对准(<12°)但机器人还在转: 停转等待稳定, 不踢
            self._clear_kick_state()
            decision.state = "face_ball"
            decision.wz = 0.0
            self.kick_face_frames = 0
        else:
            # 朝向对准且转动平稳: 连续 kick_stable_frames 帧 且 球位置稳定 后才踢球
            self.kick_face_frames += 1
            if self.kick_face_frames < self.kick_stable_frames:
                decision.state = "face_ball"
                decision.wz = 0.0
            elif ball_x > self.kick_distance:
                self._clear_kick_state()
                decision.state = "final_approach"
                # 逼近减速: 球越近 vx 越小, 避免冲刺撞球, 给 kick 留缓冲
                decision.vx = self._approach_speed(ball_x)
            else:
                # 方案A: 踢球方向 = 球相对机器人的方向, 朝球踢 (不依赖 ROS1 goal_yaw_b)。
                self._fill_kick_decision(decision, face_ball_error)
                self._set_kick_latch(decision)

        return decision

    def _fill_kick_decision(self, decision: SoccerDecision, kick_dir_yaw_b: float) -> None:
        decision.state = "kick"
        decision.kick = True
        decision.kick_dir_yaw_b = kick_dir_yaw_b
        decision.kick_speed = self.kick_speed
        self.kick_active = True

    def _set_kick_latch(self, decision: SoccerDecision) -> None:
        self.kick_latch_until = time.time() + self.kick_latch_duration
        self.kick_latch_decision = SoccerDecision(**vars(decision))

    def _clear_kick_state(self) -> None:
        self.kick_latch_until = 0.0
        self.kick_latch_decision = None
        self.kick_active = False

    def _signed_speed(self, raw: float, minimum: float, maximum: float) -> float:
        if abs(raw) < 1e-6:
            return 0.0
        value = clip(abs(raw), minimum, maximum)
        return math.copysign(value, raw)

    def _yaw_speed(self, error: float) -> float:
        if abs(error) < 1e-6:
            return 0.0
        value = clip(abs(error) * self.yaw_kp, self.min_angular_speed, self.max_angular_speed)
        return math.copysign(value, error)

    def _approach_speed(self, ball_x: float) -> float:
        """final_approach 逼近速度: 球越近越慢, 到 kick_distance 就完全停住。

        避免冲球导致脚撞到球, 也给 ball_stable 攒稳定帧留缓冲。
        ball_x >= hi(close_to_ball) 全速; ball_x <= kick_distance 停住并等待踢球。
        """
        lo = self.kick_distance
        hi = max(self.close_to_ball, self.kick_exit_distance)
        if ball_x <= lo:
            return 0.0
        t = clip((ball_x - lo) / max(hi - lo, 1e-3), 0.0, 1.0)
        return clip(t * self.max_linear_speed, 0.0, self.max_linear_speed)

    def _ball_stable(self) -> bool:
        """踢球前要求球位置在窗口内基本不动, 避免视觉抖动导致踢球方向飘/切踢不稳。

        只检查**横向(y)** 漂移: final_approach 时机器人自己在逼近, 球相对 base 的
        x 会持续变小(每帧约 0.008m, 10帧窗口约 0.08m), 若把 x 也算进去,
        ball_stable 恒 False -> kick 永不触发 -> 机器人一直走 -> 脚碰到球。
        y 方向(横向)漂移才是球/视觉真正的不稳定。
        """
        if len(self._ball_pos_hist) < 2:
            return False
        ref_y = self._ball_pos_hist[-1][1]
        return all(
            abs(p[1] - ref_y) <= self.kick_stable_ball_dist
            for p in self._ball_pos_hist
        )

    def _scan_head_target(self, cur_pitch: float, cur_yaw: float) -> Tuple[float, float]:
        target_pitch, target_yaw = self.head_scan_targets[self.head_scan_index]
        arrived = (
            abs(cur_pitch - target_pitch) < self.head_arrive_thresh
            and abs(cur_yaw - target_yaw) < self.head_arrive_thresh
        )
        now = time.time()
        if arrived:
            if self.head_scan_arrived_since is None:
                self.head_scan_arrived_since = now
            elif now - self.head_scan_arrived_since >= self.head_scan_hold_s:
                self.head_scan_index = (self.head_scan_index + 1) % len(self.head_scan_targets)
                self.head_scan_arrived_since = None
                target_pitch, target_yaw = self.head_scan_targets[self.head_scan_index]
        else:
            self.head_scan_arrived_since = None
        return target_pitch, target_yaw

    def _track_head_target(
        self,
        ball_xyz_camera: np.ndarray,
        ball_x_b: float,
        ball_y_b: float,
        cur_pitch: float,
        cur_yaw: float,
    ) -> Tuple[float, float]:
        if 0.20 < ball_x_b < 0.40 and abs(ball_y_b) < 0.25:
            return math.radians(45.0), 0.0

        cam_x, cam_y, cam_z = [float(v) for v in np.asarray(ball_xyz_camera).reshape(3)]
        cam_z = max(1e-3, cam_z)
        yaw_target = cur_yaw - self.head_yaw_sign * math.atan2(cam_x, cam_z)
        pitch_target = cur_pitch + self.head_pitch_sign * math.atan2(cam_y, cam_z)

        if abs(yaw_target - cur_yaw) < self.track_thresh:
            yaw_target = cur_yaw
        if abs(pitch_target - cur_pitch) < self.track_thresh:
            pitch_target = cur_pitch

        yaw_target = clip(yaw_target, self.head_yaw_min, self.head_yaw_max)
        pitch_target = clip(pitch_target, self.head_pitch_min, self.head_pitch_max)
        return pitch_target, yaw_target

    def _slew_head_target(
        self,
        pitch_target: float,
        yaw_target: float,
        cur_pitch: float,
        cur_yaw: float,
    ) -> Tuple[float, float]:
        if self.head_cmd_pitch is None:
            self.head_cmd_pitch = float(cur_pitch)
        if self.head_cmd_yaw is None:
            self.head_cmd_yaw = float(cur_yaw)

        def step(current: float, target: float) -> float:
            delta = clip(float(target) - float(current), -self.head_max_delta, self.head_max_delta)
            return float(current) + delta

        self.head_cmd_pitch = step(self.head_cmd_pitch, pitch_target)
        self.head_cmd_yaw = step(self.head_cmd_yaw, yaw_target)
        return self.head_cmd_pitch, self.head_cmd_yaw
