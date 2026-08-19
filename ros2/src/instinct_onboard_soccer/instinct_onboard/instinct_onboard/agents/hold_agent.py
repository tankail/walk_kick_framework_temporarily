from __future__ import annotations

from typing import Tuple

import numpy as np


class PoseHoldAgent:
    """Hold a fixed joint-space pose using the onboard action interface."""

    def __init__(
        self,
        target_joint_pos: np.ndarray,
        action_scale: np.ndarray,
        action_offset: np.ndarray,
        p_gains: np.ndarray,
        d_gains: np.ndarray,
    ):
        self.target_joint_pos = np.asarray(target_joint_pos, dtype=np.float32).copy()
        self._action_scale = np.asarray(action_scale, dtype=np.float32).copy()
        self._action_offset = np.asarray(action_offset, dtype=np.float32).copy()
        self._p_gains = np.asarray(p_gains, dtype=np.float32).copy()
        self._d_gains = np.asarray(d_gains, dtype=np.float32).copy()

        if np.any(self._action_scale == 0.0):
            raise ValueError("action_scale contains zero, cannot convert hold pose to action")

        expected_shape = self.target_joint_pos.shape
        for name, value in (
            ("action_scale", self._action_scale),
            ("action_offset", self._action_offset),
            ("p_gains", self._p_gains),
            ("d_gains", self._d_gains),
        ):
            if value.shape != expected_shape:
                raise ValueError(f"{name} shape {value.shape} does not match target_joint_pos shape {expected_shape}")

        self._action = (self.target_joint_pos - self._action_offset) / self._action_scale

    def step(self) -> Tuple[np.ndarray, bool]:
        return self._action.copy(), False

    def reset(self):
        pass

    @property
    def action_scale(self) -> np.ndarray:
        return self._action_scale

    @property
    def action_offset(self) -> np.ndarray:
        return self._action_offset

    @property
    def p_gains(self) -> np.ndarray:
        return self._p_gains

    @property
    def d_gains(self) -> np.ndarray:
        return self._d_gains
