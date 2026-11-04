"""Model-difference feedforward for the Dragonfly base controller."""
from dataclasses import dataclass

import numpy as np

from .coupled_dynamics import DynamicsResult


def _limit(values, limit, name):
    if limit is None:
        return np.asarray(values, dtype=float), False
    bound = np.broadcast_to(np.asarray(limit, dtype=float), (3,))
    if not np.all(np.isfinite(bound)) or np.any(bound < 0):
        raise ValueError(f"{name} must be finite and nonnegative")
    clipped = np.clip(values, -bound, bound)
    return clipped, bool(np.any(clipped != values))


@dataclass(frozen=True)
class FeedforwardResult:
    nominal_wrench: np.ndarray
    full_wrench: np.ndarray
    raw_delta_wrench: np.ndarray
    delta_wrench: np.ndarray
    command_wrench: np.ndarray
    saturated: bool
    enabled: bool
    dynamics: DynamicsResult


class ModelFeedforward:
    """Compute current-model minus nominal-model wrench at the same base accel."""

    def __init__(self, dynamics, nominal_joint_position=None,
                 force_limit=None, torque_limit=None):
        self.dynamics = dynamics
        count = len(dynamics.active_joint_names)
        self.nominal_joint_position = (np.zeros(count) if nominal_joint_position is None else
                                       np.asarray(nominal_joint_position, dtype=float).copy())
        if self.nominal_joint_position.shape != (count,) or not np.all(np.isfinite(self.nominal_joint_position)):
            raise ValueError(f"nominal_joint_position must contain {count} finite values")
        # Validate limits at construction rather than in the control loop.
        _limit(np.zeros(3), force_limit, "force_limit")
        _limit(np.zeros(3), torque_limit, "torque_limit")
        self.force_limit = force_limit
        self.torque_limit = torque_limit

    def evaluate(self, base_rotation, base_velocity, base_angular_velocity,
                 joint_position, joint_velocity, joint_acceleration,
                 desired_base_acceleration, gravity_world=(0.0, 0.0, -9.81), enabled=True):
        desired_base_acceleration = np.asarray(desired_base_acceleration, dtype=float)
        full = self.dynamics.evaluate(
            base_rotation, base_velocity, base_angular_velocity,
            joint_position, joint_velocity, joint_acceleration,
            desired_base_acceleration, gravity_world,
        )
        nominal = self.dynamics.evaluate(
            base_rotation, base_velocity, base_angular_velocity,
            self.nominal_joint_position,
            np.zeros_like(self.nominal_joint_position),
            np.zeros_like(self.nominal_joint_position),
            desired_base_acceleration, gravity_world,
        )
        raw_delta = full.wrench - nominal.wrench
        force, force_saturated = _limit(raw_delta[:3], self.force_limit, "force_limit")
        torque, torque_saturated = _limit(raw_delta[3:], self.torque_limit, "torque_limit")
        limited_delta = np.r_[force, torque]
        delta = limited_delta if enabled else np.zeros(6)
        return FeedforwardResult(
            nominal_wrench=nominal.wrench,
            full_wrench=full.wrench,
            raw_delta_wrench=raw_delta,
            delta_wrench=delta,
            command_wrench=nominal.wrench + delta,
            saturated=force_saturated or torque_saturated,
            enabled=bool(enabled),
            dynamics=full,
        )
