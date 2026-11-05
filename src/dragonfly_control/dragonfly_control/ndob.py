"""Six-dimensional momentum NDOB for the flying base, expressed in FLU about O.

The observer estimates the net residual wrench in

    M_bb * nu_dot + M_ba * ddq + h_b = W_actuator + d.

It does not estimate either arm's mounting wrench separately.  With the
complete coupled model, arm inertia and gravity are already in M and h; arm
wrenches must not be added a second time.  W_actuator should be reconstructed
from measured/estimated rotor speeds and tilt angles, including the effect of
all feedback, feedforward and NDOB compensation already applied.

No acceleration differentiation is used.  For generalized base momentum p,
transport/gravity bias b and positive diagonal gain L:

    d_hat = L (p - xi)
    xi_dot = W_actuator + d_hat - b.

The auxiliary equation uses forward Euler with a gain/step stability check;
this is consistent with sampled linear momentum ramps and avoids steady-state
discrete bias for constant known inputs.
"""
from dataclasses import dataclass
import math

import numpy as np


def _vector(values, name, size=6):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _positive(values, name, size=3, allow_zero=False):
    result = _vector(values, name, size)
    if np.any(result < 0) or (not allow_zero and np.any(result == 0)):
        comparison = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{name} must be {comparison}")
    return result


@dataclass(frozen=True)
class NdobConfig:
    """Configuration values are simulation starting points, not hardware limits."""

    enabled: bool = False
    shadow_mode: bool = True
    compensation_enabled: bool = False
    translation_gain: np.ndarray = (1.0, 1.0, 1.0)
    rotation_gain: np.ndarray = (1.0, 1.0, 1.0)
    force_limit: np.ndarray = (20.0, 20.0, 20.0)
    torque_limit: np.ndarray = (3.0, 3.0, 3.0)
    compensation_force_rate: np.ndarray = (50.0, 50.0, 50.0)
    compensation_torque_rate: np.ndarray = (10.0, 10.0, 10.0)
    state_timeout: float = 0.2
    max_input_future: float = 0.02
    max_dt: float = 0.05
    max_time_jump: float = 0.5
    saturation_hold_time: float = 0.2

    def __post_init__(self):
        _positive(self.translation_gain, "translation_gain")
        _positive(self.rotation_gain, "rotation_gain")
        _positive(self.force_limit, "force_limit", allow_zero=True)
        _positive(self.torque_limit, "torque_limit", allow_zero=True)
        _positive(self.compensation_force_rate, "compensation_force_rate")
        _positive(self.compensation_torque_rate, "compensation_torque_rate")
        for name in ("state_timeout", "max_dt", "max_time_jump"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("max_input_future", "saturation_hold_time"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.max_time_jump < self.max_dt:
            raise ValueError("max_time_jump must be greater than or equal to max_dt")

    @classmethod
    def from_mapping(cls, values):
        values = dict(values or {})
        return cls(**{name: values[name] for name in cls.__dataclass_fields__ if name in values})

    @property
    def gain(self):
        return np.r_[self.translation_gain, self.rotation_gain].astype(float)

    @property
    def output_limit(self):
        return np.r_[self.force_limit, self.torque_limit].astype(float)

    @property
    def compensation_rate(self):
        return np.r_[self.compensation_force_rate, self.compensation_torque_rate].astype(float)


@dataclass(frozen=True)
class NdobResult:
    timestamp: float
    input_timestamp: float
    input_age: float
    dt: float
    observer_valid: bool
    shadow_mode: bool
    compensation_enabled: bool
    compensation_active: bool
    estimated_wrench: np.ndarray
    raw_estimated_wrench: np.ndarray
    compensation_wrench: np.ndarray
    momentum: np.ndarray
    known_bias_wrench: np.ndarray
    actuator_wrench: np.ndarray
    allocator_saturation_blocked: bool
    output_limited: bool
    rate_limited: bool
    events: tuple


class MomentumNdob:
    """Stateful six-channel NDOB with safe compensation withdrawal."""

    def __init__(self, config=None):
        self.config = NdobConfig.from_mapping(config) if isinstance(config, dict) else (config or NdobConfig())
        self._auxiliary = None
        self._last_timestamp = None
        self._compensation = np.zeros(6)
        self._saturation_time = 0.0
        self.event_counts = {}

    def reset(self, reason="external_reset"):
        """Invalidate momentum state while retaining compensation for smooth withdrawal."""
        self._auxiliary = None
        self._last_timestamp = None
        self._saturation_time = 0.0
        self.event_counts[reason] = self.event_counts.get(reason, 0) + 1

    def _event(self, events, name):
        events.append(name)
        self.event_counts[name] = self.event_counts.get(name, 0) + 1

    def _compensation_step(self, target, dt):
        # A large timestamp jump must not become a large compensation step.
        step_dt = min(max(float(dt), 0.0), self.config.max_dt)
        difference = target - self._compensation
        bound = self.config.compensation_rate * step_dt
        change = np.clip(difference, -bound, bound)
        self._compensation = self._compensation + change
        return bool(np.any(np.abs(difference - change) > 1e-12))

    def _invalid(self, timestamp, input_timestamp, dt, reason,
                 momentum, bias, actuator, allocator_saturated=False):
        events = []
        self._auxiliary = None
        self._event(events, "invalid:" + reason)
        rate_limited = self._compensation_step(np.zeros(6), dt)
        if rate_limited:
            self._event(events, "compensation_withdrawal_rate_limited")
        age = timestamp - input_timestamp
        return NdobResult(
            timestamp=timestamp, input_timestamp=input_timestamp, input_age=age, dt=dt,
            observer_valid=False, shadow_mode=self.config.shadow_mode,
            compensation_enabled=self.config.compensation_enabled, compensation_active=False,
            estimated_wrench=np.zeros(6), raw_estimated_wrench=np.zeros(6),
            compensation_wrench=self._compensation.copy(), momentum=momentum,
            known_bias_wrench=bias, actuator_wrench=actuator,
            allocator_saturation_blocked=bool(allocator_saturated),
            output_limited=False, rate_limited=rate_limited, events=tuple(events),
        )

    def update(self, timestamp, input_timestamp, momentum, known_bias_wrench,
               actuator_wrench, allocator_saturated=False):
        """Update the observer using one time-aligned sample.

        All wrench inputs and outputs use [N, N, N, N·m, N·m, N·m], body FLU,
        with torque about uav_base_link origin O.  Positive estimated_wrench is
        the residual acting on the flying base.  Positive compensation_wrench
        must be subtracted from the requested whole-vehicle wrench.
        """
        try:
            timestamp = float(timestamp)
            input_timestamp = float(input_timestamp)
            if not math.isfinite(timestamp) or not math.isfinite(input_timestamp):
                raise ValueError("timestamps must be finite")
            momentum = _vector(momentum, "momentum")
            bias = _vector(known_bias_wrench, "known_bias_wrench")
            actuator = _vector(actuator_wrench, "actuator_wrench")
        except (TypeError, ValueError):
            safe_time = float(timestamp) if isinstance(timestamp, (int, float)) and math.isfinite(timestamp) else 0.0
            safe_input = float(input_timestamp) if isinstance(input_timestamp, (int, float)) and math.isfinite(input_timestamp) else safe_time
            dt = 0.0 if self._last_timestamp is None else safe_time - self._last_timestamp
            self._last_timestamp = safe_time
            return self._invalid(safe_time, safe_input, dt, "non_finite_input",
                                 np.zeros(6), np.zeros(6), np.zeros(6))

        dt = 0.0 if self._last_timestamp is None else timestamp - self._last_timestamp
        previous_timestamp = self._last_timestamp
        self._last_timestamp = timestamp
        if previous_timestamp is not None and (dt < 0 or dt > self.config.max_time_jump):
            self._auxiliary = None
            return self._invalid(timestamp, input_timestamp, dt, "time_jump",
                                 momentum, bias, actuator)
        if previous_timestamp is not None and (dt <= 0 or dt > self.config.max_dt):
            self._auxiliary = None
            return self._invalid(timestamp, input_timestamp, dt, "abnormal_dt",
                                 momentum, bias, actuator)
        if dt > 0 and np.max(self.config.gain * dt) >= 2.0:
            self._auxiliary = None
            return self._invalid(timestamp, input_timestamp, dt, "unstable_gain_dt",
                                 momentum, bias, actuator)

        age = timestamp - input_timestamp
        if age > self.config.state_timeout:
            return self._invalid(timestamp, input_timestamp, dt, "stale_state",
                                 momentum, bias, actuator)
        if age < -self.config.max_input_future:
            return self._invalid(timestamp, input_timestamp, dt, "future_state",
                                 momentum, bias, actuator)

        if allocator_saturated:
            self._saturation_time += min(max(dt, 0.0), self.config.max_dt)
        else:
            self._saturation_time = 0.0
        saturation_blocked = self._saturation_time >= self.config.saturation_hold_time

        events = []
        if self._auxiliary is None:
            self._auxiliary = momentum.copy()
            raw = np.zeros(6)
            self._event(events, "initialized")
        else:
            raw = self.config.gain * (momentum - self._auxiliary)

        limited = np.clip(raw, -self.config.output_limit, self.config.output_limit)
        output_limited = bool(np.any(np.abs(raw - limited) > 1e-12))
        if output_limited:
            self._event(events, "output_limited")
        if saturation_blocked:
            self._event(events, "allocator_saturation_block")

        compensation_active = bool(
            self.config.enabled and self.config.compensation_enabled
            and not self.config.shadow_mode and not saturation_blocked
        )
        target = limited if compensation_active else np.zeros(6)
        rate_limited = self._compensation_step(target, dt)
        if rate_limited:
            self._event(events, "compensation_rate_limited")

        if dt > 0:
            # Forward Euler is consistent with sampled linear momentum ramps:
            # constant known inputs produce zero steady-state discrete bias.
            self._auxiliary += dt * (actuator + raw - bias)

        return NdobResult(
            timestamp=timestamp, input_timestamp=input_timestamp, input_age=age, dt=dt,
            observer_valid=True, shadow_mode=self.config.shadow_mode,
            compensation_enabled=self.config.compensation_enabled,
            compensation_active=compensation_active,
            estimated_wrench=limited, raw_estimated_wrench=raw,
            compensation_wrench=self._compensation.copy(), momentum=momentum,
            known_bias_wrench=bias, actuator_wrench=actuator,
            allocator_saturation_blocked=saturation_blocked,
            output_limited=output_limited, rate_limited=rate_limited, events=tuple(events),
        )


def coupled_momentum_and_bias(dynamics, base_rotation, base_velocity, base_angular_velocity,
                              joint_position, joint_velocity, gravity_world=(0.0, 0.0, -9.81),
                              evaluation=None):
    """Return p and b, optionally reusing an aligned full-dynamics evaluation.

    The mass-matrix base block supplies total mass and first mass moment, so
    gravity does not require a second dynamics evaluation.  No arm wrench is
    added again and no state is differentiated.
    """
    v = _vector(base_velocity, "base_velocity", 3)
    omega = _vector(base_angular_velocity, "base_angular_velocity", 3)
    dq = _vector(joint_velocity, "joint_velocity", len(dynamics.active_joint_names))
    gravity_world = _vector(gravity_world, "gravity_world", 3)
    result = dynamics.evaluate(
        base_rotation, v, omega, joint_position, dq,
        np.zeros(len(dynamics.active_joint_names)), np.zeros(6), gravity_world,
    ) if evaluation is None else evaluation
    mass = np.trace(result.m_bb[:3, :3]) / 3.0
    first_moment_skew = -result.m_bb[:3, 3:6]
    first_moment = np.array([first_moment_skew[2, 1], first_moment_skew[0, 2],
                             first_moment_skew[1, 0]])
    gravity_body = np.asarray(base_rotation).T @ gravity_world
    gravity_force = -mass * gravity_body
    gravity = np.r_[gravity_force, np.cross(first_moment, -gravity_body)]
    momentum = result.m_bb @ np.r_[v, omega] + result.m_ba @ dq
    linear, angular = momentum[:3], momentum[3:]
    transport = np.r_[np.cross(omega, linear), np.cross(omega, angular) + np.cross(v, linear)]
    return momentum, transport + gravity, result


def subtract_disturbance(nominal_wrench, compensation_wrench):
    """Apply positive NDOB compensation with the verified opposite sign."""
    nominal = _vector(nominal_wrench, "nominal_wrench")
    compensation = _vector(compensation_wrench, "compensation_wrench")
    return nominal - compensation
