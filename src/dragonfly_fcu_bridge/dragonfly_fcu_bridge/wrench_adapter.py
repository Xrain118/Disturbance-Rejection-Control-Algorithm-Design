"""Pure FLU physical-wrench to PX4 FRD normalized-setpoint conversion."""
from dataclasses import dataclass

import numpy as np


FLU_FROM_FRD = np.diag([1.0, -1.0, -1.0])


def _vector(values, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    return result


@dataclass(frozen=True)
class WrenchScale:
    mass: float
    com: np.ndarray
    inertia: np.ndarray
    acceleration_scale: float
    angular_acceleration_scale: float = 4.0 / 0.15

    def __post_init__(self):
        if not np.isfinite(self.mass) or self.mass <= 0:
            raise ValueError("mass must be positive")
        if not np.isfinite(self.acceleration_scale) or self.acceleration_scale <= 0:
            raise ValueError("acceleration_scale must be positive")
        if not np.isfinite(self.angular_acceleration_scale) or self.angular_acceleration_scale <= 0:
            raise ValueError("angular_acceleration_scale must be positive")
        _vector(self.com, "com")
        inertia = np.asarray(self.inertia, dtype=float)
        if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
            raise ValueError("inertia must be a finite 3x3 matrix")
        if not np.allclose(inertia, inertia.T, atol=1e-12) or np.linalg.eigvalsh(inertia).min() <= 0:
            raise ValueError("inertia must be symmetric positive definite")

    @classmethod
    def from_model_metadata(cls, metadata):
        config = metadata["config"]
        return cls(
            mass=float(metadata["mass"]),
            com=np.asarray(metadata["com"], dtype=float),
            inertia=np.asarray(metadata["inertia"], dtype=float),
            acceleration_scale=9.81 * float(config["thrust_to_weight"]),
        )


@dataclass(frozen=True)
class NormalizedWrench:
    thrust: np.ndarray
    torque: np.ndarray


def physical_to_normalized(force_flu, torque_about_base_flu, scale):
    """Convert N and N·m about O into PX4 FRD normalized thrust/torque."""
    force_flu = _vector(force_flu, "force_flu")
    torque_about_base_flu = _vector(torque_about_base_flu, "torque_about_base_flu")
    thrust = FLU_FROM_FRD @ force_flu / (scale.mass * scale.acceleration_scale)
    torque_about_com = torque_about_base_flu - np.cross(scale.com, force_flu)
    torque = FLU_FROM_FRD @ np.linalg.solve(scale.inertia, torque_about_com)
    torque = torque / scale.angular_acceleration_scale
    return NormalizedWrench(thrust=thrust, torque=torque)


def normalized_to_physical(thrust, torque, scale):
    """Reconstruct FLU force and torque about O from PX4 normalized inputs."""
    thrust = _vector(thrust, "thrust")
    torque = _vector(torque, "torque")
    force_flu = FLU_FROM_FRD @ thrust * (scale.mass * scale.acceleration_scale)
    torque_about_com = scale.inertia @ (FLU_FROM_FRD @ torque)
    torque_about_com *= scale.angular_acceleration_scale
    torque_about_base = torque_about_com + np.cross(scale.com, force_flu)
    return force_flu, torque_about_base
