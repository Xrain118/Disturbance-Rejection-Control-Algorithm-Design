"""Analytic five-axis kinematics for the RoArm-M3 geometry.

The model follows the angle convention used by RoArm-M3_module.h.  All
public lengths are metres and all angles are radians.  Poses are expressed
in an arm's ``*_base_link`` frame.
"""

from dataclasses import dataclass
from enum import Enum
from itertools import product
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml


JOINT_NAMES = ("base", "shoulder", "elbow", "wrist", "roll")
_EPS = 1.0e-9
_ORIENTATION_TOLERANCE = 1.0e-6


def _rotation_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def _rotation_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def _transform(rotation: Optional[np.ndarray] = None, translation=(0.0, 0.0, 0.0)) -> np.ndarray:
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _wrap_pi(angle: float) -> float:
    wrapped = (angle + np.pi) % (2.0 * np.pi) - np.pi
    return np.pi if abs(wrapped + np.pi) < 1.0e-12 and angle > 0.0 else float(wrapped)


def _rotation_error(a: np.ndarray, b: np.ndarray) -> float:
    cosine = np.clip((np.trace(a.T @ b) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


@dataclass(frozen=True)
class ArmParameters:
    base_to_shoulder: float
    shoulder_to_elbow_x: float
    shoulder_to_elbow_z: float
    elbow_to_wrist_x: float
    elbow_to_wrist_z: float
    wrist_to_tool_x: float
    wrist_to_tool_z: float
    home: Tuple[float, float, float, float, float]
    lower: Tuple[float, float, float, float, float]
    upper: Tuple[float, float, float, float, float]

    def __post_init__(self):
        geometry = (
            self.base_to_shoulder, self.shoulder_to_elbow_x,
            self.shoulder_to_elbow_z, self.elbow_to_wrist_x,
            self.elbow_to_wrist_z, self.wrist_to_tool_x, self.wrist_to_tool_z,
        )
        if not np.all(np.isfinite(geometry)) or self.base_to_shoulder < 0.0:
            raise ValueError("Geometry must be finite with nonnegative base height")
        if min(self.shoulder_link_length, self.forearm_length, self.tool_length) <= _EPS:
            raise ValueError("Moving link lengths must be positive")
        for vector in (self.home, self.lower, self.upper):
            if len(vector) != 5 or not np.all(np.isfinite(vector)):
                raise ValueError("Home and limits must each contain five finite values")
        if np.any(np.asarray(self.lower) > np.asarray(self.upper)):
            raise ValueError("Lower limits cannot exceed upper limits")
        if np.any(np.asarray(self.home) < self.lower) or np.any(np.asarray(self.home) > self.upper):
            raise ValueError("Home configuration must be inside joint limits")

    @classmethod
    def from_mapping(cls, document: Mapping) -> "ArmParameters":
        data = document["arm_kinematics"] if "arm_kinematics" in document else document
        limits = data["joint_limits"]
        return cls(
            base_to_shoulder=float(data["base_to_shoulder"]),
            shoulder_to_elbow_x=float(data["shoulder_to_elbow_x"]),
            shoulder_to_elbow_z=float(data["shoulder_to_elbow_z"]),
            elbow_to_wrist_x=float(data["elbow_to_wrist_x"]),
            elbow_to_wrist_z=float(data["elbow_to_wrist_z"]),
            wrist_to_tool_x=float(data["wrist_to_tool_x"]),
            wrist_to_tool_z=float(data["wrist_to_tool_z"]),
            home=tuple(float(value) for value in data["home"]),
            lower=tuple(float(limits[name]["lower"]) for name in JOINT_NAMES),
            upper=tuple(float(limits[name]["upper"]) for name in JOINT_NAMES),
        )

    @classmethod
    def from_yaml(cls, path: Path) -> "ArmParameters":
        with Path(path).open(encoding="utf-8") as stream:
            return cls.from_mapping(yaml.safe_load(stream))

    @property
    def shoulder_link_length(self) -> float:
        return float(np.hypot(self.shoulder_to_elbow_x, self.shoulder_to_elbow_z))

    @property
    def shoulder_link_angle(self) -> float:
        return float(np.arctan2(self.shoulder_to_elbow_x, self.shoulder_to_elbow_z))

    @property
    def forearm_length(self) -> float:
        return float(np.hypot(self.elbow_to_wrist_x, self.elbow_to_wrist_z))

    @property
    def forearm_angle(self) -> float:
        return float(np.arctan2(self.elbow_to_wrist_x, self.elbow_to_wrist_z))

    @property
    def tool_length(self) -> float:
        return float(np.hypot(self.wrist_to_tool_x, self.wrist_to_tool_z))

    @property
    def tool_mount_angle(self) -> float:
        return float(np.arctan2(self.wrist_to_tool_x, self.wrist_to_tool_z))


@dataclass(frozen=True)
class TaskPose:
    x: float
    y: float
    z: float
    pitch: float
    roll: float


class IKStatus(str, Enum):
    OK = "ok"
    INVALID_INPUT = "invalid_input"
    UNREACHABLE = "unreachable"
    ORIENTATION_MISMATCH = "orientation_mismatch"
    JOINT_LIMIT = "joint_limit"


@dataclass(frozen=True)
class IKSolution:
    positions: Tuple[float, float, float, float, float]
    position_error: float
    orientation_error: float
    seed_distance: float


@dataclass(frozen=True)
class IKResult:
    status: IKStatus
    message: str
    solutions: Tuple[IKSolution, ...] = ()
    selected: Optional[IKSolution] = None

    @property
    def success(self) -> bool:
        return self.status is IKStatus.OK


class RoArmM3Kinematics:
    """Forward, inverse and differential kinematics for one RoArm-M3."""

    def __init__(self, parameters: ArmParameters):
        self.parameters = parameters

    @staticmethod
    def _joint_vector(values: Iterable[float]) -> np.ndarray:
        result = np.asarray(tuple(values), dtype=float)
        if result.shape != (5,) or not np.all(np.isfinite(result)):
            raise ValueError("Expected five finite joint positions")
        return result

    def within_limits(self, positions: Iterable[float], tolerance: float = 1.0e-9) -> bool:
        q = self._joint_vector(positions)
        return bool(
            np.all(q >= np.asarray(self.parameters.lower) - tolerance)
            and np.all(q <= np.asarray(self.parameters.upper) + tolerance)
        )

    def forward(self, positions: Iterable[float]) -> np.ndarray:
        q = self._joint_vector(positions)
        p = self.parameters
        transform = _transform(_rotation_z(q[0]))
        transform = transform @ _transform(translation=(0.0, 0.0, p.base_to_shoulder))
        transform = transform @ _transform(_rotation_y(q[1]))
        transform = transform @ _transform(
            translation=(p.shoulder_to_elbow_x, 0.0, p.shoulder_to_elbow_z)
        )
        transform = transform @ _transform(_rotation_y(q[2]))
        transform = transform @ _transform(
            translation=(p.elbow_to_wrist_x, 0.0, p.elbow_to_wrist_z)
        )
        transform = transform @ _transform(_rotation_y(q[3] + p.tool_mount_angle))
        transform = transform @ _transform(_rotation_z(q[4]))
        return transform @ _transform(translation=(0.0, 0.0, p.tool_length))

    def task_pose(self, positions: Iterable[float]) -> TaskPose:
        q = self._joint_vector(positions)
        pose = self.forward(q)
        return TaskPose(
            x=float(pose[0, 3]),
            y=float(pose[1, 3]),
            z=float(pose[2, 3]),
            pitch=float(q[1] + q[2] + q[3] - np.pi / 2.0),
            roll=float(q[4]),
        )

    def pose_from_task(self, target: TaskPose, base_angle: Optional[float] = None) -> np.ndarray:
        if base_angle is None:
            base_angle = float(np.arctan2(target.y, target.x))
        beta = target.pitch + np.pi / 2.0 + self.parameters.tool_mount_angle
        result = _transform(
            _rotation_z(base_angle) @ _rotation_y(beta) @ _rotation_z(target.roll),
            (target.x, target.y, target.z),
        )
        return result

    def jacobian(self, positions: Iterable[float]) -> np.ndarray:
        q = self._joint_vector(positions)
        p = self.parameters
        origins = []
        axes = []
        transform = np.eye(4)

        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.array((0.0, 0.0, 1.0)))
        transform = transform @ _transform(_rotation_z(q[0]))
        transform = transform @ _transform(translation=(0.0, 0.0, p.base_to_shoulder))

        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.array((0.0, 1.0, 0.0)))
        transform = transform @ _transform(_rotation_y(q[1]))
        transform = transform @ _transform(
            translation=(p.shoulder_to_elbow_x, 0.0, p.shoulder_to_elbow_z)
        )

        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.array((0.0, 1.0, 0.0)))
        transform = transform @ _transform(_rotation_y(q[2]))
        transform = transform @ _transform(
            translation=(p.elbow_to_wrist_x, 0.0, p.elbow_to_wrist_z)
        )

        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.array((0.0, 1.0, 0.0)))
        transform = transform @ _transform(_rotation_y(q[3] + p.tool_mount_angle))

        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.array((0.0, 0.0, 1.0)))
        transform = transform @ _transform(_rotation_z(q[4]))
        tool = (transform @ _transform(translation=(0.0, 0.0, p.tool_length)))[:3, 3]

        jacobian = np.zeros((6, 5))
        for index, (origin, axis) in enumerate(zip(origins, axes)):
            jacobian[:3, index] = np.cross(axis, tool - origin)
            jacobian[3:, index] = axis
        return jacobian

    def inverse_task(self, target: TaskPose, seed: Optional[Iterable[float]] = None) -> IKResult:
        """Solve firmware XYZ/pitch/roll, considering both signed radial planes."""
        try:
            seed_q = self._joint_vector(self.parameters.home if seed is None else seed)
            values = (target.x, target.y, target.z, target.pitch, target.roll)
            if not np.all(np.isfinite(values)):
                raise ValueError("Task pose must contain five finite values")
        except (TypeError, ValueError, AttributeError) as error:
            return IKResult(IKStatus.INVALID_INPUT, str(error))
        if np.hypot(target.x, target.y) <= _EPS:
            bases = (seed_q[0],)
        else:
            azimuth = float(np.arctan2(target.y, target.x))
            bases = (azimuth, _wrap_pi(azimuth + np.pi))
        results = [self.inverse(self.pose_from_task(target, base), seed_q) for base in bases]
        solutions = []
        for result in results:
            for solution in result.solutions:
                computed = self.task_pose(solution.positions)
                if max(abs(_wrap_pi(computed.pitch - target.pitch)),
                       abs(_wrap_pi(computed.roll - target.roll))) > _ORIENTATION_TOLERANCE:
                    continue
                if not any(np.allclose(solution.positions, other.positions, atol=1e-9, rtol=0)
                           for other in solutions):
                    solutions.append(solution)
        if solutions:
            ordered = tuple(sorted(solutions, key=lambda item: item.seed_distance))
            return IKResult(IKStatus.OK, "Task IK solution found", ordered, ordered[0])
        if any(result.status in (IKStatus.OK, IKStatus.JOINT_LIMIT) for result in results):
            return IKResult(IKStatus.JOINT_LIMIT, "No joint-limit-valid solution for requested task angles")
        return results[0]

    def inverse(self, target: np.ndarray, seed: Optional[Iterable[float]] = None) -> IKResult:
        try:
            pose = np.asarray(target, dtype=float)
        except (TypeError, ValueError) as error:
            return IKResult(IKStatus.INVALID_INPUT, str(error))
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            return IKResult(IKStatus.INVALID_INPUT, "Target must be a finite 4x4 transform")
        if not np.allclose(pose[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-9):
            return IKResult(IKStatus.INVALID_INPUT, "Target has an invalid homogeneous row")
        rotation = pose[:3, :3]
        if (
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-7)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-7)
        ):
            return IKResult(IKStatus.INVALID_INPUT, "Target orientation is not a rotation matrix")

        try:
            seed_q = self._joint_vector(self.parameters.home if seed is None else seed)
        except (TypeError, ValueError) as error:
            return IKResult(IKStatus.INVALID_INPUT, str(error))

        x, y, z = pose[:3, 3]
        radius = float(np.hypot(x, y))
        if radius <= _EPS:
            base_candidates = [(float(seed_q[0]), 0.0)]
        else:
            azimuth = float(np.arctan2(y, x))
            base_candidates = [(azimuth, radius), (_wrap_pi(azimuth + np.pi), -radius)]

        p = self.parameters
        l2 = p.shoulder_link_length
        l3 = p.forearm_length
        orientation_compatible = False
        geometry_solutions = []

        for base, signed_radius in base_candidates:
            local_rotation = _rotation_z(-base) @ rotation
            beta = float(np.arctan2(local_rotation[0, 2], local_rotation[2, 2]))
            roll = _wrap_pi(float(np.arctan2(local_rotation[1, 0], local_rotation[1, 1])))
            reconstructed = _rotation_z(base) @ _rotation_y(beta) @ _rotation_z(roll)
            orientation_error = _rotation_error(reconstructed, rotation)
            if orientation_error > _ORIENTATION_TOLERANCE:
                continue
            orientation_compatible = True

            wrist_r = signed_radius - np.sin(beta) * p.tool_length
            wrist_z = z - p.base_to_shoulder - np.cos(beta) * p.tool_length
            distance_squared = wrist_r * wrist_r + wrist_z * wrist_z
            cosine_delta = (distance_squared - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
            if cosine_delta < -1.0 - 1.0e-9 or cosine_delta > 1.0 + 1.0e-9:
                continue
            cosine_delta = float(np.clip(cosine_delta, -1.0, 1.0))
            alpha = float(np.arctan2(wrist_r, wrist_z))
            delta_magnitude = float(np.arccos(cosine_delta))
            deltas = (delta_magnitude,) if delta_magnitude < 1.0e-10 else (delta_magnitude, -delta_magnitude)
            for delta in deltas:
                theta2 = alpha - np.arctan2(l3 * np.sin(delta), l2 + l3 * np.cos(delta))
                shoulder = theta2 - p.shoulder_link_angle
                elbow = delta + p.shoulder_link_angle - p.forearm_angle
                phi = beta - p.tool_mount_angle
                wrist = phi - shoulder - elbow
                candidate = np.array((base, shoulder, elbow, wrist, roll), dtype=float)
                geometry_solutions.append(candidate)

        if not orientation_compatible:
            return IKResult(
                IKStatus.ORIENTATION_MISMATCH,
                "Target orientation is outside the RoArm-M3 five-axis orientation manifold",
            )
        if not geometry_solutions:
            return IKResult(IKStatus.UNREACHABLE, "Target position is outside the arm workspace")

        valid = []
        # atan2 returns principal angles. Enumerate each joint's equivalent
        # turns before applying limits; cumulative wrist pitch may exceed pi.
        candidates = []
        for geometric in geometry_solutions:
            equivalents = []
            for angle, lower, upper in zip(geometric, p.lower, p.upper):
                first = int(np.ceil((lower - angle - 1e-9) / (2 * np.pi)))
                last = int(np.floor((upper - angle + 1e-9) / (2 * np.pi)))
                equivalents.append([
                    float(np.clip(angle + turn * 2 * np.pi, lower, upper))
                    for turn in range(first, last + 1)
                ])
            candidates.extend(np.asarray(values) for values in product(*equivalents))
        for candidate in candidates:
            if any(np.allclose(candidate, item.positions, atol=1.0e-8, rtol=0) for item in valid):
                continue
            # These are bounded revolute joints, not continuous joints. Use
            # actual coordinate travel so +/-pi does not induce a full-turn jump.
            differences = candidate - seed_q
            computed = self.forward(candidate)
            valid.append(
                IKSolution(
                    positions=tuple(float(value) for value in candidate),
                    position_error=float(np.linalg.norm(computed[:3, 3] - pose[:3, 3])),
                    orientation_error=_rotation_error(computed[:3, :3], rotation),
                    seed_distance=float(np.linalg.norm(differences)),
                )
            )

        if not valid:
            return IKResult(
                IKStatus.JOINT_LIMIT,
                "Geometric inverse solutions exist, but all violate configured joint limits",
            )
        solutions = tuple(sorted(valid, key=lambda solution: solution.seed_distance))
        return IKResult(IKStatus.OK, "IK solution found", solutions, solutions[0])


def quaternion_from_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Return a normalized ROS-order quaternion (x, y, z, w)."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape == (4, 4):
        matrix = matrix[:3, :3]
    if matrix.shape != (3, 3):
        raise ValueError("Expected a 3x3 or 4x4 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            x, y, z, w = 0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            x, y, z, w = (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            x, y, z, w = (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale, (matrix[1, 0] - matrix[0, 1]) / scale
    quaternion = np.asarray((x, y, z, w), dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def matrix_from_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    """Return a rotation matrix from a ROS-order quaternion (x, y, z, w)."""
    q = np.asarray(tuple(quaternion), dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.max(np.abs(q)) <= _EPS:
        raise ValueError("Quaternion must contain four finite values and have nonzero norm")
    q = q / np.max(np.abs(q))
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        )
    )
