"""Analytic floating-base dynamics for the Dragonfly body and two arms.

The implementation is deliberately independent of ROS.  It uses one URDF as
its geometry/inertia source, keeps the ten arm joints active and locks every
other joint.  No numerical differentiation is used at runtime.

Conventions:
    world: ENU; body: FLU; wrench order: force then torque.
    base_velocity: v in body coordinates, where R.T @ world_position_dot = v.
    desired_base_acceleration: [v_dot, omega_dot], not physical acceleration.
    torque reference point: the base-link origin O.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


ARM_JOINT_NAMES = tuple(
    f"{side}_joint_{index}" for side in ("left", "right") for index in range(1, 6)
)
ARM_ROOT_JOINTS = {
    "left": "left_uav_to_arm_base",
    "right": "right_uav_to_arm_base",
}


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def _vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _origin(node):
    transform = np.eye(4)
    if node is not None:
        transform[:3, 3] = np.fromstring(node.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(node.get("rpy", "0 0 0"), sep=" ")
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return transform


def _translation(vector):
    transform = np.eye(4)
    transform[:3, 3] = vector
    return transform


@dataclass(frozen=True)
class _Inertial:
    origin: np.ndarray
    mass: float
    inertia: np.ndarray


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    active_index: int | None


@dataclass
class _BodyState:
    name: str
    mass: float
    position: np.ndarray
    inertia: np.ndarray
    jv: np.ndarray
    jw: np.ndarray
    dot_jv_dq: np.ndarray
    dot_jw_dq: np.ndarray
    arm: str | None


@dataclass(frozen=True)
class DynamicsResult:
    """Base dynamics and diagnostic arm reactions for one aligned state."""

    mass_matrix: np.ndarray
    m_bb: np.ndarray
    m_ba: np.ndarray
    h_b: np.ndarray
    wrench: np.ndarray
    left_arm_wrench: np.ndarray
    right_arm_wrench: np.ndarray

    def wrenches(self):
        return self.left_arm_wrench, self.right_arm_wrench


class CoupledDynamics:
    """URDF-driven 16-DOF model: six base velocities plus ten arm joints."""

    def __init__(self, root, base_link="uav_base_link", active_joint_names=ARM_JOINT_NAMES,
                 locked_positions=None, arm_root_joints=None):
        self.base_link = base_link
        self.active_joint_names = tuple(active_joint_names)
        self.locked_positions = dict(locked_positions or {})
        self.arm_root_joints = dict(arm_root_joints or ARM_ROOT_JOINTS)
        active_lookup = {name: index for index, name in enumerate(self.active_joint_names)}
        if len(active_lookup) != len(self.active_joint_names):
            raise ValueError("Active joint names must be unique")

        self._links = {link.get("name"): link for link in root.findall("link")}
        joints = []
        for node in root.findall("joint"):
            kind = node.get("type")
            name = node.get("name")
            axis_node = node.find("axis")
            axis = (np.fromstring(axis_node.get("xyz", "0 0 0"), sep=" ")
                    if axis_node is not None else np.zeros(3))
            if kind in ("revolute", "continuous", "prismatic"):
                norm = np.linalg.norm(axis)
                if norm <= 0:
                    raise ValueError(f"Joint {name} has no valid axis")
                axis = axis / norm
            joints.append(_Joint(
                name=name,
                kind=kind,
                parent=node.find("parent").get("link"),
                child=node.find("child").get("link"),
                origin=_origin(node.find("origin")),
                axis=axis,
                active_index=active_lookup.get(name),
            ))

        self._children = {}
        child_links = set()
        for joint in joints:
            self._children.setdefault(joint.parent, []).append(joint)
            child_links.add(joint.child)
        if base_link not in self._links:
            raise ValueError(f"Missing base link {base_link}")
        if base_link in child_links:
            # A world-fixed parent is harmless, but dynamics starts at base_link.
            pass
        self._joints = []

        def visit(link):
            for joint in self._children.get(link, []):
                self._joints.append(joint)
                visit(joint.child)

        visit(base_link)
        present = {joint.name for joint in self._joints}
        missing = set(self.active_joint_names) - present
        if missing:
            raise ValueError(f"Active joints missing below {base_link}: {sorted(missing)}")
        self._inertials = {}
        for name in [base_link] + [joint.child for joint in self._joints]:
            node = self._links[name].find("inertial")
            if node is None:
                continue
            mass = float(node.find("mass").get("value"))
            values = node.find("inertia").attrib
            inertia = np.array([
                [float(values["i" + a + b] if "i" + a + b in values else values["i" + b + a])
                 for b in "xyz"]
                for a in "xyz"
            ])
            if not np.isfinite(mass) or mass < 0 or not np.all(np.isfinite(inertia)):
                raise ValueError(f"Invalid inertial data for {name}")
            if not np.allclose(inertia, inertia.T, atol=1e-12):
                raise ValueError(f"Inertia is not symmetric for {name}")
            self._inertials[name] = _Inertial(_origin(node.find("origin")), mass, inertia)

        self._arm_links = {}
        for side, root_joint_name in self.arm_root_joints.items():
            root_joint = next((joint for joint in self._joints if joint.name == root_joint_name), None)
            if root_joint is None:
                raise ValueError(f"Missing arm root joint {root_joint_name}")
            links = {root_joint.child}

            def add_descendants(link):
                for joint in self._children.get(link, []):
                    links.add(joint.child)
                    add_descendants(joint.child)

            add_descendants(root_joint.child)
            self._arm_links[side] = links

    @classmethod
    def from_urdf(cls, source, **kwargs):
        """Load from a URDF path, XML string or ElementTree root."""
        is_file = False
        if isinstance(source, (str, Path)):
            try:
                is_file = Path(source).is_file()
            except OSError:
                # Long XML strings are not valid filesystem paths on Linux.
                is_file = False
        if is_file:
            root = ET.parse(source).getroot()
        elif isinstance(source, str):
            root = ET.fromstring(source)
        else:
            root = source
        return cls(root, **kwargs)

    @classmethod
    def from_xacro(cls, path, **kwargs):
        """Expand Xacro with the system executable, then load the URDF."""
        import subprocess
        xml = subprocess.check_output(["xacro", str(path)], text=True)
        return cls.from_urdf(xml, **kwargs)

    def _motion(self, joint, angle):
        if joint.kind in ("revolute", "continuous"):
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(joint.axis * angle).as_matrix()
            return motion
        if joint.kind == "prismatic":
            return _translation(joint.axis * angle)
        return np.eye(4)

    def _kinematics(self, joint_position):
        q = _vector(joint_position, len(self.active_joint_names), "joint_position")
        transforms = {self.base_link: np.eye(4)}
        ancestors = {self.base_link: []}
        joint_axes = {}
        joint_origins = {}
        joint_kinds = {}
        for joint in self._joints:
            joint_frame = transforms[joint.parent] @ joint.origin
            index = joint.active_index
            angle = q[index] if index is not None else self.locked_positions.get(joint.name, 0.0)
            if joint.kind in ("revolute", "continuous", "prismatic"):
                joint_axes[joint.name] = joint_frame[:3, :3] @ joint.axis
                joint_origins[joint.name] = joint_frame[:3, 3].copy()
                joint_kinds[joint.name] = joint.kind
            transforms[joint.child] = joint_frame @ self._motion(joint, angle)
            ancestors[joint.child] = ancestors[joint.parent] + ([joint.name] if index is not None else [])

        bodies = []
        for link, inertial in self._inertials.items():
            transform = transforms[link] @ inertial.origin
            position = transform[:3, 3]
            inertia = transform[:3, :3] @ inertial.inertia @ transform[:3, :3].T
            active = ancestors[link]
            count = len(self.active_joint_names)
            jv = np.zeros((3, count))
            jw = np.zeros((3, count))
            columns = {}
            for name in active:
                joint = next(item for item in self._joints if item.name == name)
                index = joint.active_index
                axis = joint_axes[name]
                if joint.kind == "prismatic":
                    linear, angular = axis, np.zeros(3)
                else:
                    linear = np.cross(axis, position - joint_origins[name])
                    angular = axis
                jv[:, index] = linear
                jw[:, index] = angular
                columns[name] = (linear, angular)

            dot_jv_dq = np.zeros(3)
            dot_jw_dq = np.zeros(3)
            position_dot = jv @ q * 0.0  # allocated here; filled after dq is supplied
            bodies.append(_BodyState(
                name=link, mass=inertial.mass, position=position, inertia=inertia,
                jv=jv, jw=jw, dot_jv_dq=dot_jv_dq, dot_jw_dq=dot_jw_dq,
                arm=next((side for side, links in self._arm_links.items() if link in links), None),
            ))
            # Store joint path data without exposing it in the public result.
            bodies[-1].active = active
            bodies[-1].columns = columns
        return bodies, joint_axes, joint_origins, joint_kinds

    @staticmethod
    def _body_wrench(body, base_rotation, base_velocity, omega, dot_v, dot_omega,
                     dq, ddq, gravity_body):
        relative_v = body.jv @ dq
        relative_omega = body.jw @ dq
        omega_i = omega + relative_omega
        acceleration = (
            dot_v + np.cross(omega, base_velocity)
            + np.cross(dot_omega, body.position)
            + np.cross(omega, np.cross(omega, body.position))
            + 2.0 * np.cross(omega, relative_v)
            + body.jv @ ddq + body.dot_jv_dq
        )
        alpha = dot_omega + body.jw @ ddq + body.dot_jw_dq + np.cross(omega, relative_omega)
        force = body.mass * (acceleration - gravity_body)
        torque = body.inertia @ alpha + np.cross(omega_i, body.inertia @ omega_i)
        return np.r_[force, np.cross(body.position, force) + torque]

    def evaluate(self, base_rotation, base_velocity, base_angular_velocity,
                 joint_position, joint_velocity, joint_acceleration,
                 desired_base_acceleration=None, gravity_world=(0.0, 0.0, -9.81)):
        """Evaluate M, h and required base wrench for one state.

        The returned arm wrenches are arm-on-base reactions.  They are diagnostic
        outputs and must not be added again to the complete coupled wrench.
        """
        rotation = np.asarray(base_rotation, dtype=float)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("base_rotation must be a finite 3x3 matrix")
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-8) or np.linalg.det(rotation) <= 0:
            raise ValueError("base_rotation must be a proper rotation matrix")
        v = _vector(base_velocity, 3, "base_velocity")
        omega = _vector(base_angular_velocity, 3, "base_angular_velocity")
        q = _vector(joint_position, len(self.active_joint_names), "joint_position")
        dq = _vector(joint_velocity, len(self.active_joint_names), "joint_velocity")
        ddq = _vector(joint_acceleration, len(self.active_joint_names), "joint_acceleration")
        acceleration = (np.zeros(6) if desired_base_acceleration is None else
                        _vector(desired_base_acceleration, 6, "desired_base_acceleration"))
        gravity_world = _vector(gravity_world, 3, "gravity_world")
        gravity_body = rotation.T @ gravity_world

        bodies, axes, origins, kinds = self._kinematics(q)
        for body in bodies:
            active = body.active
            columns = body.columns
            position_dot = body.jv @ dq
            for name in active:
                joint = next(item for item in self._joints if item.name == name)
                index = joint.active_index
                axis = axes[name]
                before = active[:active.index(name) + 1]
                axis_dot = np.zeros(3)
                for ancestor in before:
                    ancestor_joint = next(item for item in self._joints if item.name == ancestor)
                    if ancestor_joint.kind == "prismatic":
                        continue
                    axis_dot += dq[ancestor_joint.active_index] * np.cross(axes[ancestor], axis)
                origin_dot = np.zeros(3)
                for ancestor in active[:active.index(name)]:
                    ancestor_joint = next(item for item in self._joints if item.name == ancestor)
                    if ancestor_joint.kind == "prismatic":
                        origin_dot += dq[ancestor_joint.active_index] * axes[ancestor]
                    else:
                        origin_dot += dq[ancestor_joint.active_index] * np.cross(
                            axes[ancestor], origins[name] - origins[ancestor])
                if joint.kind == "prismatic":
                    derivative_linear = axis_dot
                    derivative_angular = np.zeros(3)
                else:
                    derivative_linear = (np.cross(axis_dot, body.position - origins[name])
                                         + np.cross(axis, position_dot - origin_dot))
                    derivative_angular = axis_dot
                body.dot_jv_dq += dq[index] * derivative_linear
                body.dot_jw_dq += dq[index] * derivative_angular

        count = len(self.active_joint_names)
        mass_matrix = np.zeros((6 + count, 6 + count))
        for body in bodies:
            a = np.hstack((np.eye(3), -skew(body.position), body.jv))
            b = np.hstack((np.zeros((3, 3)), np.eye(3), body.jw))
            mass_matrix += body.mass * a.T @ a + b.T @ body.inertia @ b

        zero6 = np.zeros(6)
        zeroq = np.zeros(count)
        h_b = sum((self._body_wrench(body, rotation, v, omega, zero6[:3], zero6[3:],
                                     dq, zeroq, gravity_body) for body in bodies), zero6)
        wrench = sum((self._body_wrench(body, rotation, v, omega, acceleration[:3],
                                        acceleration[3:], dq, ddq, gravity_body)
                      for body in bodies), zero6)
        arm_wrenches = {}
        for side in self.arm_root_joints:
            required = sum((self._body_wrench(body, rotation, v, omega, acceleration[:3],
                                              acceleration[3:], dq, ddq, gravity_body)
                            for body in bodies if body.arm == side), zero6)
            arm_wrenches[side] = -required
        return DynamicsResult(
            mass_matrix=mass_matrix,
            m_bb=mass_matrix[:6, :6],
            m_ba=mass_matrix[:6, 6:],
            h_b=h_b,
            wrench=wrench,
            left_arm_wrench=arm_wrenches.get("left", np.zeros(6)),
            right_arm_wrench=arm_wrenches.get("right", np.zeros(6)),
        )
