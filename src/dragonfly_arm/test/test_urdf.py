"""Physical dimensions, FK and control-contract regression checks."""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from dragonfly_arm_kinematics import ArmParameters, RoArmM3Kinematics

PACKAGE = Path(__file__).parents[1]
PARAMETERS = ArmParameters.from_yaml(PACKAGE / "config/model_dimensions.yaml")
KINEMATICS = RoArmM3Kinematics(PARAMETERS)
ARMS = {
    side: [f"{side}_joint_{i}" for i in range(1, 6)]
    for side in ("left", "right")
}
ARM = ARMS["left"] + ARMS["right"]
TILT = [f"unit_{u}_{a}_joint" for u in "abc" for a in ("outer", "inner")]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


def fk(robot, positions=None):
    positions = positions or {}
    children = {j.find("child").get("link") for j in robot.findall("joint")}
    roots = {l.get("name") for l in robot.findall("link")} - children
    assert roots == {"world"}
    poses = {"world": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in poses:
                continue
            assert child not in poses, "Multiple parents in URDF"
            transform = origin(joint.find("origin"))
            if joint.get("type") == "revolute":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                transform[:3, :3] = transform[:3, :3] @ rotation(axis, positions.get(joint.get("name"), 0))
            poses[child] = poses[parent] @ transform
            pending.remove(joint)
            progressed = True
        assert progressed, "Disconnected/cyclic URDF"
    return poses


@pytest.fixture(scope="module")
def robot():
    result = subprocess.run(
        ["xacro", str(PACKAGE / "urdf/dragonfly_arm.urdf.xacro")],
        check=True, capture_output=True, text=True,
    )
    return ET.fromstring(result.stdout)


def test_tree_and_controller_contract(robot):
    poses = fk(robot)
    assert len(poses) == len(robot.findall("link"))
    link_names = [link.get("name") for link in robot.findall("link")]
    joint_names = [joint.get("name") for joint in robot.findall("joint")]
    assert len(link_names) == len(set(link_names))
    assert len(joint_names) == len(set(joint_names))
    joints = {j.get("name"): j for j in robot.findall("joint")}
    active = {name for name, j in joints.items() if j.get("type") == "revolute"}
    assert active == set(ARM + TILT)
    assert not any(name.endswith("joint_6") for name in joints)
    expected_axes = ["0 0 1", "0 1 0", "0 1 0", "0 1 0", "0 0 1"]
    for names in ARMS.values():
        assert [joints[name].find("axis").get("xyz") for name in names] == expected_axes
    controls = {j.get("name"): j for j in robot.findall("ros2_control/joint")}
    assert set(controls) == active
    assert robot.find("ros2_control/hardware/plugin").text == "gz_ros2_control/GazeboSimSystem"
    simulation_plugins = {
        plugin.get("filename"): plugin
        for plugin in robot.findall("gazebo/plugin")
    }
    assert "gz_ros2_control-system" in simulation_plugins
    assert simulation_plugins["gz_ros2_control-system"].get("name") == (
        "gz_ros2_control::GazeboSimROS2ControlPlugin"
    )
    config = yaml.safe_load((PACKAGE / "config/controllers.yaml").read_text())
    assert "arm_controller" not in config
    assert config["left_arm_controller"]["ros__parameters"]["joints"] == ARMS["left"]
    assert config["right_arm_controller"]["ros__parameters"]["joints"] == ARMS["right"]
    assert config["tilt_controller"]["ros__parameters"]["joints"] == TILT
    for name in active:
        limit = joints[name].find("limit")
        params = {p.get("name"): float(p.text) for p in controls[name].findall("command_interface/param")}
        assert float(limit.get("lower")) == params["min"]
        assert float(limit.get("upper")) == params["max"]
        assert [e.get("name") for e in controls[name].findall("state_interface")] == ["position", "velocity"]


def test_owner_dimensions_and_arm_mount(robot):
    arm_home = {
        name: value
        for names in ARMS.values()
        for name, value in zip(names, PARAMETERS.home)
    }
    poses = fk(robot, arm_home)
    centres = [poses[f"unit_{u}_mount_link"][:3, 3] for u in "abc"]
    for i in range(3):
        assert np.linalg.norm(centres[i] - centres[(i+1) % 3]) == pytest.approx(1.2)
    base = poses["uav_base_link"]
    assert base[2, 3] == pytest.approx(1.6)
    plate = robot.find("./link[@name='dual_arm_mount_plate']")
    plate_size = np.fromstring(
        plate.find("visual/geometry/box").get("size"), sep=" "
    )
    assert plate_size == pytest.approx([0.14, 0.40, 0.012])
    plate_in_body = np.linalg.inv(base) @ poses["dual_arm_mount_plate"]
    assert plate_in_body[:3, 3] == pytest.approx([0, 0, -0.026])
    arm_bases = {}
    for side, y in (("left", 0.15), ("right", -0.15)):
        arm_in_body = np.linalg.inv(base) @ poses[f"{side}_base_link"]
        arm_bases[side] = arm_in_body[:3, 3]
        assert arm_in_body[:3, 3] == pytest.approx([0, y, -0.05])
        # The complete 0.06 m arm-base footprint is covered by the common
        # under-deck plate, not floating beside the 0.20 m avionics deck.
        assert abs(y) + 0.060 / 2 <= plate_size[1] / 2
        adapter_in_body = np.linalg.inv(base) @ poses[f"{side}_arm_mount_adapter"]
        adapter = robot.find(f"./link[@name='{side}_arm_mount_adapter']")
        adapter_height = float(adapter.find("visual/geometry/box").get("size").split()[2])
        assert adapter_in_body[2, 3] + adapter_height / 2 == pytest.approx(-0.032)
        assert adapter_in_body[2, 3] - adapter_height / 2 == pytest.approx(-0.050)
        assert arm_in_body[:3, 2] == pytest.approx([0, 0, -1], abs=1e-12)
        tip_in_arm = np.linalg.inv(poses[f"{side}_base_link"]) @ poses[f"{side}_tool0"]
        assert tip_in_arm == pytest.approx(KINEMATICS.forward(PARAMETERS.home))
        assert tip_in_arm[:3, 3] == pytest.approx([0.34616, 0, 0.34959])
        assert poses[f"{side}_tool0"][2, 3] > poses["left_skid"][2, 3] + 0.05
    assert np.linalg.norm(arm_bases["left"] - arm_bases["right"]) == pytest.approx(0.30)
    assert (arm_bases["left"] + arm_bases["right"]) / 2 == pytest.approx([0, 0, -0.05])


@pytest.mark.parametrize("side", ["left", "right"])
def test_five_axis_forward_kinematics(robot, side):
    q = [0.25, 0.15, 1.25, 0.10, 0.30]
    poses = fk(robot, dict(zip(ARMS[side], q)))
    p = np.linalg.inv(poses[f"{side}_base_link"]) @ poses[f"{side}_tool0"]
    assert p == pytest.approx(KINEMATICS.forward(q), abs=1e-10)


def test_matching_arm_kinematics(robot):
    q = [0.20, -0.10, 1.25, -0.15, 0.30]
    positions = {
        name: value
        for names in ARMS.values()
        for name, value in zip(names, q)
    }
    poses = fk(robot, positions)
    left = np.linalg.inv(poses["left_base_link"]) @ poses["left_tool0"]
    right = np.linalg.inv(poses["right_base_link"]) @ poses["right_tool0"]
    assert left == pytest.approx(right)


def test_random_urdf_fk_matches_model(robot):
    rng = np.random.default_rng(182)
    for q in rng.uniform(PARAMETERS.lower, PARAMETERS.upper, (60, 5)):
        positions = {name: value for names in ARMS.values() for name, value in zip(names, q)}
        poses = fk(robot, positions)
        for side in ARMS:
            local = np.linalg.inv(poses[f"{side}_base_link"]) @ poses[f"{side}_tool0"]
            assert local == pytest.approx(KINEMATICS.forward(q), abs=1e-10)


@pytest.mark.parametrize("outer,inner", [(0, 0), (0.3, 0), (0, -0.3), (0.3, -0.2)])
def test_tilt_order_motor_spacing_and_thrust_axes(robot, outer, inner):
    positions = {name: outer if "outer" in name else inner for name in TILT}
    poses = fk(robot, positions)
    links = {l.get("name"): l for l in robot.findall("link")}
    for unit in "abc":
        mount = poses[f"unit_{unit}_mount_link"]
        rotor_poses = [poses[f"unit_{unit}_rotor_{s}_link"] for s in ("left", "right")]
        assert np.linalg.norm(rotor_poses[0][:3, 3] - rotor_poses[1][:3, 3]) == pytest.approx(0.36)
        # R_x(outer) R_y(inner) +Z, expressed in the fixed mount frame.
        expected_z = [math.sin(inner), -math.sin(outer)*math.cos(inner), math.cos(outer)*math.cos(inner)]
        for side, pose in zip(("left", "right"), rotor_poses):
            relative = np.linalg.inv(mount) @ pose
            assert relative[:3, 2] == pytest.approx(expected_z)
            rotor = links[f"unit_{unit}_rotor_{side}_link"]
            assert len(rotor.findall("visual")) == 3
            assert float(rotor.find("collision/geometry/cylinder").get("radius")) == pytest.approx(0.116)
    if outer == 0 and inner == 0:
        all_rotors = [poses[f"unit_{u}_rotor_{s}_link"][:3, 3] for u in "abc" for s in ("left", "right")]
        for i, p in enumerate(all_rotors):
            for other in all_rotors[i+1:]:
                assert np.linalg.norm(p-other) > 2*0.116


def test_finite_positive_inertias(robot):
    for link in robot.findall("link"):
        if link.get("name") in ("world", "left_tool0", "right_tool0"):
            continue
        assert link.find("visual") is not None
        assert link.find("collision") is not None
        inertial = link.find("inertial")
        assert float(inertial.find("mass").get("value")) > 0
        i = {k: float(v) for k, v in inertial.find("inertia").attrib.items()}
        matrix = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]], [i["ixz"], i["iyz"], i["izz"]]])
        eigenvalues = np.linalg.eigvalsh(matrix)
        assert np.all(np.isfinite(eigenvalues))
        assert np.all(eigenvalues > 0)
        assert eigenvalues[-1] <= sum(eigenvalues[:-1]) + 1e-12


def test_drawing_geometry_mass_and_peak_efforts(robot):
    total_mass = sum(float(link.find('inertial/mass').get('value'))
                     for link in robot.findall('link') if link.find('inertial') is not None)
    assert total_mass == pytest.approx(5.415, abs=1e-12, rel=0)
    parts = ('base', 'column', 'upper_arm', 'forearm', 'wrist', 'gripper')
    weights = np.asarray([10, 8, 14, 10, 6, 10])
    for side in ARMS:
        links = [robot.find(f"link[@name='{side}_{part}_link']") for part in parts]
        masses = [float(link.find('inertial/mass').get('value')) for link in links]
        assert sum(masses) == pytest.approx(0.9735, abs=1e-12, rel=0)
        assert masses == pytest.approx(0.9735 * weights / weights.sum())
        base_box = links[0].find('visual/geometry/box')
        assert float(base_box.get('size').split()[2]) == pytest.approx(0.003)
        upper_rails = links[2].findall('visual')[:2]
        bounds = [abs(float(v.find('origin').get('xyz').split()[1]))
                  + float(v.find('geometry/box').get('size').split()[1]) / 2 for v in upper_rails]
        assert 2 * max(bounds) == pytest.approx(0.0378)
        column_sizes = [np.fromstring(v.find('geometry/box').get('size'), sep=' ')
                        for v in links[1].findall('visual')]
        assert sum(np.allclose(size, [0.035, 0.02472, 0.04522]) for size in column_sizes) == 2
        assert sum(np.isclose(size[2], 0.05346) for size in column_sizes) == 2
        efforts = [float(robot.find(f"joint[@name='{name}']/limit").get('effort')) for name in ARMS[side]]
        assert efforts == pytest.approx([2.941995, 5.883990, 2.941995, 2.941995, 2.941995])
    assert not any('table_clamp' in link.get('name') for link in robot.findall('link'))
