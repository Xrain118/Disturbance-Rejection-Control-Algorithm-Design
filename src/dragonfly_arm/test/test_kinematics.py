"""Numerical and analytic regression tests for RoArm-M3 kinematics."""

from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest

from dragonfly_arm_kinematics import (
    ArmParameters,
    IKStatus,
    RoArmM3Kinematics,
    TaskPose,
    matrix_from_quaternion,
    quaternion_from_matrix,
)


PACKAGE = Path(__file__).parents[1]
PARAMETERS = ArmParameters.from_yaml(PACKAGE / "config/model_dimensions.yaml")
KINEMATICS = RoArmM3Kinematics(PARAMETERS)


def rotation_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


def test_source_parameters_and_home_pose():
    assert PARAMETERS.base_to_shoulder == pytest.approx(0.12606)
    assert (PARAMETERS.shoulder_to_elbow_x, PARAMETERS.shoulder_to_elbow_z) == pytest.approx((0.03, 0.23682))
    assert (PARAMETERS.elbow_to_wrist_x, PARAMETERS.elbow_to_wrist_z) == pytest.approx((0.0, 0.14449))
    assert (PARAMETERS.wrist_to_tool_x, PARAMETERS.wrist_to_tool_z) == pytest.approx((0.01329, 0.17167))
    assert PARAMETERS.home == pytest.approx((0.0, 0.0, np.pi / 2.0, 0.0, 0.0))
    assert PARAMETERS.lower == pytest.approx((-np.pi, -np.pi / 2.0, 0.0, -np.pi / 2.0, -np.pi))
    assert PARAMETERS.upper == pytest.approx((np.pi, np.pi / 2.0, np.pi, np.pi / 2.0, np.pi))
    pose = KINEMATICS.forward(PARAMETERS.home)
    assert pose[:3, 3] == pytest.approx((0.34616, 0.0, 0.34959), abs=1.0e-12)
    assert KINEMATICS.task_pose(PARAMETERS.home) == TaskPose(0.34616, 0.0, 0.34959, 0.0, 0.0)


@pytest.mark.parametrize(
    "positions",
    (
        (0.25, 0.15, 1.25, 0.10, 0.30),
        (-0.40, -0.30, 1.80, -0.20, -0.50),
        (1.10, 0.45, 0.55, 0.30, 2.00),
    ),
)
def test_fk_ik_fk_round_trip_selects_seed_nearest_solution(positions):
    target = KINEMATICS.forward(positions)
    result = KINEMATICS.inverse(target, positions)
    assert result.status is IKStatus.OK
    assert result.selected is not None
    assert result.selected.positions == pytest.approx(positions, abs=1.0e-8)
    assert KINEMATICS.forward(result.selected.positions) == pytest.approx(target, abs=1.0e-9)


def test_inverse_returns_all_valid_branches_and_uses_seed():
    first = np.array((0.2, 0.1, 0.1, 0.0, 0.2))
    result = KINEMATICS.inverse(KINEMATICS.forward(first), first)
    assert result.status is IKStatus.OK
    assert len(result.solutions) >= 2
    assert result.selected.positions == pytest.approx(first, abs=1.0e-8)
    alternate_seed = result.solutions[1].positions
    alternate = KINEMATICS.inverse(KINEMATICS.forward(first), alternate_seed)
    assert alternate.selected.positions == pytest.approx(alternate_seed, abs=1.0e-8)


def test_base_axis_singularity_preserves_seed_angle():
    target = np.eye(4)
    target[2, 3] = 0.6
    seed = (0.7, 0.0, np.pi / 2.0, 0.0, 0.0)
    result = KINEMATICS.inverse(target, seed)
    assert result.status is IKStatus.OK
    assert result.selected.positions[0] == pytest.approx(seed[0])


def test_inverse_failure_statuses_are_distinct():
    home = KINEMATICS.forward(PARAMETERS.home)
    unreachable = home.copy()
    unreachable[0, 3] = 2.0
    assert KINEMATICS.inverse(unreachable).status is IKStatus.UNREACHABLE

    incompatible = home.copy()
    incompatible[:3, :3] = home[:3, :3] @ rotation_x(0.2)
    assert KINEMATICS.inverse(incompatible).status is IKStatus.ORIENTATION_MISMATCH

    outside_limits = KINEMATICS.forward((0.0, 1.7, 2.8, -1.5, 0.0))
    assert KINEMATICS.inverse(outside_limits).status is IKStatus.JOINT_LIMIT

    invalid = home.copy()
    invalid[0, 0] = np.nan
    assert KINEMATICS.inverse(invalid).status is IKStatus.INVALID_INPUT


def test_geometric_jacobian_matches_finite_difference():
    positions = np.array((0.35, -0.25, 1.35, 0.20, 0.45))
    expected = KINEMATICS.jacobian(positions)
    reference = KINEMATICS.forward(positions)
    step = 1.0e-7
    numerical = np.zeros((6, 5))
    for index in range(5):
        plus = positions.copy()
        minus = positions.copy()
        plus[index] += step
        minus[index] -= step
        transform_plus = KINEMATICS.forward(plus)
        transform_minus = KINEMATICS.forward(minus)
        numerical[:3, index] = (transform_plus[:3, 3] - transform_minus[:3, 3]) / (2.0 * step)
        rotation_rate = (
            (transform_plus[:3, :3] - transform_minus[:3, :3])
            / (2.0 * step)
            @ reference[:3, :3].T
        )
        numerical[3:, index] = (
            rotation_rate[2, 1],
            rotation_rate[0, 2],
            rotation_rate[1, 0],
        )
    assert expected == pytest.approx(numerical, abs=2.0e-8)


def test_quaternion_conversion_round_trip():
    rotation = KINEMATICS.forward((0.5, -0.2, 1.4, 0.1, -0.7))[:3, :3]
    quaternion = quaternion_from_matrix(rotation)
    assert matrix_from_quaternion(quaternion) == pytest.approx(rotation, abs=1.0e-12)


def test_random_full_joint_range_round_trips():
    rng = np.random.default_rng(431)
    for positions in rng.uniform(PARAMETERS.lower, PARAMETERS.upper, (300, 5)):
        target = KINEMATICS.forward(positions)
        result = KINEMATICS.inverse(target, positions)
        assert result.success, (positions, result)
        assert result.selected.positions == pytest.approx(positions, abs=1e-7)
        for candidate in result.solutions:
            assert KINEMATICS.within_limits(candidate.positions)
            assert KINEMATICS.forward(candidate.positions) == pytest.approx(target, abs=1e-8)
        task_result = KINEMATICS.inverse_task(KINEMATICS.task_pose(positions), positions)
        assert task_result.success, (positions, task_result)
        assert task_result.selected.positions == pytest.approx(positions, abs=1e-7)


@pytest.mark.parametrize("base", (-np.pi, np.pi))
@pytest.mark.parametrize("roll", (-np.pi, np.pi))
def test_angle_endpoints_choose_seed_representation(base, roll):
    q = (base, 0.3, 2.4, 0.9, roll)
    result = KINEMATICS.inverse(KINEMATICS.forward(q), q)
    assert result.success
    assert result.selected.positions == pytest.approx(q, abs=1e-8)


def test_extended_elbow_singularity_and_default_seed():
    q = (0.3, -0.2, PARAMETERS.shoulder_link_angle, 0.1, -0.4)
    target = KINEMATICS.forward(q)
    result = KINEMATICS.inverse(target, q)
    assert result.success
    assert KINEMATICS.forward(result.selected.positions) == pytest.approx(target, abs=1e-8)
    default = KINEMATICS.inverse(KINEMATICS.forward(PARAMETERS.home))
    assert default.selected.positions == pytest.approx(PARAMETERS.home, abs=1e-8)


def test_invalid_inputs_and_parameter_validation():
    target = KINEMATICS.forward(PARAMETERS.home)
    for invalid in ([[1, 2], [3]], np.zeros((4, 4)), "bad", None):
        assert KINEMATICS.inverse(invalid).status is IKStatus.INVALID_INPUT
    for seed in ([1, 2], [np.nan] * 5, 2):
        assert KINEMATICS.inverse(target, seed).status is IKStatus.INVALID_INPUT
    assert KINEMATICS.inverse_task(TaskPose(0, 0, np.nan, 0, 0)).status is IKStatus.INVALID_INPUT
    with pytest.raises(ValueError):
        replace(PARAMETERS, base_to_shoulder=-1)
    with pytest.raises(ValueError):
        replace(PARAMETERS, home=(0, 0, -1, 0, 0))
    with pytest.raises(ValueError):
        matrix_from_quaternion((0, 0, 0, 0))


@pytest.mark.parametrize("angle", (np.pi, np.pi - 1e-9, -np.pi))
def test_quaternion_near_half_turn(angle):
    rotation = rotation_x(angle)
    assert matrix_from_quaternion(quaternion_from_matrix(rotation)) == pytest.approx(rotation, abs=1e-12)
