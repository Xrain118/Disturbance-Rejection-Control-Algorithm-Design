"""ROS adapter contracts, including rejection paths and sensor timestamps."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import JointState
from dragonfly_arm.srv import SolveIK
from dragonfly_arm_kinematics import quaternion_from_matrix

SPEC = importlib.util.spec_from_file_location(
    "roarm_node", Path(__file__).parents[1] / "scripts/kinematics_node.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def node():
    rclpy.init()
    instance = MODULE.KinematicsNode()
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def request_for(node, side, q=None):
    request = SolveIK.Request()
    pose = node.kinematics.forward(node.kinematics.parameters.home if q is None else q)
    request.target.header.frame_id = f"{side}_base_link"
    request.target.pose.position.x, request.target.pose.position.y, request.target.pose.position.z = pose[:3, 3]
    orientation = request.target.pose.orientation
    orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_matrix(pose)
    return request


def test_both_services_success_and_error_statuses(node):
    for side in MODULE.SIDES:
        request = request_for(node, side)
        response = node.solve_ik(side, request, SolveIK.Response())
        assert response.success and response.status == response.STATUS_OK
        assert response.candidate_count >= 1
        assert list(response.joint_positions) == pytest.approx(node.kinematics.parameters.home)

        request.target.header.frame_id = "world"
        response = node.solve_ik(side, request, SolveIK.Response())
        assert not response.success and response.status == response.STATUS_INVALID_FRAME

        request = request_for(node, side)
        request.target.pose.orientation.x = 0.0
        request.target.pose.orientation.y = 0.0
        request.target.pose.orientation.z = 0.0
        request.target.pose.orientation.w = 0.0
        response = node.solve_ik(side, request, SolveIK.Response())
        assert not response.success and response.status == response.STATUS_INVALID_INPUT

        request = request_for(node, side)
        request.target.pose.position.x = 2.0
        response = node.solve_ik(side, request, SolveIK.Response())
        assert not response.success and response.status == response.STATUS_UNREACHABLE

        request = request_for(node, side)
        request.use_seed = True
        request.seed = [np.nan] * 5
        response = node.solve_ik(side, request, SolveIK.Response())
        assert not response.success and response.status == response.STATUS_INVALID_INPUT

        request = request_for(node, side, (0, 1.7, 2.8, -1.5, 0))
        response = node.solve_ik(side, request, SolveIK.Response())
        assert not response.success and response.status == response.STATUS_JOINT_LIMIT


def test_joint_state_named_mapping_partial_data_and_timestamps(node):
    class Recorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    recorders = {side: Recorder() for side in MODULE.SIDES}
    node._tool_publishers = recorders
    message = JointState()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.name = list(reversed(MODULE.JOINT_NAMES["left"]))
    message.position = list(reversed(node.kinematics.parameters.home))
    node.publish_forward_kinematics(message)
    assert len(recorders["left"].messages) == 1
    assert not recorders["right"].messages
    pose = recorders["left"].messages[0]
    assert pose.header.frame_id == "left_base_link"
    assert pose.header.stamp == message.header.stamp
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == pytest.approx((.34616, 0, .34959))
    message.position[0] = np.nan
    node.publish_forward_kinematics(message)
    message.position = []
    node.publish_forward_kinematics(message)
    assert len(recorders["left"].messages) == 1
