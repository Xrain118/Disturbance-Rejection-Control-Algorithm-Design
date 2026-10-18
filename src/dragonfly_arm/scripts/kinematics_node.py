#!/usr/bin/env python3
"""ROS 2 adapter for the reusable RoArm-M3 kinematics library."""

from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from dragonfly_arm_kinematics import (
    ArmParameters,
    IKStatus,
    RoArmM3Kinematics,
    matrix_from_quaternion,
    quaternion_from_matrix,
)
from dragonfly_arm.srv import SolveIK


SIDES = ("left", "right")
JOINT_NAMES = {
    side: tuple(f"{side}_joint_{index}" for index in range(1, 6))
    for side in SIDES
}


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("roarm_m3_kinematics")
        config_path = Path(get_package_share_directory("dragonfly_arm")) / "config" / "model_dimensions.yaml"
        self.kinematics = RoArmM3Kinematics(ArmParameters.from_yaml(config_path))
        self._tool_publishers = {
            side: self.create_publisher(PoseStamped, f"/{side}_arm/tool_pose", 10)
            for side in SIDES
        }
        self._ik_services = {
            side: self.create_service(
                SolveIK,
                f"/{side}_arm/solve_ik",
                lambda request, response, selected_side=side: self.solve_ik(
                    selected_side, request, response
                ),
            )
            for side in SIDES
        }
        self._joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self.publish_forward_kinematics, qos_profile_sensor_data
        )

    def publish_forward_kinematics(self, state: JointState) -> None:
        indices = {name: index for index, name in enumerate(state.name)}
        for side in SIDES:
            names = JOINT_NAMES[side]
            if not all(name in indices and indices[name] < len(state.position) for name in names):
                continue
            positions = [state.position[indices[name]] for name in names]
            if not np.all(np.isfinite(positions)):
                continue
            transform = self.kinematics.forward(positions)
            message = PoseStamped()
            message.header.stamp = state.header.stamp
            message.header.frame_id = f"{side}_base_link"
            message.pose.position.x, message.pose.position.y, message.pose.position.z = transform[:3, 3]
            quaternion = quaternion_from_matrix(transform[:3, :3])
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ) = quaternion
            self._tool_publishers[side].publish(message)

    def solve_ik(self, side: str, request: SolveIK.Request, response: SolveIK.Response):
        expected_frame = f"{side}_base_link"
        if request.target.header.frame_id != expected_frame:
            response.success = False
            response.status = SolveIK.Response.STATUS_INVALID_FRAME
            response.message = (
                f"Expected target.header.frame_id '{expected_frame}', got "
                f"'{request.target.header.frame_id}'"
            )
            return response

        pose = request.target.pose
        target = np.eye(4)
        target[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
        try:
            target[:3, :3] = matrix_from_quaternion(
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            )
            seed = request.seed if request.use_seed else None
            result = self.kinematics.inverse(target, seed)
        except (TypeError, ValueError) as error:
            response.success = False
            response.status = SolveIK.Response.STATUS_INVALID_INPUT
            response.message = str(error)
            return response

        status_map = {
            IKStatus.OK: SolveIK.Response.STATUS_OK,
            IKStatus.INVALID_INPUT: SolveIK.Response.STATUS_INVALID_INPUT,
            IKStatus.UNREACHABLE: SolveIK.Response.STATUS_UNREACHABLE,
            IKStatus.ORIENTATION_MISMATCH: SolveIK.Response.STATUS_ORIENTATION_MISMATCH,
            IKStatus.JOINT_LIMIT: SolveIK.Response.STATUS_JOINT_LIMIT,
        }
        response.success = result.success
        response.status = status_map[result.status]
        response.message = result.message
        response.candidate_count = len(result.solutions)
        if result.selected is not None:
            response.joint_positions = list(result.selected.positions)
            response.position_error = result.selected.position_error
            response.orientation_error = result.selected.orientation_error
        return response


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
