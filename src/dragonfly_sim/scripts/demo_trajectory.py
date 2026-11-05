#!/usr/bin/env python3
"""Small fixed-base simulation motions for both five-axis arms and vector units."""

import sys
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from dragonfly_arm_kinematics import ArmParameters
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINT_NAMES = {
    side: [f"{side}_joint_{index}" for index in range(1, 6)]
    for side in ("left", "right")
}
TILT_JOINT_NAMES = [
    f"unit_{unit}_{axis}_joint"
    for unit in "abc"
    for axis in ("outer", "inner")
]
ARM_ACTION_NAMES = {
    side: f"/{side}_arm_controller/follow_joint_trajectory"
    for side in ("left", "right")
}
TILT_ACTION_NAME = "/tilt_controller/follow_joint_trajectory"


class DemoTrajectory(Node):
    def __init__(self):
        super().__init__("demo_trajectory")
        self._arms = {
            side: ActionClient(self, FollowJointTrajectory, action_name)
            for side, action_name in ARM_ACTION_NAMES.items()
        }
        self._tilt = ActionClient(self, FollowJointTrajectory, TILT_ACTION_NAME)
        self._controller_manager = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    def wait_for_controllers(self, required, timeout=180.0):
        if not self._controller_manager.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Controller manager is unavailable.")
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            future = self._controller_manager.call_async(ListControllers.Request())
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if future.done() and future.result() is not None:
                active = {
                    controller.name
                    for controller in future.result().controller
                    if controller.state == "active"
                }
                if required.issubset(active):
                    return True
            time.sleep(0.2)
        self.get_logger().error(f"Controllers did not become active: {sorted(required)}")
        return False

    @staticmethod
    def make_goal(names, poses):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for index, positions in enumerate(poses, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(names)
            point.time_from_start = Duration(sec=index * 4)
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = Duration(sec=2)
        return goal

    def execute_parallel(self, commands):
        """Submit trajectories to independent controllers and await all results."""
        for label, client, _, _ in commands:
            if not client.wait_for_server(timeout_sec=180.0):
                self.get_logger().error(f"Controller unavailable: {label}")
                return False

        submissions = [
            (label, client.send_goal_async(self.make_goal(names, poses)), len(poses))
            for label, client, names, poses in commands
        ]
        deadline = time.monotonic() + 10.0
        while not all(future.done() for _, future, _ in submissions):
            if time.monotonic() >= deadline:
                self.get_logger().error("Trajectory goal submission timed out.")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        handles = []
        for label, future, point_count in submissions:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f"Trajectory rejected: {label}")
                return False
            handles.append((label, handle, handle.get_result_async(), point_count))

        deadline = time.monotonic() + max(item[3] for item in handles) * 4 + 10
        while not all(future.done() for _, _, future, _ in handles):
            if time.monotonic() >= deadline:
                for _, handle, future, _ in handles:
                    if not future.done():
                        handle.cancel_goal_async()
                self.get_logger().error("Trajectory execution timed out.")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        for label, _, future, _ in handles:
            wrapped = future.result()
            if (wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                self.get_logger().error(f"Trajectory execution failed for {label}: {wrapped}")
                return False
        return True

    def run(self):
        # The firmware home pose bends the elbow by 90 degrees, giving more
        # vertical clearance than the all-zero extended pose.
        config = Path(get_package_share_directory("dragonfly_arm")) / "config/model_dimensions.yaml"
        home = list(ArmParameters.from_yaml(config).home)
        left_arm_poses = [
            home,
            [0.20, 0.15, 1.35, 0.10, 0.20],
            [-0.20, -0.15, 1.75, -0.10, -0.20],
            home,
        ]
        right_arm_poses = [
            home,
            [-0.20, 0.10, 1.40, 0.15, -0.20],
            [0.20, -0.10, 1.70, -0.15, 0.20],
            home,
        ]
        if not self.wait_for_controllers({"left_arm_controller", "right_arm_controller"}):
            return 2
        if not self.execute_parallel([
            ("left arm", self._arms["left"], ARM_JOINT_NAMES["left"], left_arm_poses),
            ("right arm", self._arms["right"], ARM_JOINT_NAMES["right"], right_arm_poses),
        ]):
            return 2
        self.get_logger().info("Both five-axis arm trajectories completed successfully.")
        tilt_poses = [
            [0.0] * 6,
            [0.15, 0.0] * 3,  # outer axes only
            [0.0, 0.15] * 3,  # inner axes only
            [0.15, -0.15, -0.15, 0.15, 0.10, 0.10],
            [0.0] * 6,
        ]
        if not self.wait_for_controllers({"tilt_controller"}):
            return 3
        if not self.execute_parallel([
            ("tilt units", self._tilt, TILT_JOINT_NAMES, tilt_poses)
        ]):
            return 3
        self.get_logger().info("Tilt trajectory completed successfully.")
        self.get_logger().info("Trajectory completed successfully and returned to the home pose.")
        return 0


def main():
    rclpy.init()
    node = DemoTrajectory()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
