#!/usr/bin/env python3
"""Bounded runtime readiness check without the ros2 CLI discovery daemon."""
import math
import time
from collections import deque

from controller_manager_msgs.srv import ListControllers
from dragonfly_arm.srv import SolveIK
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node("verify_dragonfly_runtime")
    expected = {
        f"{side}_joint_{i}" for side in ("left", "right") for i in range(1, 6)
    } | {
        f"unit_{unit}_{axis}_joint" for unit in "abc" for axis in ("outer", "inner")
    }
    latest = deque(maxlen=1)
    sub = node.create_subscription(JointState, "/joint_states", latest.append, qos_profile_sensor_data)
    tool_poses = {}
    pose_subscriptions = [
        node.create_subscription(
            PoseStamped,
            f"/{side}_arm/tool_pose",
            lambda message, arm_side=side: tool_poses.__setitem__(arm_side, message),
            10,
        )
        for side in ("left", "right")
    ]
    ik_clients = {
        side: node.create_client(SolveIK, f"/{side}_arm/solve_ik")
        for side in ("left", "right")
    }
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    deadline = time.monotonic() + 180
    future = None
    active = set()
    next_query = 0.0
    ik_futures = {}
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is not None and future.done():
                result = future.result()
                active = {c.name for c in result.controller if c.state == "active"}
                future = None
            if latest:
                msg = latest[0]
                indices = {name: i for i, name in enumerate(msg.name)}
                valid = expected.issubset(indices) and all(
                    indices[name] < len(msg.position) and indices[name] < len(msg.velocity)
                    and math.isfinite(msg.position[indices[name]])
                    and math.isfinite(msg.velocity[indices[name]]) for name in expected
                )
                required = {
                    "joint_state_broadcaster",
                    "left_arm_controller",
                    "right_arm_controller",
                    "tilt_controller",
                }
                poses_valid = all(
                    side in tool_poses
                    and tool_poses[side].header.frame_id == f"{side}_base_link"
                    and all(
                        math.isfinite(value)
                        for value in (
                            tool_poses[side].pose.position.x,
                            tool_poses[side].pose.position.y,
                            tool_poses[side].pose.position.z,
                            tool_poses[side].pose.orientation.x,
                            tool_poses[side].pose.orientation.y,
                            tool_poses[side].pose.orientation.z,
                            tool_poses[side].pose.orientation.w,
                        )
                    )
                    for side in ("left", "right")
                )
                runtime_ready = valid and required.issubset(active) and poses_valid
                if runtime_ready and not ik_futures and all(
                    service.service_is_ready() for service in ik_clients.values()
                ):
                    for side, service in ik_clients.items():
                        request = SolveIK.Request()
                        request.target = tool_poses[side]
                        request.seed = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0]
                        request.use_seed = True
                        ik_futures[side] = service.call_async(request)
                if runtime_ready and len(ik_futures) == 2 and all(
                    future.done() for future in ik_futures.values()
                ):
                    failures = {}
                    for side, future in ik_futures.items():
                        result = future.result()
                        if result is None:
                            failures[side] = "service returned no response"
                        elif not result.success:
                            failures[side] = result.message
                    if failures:
                        raise RuntimeError(f"IK service validation failed: {failures}")
                    print(
                        "All four controllers active; 16 finite joint states, "
                        "two tool poses and two valid IK service responses received."
                    )
                    return
            if future is None and time.monotonic() >= next_query and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
                next_query = time.monotonic() + 0.5
        raise RuntimeError(
            f"Runtime not ready within 180 s; active controllers: {sorted(active)}, "
            f"tool poses: {sorted(tool_poses)}, IK responses: {sorted(ik_futures)}"
        )
    finally:
        node.destroy_subscription(sub)
        for pose_subscription in pose_subscriptions:
            node.destroy_subscription(pose_subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
