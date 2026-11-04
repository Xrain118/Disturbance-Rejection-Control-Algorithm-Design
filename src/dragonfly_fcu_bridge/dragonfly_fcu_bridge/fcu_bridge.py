"""PX4 setpoint adapter.  Mode ownership remains with the flight runtime."""
import json
import math
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dragonfly_interfaces.msg import BodyWrenchCommand
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint
from rclpy.node import Node

from .wrench_adapter import physical_to_normalized, WrenchScale


class FcuBridge(Node):
    def __init__(self):
        super().__init__("fcu_bridge")
        metadata_path = self.declare_parameter("model_metadata", "").value
        if not metadata_path:
            raise ValueError("model_metadata is required")
        self.metadata_path = Path(metadata_path)
        self.metadata = json.loads(self.metadata_path.read_text(encoding="utf-8"))
        self.scale = WrenchScale.from_model_metadata(self.metadata)
        self.expected_version = self.declare_parameter("expected_model_version", "").value
        self.stale_timeout = float(self.declare_parameter("stale_timeout", 0.2).value)
        self.max_future = float(self.declare_parameter("max_future", 0.05).value)
        if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0:
            raise ValueError("stale_timeout must be positive")
        if not math.isfinite(self.max_future) or self.max_future < 0:
            raise ValueError("max_future must be nonnegative")
        command_topic = self.declare_parameter("command_topic", "/research/body_wrench_command").value
        self.thrust_pub = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", 10)
        self.torque_pub = self.create_publisher(VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/research/fcu_bridge/diagnostics", 10)
        self.create_subscription(BodyWrenchCommand, command_topic, self.on_command, 10)
        self.accepted = 0
        self.rejected = 0
        self.last_sequence = None
        self.last_accept_ns = None
        self.detail = "waiting for wrench command"
        self.create_timer(0.5, self.publish_diagnostics)

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def _validate(self, message, now_ns):
        if not message.valid:
            return "command marked invalid"
        if message.header.frame_id != "uav_base_link":
            return "force frame must be uav_base_link/FLU"
        if message.torque_reference_point != "uav_base_link":
            return "torque reference point must be uav_base_link"
        if self.expected_version and message.model_version != self.expected_version:
            return "model version mismatch"
        stamp_ns = self._stamp_ns(message)
        if stamp_ns <= 0:
            return "missing command timestamp"
        age = (now_ns - stamp_ns) * 1e-9
        if age > self.stale_timeout:
            return f"stale wrench command: {age:.3f}s"
        if age < -self.max_future:
            return f"future wrench command: {-age:.3f}s"
        if self.last_sequence is not None and message.sequence <= self.last_sequence:
            return "duplicate or out-of-order wrench sequence"
        try:
            normalized = physical_to_normalized(message.force_n, message.torque_nm, self.scale)
        except ValueError as error:
            return str(error)
        return normalized

    def on_command(self, message):
        now_ns = self._now_ns()
        result = self._validate(message, now_ns)
        if isinstance(result, str):
            self.rejected += 1
            self.detail = result
            self.get_logger().warn(result, throttle_duration_sec=1.0)
            return
        timestamp_sample = self._stamp_ns(message) // 1000
        thrust = VehicleThrustSetpoint()
        thrust.timestamp = now_ns // 1000
        thrust.timestamp_sample = timestamp_sample
        thrust.xyz = result.thrust.astype(float).tolist()
        torque = VehicleTorqueSetpoint()
        torque.timestamp = now_ns // 1000
        torque.timestamp_sample = timestamp_sample
        torque.xyz = result.torque.astype(float).tolist()
        self.thrust_pub.publish(thrust)
        self.torque_pub.publish(torque)
        self.last_sequence = message.sequence
        self.last_accept_ns = now_ns
        self.accepted += 1
        self.detail = "forwarding normalized wrench; PX4 owns allocation"

    def publish_diagnostics(self):
        now_ns = self._now_ns()
        age = None if self.last_accept_ns is None else (now_ns - self.last_accept_ns) * 1e-9
        status = DiagnosticStatus()
        status.name = "fcu_bridge/wrench"
        status.hardware_id = str(self.metadata_path)
        if age is not None and age > self.stale_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = "stale command; no new PX4 setpoint published"
        elif self.rejected and self.accepted == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = self.detail
        else:
            status.level = DiagnosticStatus.OK if self.accepted else DiagnosticStatus.WARN
            status.message = self.detail
        status.values = [
            KeyValue(key="accepted", value=str(self.accepted)),
            KeyValue(key="rejected", value=str(self.rejected)),
            KeyValue(key="last_sequence", value=str(self.last_sequence)),
            KeyValue(key="last_accept_age_s", value="unknown" if age is None else str(age)),
            KeyValue(key="model_version", value=self.expected_version or "not enforced"),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diag_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FcuBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
