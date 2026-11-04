import unittest
from types import SimpleNamespace

import numpy as np

from dragonfly_fcu_bridge.fcu_bridge import FcuBridge
from dragonfly_fcu_bridge.wrench_adapter import WrenchScale


def command(sequence=1, stamp_ns=1_000_000_000):
    return SimpleNamespace(
        valid=True,
        header=SimpleNamespace(frame_id="uav_base_link", stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
        torque_reference_point="uav_base_link",
        model_version="model-a",
        sequence=sequence,
        force_n=[0.0, 0.0, 20.0],
        torque_nm=[0.1, -0.2, 0.3],
    )


def bridge():
    item = FcuBridge.__new__(FcuBridge)
    item.expected_version = "model-a"
    item.stale_timeout = 0.2
    item.max_future = 0.05
    item.last_sequence = None
    item.scale = WrenchScale(2.0, np.zeros(3), np.diag([0.2, 0.3, 0.4]), 20.0, 25.0)
    return item


class FcuBridgeValidationTest(unittest.TestCase):
    def test_valid_command_is_normalized(self):
        result = bridge()._validate(command(), 1_010_000_000)
        self.assertFalse(isinstance(result, str))
        self.assertTrue(np.isfinite(result.thrust).all())
        self.assertTrue(np.isfinite(result.torque).all())

    def test_rejects_frame_reference_and_version_mismatch(self):
        item = bridge()
        message = command()
        message.header.frame_id = "world"
        self.assertIn("frame", item._validate(message, 1_010_000_000))
        message = command()
        message.torque_reference_point = "com"
        self.assertIn("reference", item._validate(message, 1_010_000_000))
        message = command()
        message.model_version = "model-b"
        self.assertIn("version", item._validate(message, 1_010_000_000))

    def test_rejects_stale_future_invalid_and_out_of_order(self):
        item = bridge()
        self.assertIn("stale", item._validate(command(), 1_300_000_000))
        self.assertIn("future", item._validate(command(), 900_000_000))
        message = command()
        message.valid = False
        self.assertIn("invalid", item._validate(message, 1_010_000_000))
        item.last_sequence = 2
        self.assertIn("out-of-order", item._validate(command(2), 1_010_000_000))

    def test_rejects_non_finite_wrench(self):
        message = command()
        message.force_n[0] = float("nan")
        self.assertIn("finite", bridge()._validate(message, 1_010_000_000))


if __name__ == "__main__":
    unittest.main()
