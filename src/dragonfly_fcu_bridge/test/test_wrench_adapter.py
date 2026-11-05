import unittest

import numpy as np

from dragonfly_fcu_bridge.wrench_adapter import (
    WrenchScale, normalized_to_physical, physical_to_normalized,
)


class WrenchAdapterTest(unittest.TestCase):
    def setUp(self):
        self.scale = WrenchScale(
            mass=5.415,
            com=np.array([0.01077138, 0.0, -0.14164883]),
            inertia=np.array([
                [0.62136976, 0.0, 0.01044967],
                [0.0, 0.59689095, 0.0],
                [0.01044967, 0.0, 0.62865996],
            ]),
            acceleration_scale=19.62,
        )

    def test_hover_with_com_torque_has_zero_normalized_attitude_torque(self):
        force = np.array([0.0, 0.0, self.scale.mass * 9.81])
        torque = np.cross(self.scale.com, force)
        normalized = physical_to_normalized(force, torque, self.scale)
        self.assertTrue(np.allclose(normalized.thrust, [0.0, 0.0, -0.5]))
        self.assertTrue(np.allclose(normalized.torque, np.zeros(3), atol=1e-12))

    def test_roundtrip_with_off_diagonal_inertia(self):
        force = np.array([2.0, -3.0, 48.0])
        torque = np.array([0.4, -0.7, 0.2])
        normalized = physical_to_normalized(force, torque, self.scale)
        actual_force, actual_torque = normalized_to_physical(
            normalized.thrust, normalized.torque, self.scale)
        self.assertTrue(np.allclose(actual_force, force, atol=1e-12))
        self.assertTrue(np.allclose(actual_torque, torque, atol=1e-12))

    def test_reference_point_change_is_not_omitted(self):
        force = np.array([1.0, 2.0, 45.0])
        normalized = physical_to_normalized(force, np.zeros(3), self.scale)
        self.assertGreater(np.linalg.norm(normalized.torque), 0)

    def test_residual_is_difference_of_origin_wrenches(self):
        requested = physical_to_normalized([3., -2., 48.], [.4, -.7, .2], self.scale)
        achieved = physical_to_normalized([1., 1., 46.], [-.1, .2, -.3], self.scale)
        force, torque = normalized_to_physical(requested.thrust-achieved.thrust,
            requested.torque-achieved.torque, self.scale)
        np.testing.assert_allclose(force, [2., -3., 2.], atol=1e-12)
        np.testing.assert_allclose(torque, [.5, -.9, .5], atol=1e-12)

    def test_flu_to_frd_axis_signs(self):
        for axis, sign in enumerate((1., -1., -1.)):
            force = np.eye(3)[axis]
            result = physical_to_normalized(force, np.cross(self.scale.com, force), self.scale)
            np.testing.assert_allclose(result.thrust, sign*force/(self.scale.mass*self.scale.acceleration_scale))
            np.testing.assert_allclose(result.torque, 0., atol=1e-12)

    def test_metadata_uses_matching_nominal_scale(self):
        scale = WrenchScale.from_model_metadata({
            "mass": 4.0,
            "com": [0.0, 0.0, 0.0],
            "inertia": np.eye(3).tolist(),
            "config": {"thrust_to_weight": 2.0},
        })
        self.assertEqual(scale.acceleration_scale, 19.62)

    def test_invalid_scale_is_rejected(self):
        with self.assertRaises(ValueError):
            WrenchScale(1.0, np.zeros(3), np.zeros((3, 3)), 19.62)


if __name__ == "__main__":
    unittest.main()
