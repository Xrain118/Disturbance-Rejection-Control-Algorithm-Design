import unittest

import numpy as np

from dragonfly_control.feedforward import ModelFeedforward
from test_coupled_dynamics import _model, _state


class ModelFeedforwardTest(unittest.TestCase):
    def test_nominal_fixed_arms_have_zero_feedforward(self):
        model = _model()
        ff = ModelFeedforward(model)
        result = ff.evaluate(
            np.eye(3), [0.1, -0.2, 0.3], [0.2, 0.1, -0.1],
            np.zeros(10), np.zeros(10), np.zeros(10),
            [0.2, -0.1, 0.4, 0.1, 0.2, -0.2],
        )
        self.assertTrue(np.allclose(result.raw_delta_wrench, np.zeros(6), atol=1e-12))
        self.assertTrue(np.allclose(result.command_wrench, result.nominal_wrench))

    def test_disabled_feedforward_exactly_preserves_nominal_baseline(self):
        model = _model()
        ff = ModelFeedforward(model)
        result = ff.evaluate(
            np.eye(3), np.zeros(3), np.zeros(3),
            np.linspace(-0.2, 0.3, 10), np.linspace(0.1, -0.2, 10),
            np.linspace(0.3, -0.1, 10), np.zeros(6), enabled=False,
        )
        self.assertTrue(np.allclose(result.delta_wrench, np.zeros(6)))
        self.assertTrue(np.allclose(result.command_wrench, result.nominal_wrench))
        self.assertGreater(np.linalg.norm(result.raw_delta_wrench), 0)

    def test_static_gravity_is_not_added_twice(self):
        model = _model()
        ff = ModelFeedforward(model)
        result = ff.evaluate(
            np.eye(3), np.zeros(3), np.zeros(3),
            np.linspace(-0.4, 0.3, 10), np.zeros(10), np.zeros(10), np.zeros(6),
        )
        self.assertTrue(np.allclose(result.raw_delta_wrench[:3], np.zeros(3), atol=1e-12))

    def test_joint_acceleration_coupling_matches_mba(self):
        model = _model()
        ff = ModelFeedforward(model)
        ddq = np.linspace(0.3, -0.2, 10)
        result = ff.evaluate(
            np.eye(3), np.zeros(3), np.zeros(3),
            np.zeros(10), np.zeros(10), ddq, np.zeros(6),
        )
        self.assertTrue(np.allclose(result.raw_delta_wrench,
                                    result.dynamics.m_ba @ ddq, atol=1e-12))

    def test_component_limits_are_reported(self):
        model = _model()
        ff = ModelFeedforward(model, force_limit=[0.01, 0.01, 0.01],
                              torque_limit=[0.02, 0.02, 0.02])
        result = ff.evaluate(
            np.eye(3), np.zeros(3), np.zeros(3),
            np.linspace(-0.2, 0.3, 10), np.linspace(0.1, -0.2, 10),
            np.linspace(0.3, -0.1, 10), np.zeros(6),
        )
        self.assertTrue(result.saturated)
        self.assertLessEqual(np.max(np.abs(result.delta_wrench[:3])), 0.01)
        self.assertLessEqual(np.max(np.abs(result.delta_wrench[3:])), 0.02)


if __name__ == "__main__":
    unittest.main()
