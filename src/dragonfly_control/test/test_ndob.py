import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from dragonfly_control.ndob import (
    MomentumNdob,
    NdobConfig,
    coupled_momentum_and_bias,
    subtract_disturbance,
)
from test_coupled_dynamics import _model


def _config(**changes):
    values = dict(
        enabled=True,
        shadow_mode=True,
        compensation_enabled=False,
        translation_gain=[6.0, 6.0, 6.0],
        rotation_gain=[6.0, 6.0, 6.0],
        force_limit=[50.0, 50.0, 50.0],
        torque_limit=[20.0, 20.0, 20.0],
        compensation_force_rate=[1000.0, 1000.0, 1000.0],
        compensation_torque_rate=[1000.0, 1000.0, 1000.0],
        state_timeout=0.2,
        max_dt=0.05,
        max_time_jump=0.5,
        saturation_hold_time=0.2,
    )
    values.update(changes)
    return NdobConfig.from_mapping(values)


def _simulate(observer, disturbance, duration=3.0, dt=0.005, bias=None):
    momentum = np.zeros(6)
    actuator = np.zeros(6)
    last = None
    for index in range(int(duration / dt) + 1):
        t = index * dt
        known_bias = np.zeros(6) if bias is None else np.asarray(bias(t), dtype=float)
        last = observer.update(t, t, momentum, known_bias, actuator)
        d = disturbance(t) if callable(disturbance) else disturbance
        momentum += dt * (actuator + d - known_bias)
    return last


class MomentumNdobTest(unittest.TestCase):
    def test_zero_disturbance_with_known_bias_remains_zero(self):
        observer = MomentumNdob(_config())
        bias = np.array([1.0, -2.0, 8.0, 0.2, -0.1, 0.3])
        result = _simulate(observer, np.zeros(6), bias=lambda _: bias)
        self.assertTrue(result.observer_valid)
        self.assertLess(np.max(np.abs(result.estimated_wrench)), 1e-8)
        self.assertTrue(np.allclose(result.compensation_wrench, 0.0))

    def test_constant_disturbance_converges_in_all_six_channels(self):
        for axis in range(6):
            with self.subTest(axis=axis):
                disturbance = np.zeros(6)
                disturbance[axis] = 2.0 if axis < 3 else 0.4
                observer = MomentumNdob(_config())
                result = _simulate(observer, disturbance)
                self.assertLess(np.max(np.abs(result.estimated_wrench - disturbance)), 2e-3)
                self.assertEqual(result.estimated_wrench.shape, (6,))

    def test_compensation_is_subtracted_from_whole_vehicle_wrench(self):
        disturbance = np.array([1.0, -2.0, 0.5, 0.2, -0.1, 0.3])
        observer = MomentumNdob(_config(shadow_mode=False, compensation_enabled=True))
        result = _simulate(observer, disturbance)
        nominal = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        command = subtract_disturbance(nominal, result.compensation_wrench)
        self.assertLess(np.max(np.abs(command - (nominal - disturbance))), 2e-3)

    def test_time_varying_disturbance_is_tracked(self):
        observer = MomentumNdob(_config(
            translation_gain=[20.0] * 3, rotation_gain=[20.0] * 3,
        ))

        def disturbance(t):
            return np.array([np.sin(t), 0.5 * np.sin(t + 0.2), 0.0,
                             0.1 * np.sin(t), 0.0, 0.05 * np.sin(t + 0.4)])

        result = _simulate(observer, disturbance, duration=6.0)
        expected = disturbance(6.0)
        self.assertLess(np.max(np.abs(result.estimated_wrench - expected)), 0.08)

    def test_time_varying_known_bias_is_not_counted_as_disturbance(self):
        observer = MomentumNdob(_config())

        def bias(t):
            return np.array([np.sin(t), 0.2 * t, 8.0 + np.cos(t),
                             0.1 * np.sin(0.5 * t), 0.0, 0.0])

        result = _simulate(observer, np.zeros(6), duration=3.0, bias=bias)
        self.assertLess(np.max(np.abs(result.estimated_wrench)), 0.02)

    def test_output_limit_and_compensation_rate_limit(self):
        disturbance = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        observer = MomentumNdob(_config(
            shadow_mode=False, compensation_enabled=True,
            force_limit=[2.0, 2.0, 2.0], compensation_force_rate=[1.0, 1.0, 1.0],
        ))
        momentum = np.zeros(6)
        previous_compensation = 0.0
        saw_output_limit = False
        for index in range(20):
            t = index * 0.01
            result = observer.update(t, t, momentum, np.zeros(6), np.zeros(6))
            self.assertLessEqual(abs(result.compensation_wrench[0] - previous_compensation), 0.0100001)
            previous_compensation = result.compensation_wrench[0]
            saw_output_limit = saw_output_limit or result.output_limited
            momentum += 0.01 * disturbance
        self.assertTrue(saw_output_limit)
        self.assertLessEqual(abs(result.estimated_wrench[0]), 2.0)

    def test_stale_nan_and_abnormal_dt_invalidate_and_withdraw(self):
        observer = MomentumNdob(_config(shadow_mode=False, compensation_enabled=True))
        _simulate(observer, np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]), duration=0.2)
        stale = observer.update(0.205, 0.0, np.zeros(6), np.zeros(6), np.zeros(6))
        self.assertFalse(stale.observer_valid)
        self.assertIn("invalid:stale_state", stale.events)
        nan = observer.update(0.21, 0.21, np.full(6, np.nan), np.zeros(6), np.zeros(6))
        self.assertFalse(nan.observer_valid)
        self.assertIn("invalid:non_finite_input", nan.events)
        jump = observer.update(1.0, 1.0, np.zeros(6), np.zeros(6), np.zeros(6))
        self.assertFalse(jump.observer_valid)
        self.assertIn("invalid:time_jump", jump.events)
        recovered = observer.update(1.01, 1.01, np.zeros(6), np.zeros(6), np.zeros(6))
        self.assertTrue(recovered.observer_valid)
        self.assertIn("initialized", recovered.events)

    def test_sustained_allocator_saturation_blocks_compensation(self):
        observer = MomentumNdob(_config(
            shadow_mode=False, compensation_enabled=True, saturation_hold_time=0.03,
        ))
        momentum = np.zeros(6)
        disturbance = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        blocked = None
        for index in range(20):
            t = index * 0.01
            blocked = observer.update(t, t, momentum, np.zeros(6), np.zeros(6),
                                      allocator_saturated=index >= 4)
            momentum += 0.01 * disturbance
        self.assertTrue(blocked.allocator_saturation_blocked)
        self.assertFalse(blocked.compensation_active)
        self.assertIn("allocator_saturation_block", blocked.events)
        self.assertTrue(np.allclose(blocked.compensation_wrench, 0.0))

    def test_coupled_momentum_identity_with_rotation_and_moving_arms(self):
        model = _model()
        v = np.array([0.15, -0.1, 0.2])
        omega = np.array([0.2, -0.15, 0.1])
        q0 = np.linspace(-0.3, 0.4, 10)
        dq = np.linspace(0.2, -0.25, 10)
        gravity = np.array([0.0, 0.0, -9.81])

        def state(t):
            rotation = Rotation.from_rotvec(omega * t).as_matrix()
            q = q0 + dq * t
            return coupled_momentum_and_bias(model, rotation, v, omega, q, dq, gravity)

        step = 1e-5
        plus = state(step)
        center = state(0.0)
        minus = state(-step)
        momentum_rate = (plus[0] - minus[0]) / (2.0 * step)
        required_actuator_wrench = center[2].wrench
        self.assertTrue(np.allclose(momentum_rate + center[1], required_actuator_wrench,
                                    atol=2e-6, rtol=2e-6))

    def test_static_gravity_rotation_and_base_origin_torque(self):
        model = _model()
        rotation = Rotation.from_euler("xyz", [0.2, -0.1, 0.3]).as_matrix()
        q = np.linspace(-0.2, 0.3, 10)
        momentum, bias, result = coupled_momentum_and_bias(
            model, rotation, np.zeros(3), np.zeros(3), q, np.zeros(10),
        )
        self.assertTrue(np.allclose(momentum, 0.0))
        # The complete model has 5 kg in this fixture.  Torque is already about
        # uav_base_link O, including every body's r x F contribution.
        self.assertTrue(np.allclose(bias[:3], -5.0 * rotation.T @ [0.0, 0.0, -9.81]))
        self.assertTrue(np.allclose(bias, result.wrench))
        self.assertGreater(np.linalg.norm(bias[3:]), 0.1)

    def test_reused_dynamics_evaluation_matches_direct_computation(self):
        model = _model()
        rotation = Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_matrix()
        v = np.array([0.1, -0.2, 0.15])
        omega = np.array([0.2, 0.1, -0.1])
        q = np.linspace(-0.2, 0.3, 10)
        dq = np.linspace(0.1, -0.1, 10)
        evaluation = model.evaluate(rotation, v, omega, q, dq, np.zeros(10), np.zeros(6))
        direct = coupled_momentum_and_bias(model, rotation, v, omega, q, dq)
        reused = coupled_momentum_and_bias(
            model, rotation, v, omega, q, dq, evaluation=evaluation,
        )
        self.assertTrue(np.allclose(reused[0], direct[0], atol=1e-12, rtol=0.0))
        self.assertTrue(np.allclose(reused[1], direct[1], atol=1e-12, rtol=0.0))
        self.assertIs(reused[2], evaluation)


if __name__ == "__main__":
    unittest.main()
