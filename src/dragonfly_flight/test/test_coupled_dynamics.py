import unittest

import numpy as np

from dragonfly_flight.coupled_dynamics import CoupledDynamics


def _inertial(mass, xyz, offset=0.0):
    return f"""<inertial><origin xyz=\"{xyz}\"/><mass value=\"{mass}\"/>
      <inertia ixx=\"0.02\" ixy=\"0.001\" ixz=\"{offset}\" iyy=\"0.03\" iyz=\"0.002\" izz=\"0.04\"/>
    </inertial>"""


def _model():
    parts = ['<robot name="test">', f'<link name="uav_base_link">{_inertial(3.0, "0 0 0", 0.003)}</link>']
    for side, mount in (("left", "0 0.15 -0.05"), ("right", "0 -0.15 -0.05")):
        parts.append(f'<joint name="{side}_uav_to_arm_base" type="fixed"><parent link="uav_base_link"/><child link="{side}_base_link"/><origin xyz="{mount}"/></joint>')
        parts.append(f'<link name="{side}_base_link">{_inertial(0.2, "0 0 0")}</link>')
        parent = f"{side}_base_link"
        for index in range(1, 6):
            child = f"{side}_link_{index}"
            axis = "0 0 1" if index in (1, 5) else "0 1 0"
            xyz = "0 0 0.1" if index == 1 else "0.02 0 0.12"
            parts.append(f'''<joint name="{side}_joint_{index}" type="revolute">
              <parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/>
            </joint>''')
            parts.append(f'<link name="{child}">{_inertial(0.1 + index * 0.02, "0.01 0 0.05", 0.001)}</link>')
            parent = child
    parts.append('</robot>')
    return CoupledDynamics.from_urdf("".join(parts))


def _state(model, q=None, dq=None, ddq=None, acc=None, gravity=(0.0, 0.0, -9.81)):
    q = np.zeros(10) if q is None else q
    dq = np.zeros(10) if dq is None else dq
    ddq = np.zeros(10) if ddq is None else ddq
    acc = np.zeros(6) if acc is None else acc
    return model.evaluate(np.eye(3), np.zeros(3), np.zeros(3), q, dq, ddq, acc, gravity)


class CoupledDynamicsTest(unittest.TestCase):
    def test_mass_matrix_is_symmetric_and_positive_definite(self):
        model = _model()
        q = np.array([0.2, -0.3, 0.8, 0.1, -0.2, -0.1, 0.2, 0.6, -0.1, 0.3])
        result = _state(model, q=q)
        self.assertTrue(np.allclose(result.mass_matrix, result.mass_matrix.T, atol=1e-12))
        self.assertGreater(np.linalg.eigvalsh(result.mass_matrix).min(), 0)
        self.assertEqual(result.m_bb.shape, (6, 6))
        self.assertEqual(result.m_ba.shape, (6, 10))

    def test_direct_body_wrench_matches_mass_matrix_blocks(self):
        model = _model()
        rng = np.random.default_rng(24)
        q = rng.normal(size=10) * 0.4
        dq = rng.normal(size=10) * 0.3
        ddq = rng.normal(size=10) * 0.4
        acc = rng.normal(size=6) * 0.2
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        result = model.evaluate(rotation, [0.1, -0.2, 0.3], [0.2, -0.1, 0.15],
                                q, dq, ddq, acc)
        expected = result.m_bb @ acc + result.m_ba @ ddq + result.h_b
        self.assertTrue(np.allclose(result.wrench, expected, atol=2e-10))

    def test_velocity_bias_matches_numerical_momentum_derivative(self):
        model = _model()
        q = np.linspace(-0.3, 0.4, 10)
        dq = np.linspace(0.2, -0.25, 10)
        result = _state(model, q=q, dq=dq, gravity=np.zeros(3))
        step = 1e-6
        plus = _state(model, q=q + step * dq, dq=dq, gravity=np.zeros(3))
        minus = _state(model, q=q - step * dq, dq=dq, gravity=np.zeros(3))
        momentum_rate = (plus.m_ba @ dq - minus.m_ba @ dq) / (2 * step)
        self.assertTrue(np.allclose(result.h_b, momentum_rate, atol=2e-7))

    def test_static_gravity_and_arm_reaction_signs(self):
        model = _model()
        result = _state(model)
        arm_mass = 0.2 + sum(0.1 + index * 0.02 for index in range(1, 6))
        self.assertTrue(np.allclose(result.wrench[:3], [0.0, 0.0, (3.0 + 2 * arm_mass) * 9.81]))
        for wrench in result.wrenches():
            self.assertTrue(np.allclose(wrench[:3], [0.0, 0.0, -arm_mass * 9.81]))

    def test_common_freefall_has_zero_required_and_interface_wrench(self):
        model = _model()
        q = np.linspace(-0.2, 0.3, 10)
        result = _state(model, q=q, acc=[0.0, 0.0, -9.81, 0.0, 0.0, 0.0])
        self.assertTrue(np.allclose(result.wrench, np.zeros(6), atol=1e-12))
        self.assertTrue(np.allclose(result.left_arm_wrench, np.zeros(6), atol=1e-12))
        self.assertTrue(np.allclose(result.right_arm_wrench, np.zeros(6), atol=1e-12))

    def test_invalid_inputs_are_rejected(self):
        model = _model()
        with self.assertRaises(ValueError):
            _state(model, q=np.zeros(9))
        with self.assertRaises(ValueError):
            model.evaluate(np.eye(2), np.zeros(3), np.zeros(3), np.zeros(10), np.zeros(10), np.zeros(10))


if __name__ == "__main__":
    unittest.main()
