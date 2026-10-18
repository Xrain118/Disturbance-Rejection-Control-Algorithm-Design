"""SI/FLU flight math and bounded allocation for three shared dual-tilt units."""
import math
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation

# The flight profile has two deliberately different poses.  The firmware
# home remains folded in dragonfly_arm; flight starts folded on the ground and
# uses the straight-down pose as its airborne operating reference.
STOWED = np.array([0., 0., math.pi / 2, 0., 0.])
VERTICAL = np.zeros(5)
# Kept as a compatibility name for the flight package's operating home.
HOME = VERTICAL
ARM_NAMES = {s: [f'{s}_joint_{i}' for i in range(1, 6)] for s in ('left', 'right')}
TILT_NAMES = [f'unit_{u}_{a}_joint' for u in 'abc' for a in ('outer', 'inner')]
NED_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_FLU = np.diag([1., -1., -1.])

def px4_rotation(q_wxyz):
    return NED_ENU @ Rotation.from_quat(np.roll(q_wxyz, -1)).as_matrix() @ FRD_FLU

def smoothstep(t):
    s = np.clip(t, 0., 1.)
    return s**3 * (10 - 15*s + 6*s*s)

def duration(delta, speed=0.10, acceleration=0.20):
    d = np.max(np.abs(delta))
    return max(0.5, 1.875*d/speed, math.sqrt(5.78*d/acceleration))

class Platform:
    def __init__(self, dimensions, config, mass, com, inertia):
        self.mass, self.com, self.inertia = mass, np.array(com), np.array(inertia)
        self.cfg = config
        limits = dimensions['arm_kinematics']['joint_limits']
        order = ('base', 'shoulder', 'elbow', 'wrist', 'roll')
        self.arm_lower = np.array([limits[name]['lower'] for name in order])
        self.arm_upper = np.array([limits[name]['upper'] for name in order])
        self.radius = dimensions['measured']['triangle_side'] / math.sqrt(3)
        self.spacing = dimensions['measured']['motor_spacing']
        e = dimensions['estimated']
        self.height = e['unit_height']
        self.hub_height = e['motor_center_height'] + e['motor_height']/2 + .003
        self.max_thrust = mass * 9.81 * config['thrust_to_weight'] / 6
        self.kf = self.max_thrust / config['max_rotor_speed']**2
        self.yaws = [Rotation.from_euler('z', a).as_matrix() for a in (0, 2*math.pi/3, 4*math.pi/3)]

    def geometry(self, angles):
        positions, axes = [], []
        for k, yaw in enumerate(self.yaws):
            rot = yaw @ Rotation.from_euler('XY', angles[2*k:2*k+2]).as_matrix()
            center = yaw @ np.array([self.radius, 0., self.height])
            for side in (1., -1.):
                positions.append(center + rot @ [0., side*self.spacing/2, self.hub_height])
                axes.append(rot[:, 2])
        return np.array(positions), np.array(axes)

    def matrix(self, angles):
        p, n = self.geometry(angles)
        # Gazebo ccw rotors apply -momentConstant * thrust about local +Z.
        reaction = np.tile([-1., 1.], 3) * self.cfg['moment_constant']
        return np.vstack((n.T, (np.cross(p, n) + reaction[:, None]*n).T))

    def wrench(self, z):
        return self.matrix(z[6:]) @ z[:6]

    def control(self, p, v, rot, omega, target):
        cfg = self.cfg
        acc = np.array(cfg['position_kp'])*(target-p) - np.array(cfg['position_kd'])*v
        force = rot.T @ (self.mass*(acc + [0., 0., 9.81]))
        # desired orientation is level, yaw=0 in ENU; omega is body FLU.
        er = np.array([rot[2, 1]-rot[1, 2], rot[0, 2]-rot[2, 0], rot[1, 0]-rot[0, 1]])/2
        torque_com = self.inertia @ (-np.array(cfg['attitude_kp'])*er - np.array(cfg['attitude_kd'])*omega)
        return np.r_[force, torque_com + np.cross(self.com, force)]

class Allocator:
    def __init__(self, model):
        self.model = model
        self.z = np.r_[np.full(6, model.mass*9.81/6), np.zeros(6)]

    def solve(self, desired, dt):
        # Measured angles are logged by the caller. This allocator predicts the
        # quasi-static target wrench; servo lag is evaluated separately.
        m = self.model
        limit, rate = m.cfg['tilt_limit'], m.cfg['tilt_rate']
        # Limit the commanded target, not a moving measured angle: anchoring
        # each target to measured deflection makes the servo yield under load.
        lo = np.r_[np.zeros(6), np.maximum(-limit, self.z[6:]-rate*dt)]
        hi = np.r_[np.full(6, m.max_thrust), np.minimum(limit, self.z[6:]+rate*dt)]
        z = np.clip(self.z, lo, hi)
        weights = np.array([1., 1., 2., 3., 3., 2.])
        scales = np.r_[np.full(6, m.max_thrust), np.ones(6)]
        nominal=np.r_[np.full(6,np.clip(desired[2]/6,0,m.max_thrust)),np.zeros(6)]
        for _ in range(3):
            w = m.wrench(z)
            j = np.zeros((6, 12)); j[:, :6] = m.matrix(z[6:])
            for k in range(6, 12):
                d = np.zeros(12); d[k] = 1e-5
                j[:, k] = (m.wrench(z+d)-m.wrench(z-d))/2e-5
            a = np.vstack((weights[:, None]*j*scales, .08*np.eye(12)))
            # Absolute trim regularization prevents nullspace drift to a motor
            # limit after many individually small allocation steps.
            b = np.r_[weights*(desired-w), .08*(nominal-z)/scales]
            sol = lsq_linear(a, b, bounds=((lo-z)/scales, (hi-z)/scales), method='bvls', tol=1e-6, max_iter=30)
            if not sol.success or not np.isfinite(sol.x).all():
                raise RuntimeError('allocation failed')
            z += scales*sol.x
        self.z = z
        return z.copy(), m.wrench(z)-desired
