"""Generate flight assets from the existing arm Xacro; no second geometry source."""
import copy
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .core import HOME, STOWED, VERTICAL, ARM_NAMES, TILT_NAMES, Platform
from .effort import ordered_arm_effort_limits

def element(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value)
    return node

def origin(node):
    t = np.eye(4)
    if node is not None:
        t[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return t

def link_poses(robot, positions=None):
    positions=positions if positions is not None else {n:HOME[i] for names in ARM_NAMES.values() for i,n in enumerate(names)}
    poses = {'uav_base_link': np.eye(4)}
    joints = list(robot.findall('joint'))
    while joints:
        ready = [j for j in joints if j.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('Disconnected URDF')
        for j in ready:
            t = origin(j.find('origin'))
            if j.get('name') in positions:
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                t[:3, :3] = t[:3, :3] @ Rotation.from_rotvec(axis*positions[j.get('name')]).as_matrix()
            poses[j.find('child').get('link')] = poses[j.find('parent').get('link')] @ t
            joints.remove(j)
    return poses

def inertial_properties(robot,positions=None):
    poses=link_poses(robot,positions)
    bodies = []
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        t = poses[link.get('name')] @ origin(inertial.find('origin'))
        v = inertial.find('inertia').attrib
        inertia = np.array([[float(v['i'+a+b] if 'i'+a+b in v else v['i'+b+a]) for b in 'xyz'] for a in 'xyz'])
        if min(np.linalg.eigvalsh(inertia)) <= 0:
            raise ValueError(f'Invalid inertia: {link.get("name")}')
        bodies.append((mass, t[:3, 3], t[:3, :3]@inertia@t[:3, :3].T))
    mass = sum(b[0] for b in bodies)
    com = sum(m*p for m, p, _ in bodies)/mass
    inertia = sum(i+m*((p-com)@(p-com)*np.eye(3)-np.outer(p-com, p-com)) for m, p, i in bodies)
    return mass, com, inertia

def generate(arm_share, flight_share, output, mode='px4_actuators', step=.002):
    arm_share, flight_share, output = map(Path, (arm_share, flight_share, output))
    output.mkdir(parents=True, exist_ok=True)
    dimensions = yaml.safe_load((arm_share/'config/model_dimensions.yaml').read_text())
    cfg = yaml.safe_load((flight_share/'config/flight.yaml').read_text())
    robot = ET.fromstring(subprocess.check_output(['xacro', str(arm_share/'urdf/dragonfly_arm.urdf.xacro')]))
    robot.remove(robot.find("link[@name='world']"))
    robot.remove(robot.find("joint[@name='world_to_uav']"))
    robot.remove(robot.find("gazebo[@reference='world']"))
    arm_names = [name for side in ('left', 'right') for name in ARM_NAMES[side]]
    arm_effort_limits = {
        name: float(robot.find(f"joint[@name='{name}']/limit").get('effort'))
        for name in arm_names
    }
    ordered_arm_effort_limits({'arm_effort_limits': arm_effort_limits})
    for j in robot.findall('joint'):
        if '_rotor_' in j.get('name'):
            j.set('type', 'continuous')
            element(j, 'axis', xyz='0 0 1')
            element(j, 'limit', effort='10', velocity='1000')
    # The controller is trimmed for the airborne straight-down configuration.
    # Ground PX4 runs still spawn folded so the tool remains above the skids.
    mass, com, inertia = inertial_properties(robot)
    stowed_positions={n:STOWED[i] for names in ARM_NAMES.values() for i,n in enumerate(names)}
    stowed_mass, stowed_com, stowed_inertia = inertial_properties(robot,stowed_positions)
    if not np.isclose(stowed_mass,mass):raise ValueError('Arm pose changed total mass')
    model = Platform(dimensions, cfg, mass, com, inertia)
    initial_arm_pose = STOWED if mode in ('px4_actuators', 'px4_native') else VERTICAL
    metadata = dict(dimensions=dimensions, config=cfg, mass=mass, com=com.tolist(), inertia=inertia.tolist(),
                    arm_effort_limits=arm_effort_limits,
                    motor_constant=model.kf, max_thrust=model.max_thrust, mode=mode,
                    initial_arm_pose=initial_arm_pose.tolist(),stowed_com=stowed_com.tolist(),
                    stowed_inertia=stowed_inertia.tolist())
    (output/'model.json').write_text(json.dumps(metadata, indent=2))
    control = robot.find('ros2_control')
    for j in list(control.findall('joint')):
        name = j.get('name')
        if name in TILT_NAMES:
            # State-only: flight servo torque is owned by the Gazebo system.
            j.remove(j.find('command_interface'))
        else:
            interface = j.find('command_interface'); interface.set('name', 'effort')
            interface.find("param[@name='min']").text = str(-arm_effort_limits[name])
            interface.find("param[@name='max']").text = str(arm_effort_limits[name])
            element(j.find("state_interface[@name='position']"), 'param', initial_arm_pose[int(name[-1])-1], name='initial_value')
            element(j, 'state_interface', name='effort')
    controllers = yaml.safe_load((arm_share/'config/controllers.yaml').read_text())
    controllers['controller_manager']['ros__parameters']['update_rate'] = 500
    del controllers['controller_manager']['ros__parameters']['tilt_controller']
    del controllers['tilt_controller']
    for side in ARM_NAMES:
        p = controllers[f'{side}_arm_controller']['ros__parameters']
        p['command_interfaces'] = ['effort']
        p['gains'] = {n: dict(p=kp, i=ki, d=kd, i_clamp=2.0, ff_velocity_scale=0.0)
                      for n,kp,ki,kd in zip(ARM_NAMES[side],cfg['arm_kp'],cfg['arm_ki'],cfg['arm_kd'])}
    (output/'controllers.yaml').write_text(yaml.safe_dump(controllers, sort_keys=False))
    robot.find('.//plugin/parameters').text = str(output/'controllers.yaml')
    ET.ElementTree(robot).write(output/'robot.urdf')
    sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(output/'robot.urdf')], stderr=subprocess.DEVNULL))
    vehicle = sdf.find('model'); vehicle.set('name', 'dragonfly')
    vehicle.set('canonical_link', 'uav_base_link')
    element(vehicle, 'pose', '0 0 '+('1.6' if mode in ('ideal_wrench', 'physics') else '0.60')+' 0 0 0')
    base = vehicle.find("link[@name='uav_base_link']")
    for name, kind, rate, suffix in [('imu_sensor','imu',250,'imu'), ('magnetometer_sensor','magnetometer',100,'magnetometer'),
                                     ('air_pressure_sensor','air_pressure',50,'air_pressure'), ('navsat_sensor','navsat',30,'navsat')]:
        sensor = element(base, 'sensor', name=name, type=kind)
        element(sensor, 'always_on', 1); element(sensor, 'update_rate', rate)
        element(sensor, 'topic', f'/world/default/model/dragonfly/link/base_link/sensor/{name}/{suffix}')
        # Small sensor noise also avoids PX4 declaring perfectly constant values stuck.
        body=element(sensor,kind)
        groups=[(body, .0001)] if kind=='magnetometer' else []
        if kind=='imu':
            groups=[(element(body,'angular_velocity'),.0008726646),(element(body,'linear_acceleration'),.006)]
        for group,std in groups:
            for axis in 'xyz':
                noise=element(element(group,axis),'noise',type='gaussian')
                element(noise,'mean',0);element(noise,'stddev',std)
        if kind=='air_pressure':
            noise=element(element(body,'pressure'),'noise',type='gaussian');element(noise,'stddev',.1)
    physics = element(vehicle, 'plugin', filename='libdragonfly_physics.so', name='dragonfly::Physics')
    element(physics, 'gz:systems:priority', 10)
    for key in ('tilt_rate', 'tilt_effort', 'tilt_kp', 'tilt_kd'):
        element(physics, key, cfg[key])
    for name, limit in arm_effort_limits.items():
        element(physics, 'arm_effort_limit', limit, joint=name)
    element(physics, 'ideal', str(mode == 'ideal_wrench').lower())
    for k, (unit, side) in enumerate((u, s) for u in 'abc' for s in ('left', 'right')):
        if mode not in ('px4_actuators', 'px4_native'):
            continue
        plugin = element(vehicle, 'plugin', filename='gz-sim-multicopter-motor-model-system', name='gz::sim::systems::MulticopterMotorModel')
        values = dict(jointName=f'unit_{unit}_rotor_{side}_joint', linkName=f'unit_{unit}_rotor_{side}_link',
                      turningDirection='ccw' if k%2 == 0 else 'cw', timeConstantUp=cfg['motor_time_constant'],
                      timeConstantDown=cfg['motor_time_constant'], maxRotVelocity=cfg['max_rotor_speed'],
                      motorConstant=model.kf, momentConstant=cfg['moment_constant'], commandSubTopic='command/motor_speed',
                      motorNumber=k, rotorDragCoefficient=0, rollingMomentCoefficient=0,
                      rotorVelocitySlowdownSim=10, motorType='velocity')
        for key, value in values.items(): element(plugin, key, value)
    world = ET.parse(arm_share/'worlds/model_preview.world')
    w = world.getroot().find('world')
    w.find('physics/max_step_size').text = str(step)
    # Slower than wall time gives Python control deterministic scheduling headroom.
    w.find('physics/real_time_factor').text = '0.5'
    if mode == 'physics':
        element(w, 'gravity', '0 0 0')
        w.remove(w.find("model[@name='ground_plane']"))
    spherical = element(w, 'spherical_coordinates')
    for key, value in dict(surface_model='EARTH_WGS84', latitude_deg=47.397742, longitude_deg=8.545594,
                           elevation=488, heading_deg=0).items(): element(spherical, key, value)
    for name, system in [('imu','Imu'),('magnetometer','Magnetometer'),('air-pressure','AirPressure'),('navsat','NavSat')]:
        plugin=element(w, 'plugin', filename=f'gz-sim-{name}-system', name=f'gz::sim::systems::{system}')
        if name=='magnetometer':
            element(plugin,'use_units_gauss','true');element(plugin,'use_earth_frame_ned','false')
    w.append(vehicle)
    world.write(output/'world.sdf')
    return metadata
