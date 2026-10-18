import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from dragonfly_flight.native import px4_parameters
from dragonfly_flight.core import Platform
from dragonfly_flight.runtime import Runtime
from dragonfly_flight.model import inertial_properties


def metadata():
    root=Path(__file__).resolve().parents[2]
    dimensions=yaml.safe_load((root/'dragonfly_arm/config/model_dimensions.yaml').read_text())
    cfg=yaml.safe_load((root/'dragonfly_flight/config/flight.yaml').read_text())
    robot=ET.fromstring(subprocess.check_output([
        'xacro',str(root/'dragonfly_arm/urdf/dragonfly_arm.urdf.xacro')]))
    robot.remove(robot.find("joint[@name='world_to_uav']"))
    robot.remove(robot.find("link[@name='world']"))
    mass,com,inertia=inertial_properties(robot)
    model=Platform(dimensions,cfg,mass,com,inertia)
    return dict(dimensions=dimensions,config=cfg,mass=model.mass,com=model.com.tolist(),
                inertia=model.inertia.tolist(),max_thrust=model.max_thrust)


def test_native_parameters_follow_geometry_and_thrust_scale():
    model=metadata();params=px4_parameters(model)
    assert params['DF_MASS']==model['mass']
    assert math.isclose(params['DF_RADIUS'],1.2/math.sqrt(3))
    assert math.isclose(params['DF_HUB'],.038)
    assert math.isclose(params['DF_FMAX']*6,model['mass']*params['DF_ACC'])
    assert params['MPC_USE_HTE']==0 and params['DF_EN']==1
    assert params['CA_METHOD']==3 and params['CA_AIRFRAME']==16


def test_native_world_target_is_transformed_to_local_ned_with_nan_feedforward():
    sent=[]
    item=SimpleNamespace(offset=np.array([1.,2.,3.]),local=None,now=lambda:10.,
        trajectory=SimpleNamespace(publish=sent.append))
    Runtime.send_native_target(item,np.array([1.5,1.6,4.6]))
    message=sent[-1]
    assert np.allclose(message.position,[-.4,.5,-1.6])
    assert math.isclose(message.yaw,math.pi/2)
    assert all(math.isnan(v) for v in message.velocity+message.acceleration)


def test_native_arming_warmup_holds_estimated_position():
    sent=[]
    item=SimpleNamespace(offset=None,local=SimpleNamespace(x=4.,y=-2.,z=-1.),now=lambda:3.,
        trajectory=SimpleNamespace(publish=sent.append))
    Runtime.send_native_target(item,None)
    assert np.array_equal(sent[-1].position,[4.,-2.,-1.])


def test_native_output_expiry_aborts_instead_of_using_old_actuator_commands():
    item=SimpleNamespace(native_outputs={},native_times={})
    with pytest.raises(RuntimeError,match='telemetry missing'):
        Runtime.native_state(item,5.)
