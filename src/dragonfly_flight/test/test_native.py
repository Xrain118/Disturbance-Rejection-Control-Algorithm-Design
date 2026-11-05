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
from dragonfly_flight.native import CONTROL_FLAGS, FEEDBACK_FLAGS, ACTUATOR_INPUT_TOPICS, control_chain_errors
from dragonfly_fcu_bridge.wrench_adapter import WrenchScale, physical_to_normalized


def chain(mode):
    evidence={key:mode=='px4_native' or key not in FEEDBACK_FLAGS for key in CONTROL_FLAGS}
    evidence.update(mode=mode,ros_publishes_actuators=False,
        actuator_input_publishers={topic:[] for topic in ACTUATOR_INPUT_TOPICS})
    return evidence


@pytest.mark.parametrize('mode',['px4_native','px4_wrench'])
def test_control_ownership_requires_every_flag_and_graph_evidence(mode):
    evidence=chain(mode)
    assert not control_chain_errors(mode,evidence)
    for key in CONTROL_FLAGS:
        wrong=dict(evidence);wrong[key]=not wrong[key]
        assert control_chain_errors(mode,wrong)
        missing=dict(evidence);del missing[key]
        assert control_chain_errors(mode,missing)
    del evidence['actuator_input_publishers']
    assert control_chain_errors(mode,evidence)


def test_direct_actuator_publisher_is_rejected():
    evidence=chain('px4_wrench')
    evidence['actuator_input_publishers'][ACTUATOR_INPUT_TOPICS[0]]=[{'node_name':'rogue'}]
    assert control_chain_errors('px4_wrench',evidence)


def test_live_residual_uses_origin_and_checks_mode_after_first_success(tmp_path):
    scale=WrenchScale(5.415,np.array([.02,.03,-.14]),np.array([[.6,.01,.02],[.01,.7,.03],[.02,.03,.8]]),19.62)
    requested=physical_to_normalized([3.,-2.,48.],[.4,-.7,.2],scale)
    difference=physical_to_normalized([2.,-3.,2.],[.5,-.9,.5],scale)
    outputs={
        'actuator_motors':SimpleNamespace(control=[.5]*6),
        'actuator_servos':SimpleNamespace(control=[0.]*6),
        'control_allocator_status':SimpleNamespace(unallocated_thrust=difference.thrust,unallocated_torque=difference.torque),
        'vehicle_thrust_setpoint':SimpleNamespace(xyz=requested.thrust),
        'vehicle_torque_setpoint':SimpleNamespace(xyz=requested.torque),
        'vehicle_control_mode':SimpleNamespace(**{k:v for k,v in chain('px4_wrench').items() if k in CONTROL_FLAGS})}
    item=SimpleNamespace(native_outputs=outputs,native_times={key:2. for key in outputs},
        model=SimpleNamespace(max_thrust=17.7,cfg={'tilt_limit':np.pi/2}),wrench_scale=scale,
        mode='px4_wrench',out=tmp_path,native_proven=False,get_publishers_info_by_topic=lambda topic:[])
    _,residual,desired=Runtime.native_state(item,2.)
    np.testing.assert_allclose(residual,[2.,-3.,2.,.5,-.9,.5],atol=1e-12)
    np.testing.assert_allclose(desired,[3.,-2.,48.,.4,-.7,.2],atol=1e-12)
    outputs['vehicle_control_mode'].flag_control_rates_enabled=True
    with pytest.raises(RuntimeError,match='ownership'):
        Runtime.native_state(item,2.01)


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
    with pytest.raises(RuntimeError,match='telemetry stale'):
        Runtime.native_state(item,5.)
