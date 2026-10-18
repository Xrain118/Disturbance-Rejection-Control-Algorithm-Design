import math
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time
from dragonfly_flight.core import VERTICAL
from dragonfly_flight.runtime import Runtime


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)


class DeploymentArms:
    def __init__(self):self.busy_until=0.;self.reached=False;self.moves=[]
    def move_both(self,target):
        self.moves.append(np.asarray(target));self.busy_until=52.;return True
    def at(self,target):return self.reached and np.allclose(target,VERTICAL)


def deployment(tmp_path):
    item=SimpleNamespace(deployment_state='pending',stable_since=None,deployment_deadline=None,
                         experiment_start=None,events={},out=tmp_path,arms=DeploymentArms(),error=None,
                         model=SimpleNamespace(com=None,inertia=None),vertical_com=np.zeros(3),vertical_inertia=np.eye(3))
    item.abort=lambda reason:setattr(item,'error',reason)
    return item


def test_deployment_requires_hover_and_recovery_dwells(tmp_path):
    item=deployment(tmp_path)
    assert not Runtime.update_deployment(item,10.,14.,True)
    assert not Runtime.update_deployment(item,20.,15.,True)
    assert not Runtime.update_deployment(item,21.9,16.9,True)
    assert not Runtime.update_deployment(item,22.1,17.1,True)
    assert item.deployment_state=='moving' and len(item.arms.moves)==1
    assert np.allclose(item.model.com,item.vertical_com)
    assert np.allclose(item.model.inertia,item.vertical_inertia)
    item.arms.reached=True
    assert not Runtime.update_deployment(item,52.1,47.1,True)
    assert item.deployment_state=='recovering'
    assert not Runtime.update_deployment(item,53.,48.,True)
    assert Runtime.update_deployment(item,55.1,50.1,True)
    assert item.deployment_state=='complete' and item.experiment_start==50.1


def test_deployment_timeout_is_reported(tmp_path):
    item=deployment(tmp_path);item.deployment_state='moving';item.deployment_deadline=57.
    Runtime.update_deployment(item,58.,50.,True)
    assert item.error and 'within 1 degree' in item.error


def test_monitor_messages_use_actual_minus_reference_and_enu_rpy():
    publishers={name:Publisher() for name in ('reference_pose','position_error','position_error_norm',
        'attitude_error','attitude_error_norm','arm_active','ready_pub')}
    clock=SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=12,nanosec=0)))
    item=SimpleNamespace(**publishers,get_clock=lambda:clock,arms=SimpleNamespace(ready=True))
    actual=np.array([1.,2.,3.]);reference=np.array([.9,2.2,2.7])
    rpy=np.array([.1,-.2,.3]);returned=Runtime.publish_monitor(item,actual,reference,Rotation.from_euler('xyz',rpy).as_matrix(),True)
    position=item.position_error.messages[-1]
    assert position.header.frame_id=='world'
    assert np.allclose([position.vector.x,position.vector.y,position.vector.z],[.1,-.2,.3])
    assert math.isclose(item.position_error_norm.messages[-1].data,math.sqrt(.14))
    assert np.allclose(returned,rpy)
    assert item.arm_active.messages[-1].data and item.ready_pub.messages[-1].data
