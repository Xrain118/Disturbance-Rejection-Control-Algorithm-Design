import math
from pathlib import Path
import numpy as np
import yaml
from dragonfly_flight.core import Platform, Allocator, px4_rotation, HOME, STOWED, VERTICAL
from dragonfly_flight.arms import trajectory

def platform():
    root=Path(__file__).resolve().parents[2]
    dimensions=yaml.safe_load((root/'dragonfly_arm/config/model_dimensions.yaml').read_text())
    cfg=yaml.safe_load((root/'dragonfly_flight/config/flight.yaml').read_text())
    return Platform(dimensions,cfg,4.0,[.03,0,-.05],np.diag([.3,.3,.6]))

def test_stowed_and_vertical_pose_constants():
    assert np.allclose(STOWED,[0,0,math.pi/2,0,0])
    assert np.allclose(VERTICAL,np.zeros(5))
    assert np.array_equal(HOME,VERTICAL)

def test_group_axes_and_lever_arms():
    m=platform();p,n=m.geometry([.2,-.1,-.3,.15,.1,.3])
    for k in range(3):
        assert np.allclose(n[2*k],n[2*k+1])
        assert np.isclose(np.linalg.norm(p[2*k]-p[2*k+1]),.36)
        assert abs(np.dot(p[2*k]-p[2*k+1],n[2*k]))<1e-12

def test_level_equal_thrust_cancels_torque():
    m=platform();w=m.wrench(np.r_[np.full(6,4*9.81/6),np.zeros(6)])
    assert np.allclose(w,[0,0,39.24,0,0,0],atol=1e-12)

def test_allocation_tracks_hover_and_lateral_force():
    m=platform();a=Allocator(m);angles=np.zeros(6)
    target=np.array([1.,-.7,39.24,0.,-1.2,0.])
    for _ in range(80):
        z,res=a.solve(target,.01)
        assert np.max(abs(z[6:]-angles))<=.005001
        assert min(z[:6])>=-1e-8 and max(z[:6])<=m.max_thrust+1e-8
        angles=z[6:]
    assert np.linalg.norm(res)<.03

def test_px4_enu_flu_conversion():
    # ENU yaw zero corresponds to a +90deg NED heading and inverted Z.
    assert np.allclose(px4_rotation([math.sqrt(.5),0,0,math.sqrt(.5)]),np.eye(3),atol=1e-12)

def test_pointing_trajectory_limits_and_endpoints():
    end=HOME.copy();end[1]+=.17
    msg,_=trajectory(['a','b','c','d','e'],HOME,end)
    assert np.allclose(msg.points[0].positions,HOME)
    assert np.allclose(msg.points[-1].positions,end)
    assert max(abs(v) for p in msg.points for v in p.velocities)<=.10001
    assert max(abs(v) for p in msg.points for v in p.accelerations)<=.20001
