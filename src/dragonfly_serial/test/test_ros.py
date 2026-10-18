import json
import math
import time
import pytest
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('dragonfly_interfaces.msg')
from dragonfly_serial.node import TelemetryNode
from dragonfly_interfaces.srv import ConfigureTelemetry
from test_protocol import sample

class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)

def test_ros_semantics_identity_errors_and_independent_watchdog():
    rclpy.init()
    node=TelemetryNode(replay=True)
    node.telemetry_pub={s:Capture() for s in ('left','right')}
    node.torque_pub={s:Capture() for s in ('left','right')}
    node.joint_pub=Capture();node.diag_pub=Capture()
    def handle(side,**kw):
        node.handle(dict(side=side,kind='line',raw=json.dumps(sample(dev=side,**kw)),
                         mono_ns=time.monotonic_ns(),wall_ns=time.time_ns()))
    try:
        recorded=dict(node.config['left'],position_offset=[1.,0.,0.,0.,0.])
        node.handle(dict(side='left',kind='configuration',config=recorded,
                         filter_alpha=0.,stale_timeout=.5))
        handle('left')
        assert node.joint_pub.messages[-1].position[0]==pytest.approx(1.)
        assert not node.workers  # Recorded port metadata never opens a device.
        handle('right')
        msg=node.joint_pub.messages[-1]
        assert len(msg.name)==5 and not msg.velocity and not msg.effort
        assert msg.name[0]=='right_joint_1'
        assert len(node.telemetry_pub['left'].messages[-1].servos)==7
        assert node.telemetry_pub['left'].messages[-1].servos[0].current_raw==-1
        node.config['left']['device_id']='different-device'
        handle('left',seq=1,t_us=40000)
        assert len(node.telemetry_pub['left'].messages)==1
        assert node.errors['left']==1
        node.quality['left'].last_receive=time.monotonic()-1
        node.diagnostics()
        statuses=node.diag_pub.messages[-1].status
        assert statuses[0].level==2 and statuses[1].level==1
        d=sample(seq=1,t_us=40000);d['ok'][2]=False
        from dragonfly_serial.protocol import ARRAYS
        for k in ARRAYS:d[k][2]=None
        d['dev']='right'
        node.handle(dict(side='right',kind='line',raw=json.dumps(d),
                         mono_ns=time.monotonic_ns(),wall_ns=time.time_ns()))
        torque=node.torque_pub['right'].messages[-1]
        assert not torque.valid[1] and math.isnan(torque.estimate_nm[1])
        servo=node.telemetry_pub['right'].messages[-1].servos[2]
        assert not servo.valid and math.isnan(servo.torque_estimate_nm)
        req=ConfigureTelemetry.Request(operation=2,rate_hz=50)
        res=node.configure('left',req,ConfigureTelemetry.Response())
        assert not res.accepted and not node.workers
    finally:
        node.destroy_node();rclpy.shutdown()
