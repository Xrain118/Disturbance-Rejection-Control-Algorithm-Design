"""Exercise the real terminal client against effort-driven Gazebo joints."""
import json
import math
import os
from pathlib import Path
import pty
import signal
import sys
import subprocess
import time
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from dragonfly_flight.core import VERTICAL

out=Path(sys.argv[1] if len(sys.argv)>1 else str(Path(__file__).resolve().parents[3]/'log/keyboard_live'));out.mkdir(exist_ok=True)
os.environ.update(ROS_DOMAIN_ID='76',ROS_LOCALHOST_ONLY='0')
log=(out/'runner.log').open('w')
runner=subprocess.Popen(['ros2','run','dragonfly_flight','flight','--mode','ideal_wrench','--scenario','manual','--output',str(out)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('keyboard_probe');state={};status=[];clock=[0.]
node.create_subscription(JointState,'/joint_states',lambda m:state.update(zip(m.name,m.position)),10)
node.create_subscription(String,'/dragonfly/arm_status',lambda m:status.append(m.data),10)
ready=[False]
node.create_subscription(String,'/dragonfly/status',lambda m:ready.__setitem__(0,'ready=True' in m.data),10)
node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
client=None;master=None;checks=[]

def spin_until(predicate,timeout=60):
    deadline=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>deadline:raise TimeoutError('Probe wait timed out; '+str(status[-3:]))
        if runner.poll() is not None:raise RuntimeError('Simulation ended during keyboard probe')
        rclpy.spin_once(node,timeout_sec=.02)

def delay(seconds):
    start=clock[0];spin_until(lambda:clock[0]-start>=seconds,20)

def wait_joint(side,joint,target,tolerance=math.radians(.8),timeout=15):
    """Wait for the effort-driven joint, not merely the trajectory end time."""
    spin_until(lambda:abs(state.get(f'{side}_joint_{joint+1}',math.inf)-target)<tolerance,timeout)

def keyboard():
    master,slave=pty.openpty()
    child=subprocess.Popen(['ros2','run','dragonfly_flight','keyboard'],stdin=slave,stdout=log,stderr=log,start_new_session=True)
    os.close(slave);delay(.7)
    return master,child

try:
    spin_until(lambda:ready[0],100);master,client=keyboard()
    for side,key in [('left','L'),('right','R')]:
        for joint in range(5):
            if len(sys.argv)>2 and joint!=int(sys.argv[2]):continue
            os.write(master,(key+str(joint+1)+'+').encode())
            wait_joint(side,joint,VERTICAL[joint]+math.radians(5))
            delay(2.)
            measured=state[f'{side}_joint_{joint+1}']-VERTICAL[joint]
            assert abs(measured-math.radians(5))<math.radians(.8),(side,joint,'+',measured,status[-3:])
            checks.append(dict(side=side,joint=joint+1,key='+',measured_deg=math.degrees(measured)))
            os.write(master,b'-')
            wait_joint(side,joint,VERTICAL[joint])
            delay(2.)
            returned=state[f'{side}_joint_{joint+1}']-VERTICAL[joint]
            assert abs(returned)<math.radians(.8),(side,joint,'-',returned,status[-3:])
            checks.append(dict(side=side,joint=joint+1,key='-',step_deg=math.degrees(returned-measured)))
            os.write(master,b'H');delay(.7)
    os.write(master,b'P');delay(.8);os.write(master,b' ');delay(1.)
    assert any('Stopped arms' in value for value in status[-10:])
    os.write(master,b'\x1b');client.wait(timeout=5);os.close(master);master=None;client=None
    delay(.7);assert runner.poll() is None
    master,client=keyboard();status.clear()
    os.killpg(client.pid,signal.SIGINT);client.wait(timeout=5)
    os.close(master);master=None;client=None;delay(.7)
    assert any('Stopped arms' in value for value in status) and runner.poll() is None
    delay(3.)
    master,client=keyboard();os.write(master,b'P');delay(.8)
    os.killpg(client.pid,signal.SIGKILL);client.wait();client=None;os.close(master);master=None
    status.clear();delay(1.5);assert any('Stopped arms' in value for value in status)
    # Freeze only this run's controller to verify the independent process monitor.
    children=psutil.Process(runner.pid).children(recursive=True)
    control=next(p for p in children if 'dragonfly_flight.runtime' in p.cmdline())
    control.send_signal(signal.SIGSTOP)
    runner.wait(timeout=12)
    assert runner.returncode!=0 and 'stalled' in (out/'failure.txt').read_text()
    result=dict(passed=True,jog_checks=checks,space=True,escape_keeps_flight=True,ctrl_c_keeps_flight=True,heartbeat_stop=True,controller_stall_abort=True)
    (out/'keyboard_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
    if client and client.poll() is None:os.killpg(client.pid,signal.SIGKILL);client.wait()
    if master is not None:os.close(master)
    if runner.poll() is None:
        os.killpg(runner.pid,signal.SIGINT)
        try:runner.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(runner.pid,signal.SIGKILL);runner.wait()
    node.destroy_node();rclpy.try_shutdown();log.close()
