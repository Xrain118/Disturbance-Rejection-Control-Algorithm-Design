"""Single arm command owner and small terminal client; no hardware connection."""
import math
import os
import select
import sys
import termios
import time
import tty
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from .core import VERTICAL, ARM_NAMES, duration

def trajectory(names, start, end, speed=.1, acceleration=.2):
    total=duration(end-start,speed,acceleration)
    msg=JointTrajectory(joint_names=names)
    for t in np.linspace(0,total,max(3,int(total/.04)+1)):
        s=t/total; delta=end-start
        p=JointTrajectoryPoint()
        p.positions=(start+delta*(10*s**3-15*s**4+6*s**5)).tolist()
        p.velocities=(delta*(30*s*s-60*s**3+30*s**4)/total).tolist()
        p.accelerations=(delta*(60*s-180*s*s+120*s**3)/total**2).tolist()
        p.time_from_start=Duration(sec=int(t),nanosec=int(t%1*1e9))
        msg.points.append(p)
    return msg,total

class ArmCommands:
    def __init__(self,node,cfg,clearance=None,lower=None,upper=None,initial_pose=None):
        self.node,self.cfg=node,cfg
        self.clearance=clearance
        self.lower=np.full(5,-np.inf) if lower is None else np.asarray(lower,dtype=float)
        self.upper=np.full(5,np.inf) if upper is None else np.asarray(upper,dtype=float)
        self.initial_pose=VERTICAL.copy() if initial_pose is None else np.asarray(initial_pose,dtype=float).copy()
        self.q={};self.v={};self.side='left';self.joint=0
        self.ready=False;self.safe=False;self.busy_until=0.;self.heartbeat=-1.;self.manual=False
        self.target={s:self.initial_pose.copy() for s in ARM_NAMES}; self.sequence=[]
        self.pub={s:node.create_publisher(JointTrajectory,f'/{s}_arm_controller/joint_trajectory',10) for s in ARM_NAMES}
        self.status=node.create_publisher(String,'/dragonfly/arm_status',10)
        node.create_subscription(JointState,'/joint_states',self.state,qos_profile_sensor_data)
        node.create_subscription(String,'/dragonfly/arm_key',self.key,10)
        node.create_subscription(Empty,'/dragonfly/arm_heartbeat',lambda _:setattr(self,'heartbeat',time.monotonic()),10)
        self.state_time=-1.;self.last_report=-1.

    def now(self):return self.node.get_clock().now().nanoseconds*1e-9
    def state(self,msg):
        self.q.update(zip(msg.name,msg.position)); self.v.update(zip(msg.name,msg.velocity)); self.state_time=self.now()
    def positions(self,side):return np.array([self.q[n] for n in ARM_NAMES[side]])
    def report(self,text):self.status.publish(String(data=text))
    def initialized(self):return all(n in self.q for n in ARM_NAMES['left']+ARM_NAMES['right'])
    def limits(self):
        span=math.radians(self.cfg['arm_range_degrees'])
        return np.maximum(VERTICAL-span,self.lower),np.minimum(VERTICAL+span,self.upper)
    def constrain(self,target):
        lower,upper=self.limits();return np.clip(np.asarray(target,dtype=float),lower,upper)
    def at(self,target,tolerance=math.radians(1)):
        return self.initialized() and all(np.max(np.abs(self.positions(side)-target))<=tolerance for side in ARM_NAMES)

    def move(self,side,target,manual=False,constrain=True):
        now=self.now()
        if not self.initialized() or now-self.state_time>.2:
            self.report('Joint state missing or stale');return False
        if now<self.busy_until:return False
        target=self.constrain(target) if constrain else np.asarray(target,dtype=float)
        if self.clearance and not self.clearance.allows(side,self.positions(side),target):
            self.sequence=[];self.report('Rejected: path approaches landing gear');return False
        self.target[side]=target
        msg,dt=trajectory(ARM_NAMES[side],self.positions(side),self.target[side],self.cfg['arm_speed'],self.cfg['arm_acceleration'])
        self.pub[side].publish(msg);self.busy_until=now+dt+.3;self.manual=manual
        self.report(f'{side} target {np.round(self.target[side],3).tolist()}');return True

    def move_both(self,target,manual=False,constrain=True):
        now=self.now()
        if not self.initialized() or now-self.state_time>.2:
            self.report('Joint state missing or stale');return False
        if now<self.busy_until:return False
        target=self.constrain(target) if constrain else np.asarray(target,dtype=float)
        starts={side:self.positions(side) for side in ARM_NAMES}
        if self.clearance and any(not self.clearance.allows(side,starts[side],target) for side in ARM_NAMES):
            self.sequence=[];self.report('Rejected: deployment path approaches landing gear');return False
        commands={side:trajectory(ARM_NAMES[side],starts[side],target,self.cfg['arm_speed'],self.cfg['arm_acceleration']) for side in ARM_NAMES}
        for side,(msg,_) in commands.items():
            self.pub[side].publish(msg);self.target[side]=target.copy()
        self.busy_until=now+max(dt for _,dt in commands.values())+.3;self.manual=manual
        self.report(f'both arms target {np.round(target,3).tolist()}');return True

    def stop(self):
        self.sequence=[];self.manual=False
        if not self.initialized():return
        longest=0.
        for side in ARM_NAMES:
            q=self.positions(side);v=np.array([self.v.get(n,0.) for n in ARM_NAMES[side]])
            total=max(.1,float(np.max(np.abs(v)))/self.cfg['arm_acceleration'])
            end=self.constrain(q+.5*v*total)
            msg=JointTrajectory(joint_names=ARM_NAMES[side])
            for t in np.linspace(0,total,11):
                p=JointTrajectoryPoint();p.positions=self.constrain(q+v*t-.5*v*t*t/total).tolist()
                p.velocities=(v*(1-t/total)).tolist()
                p.time_from_start=Duration(sec=int(t),nanosec=int(t%1*1e9));msg.points.append(p)
            msg.points[-1].positions=end.tolist();self.pub[side].publish(msg);self.target[side]=end;longest=max(longest,total)
        self.busy_until=self.now()+longest+.3;self.report('Stopped arms; flight controller remains active')

    def replay(self,side):
        target=VERTICAL.copy();target[1]+=math.radians(5)
        self.sequence=[(side,VERTICAL.copy(),0.),(side,target,2.),(side,VERTICAL.copy(),0.)]

    def key(self,msg):
        key=msg.data
        if key==' ':self.stop();return
        if key.lower() in ('l','r'):self.side='left' if key.lower()=='l' else 'right'
        elif key in ('1','2','3','4','5'):self.joint=int(key)-1
        elif key in ('+','=','-','h','H','p','P'):
            if not self.ready:self.report('Not ready: wait for stable hover');return
            if time.monotonic()-self.heartbeat>.5:self.report('Keyboard heartbeat missing');return
            if self.sequence or self.now()<self.busy_until:return
            if key.lower()=='p':self.replay(self.side);self.manual=True;return
            target=self.target[self.side].copy()
            if key.lower()=='h':target=VERTICAL.copy()
            else:target[self.joint]+=math.radians(self.cfg['arm_step_degrees'])*(1 if key in ('+','=') else -1)
            bounded=self.constrain(target)
            if np.allclose(bounded,self.target[self.side],atol=1e-12,rtol=0):
                self.report(f'{self.side} J{self.joint+1}: joint or work-envelope limit reached');return
            self.move(self.side,bounded,True)
        self.report(f'Selected {self.side} joint {self.joint+1}')

    def tick(self):
        now=self.now()
        if self.manual:
            reason=None
            if time.monotonic()-self.heartbeat>.5:reason='keyboard heartbeat missing'
            elif now-self.state_time>.2:reason='joint state stale'
            elif not self.safe:reason='flight left the 0.25 m / 8 deg arm-motion envelope'
            if reason:
                self.stop();self.report('Stopped arms: '+reason)
        if self.sequence and now>=self.busy_until:
            side,target,pause=self.sequence[0]
            if self.move(side,target,self.manual):self.sequence.pop(0);self.busy_until+=pause
        if now-self.last_report>1 and self.initialized():
            self.report(f'{self.side} J{self.joint+1}: actual={self.positions(self.side)[self.joint]:.3f} target={self.target[self.side][self.joint]:.3f} '+('moving' if now<self.busy_until else 'idle'))
            self.last_report=now

def keyboard():
    if not sys.stdin.isatty():raise SystemExit('Run keyboard in an interactive WSL terminal.')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('dragonfly_keyboard')
    pub=node.create_publisher(String,'/dragonfly/arm_key',10)
    hb=node.create_publisher(Empty,'/dragonfly/arm_heartbeat',10)
    node.create_subscription(String,'/dragonfly/arm_status',lambda m:print('\r'+m.data+' '*8,flush=True),10)
    saved=termios.tcgetattr(sys.stdin);last=0.
    print('L/R arm | 1-5 joint | +/- 5 degrees | Space stop | H vertical | P replay | Esc exit')
    try:
        tty.setcbreak(sys.stdin)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()-last>.1:hb.publish(Empty());last=time.monotonic()
            if select.select([sys.stdin],[],[],.01)[0]:
                key=os.read(sys.stdin.fileno(),1).decode()
                if key in ('\x1b','\x03'):break
                pub.publish(String(data=key))
    except KeyboardInterrupt:pass
    finally:
        pub.publish(String(data=' '));rclpy.spin_once(node,timeout_sec=.1)
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,saved);node.destroy_node();rclpy.shutdown()
