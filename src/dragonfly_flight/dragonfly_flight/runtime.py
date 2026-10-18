"""Control, PX4 adapter and experiment sequencing. Truth never feeds PX4-mode control."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, PoseStamped, TransformStamped, WrenchStamped, Vector3Stamped
from std_msgs.msg import Bool, String, Float32MultiArray, Float64
from tf2_ros import TransformBroadcaster
from control_msgs.msg import JointTrajectoryControllerState
from px4_msgs.msg import (VehicleLocalPosition, VehicleOdometry, VehicleStatus, VehicleCommand,
                          VehicleCommandAck, OffboardControlMode, ActuatorMotors, ActuatorServos,
                          TrajectorySetpoint, ControlAllocatorStatus, VehicleThrustSetpoint,
                          VehicleTorqueSetpoint, VehicleControlMode)
from .core import Platform, Allocator, NED_ENU, FRD_FLU, px4_rotation, smoothstep, STOWED, VERTICAL
from .arms import ArmCommands
from .clearance import GearClearance
from .effort import ordered_arm_effort_limits, arm_effort_ratios

class Runtime(Node):
    def __init__(self,out,scenario):
        super().__init__('dragonfly_flight',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        data=json.loads((out/'model.json').read_text());self.mode=data['mode'];self.out=out;self.scenario=scenario
        self.model=Platform(data['dimensions'],data['config'],data['mass'],data['com'],data['inertia'])
        self.arm_effort_limits=ordered_arm_effort_limits(data)
        self.vertical_com=self.model.com.copy();self.vertical_inertia=self.model.inertia.copy()
        if self.mode in ('px4_actuators','px4_native'):
            self.model.com=np.asarray(data['stowed_com']);self.model.inertia=np.asarray(data['stowed_inertia'])
        self.alloc=Allocator(self.model) if self.mode!='px4_native' else None
        self.native_outputs={};self.native_times={};self.native_reference=None;self.native_proven=False
        self.truth=None;self.local=None;self.odom=None;self.vehicle=None;self.offset=None
        self.local_time=-1.;self.odom_time=-1.;self.begin=None;self.start_height=None;self.home_xy=None
        self.last_command=-10.;self.last_t=None;self.failures=0;self.done=False;self.error=None;self.last_log=-1.
        self.target=np.array([0.,0.,self.model.cfg['hover_height']]);self.external_target=None
        self.initial_arm_pose=np.asarray(data.get('initial_arm_pose',VERTICAL),dtype=float)
        self.arms=ArmCommands(self,self.model.cfg,GearClearance(out/'robot.urdf'),self.model.arm_lower,self.model.arm_upper,self.initial_arm_pose)
        self.initialized_arms=False;self.replay_index=0;self.replay_started=False
        self.deployment_state='pending' if self.mode in ('px4_actuators','px4_native') else 'complete'
        self.stable_since=None;self.deployment_deadline=None;self.experiment_start=None
        self.events={};self.previous_arm_busy=False
        self.force_probe=None;self.probe_samples=[]
        self.momentum=[];self.last_momentum=None;self.efforts=np.zeros(10);self.rotor_speeds=np.zeros(6)
        self.arm_reference=np.tile(self.initial_arm_pose,2)
        self.tilt_efforts=np.zeros(6)
        for index,side in enumerate(('left','right')):
            self.create_subscription(JointTrajectoryControllerState,f'/{side}_arm_controller/controller_state',
                                     lambda m,i=index:self.on_arm_reference(m,i),qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/dragonfly/status',10)
        self.diagnostics=self.create_publisher(Float32MultiArray,'/dragonfly/diagnostics',10)
        self.reference_pose=self.create_publisher(PoseStamped,'/dragonfly/reference_pose',10)
        self.position_error=self.create_publisher(Vector3Stamped,'/dragonfly/position_error',10)
        self.position_error_norm=self.create_publisher(Float64,'/dragonfly/position_error_norm',10)
        self.attitude_error=self.create_publisher(Vector3Stamped,'/dragonfly/attitude_error',10)
        self.attitude_error_norm=self.create_publisher(Float64,'/dragonfly/attitude_error_norm',10)
        self.arm_active=self.create_publisher(Bool,'/dragonfly/arm_active',10)
        self.ready_pub=self.create_publisher(Bool,'/dragonfly/ready',10)
        self.create_subscription(Odometry,'/dragonfly/truth',self.on_truth,qos_profile_sensor_data)
        self.create_subscription(WrenchStamped,'/dragonfly/momentum',self.on_momentum,qos_profile_sensor_data)
        self.create_subscription(String,'/dragonfly/efforts',lambda m:setattr(self,'efforts',np.fromstring(m.data,sep=',')),qos_profile_sensor_data)
        self.create_subscription(String,'/dragonfly/rotor_speeds',lambda m:setattr(self,'rotor_speeds',np.fromstring(m.data,sep=',')),qos_profile_sensor_data)
        self.create_subscription(String,'/dragonfly/tilt_efforts',lambda m:setattr(self,'tilt_efforts',np.fromstring(m.data,sep=',')),qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/dragonfly/target',self.on_target,10)
        self.tf=TransformBroadcaster(self)
        if self.mode in ('px4_actuators','px4_native'):
            for name,kind,callback in [('vehicle_local_position',VehicleLocalPosition,self.on_local),('vehicle_odometry',VehicleOdometry,self.on_odom),
                                       ('vehicle_status',VehicleStatus,lambda m:setattr(self,'vehicle',m)),
                                       ('vehicle_command_ack',VehicleCommandAck,lambda m:self.get_logger().info(f'PX4 ack {m.command}: {m.result}'))]:
                version=getattr(kind,'MESSAGE_VERSION',0)
                topic='/fmu/out/'+name+(f'_v{version}' if version else '')
                self.create_subscription(kind,topic,callback,qos_profile_sensor_data)
            self.offboard=self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',10)
            self.commands=self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
            if self.mode=='px4_native':
                self.trajectory=self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',10)
                for name,kind in [('actuator_motors',ActuatorMotors),('actuator_servos',ActuatorServos),
                                  ('control_allocator_status',ControlAllocatorStatus),
                                  ('vehicle_thrust_setpoint',VehicleThrustSetpoint),
                                  ('vehicle_torque_setpoint',VehicleTorqueSetpoint),
                                  ('vehicle_control_mode',VehicleControlMode)]:
                    self.create_subscription(kind,'/fmu/out/'+name,
                        lambda m,n=name:self.on_native(n,m),qos_profile_sensor_data)
            else:
                self.motors=self.create_publisher(ActuatorMotors,'/fmu/in/actuator_motors',10)
                self.servos=self.create_publisher(ActuatorServos,'/fmu/in/actuator_servos',10)
        self.ideal=self.create_publisher(Wrench,'/dragonfly/ideal_wrench',10) if self.mode=='ideal_wrench' else None
        self.file=open(out/'flight.csv','w',newline='');self.csv=csv.writer(self.file)
        self.csv.writerow(['t','phase','x','y','z','rx','ry','rz','ex','ey','ez','angle','eroll','epitch','eyaw','est_error','force_residual','torque_residual','saturated']+
                          [f'q{i}' for i in range(10)]+[f'effort{i}' for i in range(10)]+[f'f{i}' for i in range(6)]+[f'tilt{i}' for i in range(6)]+['est_angle']+[f'actual_tilt{i}' for i in range(6)]+[f'actual_f{i}' for i in range(6)]+[f'qref{i}' for i in range(10)]+[f'tilt_effort{i}' for i in range(6)])
        self.timer=self.create_timer(.01,self.tick)
        self.wall_start=time.monotonic();self.last_heartbeat=0.;self.get_logger().info(f'Mode {self.mode}; mass={self.model.mass:.3f}, com={self.model.com}')

    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def on_arm_reference(self,m,index):
        reference=m.reference if hasattr(m,'reference') else m.desired
        if len(reference.positions)==5:self.arm_reference[index*5:index*5+5]=reference.positions
    def on_truth(self,m):
        self.truth=m
        tf=TransformStamped();tf.header.stamp=m.header.stamp;tf.header.frame_id='world';tf.child_frame_id='uav_base_link'
        tf.transform.translation.x=m.pose.pose.position.x;tf.transform.translation.y=m.pose.pose.position.y;tf.transform.translation.z=m.pose.pose.position.z
        tf.transform.rotation=m.pose.pose.orientation;self.tf.sendTransform(tf)
    def on_local(self,m):self.local=m;self.local_time=self.now()
    def on_odom(self,m):self.odom=m;self.odom_time=self.now()
    def on_momentum(self,m):
        self.last_momentum=np.array([m.wrench.force.x,m.wrench.force.y,m.wrench.force.z])
        self.last_momentum_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.mode=='physics':self.momentum.append([self.now(),m.wrench.force.x,m.wrench.force.y,m.wrench.force.z,m.wrench.torque.x,m.wrench.torque.y,m.wrench.torque.z])
    def on_target(self,m):
        values=np.array([m.pose.position.x,m.pose.position.y,m.pose.position.z])
        if m.header.frame_id!='world' or not np.isfinite(values).all():return
        self.external_target=values
    def on_native(self,name,message):
        self.native_outputs[name]=message;self.native_times[name]=self.now()
    def send_native_target(self,target):
        position=NED_ENU@(target-self.offset) if target is not None and self.offset is not None else np.zeros(3)
        if target is None and self.local is not None:position=np.array([self.local.x,self.local.y,self.local.z])
        message=TrajectorySetpoint(timestamp=int(self.now()*1e6),position=position.astype(float).tolist(),
            velocity=[math.nan]*3,acceleration=[math.nan]*3,jerk=[math.nan]*3,yaw=math.pi/2,yawspeed=0.)
        self.trajectory.publish(message)
    def native_state(self,t):
        required=('actuator_motors','actuator_servos','control_allocator_status','vehicle_thrust_setpoint','vehicle_torque_setpoint')
        if any(n not in self.native_outputs or t-self.native_times[n]>.25 for n in required):
            raise RuntimeError('Native PX4 actuator/allocation telemetry missing or older than 0.25 simulation seconds')
        outputs=self.native_outputs
        motors=np.array(outputs['actuator_motors'].control[:6]);servos=np.array(outputs['actuator_servos'].control[:6])
        if not np.isfinite(np.r_[motors,servos]).all():raise RuntimeError('Native PX4 produced non-finite actuator commands')
        z=np.r_[np.clip(motors,0,1)**2*self.model.max_thrust,np.clip(servos,-1,1)*self.model.cfg['tilt_limit']]
        status=outputs['control_allocator_status'];acceleration=9.81*self.model.cfg['thrust_to_weight']
        force=FRD_FLU@np.array(outputs['vehicle_thrust_setpoint'].xyz)*self.model.mass*acceleration
        torque=self.vertical_inertia@(FRD_FLU@np.array(outputs['vehicle_torque_setpoint'].xyz))*(4/.15)
        desired=np.r_[force,torque+np.cross(self.vertical_com,force)]
        residual=np.r_[FRD_FLU@np.array(status.unallocated_thrust)*self.model.mass*acceleration,
            self.vertical_inertia@(FRD_FLU@np.array(status.unallocated_torque))*(4/.15)]
        if not self.native_proven:
            mode=outputs.get('vehicle_control_mode')
            if mode is not None:
                evidence={key:bool(getattr(mode,key)) for key in ('flag_control_position_enabled',
                    'flag_control_velocity_enabled','flag_control_attitude_enabled','flag_control_rates_enabled',
                    'flag_control_allocation_enabled','flag_control_offboard_enabled')}
                evidence.update(mode='px4_native',ros_publishes_actuators=False,
                    actuator_source='PX4 control_allocator',position_setpoint_frame='NED',thrust_frame='FRD')
                if not all(evidence[key] for key in evidence if key.startswith('flag_')):
                    raise RuntimeError('PX4 native control stages are not all enabled: '+str(evidence))
                (self.out/'native_chain.json').write_text(json.dumps(evidence,indent=2));self.native_proven=True
        return z,residual,desired
    def send_command(self,command,**params):
        m=VehicleCommand(timestamp=int(self.now()*1e6),command=command,target_system=1,target_component=1,source_system=1,source_component=1,from_external=True,**params)
        self.commands.publish(m)
    def abort(self,reason):
        self.error=reason;self.done=True;self.arms.ready=False;self.arms.safe=False;self.arms.stop()
        self.get_logger().error(reason);(self.out/'failure.txt').write_text(reason)
    def truth_state(self):
        m=self.truth;v=m.twist.twist;p=m.pose.pose.position;q=m.pose.pose.orientation
        rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        return np.array([p.x,p.y,p.z]),np.array([v.linear.x,v.linear.y,v.linear.z]),rot,rot.T@np.array([v.angular.x,v.angular.y,v.angular.z])

    def publish_monitor(self,tp,target,tr,arm_busy):
        stamp=self.get_clock().now().to_msg();error=tp-target
        reference=PoseStamped();reference.header.stamp=stamp;reference.header.frame_id='world'
        reference.pose.position.x,reference.pose.position.y,reference.pose.position.z=target.tolist();reference.pose.orientation.w=1.
        position=Vector3Stamped();position.header=reference.header
        position.vector.x,position.vector.y,position.vector.z=error.tolist()
        euler=Rotation.from_matrix(tr).as_euler('xyz')
        attitude=Vector3Stamped();attitude.header=reference.header
        attitude.vector.x,attitude.vector.y,attitude.vector.z=euler.tolist()
        self.reference_pose.publish(reference);self.position_error.publish(position)
        self.position_error_norm.publish(Float64(data=float(np.linalg.norm(error))))
        self.attitude_error.publish(attitude)
        self.attitude_error_norm.publish(Float64(data=float(Rotation.from_matrix(tr).magnitude())))
        self.arm_active.publish(Bool(data=arm_busy));self.ready_pub.publish(Bool(data=self.arms.ready))
        return euler

    def update_deployment(self,t,elapsed,stable):
        """Deploy after a genuine hover dwell; return whether airborne work may start."""
        if self.deployment_state=='complete':return True
        if self.deployment_state=='pending':
            if elapsed<=14 or not stable:
                self.stable_since=None;return False
            if self.stable_since is None:self.stable_since=t
            if t-self.stable_since<2:return False
            if not self.arms.move_both(VERTICAL):return False
            # Use one fixed vertical-workspace model for the whole deployment.
            # Do not follow the changing multibody CoM/inertia: its reaction is
            # deliberately left for the flight feedback loop to reject.
            self.model.com=self.vertical_com.copy();self.model.inertia=self.vertical_inertia.copy()
            self.deployment_state='moving';self.deployment_deadline=self.arms.busy_until+5
            self.events['deployment_start']=elapsed;(self.out/'events.json').write_text(json.dumps(self.events,indent=2))
            self.stable_since=None;return False
        if self.deployment_state=='moving':
            if t<self.arms.busy_until:return False
            if not self.arms.at(VERTICAL):
                if t>self.deployment_deadline:self.abort('Arm deployment did not reach the vertical pose within 1 degree')
                return False
            self.deployment_state='recovering';self.events['deployment_end']=elapsed
            (self.out/'events.json').write_text(json.dumps(self.events,indent=2));self.stable_since=None
            return False
        if not stable:
            self.stable_since=None;return False
        if self.stable_since is None:self.stable_since=t
        if t-self.stable_since<2:return False
        self.deployment_state='complete';self.experiment_start=elapsed;self.events['experiment_start']=elapsed
        (self.out/'events.json').write_text(json.dumps(self.events,indent=2));return True

    def tick(self):
        try:self.step()
        except (ValueError,RuntimeError,FloatingPointError) as e:self.abort(str(e))
    def step(self):
        t=self.now()
        if self.done:return
        if self.truth is None or not self.arms.initialized():return
        tp,tv,tr,tw=self.truth_state()
        if not self.initialized_arms:
            self.arms.busy_until=0.;self.arms.move_both(self.initial_arm_pose,constrain=False)
            self.initialized_arms=True
        self.arms.tick()
        arm_busy=bool(self.arms.sequence or t<self.arms.busy_until)
        if self.previous_arm_busy and not arm_busy and self.replay_index:
            self.events['left_end' if self.replay_index==1 else 'right_end']=t-self.begin
            (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
        self.previous_arm_busy=arm_busy
        if self.mode=='physics':
            self.arms.ready=True;self.arms.safe=True
            if t>3 and not self.replay_started:self.arms.replay('left');self.replay_started=True
            q=[self.arms.q[n] for n in sum(ARM_NAMES_LIST,[])]
            target=np.array([0.,0.,1.6]);euler=self.publish_monitor(tp,target,tr,arm_busy)
            self.csv.writerow([t,'physics',*tp,*target,*(tp-target),Rotation.from_matrix(tr).magnitude(),*euler,0,0,0,0,*q,*self.efforts,*np.zeros(25),*self.arm_reference,*self.tilt_efforts])
            if t>16:
                np.savetxt(self.out/'momentum.csv',self.momentum,delimiter=',',header='t,px,py,pz,lx,ly,lz');self.done=True
            return
        if self.mode in ('px4_actuators','px4_native'):
            self.offboard.publish(OffboardControlMode(timestamp=int(t*1e6),
                position=self.mode=='px4_native',direct_actuator=self.mode=='px4_actuators'))
            if self.mode=='px4_native':self.send_native_target(self.native_reference)
            if self.local is None or self.odom is None or self.vehicle is None:return
            if not self.local.xy_valid or not self.local.z_valid:
                if self.begin is not None:self.abort('PX4 position estimate invalid')
                return
            p=NED_ENU@np.array([self.local.x,self.local.y,self.local.z])
            if self.offset is None:self.offset=tp-p;self.home_xy=tp[:2].copy()
            p=p+self.offset;v=NED_ENU@np.array([self.local.vx,self.local.vy,self.local.vz])
            rot=px4_rotation(self.odom.q);omega=FRD_FLU@np.array(self.odom.angular_velocity)
            if self.vehicle.nav_state!=14 or self.vehicle.arming_state!=2:
                if self.begin is not None:self.abort('PX4 left armed Offboard mode');return
                # Align the two origins while landed, before the estimator's
                # altitude reference settles. Never use truth for flight feedback.
                self.offset=tp-NED_ENU@np.array([self.local.x,self.local.y,self.local.z])
                if t-self.last_command>2 and t>5:
                    if self.vehicle.nav_state!=14:self.send_command(176,param1=1.,param2=6.)
                    else:self.send_command(400,param1=1.)
                    self.last_command=t
                return
            if t-min(self.local_time,self.odom_time)>.2:self.abort('PX4 state older than 0.2 simulation seconds');return
        else:p,v,rot,omega=tp,tv,tr,tw
        if not np.isfinite(np.r_[p,v,rot.ravel(),omega]).all():self.abort('Non-finite state');return
        if self.begin is None:
            self.begin=t;self.start_height=p[2];self.home_xy=p[:2].copy();self.get_logger().info('Flight started')
        elapsed=t-self.begin
        target=np.r_[self.home_xy,self.start_height+(self.model.cfg['hover_height']-self.start_height)*smoothstep(elapsed/6)]
        angle=Rotation.from_matrix(tr).magnitude();base_error=np.linalg.norm(tp-target)
        stable=elapsed>14 and base_error<.10 and angle<math.radians(3)
        deployed=self.update_deployment(t,elapsed,stable)
        if self.done:return
        if self.mode not in ('px4_actuators','px4_native') and self.experiment_start is None and stable:
            self.experiment_start=elapsed;self.events['experiment_start']=elapsed
            (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
        test_elapsed=None if self.experiment_start is None else elapsed-self.experiment_start
        if elapsed<14:phase='takeoff'
        elif self.deployment_state=='pending':phase='predeploy'
        elif self.deployment_state=='moving':phase='deploying'
        elif self.deployment_state=='recovering':phase='deployment_recovery'
        else:phase='hover'
        if self.scenario=='acceptance' and test_elapsed is not None and test_elapsed>=30:
            targets=[np.array([.2,0,0]),np.array([.2,.2,0]),np.array([.2,.2,.1]),np.zeros(3)]
            track_elapsed=test_elapsed-30
            k=min(3,int(track_elapsed/10));s=smoothstep((track_elapsed%10)/3)
            start=np.zeros(3) if k==0 else targets[k-1]
            target+=start+(targets[k]-start)*s if track_elapsed<40 else np.zeros(3)
            phase=f'track{k}' if track_elapsed<40 else 'arms'
        if self.scenario=='manual' and deployed and self.external_target is not None:target=self.external_target
        self.target=target
        error=np.linalg.norm(tp-target);stable=elapsed>14 and error<.10 and angle<math.radians(3)
        available=not self.arms.sequence and t>=self.arms.busy_until
        self.arms.ready=bool(deployed and stable and available)
        self.arms.safe=bool(deployed and error<.25 and angle<math.radians(8))
        if self.scenario=='acceptance' and test_elapsed is not None and test_elapsed>73 and self.replay_index<2:
            if self.arms.ready and not self.arms.sequence and t>self.arms.busy_until+5:
                side='left' if self.replay_index==0 else 'right'
                self.arms.replay(side);self.arms.ready=False
                self.events[side+'_start']=elapsed;self.replay_index+=1
                (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
        if phase=='arms':phase='arms_'+('left' if self.replay_index<=1 else 'right')
        if self.mode=='px4_native':
            self.native_reference=target.copy();self.send_native_target(target)
            desired=np.zeros(6)
        else:desired=self.model.control(p,v,rot,omega,target)
        if self.scenario in ('freefall','force_x','force_y','force_z') and elapsed>=14 and self.last_momentum is not None:
            if self.force_probe is None:self.force_probe=(t,self.last_momentum.copy())
            if t-self.force_probe[0]>.1:self.probe_samples.append([self.last_momentum_time,*self.last_momentum])
            force=np.zeros(3)
            if self.scenario!='freefall':
                force[2]=self.model.mass*9.81;force['xyz'.index(self.scenario[-1])]+=1.
            body_force=rot.T@force
            desired=np.r_[body_force,np.cross(self.model.com,body_force)]
            if t-self.force_probe[0]>=.35:
                # Fit after the transport transition, using sensor timestamps.
                samples=np.unique(np.array(self.probe_samples),axis=0)
                measured=np.polyfit(samples[:,0]-samples[0,0],samples[:,1:],1)[0]
                expected=force-np.array([0.,0.,self.model.mass*9.81])
                result=dict(measured_force=measured.tolist(),expected_force=expected.tolist(),
                            passed=bool(np.linalg.norm(measured-expected)<.1))
                (self.out/'diagnostic.json').write_text(json.dumps(result,indent=2));self.done=True
        actual=np.array([self.arms.q.get(f'unit_{u}_{a}_joint',0.) for u in 'abc' for a in ('outer','inner')])
        dt=.01 if self.last_t is None else np.clip(t-self.last_t,.001,.03);self.last_t=t
        if self.mode=='px4_native':
            z,residual,desired=self.native_state(t)
        elif self.ideal:
            msg=Wrench();msg.force.x,msg.force.y,msg.force.z=desired[:3].tolist();msg.torque.x,msg.torque.y,msg.torque.z=desired[3:].tolist()
            self.ideal.publish(msg);z=np.zeros(12);residual=np.zeros(6)
        else:
            try:z,residual=self.alloc.solve(desired,dt);self.failures=0
            except RuntimeError:
                self.failures+=1
                if self.failures>=3:raise
                return
            motors=ActuatorMotors(timestamp=int(t*1e6),timestamp_sample=self.odom.timestamp_sample)
            # THR_MDL_FAC=0, EC_MIN=0: PX4 maps [0,1] linearly to rotor rad/s.
            motors.control=np.r_[np.sqrt(np.clip(z[:6]/self.model.max_thrust,0,1)),np.full(6,np.nan)].astype(float).tolist()
            servos=ActuatorServos(timestamp=int(t*1e6),timestamp_sample=self.odom.timestamp_sample)
            servos.control=np.r_[z[6:]/self.model.cfg['tilt_limit'],np.full(2,np.nan)].astype(float).tolist()
            self.motors.publish(motors);self.servos.publish(servos)
        saturated=bool(np.any(z[:6]>.99*self.model.max_thrust) or np.any(abs(z[6:])>self.model.cfg['tilt_limit']-.01)
                       or np.any(arm_effort_ratios(self.efforts,self.arm_effort_limits)>.99) or np.any(abs(self.tilt_efforts)>.99*self.model.cfg['tilt_effort']))
        q=[self.arms.q[n] for n in sum(ARM_NAMES_LIST,[])]
        actual_f=self.model.kf*self.rotor_speeds**2
        arm_busy=bool(self.arms.sequence or t<self.arms.busy_until)
        euler=self.publish_monitor(tp,target,tr,arm_busy)
        self.csv.writerow([elapsed,phase,*tp,*target,*(tp-target),angle,*euler,np.linalg.norm(p-tp),np.linalg.norm(residual[:3]),np.linalg.norm(residual[3:]),int(saturated),*q,*self.efforts,*z,Rotation.from_matrix(rot.T@tr).magnitude(),*actual,*actual_f,*self.arm_reference,*self.tilt_efforts])
        self.diagnostics.publish(Float32MultiArray(data=[*desired.tolist(),*residual.tolist(),*z.tolist()]))
        if t-self.last_log>2:
            self.status.publish(String(data=f'{phase}: position_error={error:.3f} attitude_deg={math.degrees(angle):.2f} ready={self.arms.ready}'))
            self.get_logger().info(f'{elapsed:.1f}s {phase} p={np.round(tp,3)} error={error:.3f} angle={math.degrees(angle):.2f}');self.file.flush();self.last_log=t
        if elapsed>8 and (error>1.5 or angle>math.radians(60)):self.abort('Flight departed test envelope')
        if self.scenario=='hover' and test_elapsed is not None and test_elapsed>=30:self.done=True
        if self.scenario=='acceptance' and self.replay_index==2 and not self.arms.sequence and t>self.arms.busy_until+6:self.done=True
        if self.scenario=='acceptance' and elapsed>220:self.abort('Acceptance sequence did not finish in 220 seconds')

ARM_NAMES_LIST=[[f'{s}_joint_{i}' for i in range(1,6)] for s in ('left','right')]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--scenario',default='manual')
    args=parser.parse_args();rclpy.init();node=Runtime(args.output,args.scenario)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
            # Keep process liveness separate from paused simulation time.
            wall=time.monotonic()
            if wall-node.last_heartbeat>.1:
                (node.out/'controller.heartbeat').touch();node.last_heartbeat=wall
            if node.begin is None and node.mode!='physics' and time.monotonic()-node.wall_start>150:node.abort('Startup timed out; inspect PX4/controller logs')
    except KeyboardInterrupt:pass
    finally:node.file.close();node.destroy_node();rclpy.try_shutdown()
    if node.error:raise SystemExit(1)

if __name__=='__main__':main()
