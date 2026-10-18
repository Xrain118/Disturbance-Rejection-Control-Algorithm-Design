"""ROS adapter. Launching acquisition never launches flight or sends motion commands."""
import argparse
import json
import math
import queue
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dragonfly_interfaces.msg import ServoTelemetry, ArmTelemetry, JointTorqueEstimate
from dragonfly_interfaces.srv import ConfigureTelemetry
from .protocol import parse, ProtocolError, Quality, TorqueFilter, joint_positions, joint_torques
from .transport import PortWorker, Recorder, replay_events, validate_ports

SIDES = ('left','right')

class TelemetryNode(Node):
    def __init__(self, replay=False):
        super().__init__('dual_arm_serial')
        self.replay = replay
        self.events = queue.Queue(maxsize=512)
        self.workers = {}
        self.config = {}
        self.quality = {s:Quality() for s in SIDES}
        self.connected = {s:False for s in SIDES}
        self.last_metrics = {s:{} for s in SIDES}
        self.errors = {s:0 for s in SIDES}
        self.detail = {s:'waiting for telemetry' for s in SIDES}
        self.stale_timeout = self.declare_parameter('stale_timeout',.5).value
        reconnect = self.declare_parameter('reconnect_interval',2.).value
        alpha = self.declare_parameter('filter_alpha',0.).value
        if not all(math.isfinite(x) and x>0 for x in (self.stale_timeout,reconnect)):
            raise ValueError('timeouts must be positive')
        self.filters = {s:TorqueFilter(alpha) for s in SIDES}
        self.record = Recorder(self.declare_parameter('record_path','').value)
        self.joint_pub = self.create_publisher(JointState,'/hardware/joint_states',qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(DiagnosticArray,'/hardware/diagnostics',10)
        self.telemetry_pub, self.torque_pub, self.filtered_pub, self.ack_pub = {}, {}, {}, {}
        for side in SIDES:
            defaults = dict(port='/dev/serial/by-id/REPLACE_'+side.upper()+'_ESP32', device_id='',
                protocol='v1', baud=0, position_sign=[1.]*5, position_offset=[0.]*5,
                torque_sign=[-1.,1.,-1.,1.,1.,-1.,1.], speed_rad_s_per_raw=0.)
            cfg = {k:self.declare_parameter(side+'.'+k,v).value for k,v in defaults.items()}
            if cfg['protocol'] not in ('v1','legacy'):
                raise ValueError('protocol must be v1 or legacy')
            if cfg['protocol']=='legacy' and cfg['device_id']:
                raise ValueError('legacy protocol cannot verify device_id')
            cfg['baud'] = cfg['baud'] or (115200 if cfg['protocol']=='legacy' else 921600)
            for key,n in [('position_sign',5),('position_offset',5),('torque_sign',7)]:
                if len(cfg[key]) != n or not all(math.isfinite(x) for x in cfg[key]):
                    raise ValueError('invalid '+key)
            if not math.isfinite(cfg['speed_rad_s_per_raw']) or cfg['speed_rad_s_per_raw'] < 0:
                raise ValueError('speed scale must be nonnegative')
            self.config[side] = cfg
            prefix = '/hardware/'+side+'_arm'
            self.telemetry_pub[side] = self.create_publisher(ArmTelemetry,prefix+'/telemetry',qos_profile_sensor_data)
            self.torque_pub[side] = self.create_publisher(JointTorqueEstimate,prefix+'/joint_torque_estimate',qos_profile_sensor_data)
            self.ack_pub[side] = self.create_publisher(String,prefix+'/config_ack',10)
            if alpha:
                self.filtered_pub[side] = self.create_publisher(JointTorqueEstimate,prefix+'/joint_torque_estimate_filtered',qos_profile_sensor_data)
            self.create_service(ConfigureTelemetry,prefix+'/configure_telemetry',
                lambda req,res,s=side:self.configure(s,req,res))
        if self.config['left']['device_id'] and self.config['left']['device_id']==self.config['right']['device_id']:
            raise ValueError('left and right device identities must differ')
        for side,cfg in self.config.items():
            self.record.write(dict(side=side,kind='configuration',mono_ns=time.monotonic_ns(),
                wall_ns=time.time_ns(),config=cfg,filter_alpha=alpha,stale_timeout=self.stale_timeout))
        if not replay:
            validate_ports(self.config['left']['port'],self.config['right']['port'])
            for side,cfg in self.config.items():
                worker = PortWorker(side,cfg['port'],cfg['baud'],self.events,self.record,reconnect)
                self.workers[side] = worker
                worker.start()
        self.create_timer(.005,self.drain)
        self.create_timer(.5,self.diagnostics)

    def configure(self,side,req,res):
        if side not in self.workers or self.config[side]['protocol']!='v1' or not self.connected[side]:
            res.accepted=False; res.message='No connected v1 device (replay never writes).'
            return res
        try:
            self.workers[side].configure(req.operation,req.rate_hz)
            res.accepted=True; res.message='Queued; inspect config_ack for device confirmation.'
        except (ValueError,queue.Full) as exc:
            res.accepted=False; res.message=str(exc)
        return res

    def drain(self):
        for _ in range(256):
            try:
                self.handle(self.events.get_nowait())
            except queue.Empty:
                break

    def handle(self,event):
        side=event['side']; kind=event['kind']
        if kind=='configuration':
            if not self.replay:
                return
            try:
                source=event['config']
                keys=('device_id','protocol','position_sign','position_offset','torque_sign','speed_rad_s_per_raw')
                cfg={**self.config[side],**{k:source[k] for k in keys}}
                if cfg['protocol'] not in ('v1','legacy') or not isinstance(cfg['device_id'],str):
                    raise ValueError('invalid recorded protocol/identity')
                for key,size in [('position_sign',5),('position_offset',5),('torque_sign',7)]:
                    if len(cfg[key])!=size or not all(math.isfinite(x) for x in cfg[key]):
                        raise ValueError('invalid recorded '+key)
                scale=cfg['speed_rad_s_per_raw'];timeout=event['stale_timeout']
                if not math.isfinite(scale) or scale<0 or not math.isfinite(timeout) or timeout<=0:
                    raise ValueError('invalid recorded scale/timeout')
                filt=TorqueFilter(event['filter_alpha'])
                self.config[side]=cfg;self.filters[side]=filt;self.stale_timeout=timeout
                if filt.alpha and side not in self.filtered_pub:
                    self.filtered_pub[side]=self.create_publisher(JointTorqueEstimate,
                        '/hardware/'+side+'_arm/joint_torque_estimate_filtered',qos_profile_sensor_data)
            except (ValueError,TypeError,KeyError) as exc:
                self.errors[side]+=1;self.detail[side]=str(exc)
                raise ValueError('invalid replay configuration; refusing different conversion') from exc
            return
        if kind in ('connected','disconnected'):
            self.connected[side]=kind=='connected'
            self.quality[side].last_receive=None
            self.filters[side].previous=[None]*5
            self.detail[side]=event.get('error',kind)
            return
        if kind=='framing_error':
            self.errors[side]+=event.get('count',1)
            return
        if kind!='line':
            return
        raw=event['raw']
        try:
            cfg=self.config[side]
            d=json.loads(raw)
            if isinstance(d,dict) and d.get('T')==6061:
                self.ack_pub[side].publish(String(data=raw))
                return
            expected=cfg['device_id'] or self.quality[side].last_device or ''
            frame=parse(raw,cfg['protocol'],expected)
            if frame is None:
                return
            other='right' if side=='left' else 'left'
            if not frame.legacy and frame.data['dev']==self.quality[other].last_device:
                raise ProtocolError('same device identity observed on both arms')
            was_stale=self.quality[side].stale(event['mono_ns']/1e9,self.stale_timeout)
            reset=self.quality[side].accept(frame,event['mono_ns']/1e9) or was_stale
            if not frame.legacy:
                self.last_metrics[side]=dict(invalid_servos=frame.data['ok'].count(False),
                    scan_us=frame.data['scan_us'],overruns=frame.data['overrun'],
                    device_dropped=frame.data['dropped'],target_hz=frame.data['rate'],
                    fault_servos=sum(bool(a or b) for a,b in zip(frame.data['status'],frame.data['error'])))
            self.connected[side]=True
            self.detail[side]='legacy measurement validity unknown' if frame.legacy else (
                'uncalibrated; one or more servo reads invalid' if not all(frame.data['ok'])
                else 'bus reads valid; torque uncalibrated; ADC sample times unknown')
            self.publish_frame(side,frame,event['wall_ns'],reset)
        except (ProtocolError,ValueError,TypeError,KeyError) as exc:
            self.errors[side]+=1
            self.detail[side]=str(exc)
            self.record.write(dict(side=side,kind='parse_error',mono_ns=event['mono_ns'],
                                   wall_ns=event['wall_ns'],error=str(exc),raw=raw))

    def publish_frame(self,side,frame,wall_ns,reset=False):
        cfg=self.config[side]; d=frame.data
        msg=ArmTelemetry()
        msg.header.stamp.sec=wall_ns//10**9
        msg.header.stamp.nanosec=wall_ns%10**9
        msg.header.frame_id=side+'_base_link'
        msg.raw_json=frame.raw; msg.legacy=frame.legacy
        msg.estimate_method='unavailable' if frame.legacy else d['method']
        msg.calibration_status='unknown' if frame.legacy else d['cal']
        msg.parameter_version='' if frame.legacy else d['params']
        if not frame.legacy:
            msg.device_id=d['dev']; msg.boot_id=d['boot']; msg.sequence=d['seq']
            msg.device_time_us=d['t_us']; msg.target_rate_hz=float(d['rate'])
            msg.scan_duration_us=d['scan_us']; msg.overruns=d['overrun']; msg.dropped_frames=d['dropped']
            for i,servo_id in enumerate(d['ids']):
                servo=ServoTelemetry(); servo.id=servo_id; servo.valid=d['ok'][i]
                if servo.valid:
                    servo.read_completed_us=d['read_us'][i]
                    servo.position_raw=d['pos'][i]; servo.speed_raw=d['speed'][i]
                    servo.current_raw=d['current'][i]; servo.load_raw=d['load'][i]; servo.status=d['status'][i]; servo.response_error=d['error'][i]
                    servo.voltage_v=d['voltage'][i]*.1; servo.temperature_c=float(d['temperature'][i])
                    servo.current_a=d['current'][i]*.0065; servo.torque_estimate_nm=float(d['tau'][i])
                else:
                    servo.voltage_v=servo.temperature_c=servo.current_a=servo.torque_estimate_nm=float('nan')
                msg.servos.append(servo)
        self.telemetry_pub[side].publish(msg)
        joints=JointState(); joints.header=msg.header
        for j,value in joint_positions(frame,cfg['position_sign'],cfg['position_offset']):
            joints.name.append(f'{side}_joint_{j+1}'); joints.position.append(value)
            if not frame.legacy and cfg['speed_rad_s_per_raw']>0:
                i=(0,1,3,4,5)[j]; direction=(-1,1,1,1,-1)[j]*cfg['position_sign'][j]
                joints.velocity.append(d['speed'][i]*cfg['speed_rad_s_per_raw']*direction)
        if joints.name:
            self.joint_pub.publish(joints)
        torque=JointTorqueEstimate(); torque.header=msg.header
        torque.names=[f'{side}_joint_{j}' for j in range(1,6)]
        values=joint_torques(frame,cfg['torque_sign'])
        torque.valid=[x is not None for x in values]
        torque.estimate_nm=[float('nan') if x is None else float(x) for x in values]
        torque.estimate_method=msg.estimate_method; torque.calibration_status=msg.calibration_status
        torque.parameter_version=msg.parameter_version
        self.torque_pub[side].publish(torque)
        if side in self.filtered_pub and self.filters[side].alpha:
            filtered=self.filters[side].update(values,reset)
            torque.estimate_nm=[float('nan') if x is None else float(x) for x in filtered]
            torque.estimate_method += '_ema'
            self.filtered_pub[side].publish(torque)

    def diagnostics(self):
        msg=DiagnosticArray(); msg.header.stamp=self.get_clock().now().to_msg()
        now=time.monotonic()
        for side in SIDES:
            q=self.quality[side]; cfg=self.config[side]
            stale=q.stale(now,self.stale_timeout)
            if stale:
                self.filters[side].previous=[None]*5
            stat=DiagnosticStatus(); stat.name=side+'_arm/telemetry'; stat.hardware_id=q.last_device or cfg['port']
            stat.level=DiagnosticStatus.ERROR if stale or not self.connected[side] else DiagnosticStatus.WARN
            stat.message='stale/disconnected' if stale or not self.connected[side] else self.detail[side]
            elapsed=0 if q.first_receive is None else now-q.first_receive
            values=dict(connected=self.connected[side],frames=q.frames,parse_errors=self.errors[side],
                        sequence_gaps=q.gaps,device_resets=q.resets,
                        age_s='unknown' if q.last_receive is None else now-q.last_receive,
                        average_hz=0 if elapsed<=0 else max(0,q.frames-1)/elapsed,
                        host_queue_dropped=self.workers[side].local_dropped if side in self.workers else 0,
                        recording_error=self.record.error or "",detail=self.detail[side],**self.last_metrics[side])
            stat.values=[KeyValue(key=k,value=str(v)) for k,v in values.items()]
            msg.status.append(stat)
        self.diag_pub.publish(msg)

    def destroy_node(self):
        for worker in self.workers.values():
            worker.close()
        self.record.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=TelemetryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def replay_main():
    parser=argparse.ArgumentParser()
    parser.add_argument('path')
    parser.add_argument('--speed',type=float,default=1.)
    args,ros_args=parser.parse_known_args()
    if not math.isfinite(args.speed) or args.speed<=0:
        parser.error('--speed must be positive')
    rclpy.init(args=ros_args)
    node=TelemetryNode(replay=True)
    try:
        previous=None
        for event in replay_events(args.path):
            if previous is not None:
                deadline=time.monotonic()+max(0,event['mono_ns']-previous)/1e9/args.speed
                while rclpy.ok() and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=min(.05,max(0,deadline-time.monotonic())))
            previous=event['mono_ns']
            event=dict(event,mono_ns=time.monotonic_ns())
            node.handle(event)
            rclpy.spin_once(node,timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
