"""One process owner for a reproducible run; only children of this run are stopped."""
import argparse
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shlex
import socket
import subprocess
import sys
import time
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from .model import generate

PX4_DDS_PORT = 8888
PX4_RUN_LOCK = Path('/tmp/dragonfly_flight_px4.lock')


def _processes_using_executable(executable):
    """Return (pid, command) entries whose argv[0] is the requested executable."""
    executable = Path(executable).resolve()
    matches = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry/'cmdline').read_bytes().split(b'\0')
            if argv[0] and Path(os.fsdecode(argv[0])).resolve() == executable:
                matches.append((int(entry.name), ' '.join(os.fsdecode(arg) for arg in argv if arg)))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return matches


def _check_px4_resources(px4_binary, port=PX4_DDS_PORT):
    """Fail before starting Gazebo when PX4 instance 0 or its DDS port is busy."""
    blockers = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(('0.0.0.0', port))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        blockers.append(f'UDP {port} is already in use')
    finally:
        probe.close()

    px4_processes = _processes_using_executable(px4_binary)
    if px4_processes:
        pids = ', '.join(str(pid) for pid, _ in px4_processes)
        blockers.append(f'PX4 instance 0 is already running (PID {pids})')
    if blockers:
        details = '; '.join(blockers)
        raise RuntimeError(
            'Cannot start a second Dragonfly PX4 simulation: '+details+'. '
            'The previous simulation is still active. Return to its main terminal and press Ctrl+C, '
            'wait for "Stopping this run.", then start again.'
        )


def _acquire_px4_lock(px4_binary):
    """Serialize preflight/startup and retain the lock for this runner's lifetime."""
    lock = PX4_RUN_LOCK.open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(
            'Cannot start a second Dragonfly PX4 simulation: another launcher owns the run lock. '
            'Return to its main terminal and press Ctrl+C before starting again.'
        ) from None
    try:
        _check_px4_resources(px4_binary)
    except Exception:
        lock.close()
        raise
    lock.seek(0)
    lock.truncate()
    lock.write(str(os.getpid()))
    lock.flush()
    return lock

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=['ideal_wrench','px4_native','px4_actuators','px4_wrench','physics'],default='px4_native')
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--scenario',choices=['hover','acceptance','physics','manual','freefall','force_x','force_y','force_z'],default='manual')
    parser.add_argument('--output',default='log/flight_'+time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--step',type=float,default=.002)
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--feedforward',action='store_true',help='Enable model-difference feedforward in px4_wrench mode')
    parser.add_argument('--px4',default=os.environ.get('PX4_ROOT', str(Path.home()/'PX4-Autopilot')))
    args=parser.parse_args()
    if args.scenario in ('freefall','force_x','force_y','force_z') and args.mode!='ideal_wrench':
        parser.error('Force diagnostics require --mode ideal_wrench')
    if args.scenario=='physics' and args.mode!='physics':parser.error('Use --mode physics --scenario physics together')
    if args.feedforward and args.mode!='px4_wrench':parser.error('--feedforward requires --mode px4_wrench')
    run_lock = None
    if args.mode in ('px4_actuators', 'px4_native', 'px4_wrench'):
        px4_binary = Path(args.px4)/'build/px4_sitl_default/bin/px4'
        try:
            run_lock = _acquire_px4_lock(px4_binary)
        except RuntimeError as error:
            parser.exit(1, f'ERROR: {error}\n')
    out=Path(args.output).resolve()
    if (out/'world.sdf').exists():raise FileExistsError('Use a new output directory to preserve previous evidence: '+str(out))
    out.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID','73')
    # The native XRCE Agent does not use ROS_LOCALHOST_ONLY; both sides must
    # discover the same FastDDS interfaces. Domain and GZ partition isolate runs.
    os.environ['ROS_LOCALHOST_ONLY']='0'
    os.environ.setdefault('GZ_PARTITION','dragonfly_flight_'+str(os.getpid()))
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']=get_package_prefix('dragonfly_flight')+'/lib:'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    metadata=generate(get_package_share_directory('dragonfly_arm'),get_package_share_directory('dragonfly_flight'),out,args.mode,args.step)
    (out/'run.json').write_text(json.dumps(vars(args),indent=2))
    children=[]; logs=[]; critical=[]
    def start(cmd,name,**kwargs):
        log=open(out/(name+'.log'),'w'); logs.append(log)
        proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,**kwargs)
        children.append(proc); return proc
    try:
        gz=start(['gz','sim','-r']+([] if args.gui else ['-s'])+['-v','2',str(out/'world.sdf')],'gazebo')
        bridge_args=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                     '/dragonfly/truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     '/dragonfly/momentum@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
                     '/dragonfly/efforts@std_msgs/msg/String[gz.msgs.StringMsg',
                     '/dragonfly/rotor_speeds@std_msgs/msg/String[gz.msgs.StringMsg']
        bridge_args.append('/dragonfly/tilt_efforts@std_msgs/msg/String[gz.msgs.StringMsg')
        if args.mode=='ideal_wrench': bridge_args.append('/dragonfly/ideal_wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench')
        start(['ros2','run','ros_gz_bridge','parameter_bridge']+bridge_args,'bridge')
        start(['ros2','run','robot_state_publisher','robot_state_publisher',str(out/'robot.urdf'),
               '--ros-args','-p','use_sim_time:=true'],'state_publisher')
        start(['ros2','run','dragonfly_arm','kinematics_node','--ros-args','-p','use_sim_time:=true'],'kinematics')
        spawners=[]
        for c in ('joint_state_broadcaster','left_arm_controller','right_arm_controller'):
            spawners.append(start(['ros2','run','controller_manager','spawner',c,'--controller-manager-timeout','90',
                   '--param-file',str(out/'controllers.yaml')],c))
        for process in spawners:
            if process.wait(timeout=100):raise RuntimeError('Controller activation failed; see spawner logs')
        if args.mode in ('px4_actuators', 'px4_native', 'px4_wrench'):
            px4=Path(args.px4)
            version=subprocess.check_output(['git','-C',str(px4),'describe','--tags','--always'],text=True).strip()
            if version!='v1.16.2': raise RuntimeError('This profile is verified for PX4 v1.16.2, found '+version)
            params={'SIM_GZ_EN':1,'CA_ROTOR_COUNT':6,'COM_RC_IN_MODE':4,'COM_ARM_WO_GPS':0,'NAV_DLL_ACT':0,
                    # Direct actuator mode bypasses the thrust setpoint used by
                    # the stock land detector. This SITL runner owns shutdown.
                    'COM_DISARM_PRFLT':-1,'COM_DISARM_LAND':-1,
                    'COM_OF_LOSS_T':.5,'COM_OBL_RC_ACT':6,'THR_MDL_FAC':0,
                    'UXRCE_DDS_DOM_ID':int(os.environ['ROS_DOMAIN_ID']), 'UXRCE_DDS_SYNCT':1,
                    # Match Gazebo Harmonic's magnetic table at the configured
                    # Zurich origin; PX4's newer WMM table otherwise adds bias.
                    'EKF2_DECL_TYPE':2,'EKF2_MAG_DECL':2.5636782}
            for i in range(1,7):
                params.update({f'SIM_GZ_EC_FUNC{i}':100+i,f'SIM_GZ_EC_MIN{i}':0,f'SIM_GZ_EC_MAX{i}':1000,
                               f'SIM_GZ_SV_FUNC{i}':200+i,f'SIM_GZ_SV_MINA{i}':-90,f'SIM_GZ_SV_MAXA{i}':90})
            if args.mode in ('px4_native','px4_wrench'):
                from .native import px4_parameters
                params.update(px4_parameters(metadata))
            else:
                params.update(DF_EN=0, CA_AIRFRAME=0, CA_METHOD=1)
            env=os.environ.copy(); env.update({f'PX4_PARAM_{k}':str(v) for k,v in params.items()})
            env.update(PX4_SYS_AUTOSTART='40160' if args.mode in ('px4_native','px4_wrench') else '4001',PX4_GZ_STANDALONE='1',PX4_GZ_MODEL_NAME='dragonfly',PX4_GZ_WORLD='default',
                       PX4_UXRCE_DDS_PORT='8888',PX4_GZ_MAG_ENU='1')
            (out/'px4_parameters.json').write_text(json.dumps(params,indent=2))
            # Apply after the stock airframe's set-default commands; setting a
            # parameter to its compiled default beforehand can be overwritten.
            startup=out/'px4_startup.sh'
            startup.write_text('. '+shlex.quote(str(px4/'build/px4_sitl_default/etc/init.d-posix/rcS'))+'\n'+
                               '\n'.join(f'param set {k} {v}' for k,v in params.items())+'\n')
            instance=out/'px4'; instance.mkdir(exist_ok=True)
            critical.append(('DDS agent',start(['MicroXRCEAgent','udp4','-p',str(PX4_DDS_PORT)],'dds')))
            critical.append(('PX4',start([str(px4/'build/px4_sitl_default/bin/px4'),'-d',str(px4/'build/px4_sitl_default/etc'),'-s',os.path.relpath(startup,instance)],
                  'px4',cwd=instance,env=env)))
            if args.mode=='px4_wrench':
                model_version=hashlib.sha256((out/'model.json').read_bytes()).hexdigest()
                critical.append(('FCU bridge',start(['ros2','launch','dragonfly_fcu_bridge','fcu_bridge.launch.py',
                    'model_metadata:='+str(out/'model.json'),'expected_model_version:='+model_version],'fcu_bridge')))
        record_topics=['/clock','/dragonfly/truth','/joint_states','/dragonfly/efforts',
                   '/dragonfly/rotor_speeds','/dragonfly/tilt_efforts','/dragonfly/arm_status','/dragonfly/status','/dragonfly/diagnostics',
                   '/dragonfly/reference_pose','/dragonfly/position_error','/dragonfly/position_error_norm',
                   '/dragonfly/attitude_error','/dragonfly/attitude_error_norm','/dragonfly/arm_active','/dragonfly/ready',
                   '/left_arm_controller/controller_state','/right_arm_controller/controller_state',
                   '/fmu/out/vehicle_odometry','/fmu/out/vehicle_status_v1','/fmu/in/offboard_control_mode',
                   '/fmu/in/actuator_motors','/fmu/in/actuator_servos','/fmu/in/trajectory_setpoint',
                   '/fmu/out/actuator_motors','/fmu/out/actuator_servos','/fmu/out/control_allocator_status',
                   '/fmu/out/vehicle_thrust_setpoint','/fmu/out/vehicle_torque_setpoint','/fmu/out/vehicle_control_mode']
        if args.mode=='px4_wrench':
            record_topics+=['/research/body_wrench_command','/research/fcu_bridge/diagnostics',
                            '/fmu/in/vehicle_thrust_setpoint','/fmu/in/vehicle_torque_setpoint']
            if metadata['config'].get('ndob',{}).get('enabled',False):
                record_topics+=['/research/ndob/estimated_wrench','/research/ndob/compensation_wrench',
                                '/research/ndob/status']
        if args.record:start(['ros2','bag','record','-o',str(out/'bag')]+record_topics,'rosbag')
        runtime_cmd=[sys.executable,'-m','dragonfly_flight.runtime','--output',str(out),'--scenario',args.scenario]
        if args.feedforward:runtime_cmd.append('--feedforward')
        runtime=start(runtime_cmd,'controller')
        print(f'Run: {out}\nKeyboard: bash src/dragonfly_flight/scripts/sim.sh keyboard\nMonitor: bash src/dragonfly_flight/scripts/sim.sh monitor',flush=True)
        while runtime.poll() is None:
            if gz.poll() is not None: raise RuntimeError('Gazebo exited; see gazebo.log')
            exited=[f'{name} (exit {process.returncode})' for name,process in critical if process.poll() is not None]
            if exited:
                raise RuntimeError('Critical process exited: '+', '.join(exited)+f'; see {out}/dds.log and px4.log')
            heartbeat=out/'controller.heartbeat'
            if heartbeat.exists() and time.time()-heartbeat.stat().st_mtime>1:
                raise RuntimeError('Flight controller stalled for one wall-clock second')
            time.sleep(.25)
        if runtime.returncode: raise RuntimeError(f'Controller exited ({runtime.returncode}); see controller.log')
    except KeyboardInterrupt:
        print('Stopping this run.',flush=True)
    except Exception as error:
        failure=out/'failure.txt'
        failure.write_text((failure.read_text()+'\n' if failure.exists() else '')+str(error));raise
    finally:
        # Stop physics before stopping the controller/transport on an abort.
        if children and children[0].poll() is None: os.killpg(children[0].pid,signal.SIGSTOP)
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGCONT); os.killpg(p.pid,signal.SIGINT)
        deadline=time.monotonic()+5
        for p in children:
            try: p.wait(timeout=max(.01,deadline-time.monotonic()))
            except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()
        for f in logs: f.close()

if __name__=='__main__': main()
