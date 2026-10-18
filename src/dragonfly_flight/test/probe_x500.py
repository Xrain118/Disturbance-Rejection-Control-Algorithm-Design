"""Read-only-to-source native PX4/Gazebo/DDS integration probe."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition

def main():
    px4=Path(os.environ.get('PX4_ROOT', str(Path.home()/'PX4-Autopilot')));out=Path('log/flight_baseline/x500').resolve();out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='74',GZ_PARTITION='dragonfly_x500_probe',HEADLESS='1',
        PX4_SYS_AUTOSTART='4001',PX4_SIM_MODEL='gz_x500',PX4_GZ_WORLD='default',PX4_UXRCE_DDS_PORT='8889',
        PX4_GZ_MODELS=str(px4/'Tools/simulation/gz/models'),PX4_GZ_WORLDS=str(px4/'Tools/simulation/gz/worlds'),
        GZ_SIM_RESOURCE_PATH=str(px4/'Tools/simulation/gz/models'))
    env['GZ_SIM_SERVER_CONFIG_PATH']=str(px4/'src/modules/simulation/gz_bridge/server.config')
    os.environ['ROS_DOMAIN_ID']='74';processes=[];logs=[]
    rclpy.init();node=rclpy.create_node('x500_probe');samples=[]
    node.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position',lambda m:samples.append(m),qos_profile_sensor_data)
    try:
        for cmd,name in [(['MicroXRCEAgent','udp4','-p','8889'],'dds'),
                         ([str(px4/'build/px4_sitl_default/bin/px4'),'-d',str(px4/'build/px4_sitl_default/etc')],'px4')]:
            log=open(out/(name+'.log'),'w');logs.append(log)
            processes.append(subprocess.Popen(cmd,cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
            if len(samples)>10 and samples[-1].xy_valid and samples[-1].z_valid:break
        result=dict(samples=len(samples),position_valid=bool(samples and samples[-1].xy_valid and samples[-1].z_valid))
        (out/'result.json').write_text(json.dumps(result,indent=2));print(result,flush=True)
        if not result['position_valid']:raise RuntimeError('Native PX4 probe failed')
    finally:
        for p in processes:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            # Gazebo can outlive PX4's main process but shares its process group.
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
