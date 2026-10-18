import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory
from dragonfly_flight.core import STOWED, VERTICAL, ARM_NAMES
from dragonfly_flight.clearance import GearClearance, boxes
from dragonfly_flight.model import link_poses

def test_jogs_replay_and_gear_rejection(tmp_path):
    root=ET.fromstring(subprocess.check_output(['xacro',get_package_share_directory('dragonfly_arm')+'/urdf/dragonfly_arm.urdf.xacro']))
    root.remove(root.find("joint[@name='world_to_uav']"))
    path=tmp_path/'robot.urdf';ET.ElementTree(root).write(path)
    guard=GearClearance(path)
    for side in ('left','right'):
        assert guard.allows(side,STOWED,VERTICAL)
        for joint in range(5):
            for sign in (-1,1):
                if joint==2 and sign<0:continue
                q=VERTICAL.copy();q[joint]+=sign*np.radians(5)
                assert guard.allows(side,VERTICAL,q)
                assert guard.allows(side,q,VERTICAL)
        replay=VERTICAL.copy();replay[1]+=np.radians(5)
        assert guard.allows(side,VERTICAL,replay) and guard.allows(side,replay,VERTICAL)

    names=[side+'_'+part+'_link' for side in ARM_NAMES for part in ('upper_arm','forearm','wrist','gripper')]
    for pose,base_height in ((STOWED,.60),(VERTICAL,1.60)):
        positions={name:pose[i] for arm in ARM_NAMES.values() for i,name in enumerate(arm)}
        moving=boxes(root,link_poses(root,positions),names)
        lowest=min(center[2]-(abs(rotation)@half)[2] for _,center,rotation,half in moving)
        assert base_height+lowest>0
