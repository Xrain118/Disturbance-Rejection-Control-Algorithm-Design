"""Conservative OBB sweep of the jog envelope against the landing gear.

Cylinders use enclosing boxes. This is an offline model check, not a planner.
"""
import itertools
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
from dragonfly_flight.core import HOME, ARM_NAMES
from dragonfly_flight.model import link_poses, origin

def boxes(robot,poses,names):
    output=[]
    for link in robot.findall('link'):
        if link.get('name') not in names:continue
        for collision in link.findall('collision'):
            g=collision.find('geometry');t=poses[link.get('name')]@origin(collision.find('origin'))
            if g.find('box') is not None:half=np.fromstring(g.find('box').get('size'),sep=' ')/2
            elif g.find('cylinder') is not None:
                c=g.find('cylinder');half=np.array([float(c.get('radius')),float(c.get('radius')),float(c.get('length'))/2])
            else:continue
            output.append((link.get('name'),t[:3,3],t[:3,:3],half))
    return output

def overlap(a,b):
    _,pa,ra,ha=a;_,pb,rb,hb=b
    delta=pb-pa
    if np.any(abs(delta)>abs(ra)@ha+abs(rb)@hb):return False
    axes=[*ra.T,*rb.T]+[np.cross(x,y) for x in ra.T for y in rb.T]
    return all(abs(axis@delta)<=abs(axis@ra)@ha+abs(axis@rb)@hb+1e-9 for axis in axes)

def check(path,span):
    robot=ET.parse(path).getroot();nominal={n:HOME[i] for ns in ARM_NAMES.values() for i,n in enumerate(ns)}
    gear=[l.get('name') for l in robot.findall('link') if any(x in l.get('name') for x in ('front_leg','rear_leg','skid'))]
    fixed=boxes(robot,link_poses(robot),gear);failures=[]
    for side in ARM_NAMES:
        names=[side+'_'+s+'_link' for s in ('upper_arm','forearm','wrist','gripper')]
        for offsets in itertools.product((-span,0,span),repeat=5):
            q=nominal.copy();q.update(zip(ARM_NAMES[side],HOME+np.radians(offsets)))
            for a in boxes(robot,link_poses(robot,q),names):
                for b in fixed:
                    if overlap(a,b):failures.append((side,offsets,a[0],b[0]));break
    return failures

class GearClearance:
    """Reject sampled paths near landing gear; deliberately not a motion planner."""
    def __init__(self,path):
        self.robot=ET.parse(path).getroot()
        gear=[l.get('name') for l in self.robot.findall('link') if any(x in l.get('name') for x in ('front_leg','rear_leg','skid'))]
        self.fixed=boxes(self.robot,link_poses(self.robot),gear)
        # Cover interpolation gaps and small tracking error conservatively.
        self.fixed=[(n,p,r,h+.003) for n,p,r,h in self.fixed]

    def allows(self,side,start,end):
        names=[side+'_'+s+'_link' for s in ('upper_arm','forearm','wrist','gripper')]
        count=max(2,int(np.ceil(np.degrees(np.max(abs(end-start)))))+1)
        for q in np.linspace(start,end,count):
            poses=link_poses(self.robot,dict(zip(ARM_NAMES[side],q)))
            if any(overlap(a,b) for a in boxes(self.robot,poses,names) for b in self.fixed):return False
        return True

if __name__=='__main__':
    failures=check(sys.argv[1],float(sys.argv[2]) if len(sys.argv)>2 else 10)
    print(json.dumps({'intersections':len(failures),'examples':failures[:6]},indent=2))
    sys.exit(bool(failures))
