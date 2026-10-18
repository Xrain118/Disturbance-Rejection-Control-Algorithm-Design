"""Evaluate recorded truth, never controller estimates, against declared criteria."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from .clearance import GearClearance
from .effort import ordered_arm_effort_limits, arm_effort_ratios
from .core import ARM_NAMES

def longest_saturation(rows):
    longest=current=0.
    for prev,row in zip(rows,rows[1:]):
        current=current+float(row['t'])-float(prev['t']) if int(row['saturated']) else 0.
        longest=max(longest,current)
    return longest

def settling_time(rows,end):
    after=[r for r in rows if float(r['t'])>=end]
    for index,row in enumerate(after):
        start=float(row['t']);window=[r for r in after[index:] if float(r['t'])<=start+1]
        if window and float(window[-1]['t'])-start>=.99 and all(
            np.linalg.norm([float(sample[k]) for k in ('ex','ey','ez')])<=.10
            and math.degrees(float(sample['angle']))<=3 for sample in window):
            return start-end
    return None

def arm_metrics(rows,events,side):
    start,end=events[side+'_start'],events[side+'_end']
    motion=[r for r in rows if start<=float(r['t'])<=end]
    pre=[r for r in rows if start-2<=float(r['t'])<start]
    if not motion or not pre:return None
    errors=np.array([[float(r[k]) for k in ('ex','ey','ez')] for r in motion])
    angles=np.array([float(r['angle']) for r in motion])
    actual=np.array([[float(r[k]) for k in ('x','y','z')] for r in motion])
    pre_position=np.mean([[float(r[k]) for k in ('x','y','z')] for r in pre],axis=0)
    euler=np.array([[float(r[k]) for k in ('eroll','epitch','eyaw')] for r in motion])
    pre_euler=np.mean([[float(r[k]) for k in ('eroll','epitch','eyaw')] for r in pre],axis=0)
    recovery=min(rows,key=lambda r:abs(float(r['t'])-(end+5)))
    shoulder='q1' if side=='left' else 'q6'
    return {
        'peak_axis_error_m':dict(zip('xyz',np.max(np.abs(errors),axis=0).tolist())),
        'rms_axis_error_m':dict(zip('xyz',np.sqrt(np.mean(errors**2,axis=0)).tolist())),
        'peak_position_error_m':float(np.max(np.linalg.norm(errors,axis=1))),
        'rms_position_error_m':float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
        'peak_attitude_error_deg':float(np.degrees(np.max(angles))),
        'rms_attitude_error_deg':float(np.degrees(np.sqrt(np.mean(angles**2)))),
        'incremental_position_peak_m':float(np.max(np.linalg.norm(actual-pre_position,axis=1))),
        'incremental_attitude_peak_deg':float(np.degrees(np.max(np.linalg.norm(euler-pre_euler,axis=1)))),
        'recovery_position_at_5s_m':float(np.linalg.norm([float(recovery[k]) for k in ('ex','ey','ez')])),
        'recovery_attitude_at_5s_deg':math.degrees(float(recovery['angle'])),
        'settling_time_s':settling_time(rows,end),
        'shoulder_travel_deg':math.degrees(np.ptp([float(r[shoulder]) for r in motion])),
        'longest_saturation_seconds':longest_saturation(motion),
    }

def evaluate(directory):
    rows=list(csv.DictReader((directory/'flight.csv').open()))
    if not rows:raise ValueError('No flight samples')
    hover=[r for r in rows if r['phase']=='hover']
    if not hover:raise ValueError('No hover interval')
    errors=np.array([[float(r[k]) for k in ('ex','ey','ez')] for r in hover])
    angles=np.array([float(r['angle']) for r in hover])
    duration=float(hover[-1]['t'])-float(hover[0]['t'])
    result=dict(hover_seconds=duration,position_rms=float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
                attitude_rms_deg=float(np.degrees(np.sqrt(np.mean(angles**2)))))
    longest=longest_saturation(rows)
    result['longest_saturation_seconds']=longest
    result['hover_pass']=duration>=29.9 and result['position_rms']<=.10 and result['attitude_rms_deg']<=3 and longest<=1
    tracks={}
    for phase in ('track0','track1','track2','track3'):
        samples=[r for r in rows if r['phase']==phase]
        if samples:
            # Reference motion takes 3 s; evaluate 5 s after its completion.
            deadline=float(samples[0]['t'])+8
            end=min(samples,key=lambda r:abs(float(r['t'])-deadline))
            tracks[phase]=float(np.linalg.norm([float(end[k]) for k in ('ex','ey','ez')]))
    result['tracking_errors_at_5s']=tracks
    arm=[r for r in rows if r['phase'].startswith('arms_')]
    if arm:
        result['arm_peak_position']=max(float(np.linalg.norm([float(r[k]) for k in ('ex','ey','ez')])) for r in arm)
        result['arm_peak_angle_deg']=max(math.degrees(float(r['angle'])) for r in arm)
    result['passed']=result['hover_pass'] and all(x<=.10 for x in tracks.values()) and not (directory/'failure.txt').exists()
    if arm:result['passed'] &= result['arm_peak_position']<=.25 and result['arm_peak_angle_deg']<=8
    if (directory/'run.json').exists() and json.loads((directory/'run.json').read_text())['scenario']=='acceptance':
        recovery={};per_arm={};events=json.loads((directory/'events.json').read_text()) if (directory/'events.json').exists() else {}
        for side in ('left','right'):
            if side+'_start' in events and side+'_end' in events:
                metrics=arm_metrics(rows,events,side)
                if metrics:
                    per_arm[side]=metrics
                    recovery[side]={'position':metrics['recovery_position_at_5s_m'],
                                    'angle_deg':metrics['recovery_attitude_at_5s_deg'],
                                    'shoulder_travel_deg':metrics['shoulder_travel_deg']}
        result['arm_metrics']=per_arm
        result['arm_recovery']=recovery
        result['passed'] &= len(tracks)==4 and len(recovery)==2 and all(r['position']<=.10 and r['angle_deg']<=3 and r['shoulder_travel_deg']>=4.5 for r in recovery.values())
        guard=GearClearance(directory/'robot.urdf')
        samples=rows[::10]
        result['sampled_gear_clearance_pass']=all(guard.allows(side,q,q) for r in samples
            for side,offset in [('left',0),('right',5)]
            for q in [np.array([float(r[f'q{i}']) for i in range(offset,offset+5)])])
        result['max_arm_effort_nm']=max(abs(float(r[f'effort{i}'])) for r in rows for i in range(10))
        if 'tilt_effort0' in rows[0]:
            result['max_tilt_effort_nm']=max(abs(float(r[f'tilt_effort{i}'])) for r in rows for i in range(6))
        model=json.loads((directory/'model.json').read_text())
        limits=ordered_arm_effort_limits(model,allow_legacy=True)
        names=[name for side in ('left','right') for name in ARM_NAMES[side]]
        peaks=np.max(np.abs([[float(r[f'effort{i}']) for i in range(10)] for r in rows]),axis=0)
        result['arm_effort_limits_nm']=dict(zip(names,limits.tolist()))
        result['max_arm_effort_per_joint_nm']=dict(zip(names,peaks.tolist()))
        result['max_arm_effort_ratio']=float(np.max(arm_effort_ratios(peaks,limits)))
        result['arm_effort_limits_pass']=bool(np.all(peaks<=limits+1e-6))
        result['rotor_mode']=model['mode'] in ('px4_actuators','px4_native')
        result['passed'] &= result['sampled_gear_clearance_pass'] and result['arm_effort_limits_pass'] and result['rotor_mode']
    model=json.loads((directory/'model.json').read_text())
    if model['mode']=='px4_native':
        evidence=json.loads((directory/'native_chain.json').read_text()) if (directory/'native_chain.json').exists() else {}
        result['native_control_chain']=evidence
        result['passed'] &= bool(evidence) and not evidence.get('ros_publishes_actuators',True)
    (directory/'metrics.json').write_text(json.dumps(result,indent=2))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
    result=evaluate(args.directory);print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)
