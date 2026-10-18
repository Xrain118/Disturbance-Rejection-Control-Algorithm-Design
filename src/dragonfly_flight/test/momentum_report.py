"""Quantify conservation error without asserting unobserved convergence."""
import csv
import json
from pathlib import Path
import sys
import numpy as np

results=[]
for name in sys.argv[1:]:
    path=Path(name);a=np.loadtxt(path/'momentum.csv',delimiter=',');a=a[a[:,0]>2]
    delta=a[:,1:]-a[0,1:];rows=list(csv.DictReader((path/'flight.csv').open()))
    linear=np.linalg.norm(delta[:,:3],axis=1);angular=np.linalg.norm(delta[:,3:],axis=1)
    r=dict(step=json.loads((path/'run.json').read_text())['step'],
           linear_peak=float(linear.max()),angular_peak=float(angular.max()),
           linear_rms=float(np.sqrt(np.mean(linear**2))),angular_rms=float(np.sqrt(np.mean(angular**2))),
           linear_final=float(linear[-1]),angular_final=float(angular[-1]),
           body_rotation_deg=float(np.degrees(max(float(row['angle']) for row in rows))),
           shoulder_travel_deg=float(np.degrees(np.ptp([float(row['q1']) for row in rows]))))
    (path/'metrics.json').write_text(json.dumps(r,indent=2));results.append(r)
results.sort(key=lambda r:-r['step'])
print(json.dumps(dict(runs=results,monotonic_momentum_rms=all(b['linear_rms']<a['linear_rms'] and b['angular_rms']<a['angular_rms'] for a,b in zip(results,results[1:]))),indent=2))
