"""Run after the batch: python3 <batch>/analyze.py. No flight data are modified."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
summary=json.loads((root/'summary.json').read_text())
records=[];provenance=[];models=[]
for attempt in summary['attempts']:
    path=root/f"{attempt['mode']}_{attempt['round']}"
    rows=list(csv.DictReader((path/'flight.csv').open()))
    item={k:attempt[k] for k in ('mode','round','passed','wall_seconds')}
    item['failure']=(path/'failure.txt').read_text() if (path/'failure.txt').exists() else None
    item['samples']=len(rows)
    item['phases']=sorted({r['phase'] for r in rows})
    if rows:
        end=float(rows[-1]['t']);tail=[r for r in rows if float(r['t'])>=end-30]
        item['last_t_s']=end
        times=np.array([float(r['t']) for r in rows]);dt=np.diff(times)
        item['logged_control_rate_hz']=(len(rows)-1)/(times[-1]-times[0])
        item['median_logged_dt_s']=float(np.median(dt))
        item['max_logged_dt_s']=float(np.max(dt))
        item['last_30s']={
            'position_rms_m':float(np.sqrt(np.mean([sum(float(r[k])**2 for k in ('ex','ey','ez')) for r in tail]))),
            'attitude_rms_deg':math.degrees(float(np.sqrt(np.mean([float(r['angle'])**2 for r in tail])))),
            'mean_rpy_deg':np.degrees(np.mean([[float(r[k]) for k in ('eroll','epitch','eyaw')] for r in tail],axis=0)).tolist(),
            'mean_force_residual_n':float(np.mean([float(r['force_residual']) for r in tail])),
            'mean_torque_residual_nm':float(np.mean([float(r['torque_residual']) for r in tail])),
        }
    if (path/'research.csv').exists():
        research=list(csv.DictReader((path/'research.csv').open()))
        item['feedforward_enabled_samples']=sum(int(r['feedforward_enabled']) for r in research)
        item['max_applied_feedforward']=max(abs(float(r['delta'+str(i)])) for r in research for i in range(6))
        item['last_100_mean_predicted_delta_wrench']=np.mean([[float(r['delta_raw'+str(i)]) for i in range(6)] for r in research[-100:]],axis=0).tolist()
        item['compute_median_ms']=1000*float(np.median([float(r['compute_s']) for r in research]))
    item['has_ndob_log']=(path/'ndob.csv').exists()
    item['has_rosbag']=bool(list((path/'bag').glob('*.db3')))
    item['has_ulog']=bool(list((path/'px4').rglob('*.ulg')))
    records.append(item)
    provenance.append(json.loads((path/'provenance.json').read_text()))
    model=json.loads((path/'model.json').read_text());model.pop('mode');models.append(model)

first=provenance[0]['source']['sha256']
# Tests, documentation and provenance recording itself can differ without changing
# the controller. Report those differences too, rather than hiding them.
changes=[{name for name,value in p['source']['sha256'].items() if first.get(name)!=value} for p in provenance]
control=lambda name: name.startswith('src/') and '/test/' not in name and name.endswith(('.py','.cpp','.hpp','.yaml','.xacro')) and not name.endswith('/provenance.py')
report={'batch_passed':summary['passed'],'attempts':records,
        'same_model_except_mode':all(m==models[0] for m in models),
        'same_flight_config':len({p['assets_sha256']['flight_config.yaml'] for p in provenance})==1,
        'same_px4_binary':len({p['px4']['binary_sha256'] for p in provenance})==1,
        'px4_manifest_matches':all(not p['px4']['manifest_mismatches'] for p in provenance),
        'source_changes_from_first':[sorted(c) for c in changes],
        'control_source_changes_from_first':[sorted(n for n in c if control(n)) for c in changes]}
(root/'diagnostics.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
