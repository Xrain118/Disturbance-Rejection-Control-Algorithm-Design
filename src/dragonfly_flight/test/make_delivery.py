import os
"""Collect compact final evidence; raw bags and logs remain under log/."""
import hashlib
import json
from pathlib import Path
import shutil
import statistics


root=Path(__file__).resolve().parents[3]
destination=root/'log/delivery'
destination.mkdir(parents=True,exist_ok=True)
names=[f'vertical_strict_{i}' for i in (1,2,3)]
metrics=[json.loads((root/'log'/name/'metrics.json').read_text()) for name in names]
models=[(root/'log'/name/'model.json').read_bytes() for name in names]
parameters=[(root/'log'/name/'px4_parameters.json').read_bytes() for name in names]
assert len(set(models))==1 and len(set(parameters))==1

for name in names:
    folder=destination/name;folder.mkdir(exist_ok=True)
    for filename in ('curves.png','metrics.json','model.json','controllers.yaml',
                     'px4_parameters.json','events.json','run.json'):
        shutil.copy2(root/'log'/name/filename,folder/filename)

momentum=[]
for step in ('.002','.001','.0005'):
    source=root/'log'/f'vertical_momentum_{step}'
    item=json.loads((source/'metrics.json').read_text());momentum.append(item)
    folder=destination/f'momentum_{step}';folder.mkdir(exist_ok=True)
    for filename in ('metrics.json','momentum.csv'):shutil.copy2(source/filename,folder/filename)

keyboard=json.loads((root/'log/vertical_keyboard_full/keyboard_result.json').read_text())
assert keyboard['passed'] and len(keyboard['jog_checks'])==20
for source,target in [
    (root/'src/dragonfly_flight/README.md','README.md'),
    (root/'src/dragonfly_flight/docs/verification.md','verification.md'),
    (root/'src/dragonfly_flight/config/plotjuggler.xml','plotjuggler.xml'),
    (root/'src/dragonfly_flight/config/versions.yaml','versions.yaml'),
    (root/'log/vertical_keyboard_full/keyboard_result.json','keyboard_result.json'),
]:shutil.copy2(source,destination/target)

keys=('position_rms','attitude_rms_deg','arm_peak_position','arm_peak_angle_deg','longest_saturation_seconds')
summary={
    'runs':dict(zip(names,metrics)),
    'passed_runs':sum(bool(item['passed']) for item in metrics),
    'run_count':len(metrics),
    'mean':{key:statistics.mean(item[key] for item in metrics) for key in keys},
    'maximum':{key:max(item[key] for item in metrics) for key in keys},
    'momentum':momentum,
}
(destination/'summary.json').write_text(json.dumps(summary,indent=2))
(destination/'configuration_sha256.txt').write_text(
    'model '+hashlib.sha256(models[0]).hexdigest()+'\n'+
    'PX4 '+hashlib.sha256(parameters[0]).hexdigest()+'\n')
print(destination)
