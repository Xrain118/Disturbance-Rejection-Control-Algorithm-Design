"""Acceptance must not pass on incomplete or conflicting control-chain evidence."""
import csv
import json
from types import SimpleNamespace
import pytest
from dragonfly_flight import verify
from dragonfly_flight.native import CONTROL_FLAGS, FEEDBACK_FLAGS, ACTUATOR_INPUT_TOPICS


@pytest.fixture
def recording(tmp_path):
    row=dict(t=0,phase='hover',ex=0,ey=0,ez=0,angle=0,saturated=0,x=0,y=0,z=1.6,
             eroll=0,epitch=0,eyaw=0)
    row.update({f'q{i}':0 for i in range(10)})
    row.update({f'effort{i}':0 for i in range(10)})
    with (tmp_path/'flight.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader()
        for t in range(32):writer.writerow(dict(row,t=t))
    (tmp_path/'model.json').write_text(json.dumps({'mode':'px4_wrench','config':{'arm_effort':5.}}))
    (tmp_path/'run.json').write_text(json.dumps({'scenario':'hover'}))
    evidence={key:key not in FEEDBACK_FLAGS for key in CONTROL_FLAGS}
    evidence.update(mode='px4_wrench',ros_publishes_actuators=False,
                    actuator_input_publishers={topic:[] for topic in ACTUATOR_INPUT_TOPICS})
    (tmp_path/'wrench_chain.json').write_text(json.dumps(evidence))
    return tmp_path,evidence


def test_offline_rejects_missing_or_active_native_feedback(recording):
    directory,evidence=recording
    assert verify.evaluate(directory)['passed']
    evidence['flag_control_rates_enabled']=True
    (directory/'wrench_chain.json').write_text(json.dumps(evidence))
    assert not verify.evaluate(directory)['passed']
    (directory/'wrench_chain.json').unlink()
    assert not verify.evaluate(directory)['passed']


def test_acceptance_requires_translation_and_both_arm_segments(recording,monkeypatch):
    directory,_=recording
    (directory/'run.json').write_text(json.dumps({'scenario':'acceptance'}))
    monkeypatch.setattr(verify,'GearClearance',lambda path:SimpleNamespace(allows=lambda *args:True))
    result=verify.evaluate(directory)
    assert result['hover_pass']
    assert not result['passed']
    assert result['tracking_errors_at_5s']=={}
    assert result['arm_recovery']=={}
