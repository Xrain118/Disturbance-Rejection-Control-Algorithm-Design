import copy
import json
from pathlib import Path
import pytest
from dragonfly_serial.protocol import (parse, ProtocolError, LineDecoder, Quality,
    TorqueFilter, joint_positions, joint_torques, configure_command, ARRAYS)

DATA = Path(__file__).parent/'data'

def sample(**changes):
    d=json.loads((DATA/'valid.json').read_text())
    d.update(changes)
    return d

def frame(**changes):
    return parse(json.dumps(sample(**changes)))

def test_negative_current_and_physical_torque():
    f=frame()
    assert f.data['current'][0] == -1
    assert f.data['tau'][0] == pytest.approx(-1.0787315*.0065,abs=1e-6)
    assert joint_torques(f)[1] == pytest.approx(f.data['tau'][1]-f.data['tau'][2])
    assert len(joint_positions(f)) == 5

def test_invalid_never_reuses_sample():
    d=sample();d['ok'][2]=False
    for k in ARRAYS:d[k][2]=None
    f=parse(json.dumps(d))
    assert joint_torques(f)[1] is None
    assert len(joint_positions(f)) == 5  # slave is not a sixth joint
    d['ok'][0]=False
    for k in ARRAYS:d[k][0]=None
    assert len(joint_positions(parse(json.dumps(d)))) == 4
    d['current'][0]=0
    with pytest.raises(ProtocolError):parse(json.dumps(d))

@pytest.mark.parametrize('changes',[{'ver':2},{'ver':True},{'rate':51},
    {'current':[False]*7},{'tau':[float('nan')]*7},{'pos':[40000]*7},
    {'read_us':[2**64]*7},{'ok':[1]*7},{'ids':list(range(10,17))}])
def test_bad_fields(changes):
    with pytest.raises(ProtocolError):frame(**changes)

def test_identity_and_legacy():
    with pytest.raises(ProtocolError):parse(json.dumps(sample()),expected_device='wrong')
    f=parse('{"T":1051,"b":0,"s":0,"e":0,"t":0,"r":0}',mode='legacy')
    assert f.legacy and joint_torques(f)==[None]*5
    assert parse('{"T":6061}') is None

def test_framing():
    d=LineDecoder(limit=10)
    assert d.feed(b'ab')==[]
    assert d.feed(b'c\r\nfirst\nnext\n')==['abc','first','next']
    assert d.feed(b'x'*20+b'\nokay\n\xff\n')==['okay']
    assert d.errors==2 and len(d.buffer)==0

def test_quality_reset_gap_duplicate_and_independent_age():
    left,right=Quality(),Quality()
    left.accept(frame(seq=0),0);right.accept(frame(seq=0),0)
    assert left.accept(frame(seq=3,t_us=40000),.02)
    assert left.gaps==2 and left.frames==2
    assert not left.stale(.11,.1) and right.stale(.11,.1)
    with pytest.raises(ProtocolError):left.accept(frame(seq=3,t_us=40000),.03)
    assert left.accept(frame(boot=2,seq=0,t_us=1,read_us=[0]*7),.04)
    assert left.resets==1

def test_wrap_and_filter():
    q=Quality();q.accept(frame(seq=2**32-1),0)
    assert not q.accept(frame(seq=0,t_us=40000),.02)
    f=TorqueFilter(.2)
    assert f.update([10]*5)==[10]*5
    assert f.update([0]*5)==[8]*5
    assert f.update([None]*5)==[None]*5
    assert f.update([0]*5)==[0]*5
    assert TorqueFilter().update([1,2,3,4,5])==[1,2,3,4,5]

def test_config_and_worst_frame_budget():
    assert json.loads(configure_command(0))=={'T':606}
    assert json.loads(configure_command(0,0))=={'T':606}
    assert json.loads(configure_command(2,50))=={'T':606,'enable':True,'rate':50}
    with pytest.raises(ValueError):configure_command(2,51)
    worst=(DATA/'worst.json').read_text().strip()
    parse(worst)
    assert len(worst.encode())+1<=1200
    assert 1200*50*10/921600<=.7
