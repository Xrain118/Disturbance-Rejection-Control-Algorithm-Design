import json
import os
import pty
import queue
import threading
import time
import pytest
from dragonfly_serial.transport import PortWorker, Recorder, replay_events, validate_ports
from dragonfly_serial.protocol import parse, Quality
from test_protocol import sample

def wait(events, predicate, timeout=3):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        try:e=events.get(timeout=.1)
        except queue.Empty:continue
        if predicate(e):return e
    raise AssertionError('event deadline exceeded')

def test_port_alias_rejected(tmp_path):
    master,slave=pty.openpty()
    try:
        path=os.ttyname(slave);alias=tmp_path/'alias';alias.symlink_to(path)
        with pytest.raises(ValueError):validate_ports(path,str(alias))
    finally:os.close(master);os.close(slave)

def test_dual_50hz_disconnect_reconnect_record_replay(tmp_path):
    events=queue.Queue(512);rec=Recorder(tmp_path/'capture.jsonl')
    handles=[pty.openpty(),pty.openpty()];workers=[];stop=threading.Event()
    links=[]
    for side,(m,s) in zip(('left','right'),handles):
        link=tmp_path/side;link.symlink_to(os.ttyname(s));links.append(link)
        worker=PortWorker(side,str(link),921600,events,rec,.05);workers.append(worker);worker.start()
    senders=[]
    def send(m,side):
        try:
            for seq in range(60):
                if stop.is_set():break
                wire=(json.dumps(sample(dev=side,seq=seq,t_us=20000+seq*20000))+'\n').encode()
                os.write(m,wire[:20]);os.write(m,wire[20:])
                time.sleep(.02)
        except OSError:pass
    try:
        connected=set()
        while len(connected)<2:connected.add(wait(events,lambda e:e['kind']=='connected')['side'])
        # Drivers must be passive until an explicit configure call.
        import select
        assert not select.select([m for m,s in handles],[],[],.05)[0]
        for (m,s),side in zip(handles,('left','right')):
            t=threading.Thread(target=send,args=(m,side));t.start();senders.append(t)
        quality={s:Quality() for s in ('left','right')}
        until=time.monotonic()+2
        while time.monotonic()<until and min(q.frames for q in quality.values())<50:
            e=wait(events,lambda e:e['kind']=='line')
            quality[e['side']].accept(parse(e['raw'],expected_device=e['side']),e['mono_ns']/1e9)
        assert min(q.frames for q in quality.values())>=50
        workers[0].configure(2,25)
        wait(events,lambda e:e['kind']=='command')
        assert b'606' in os.read(handles[0][0],1024)
        for t in senders:t.join()
        os.close(handles[0][0]);os.close(handles[0][1]);handles[0]=(-1,-1)
        wait(events,lambda e:e['side']=='left' and e['kind']=='disconnected')
        m,s=pty.openpty();handles[0]=(m,s)
        links[0].unlink();links[0].symlink_to(os.ttyname(s))
        wait(events,lambda e:e['side']=='left' and e['kind']=='connected')
        os.write(m,(json.dumps(sample(dev='left',boot=2))+'\n').encode())
        e=wait(events,lambda e:e['side']=='left' and e['kind']=='line')
        assert quality['left'].accept(parse(e['raw']),e['mono_ns']/1e9)
    finally:
        stop.set()
        for t in senders:t.join()
        for w in workers:w.close()
        rec.close()
        for m,s in handles:
            if m>=0:os.close(m);os.close(s)
    recorded=list(replay_events(tmp_path/'capture.jsonl'))
    assert sum(e['kind']=='line' for e in recorded)>=100
    for e in recorded:
        if e['kind']=='line':parse(e['raw'],expected_device=e['side'])
    with pytest.raises(FileExistsError):Recorder(tmp_path/'capture.jsonl')
