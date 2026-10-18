"""Freeze this run's DDS agent and confirm the flight-state timeout stops SITL."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import psutil

out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=True)
log=(out/'probe.log').open('w')
runner=subprocess.Popen(['bash','src/dragonfly_flight/scripts/sim.sh','--scenario','manual','--output',str(out)],
                        stdout=log,stderr=log,start_new_session=True)
try:
    deadline=time.monotonic()+100
    while True:
        if runner.poll() is not None:raise RuntimeError('Flight failed before injection')
        path=out/'controller.log'
        if path.exists() and 'hover p=' in path.read_text():break
        if time.monotonic()>deadline:raise TimeoutError('Hover not reached')
        time.sleep(.1)
    agent=next(p for p in psutil.Process(runner.pid).children(recursive=True) if 'MicroXRCEAgent' in p.cmdline())
    started=time.monotonic();agent.send_signal(signal.SIGSTOP)
    runner.wait(timeout=10)
    reason=(out/'failure.txt').read_text()
    assert runner.returncode!=0 and 'state older than 0.2' in (out/'controller.log').read_text()
    (out/'fault_result.json').write_text(json.dumps(dict(passed=True,reason=reason,wall_seconds_to_shutdown=time.monotonic()-started),indent=2))
finally:
    if runner.poll() is None:os.killpg(runner.pid,signal.SIGINT);runner.wait(timeout=12)
    log.close()
