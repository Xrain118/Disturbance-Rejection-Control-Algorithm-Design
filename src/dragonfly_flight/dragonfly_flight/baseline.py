"""Six sequential, uncompensated acceptance attempts with preserved failures."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
from .provenance import source_root
from .verify import evaluate


def baseline_config(config):
    # Copy nested dictionaries without changing the caller's/default configuration.
    config=json.loads(json.dumps(config))
    config.setdefault('ndob',{}).update(enabled=False,shadow_mode=True,compensation_enabled=False)
    return config


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=1200.,help='Wall seconds per attempt')
    parser.add_argument('--px4',default=os.environ.get('PX4_ROOT',str(Path.home()/'PX4-Autopilot')))
    args=parser.parse_args()
    if args.timeout<=0:parser.error('--timeout must be positive')
    root=source_root();out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    cfg=baseline_config(yaml.safe_load((root/'src/dragonfly_flight/config/flight.yaml').read_text()))
    config=out/'baseline_config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    summary=[]
    for index in range(1,4):
        for mode in ('px4_native','px4_wrench'):
            directory=out/f'{mode}_{index}'
            command=[sys.executable,'-m','dragonfly_flight.run','--mode',mode,
                     '--scenario','acceptance','--record','--config',str(config),
                     '--output',str(directory),'--px4',args.px4]
            result={'mode':mode,'round':index,'directory':str(directory),'command':command}
            print(f'Starting {mode} round {index}: {directory}',flush=True)
            started=time.monotonic()
            with (out/f'{mode}_{index}.launcher.log').open('w') as log:
                process=subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    result['exit_code']=process.wait(timeout=args.timeout)
                except subprocess.TimeoutExpired:
                    result['timeout']=True
                except KeyboardInterrupt:
                    result['interrupted']=True
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid,signal.SIGINT)
                        try:process.wait(timeout=20)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid,signal.SIGKILL);process.wait()
            result['wall_seconds']=time.monotonic()-started
            try:result['metrics']=evaluate(directory)
            except Exception as error:
                result['evaluation_error']=str(error)
                if directory.exists():
                    (directory/'metrics.json').write_text(json.dumps({'passed':False,'evaluation_error':str(error)},indent=2))
            result['passed']=result.get('exit_code')==0 and result.get('metrics',{}).get('passed',False)
            summary.append(result)
            (out/'summary.json').write_text(json.dumps({'passed':len(summary)==6 and all(r['passed'] for r in summary),
                'attempts':summary},indent=2))
            print(f'Finished {mode} round {index}: passed={result["passed"]}',flush=True)
            if result.get('interrupted'):raise SystemExit(130)
    if not all(r['passed'] for r in summary):raise SystemExit(1)


if __name__=='__main__':main()
