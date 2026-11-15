#!/usr/bin/env python3
"""Independent simulation owner. Ctrl-C closes the whole experiment; app x only leaves it."""
import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time

from simulation.main import Runner, load_bootstrap, warp_engine
from simulation.assets import verify_binding


def identities():
    a,b=secrets.token_hex(16),secrets.token_hex(16)
    while a==b:b=secrets.token_hex(16)
    return dict(run_id=a,runtime_id=b)


def reserve_directory(path):
    # Never reuse or unlink another run's socket/bootstrap, even after a crash.
    Path(path).mkdir(mode=0o700)


class Experiment:
    def __init__(self, config, runtime, application, normalizer=None,
                 engine_factory=warp_engine, binding=verify_binding, runner_type=Runner, application_stdin=None):
        self.config=Path(config).resolve()
        self.runtime=Path(runtime).resolve()
        self.application=Path(application).resolve()
        self.normalizer=Path(normalizer or self.runtime.with_name('simulation_normalize')).resolve()
        self.engine_factory=engine_factory
        self.binding=binding
        self.runner_type=runner_type
        self.application_stdin=application_stdin
        self.children=[]
        self.logs=[]
        self.directory_owned=False
        self.result=None
        self.thread=None
        self.runner=None
        self.shutdown_notified=False
        self.evidence={}

    def notify_shutdown(self,signum=None,frame=None):
        self.shutdown_notified=True

    def _cancel_startup(self):
        # Normal flow only: signal handlers merely assign shutdown_notified.
        if not self.shutdown_notified:return False
        self.evidence['startup_cancelled']=True
        self.evidence['runtime_spawned']=bool(self.children)
        self.close()
        if self.thread and self.thread.is_alive():
            raise RuntimeError('startup cancellation owner deadline')
        return True

    def start(self):
        if self._cancel_startup():return self
        n=json.loads(subprocess.check_output([str(self.normalizer),str(self.config)],text=True,timeout=30))
        self.directory=Path(n.pop('socket_directory'))
        self.evidence=self.binding(n)
        n.pop('asset_digest');n.pop('policy_path')
        n.update(identities())
        reserve_directory(self.directory)
        self.directory_owned=True
        bootstrap=self.directory/'bootstrap.json'
        fd=os.open(bootstrap,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as f:json.dump(n,f)
        ready=threading.Event()
        class ReadyRunner(self.runner_type):
            def _advance(inner,event):
                super()._advance(event)
                if event.kind=='feedback' and inner.coordinator.feedback is not None:ready.set()
        self.runner=ReadyRunner(load_bootstrap(bootstrap),self.directory/'bridge.sock',self.engine_factory)
        def run():self.result=self.runner.run()
        self.thread=threading.Thread(target=run,name='simulation-owner')
        self.thread.start()
        deadline=time.monotonic()+n['startup_timeout_us']/1e6
        while True:
            if self._cancel_startup():return self
            if ready.wait(.01):break
            if self.result is not None or time.monotonic()>deadline:
                raise RuntimeError('engine startup failed: '+self.runner.coordinator.failure)
        # Engine coherent initial feedback and listening socket precede both SDK peers.
        self.evidence.update(run_id=n['run_id'],runtime_id=n['runtime_id'],engine_ready_before_sdk=True)
        for name,binary in [('runtime',self.runtime),('application',self.application)]:
            if self._cancel_startup():return self
            log=open(self.directory/(name+'.log'),'w')
            self.logs.append(log)
            if self._cancel_startup():return self
            self.children.append(subprocess.Popen([str(binary),'--config',str(self.config)],
                stdin=self.application_stdin if name=='application' else subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        return self

    def wait(self):
        if self.evidence.get('startup_cancelled'):return 1
        while self.thread.is_alive():
            if self.shutdown_notified:
                self.runner.request_shutdown()
            if self.children[0].poll() is not None and self.result is None and self.runner.coordinator.state!='Closed':
                self.runner.request_stop()
            if self.children[1].poll() is not None and not self.runner.coordinator.runtime_only and self.runner.coordinator.state!='Closed':
                self.runner.request_stop()
            self.thread.join(.01)
        codes=[p.wait(timeout=5) for p in self.children]
        c=self.runner.coordinator
        self.evidence.update(coordinator_result=self.result,child_codes=codes,
            state=c.state,failure=c.failure,stop_proven=c.stop_proven,
            runtime_only=c.runtime_only,release_state=c.release_state,
            release_event_drops=c.release_event_drops,
            release_events=list(c.release_events),logical_steps=c.step)
        return 0 if self.result==0 and codes==[0,0] and c.stop_proven else 1

    def close(self):
        if self.runner and self.thread and self.thread.is_alive():
            self.runner.request_stop();self.thread.join(2)
        for p in self.children:
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        for p in self.children:
            if p.stdin and not p.stdin.closed:p.stdin.close()
        for log in self.logs:log.close()
        if self.directory_owned and self.directory.exists():
            if self.runner:
                c=self.runner.coordinator
                self.evidence.update(coordinator_result=self.result,state=c.state,failure=c.failure,
                    stop_proven=c.stop_proven,runtime_only=c.runtime_only,release_state=c.release_state,
                    release_event_drops=c.release_event_drops,release_events=list(c.release_events),logical_steps=c.step)
            self.evidence['final_child_codes']=[p.returncode for p in self.children]
            (self.directory/'evidence.json').write_text(json.dumps(self.evidence,indent=2,default=lambda x:x.hex() if isinstance(x,bytes) else str(x)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--runtime',required=True)
    parser.add_argument('--application',required=True)
    parser.add_argument('--python',required=True,help='separate pinned simulation interpreter')
    parser.add_argument('--normalizer')
    parser.add_argument('--simulation-worker',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if not args.simulation_worker:
        os.execvp(args.python,[args.python,str(Path(__file__).resolve()),*sys.argv[1:],'--simulation-worker'])
    experiment=Experiment(args.config,args.runtime,args.application,args.normalizer)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,experiment.notify_shutdown)
    try:
        experiment.start()
        if experiment.evidence.get('startup_cancelled'):return experiment.wait()
        print('Simulation engine ready; SDK peers started. App exit retains runtime; Ctrl-C shuts down.',flush=True)
        return experiment.wait()
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
        print('simulation launch failed: '+str(exc),file=sys.stderr)
        return 1
    finally:experiment.close()

if __name__=='__main__':raise SystemExit(main())
