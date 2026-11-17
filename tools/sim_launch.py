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
                 engine_factory=warp_engine, binding=verify_binding, runner_type=Runner, application_stdin=None,
                 viewer=False, viewer_follow=False, viewer_factory=None,
                 viewer_no_shadow=False):
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
        self.viewer_enabled=viewer or viewer_follow or viewer_no_shadow
        self.viewer_follow=viewer_follow
        self.viewer_no_shadow=viewer_no_shadow
        self.viewer_factory=viewer_factory
        self.viewer=None
        self.application_output=None

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

    def _open_application_console(self):
        path=self.directory/'application.log'
        self.application_output=path.open('r',encoding='utf-8',errors='replace')
        print('sim_launch: application status and log: '+str(path),flush=True)
        try:
            bindings=json.loads(self.config.read_text())['input']['bindings']
        except (OSError,ValueError,KeyError,TypeError):
            bindings={}
        keys=['{}={}'.format('space' if key==' ' else key,action)
              for action,key in bindings.items() if isinstance(key,str) and len(key)==1]
        if keys:print('sim_launch: keys: '+'  '.join(keys),flush=True)
        print('sim_launch: type keys in this terminal without Enter; wait for '
              'behavior=Ready before starting locomotion. Ctrl-C stops the whole experiment.',
              flush=True)

    def _mirror_application_output(self):
        if self.application_output is None:return
        # Follow the child-owned regular file, never put a logging pipe in the
        # SDK path. Bound each host iteration; application keeps owning stdin.
        text=self.application_output.read(8192)
        if text:
            try:
                sys.stdout.write(text)
                sys.stdout.flush()
            except (BrokenPipeError,BlockingIOError) as exc:
                self.evidence['console_output_error']=str(exc)
                self.application_output.close()
                self.application_output=None


    def start(self):
        if self._cancel_startup():return self
        n=json.loads(subprocess.check_output([str(self.normalizer),str(self.config)],text=True,timeout=30))
        self.directory=Path(n.pop('socket_directory'))
        self.evidence=self.binding(n)
        asset_digest=n['asset_digest']
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
        if self.viewer_enabled:
            factory=self.viewer_factory
            if factory is None:
                from simulation.viewer import ViewerPublisher
                factory=ViewerPublisher
            self.viewer=factory(n['engine_config'],n['run_id'],asset_digest,
                                n['physics_dt_us'],follow=self.viewer_follow,
                                no_shadow=self.viewer_no_shadow)
            self.viewer.cancelled=lambda:self.shutdown_notified
            self.viewer.start(min(30,n['startup_timeout_us']/1e6))
            if self._cancel_startup():return self
            self.runner.viewer=self.viewer
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
            if name=='application' and self.application_stdin is None:
                self._open_application_console()
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
            self._mirror_application_output()
            if self.viewer is not None:
                status=self.viewer.status()
                previous=self.evidence.get('viewer',{}).get('state')
                self.evidence['viewer']=status
                if status['state']!=previous:
                    print('simulation viewer: '+status['state']+
                          (': '+status['error'] if status.get('error') else ''),
                          file=sys.stderr,flush=True)
        codes=[p.wait(timeout=5) for p in self.children]
        self._mirror_application_output()
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
        self._mirror_application_output()
        if self.application_output is not None:
            self.application_output.close()
            self.application_output=None
        for log in self.logs:log.close()
        if self.viewer is not None:
            self.viewer.close()
            self.evidence['viewer']=self.viewer.status()
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
    parser.add_argument('--viewer',action='store_true',help='open read-only native MuJoCo window')
    parser.add_argument('--viewer-follow',action='store_true',help='open viewer with base-follow camera')
    parser.add_argument('--viewer-no-shadow',action='store_true',
        help='open viewer without the cast ground shadow: costs ~0.24 ms/cycle less on the same GPU (real-time on a laptop); lighting, sky and ground stay')
    parser.add_argument('--simulation-worker',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if not args.simulation_worker:
        os.execvp(args.python,[args.python,str(Path(__file__).resolve()),*sys.argv[1:],'--simulation-worker'])
    experiment=Experiment(args.config,args.runtime,args.application,args.normalizer,
                          viewer=args.viewer,viewer_follow=args.viewer_follow,
                          viewer_no_shadow=args.viewer_no_shadow)
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
