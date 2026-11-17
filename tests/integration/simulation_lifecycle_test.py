"""Actual built SDK processes, real DDS, test-only deterministic engine. Serial domain224."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

ROOT=Path(__file__).resolve().parents[2]
assert '224' in json.loads((ROOT/'tests/integration/dds_domains.json').read_text())['reserved']
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'tests/support/simulation'))
from simulation.main import Runner,load_bootstrap
from fake_engine import FakeEngine


def run(build, mode="leave"):
    stop_pending=mode.startswith("inference")
    with tempfile.TemporaryDirectory(prefix='irmv-t8-') as directory:
        d=Path(directory)
        cfg=json.loads((ROOT/'config/sdk.json').read_text())
        cfg.update(deployment='simulation',backend='sim')
        cfg['dds']['domain']=224
        cfg['input']['bindings'].update(exit='q',velocity_yaw_left='x')
        cfg['telemetry']['directory']=str(d)
        cfg['simulation']=dict(socket_directory=str(d),engine='mujoco_warp',
            engine_sha256='a'*64,model_path=str(ROOT/'tests/support/simulation/free_base.xml'),
            asset_sha256='b'*64,physics_dt_us=1000,base_body='base',imu_site='imu',
            weld_name='support',initial_base_position=[0,0,1],initial_base_quaternion=[1,0,0,0],
            initial_joint_position=dict(test_left=0,test_right=0),
            joint_mapping=dict(test_left=dict(joint='left',actuator='left'),test_right=dict(joint='right',actuator='right')))
        fixture=json.loads((ROOT/'tests/support/deployment/valid_two_axis.json').read_text().replace('"left"','"test_left"').replace('"right"','"test_right"'))
        fixture['action']['default_q']=[0.,0.]
        fixture['action']['offset']=[0.,0.]
        fixture['action']['scale']=[0.,0.]
        fixture['action']['kp']=[1.,1.]
        fixture['action']['kd']=[1.,1.]
        fixture_path=d/'deployment.json';fixture_path.write_text(json.dumps(fixture))
        subprocess.run([sys.executable,str(ROOT/'tools/make_test_model.py'),str(d/'models'),str(fixture_path)],check=True)
        if stop_pending:
            import importlib.util
            spec=importlib.util.spec_from_file_location('fixture_generator',ROOT/'tools/make_test_model.py')
            generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
            # Real ONNX work; no sleeps or completion injection in production workers.
            generator.save(d/'models','delayed.onnx',generator.make_model(
                generator.long_running_graph(fixture['model']['input']['dimension'],2,
                    iterations=200000),fixture))
        cfg['application']['policy']['model_path']=str(d/'models'/('delayed.onnx' if stop_pending else 'valid.onnx'))
        cfg['application']['initialization']['trajectory'].update(duration_us=20000,stable_window_us=10000)
        config=d/'sdk.json';config.write_text(json.dumps(cfg))
        # Test-only loader mode returns actual C++ normalized digests. No Python canonical hashing.
        digest=subprocess.check_output([str(build/'simulation_driver_test'),str(config)],text=True).split()
        bootstrap=dict(run_id='12'*16,runtime_id='34'*16,config_digest=digest[0],
            application_digest=digest[1],engine_digest='a'*64,physics_dt_us=1000,
            control_dt_us=2000,engine_config=dict(q=[0,0]),phase_timeout_us=1000000)
        (d/'bootstrap.json').write_text(json.dumps(bootstrap))
        (d/'bootstrap.json').chmod(0o600)
        physics_wait=threading.Event()
        unblock=threading.Event()
        engines=[]
        class DelayedEngine(FakeEngine):
            def read_feedback(self):
                value=super().read_feedback()
                if mode=='fault_leave' and self.physics_step>=260:
                    from dataclasses import replace
                    return replace(value,dq=(10000.,)*len(self.q))
                return value
            def initialize(self,config):
                super().initialize(config)
                self.releases=[]
                self.steps=[]
                engines.append(self)
            def release_base(self):
                before=self.read_feedback()
                super().release_base()
                self.releases.append((self.physics_step,before,self.read_feedback()))
            def close(self):
                if mode=='engine_stop_hang' and runner.coordinator.state=='StopEngine':
                    physics_wait.set()
                    unblock.wait(8)
                super().close()
            def step(self,n):
                if self.physics_step>=260 and mode in ('physics_hang','signal_physics'):
                    physics_wait.set()
                    unblock.wait(8)
                self.steps.append((self.physics_step,self.released))
                return super().step(n)
        class ObservedRunner(Runner):
            application=None
            facts=[]
            arms=[]
            leaves=[]
            commands=[]
            stop_sent=False
            stop_at=None
            kill_triggered=False
            kill_exit=None
            def _advance(self,event):
                f=event.frame
                if f is not None:
                    if f.kind==6:self.arms.append(f)
                    if f.kind==7:self.leaves.append(f)
                    if f.kind==3:self.facts.append((f.control_step,f.payload))
                    if f.kind==4 and f.payload['message']['kind']==5:
                        self.commands.append((f.control_step,f.payload['message']))
                    if f.kind==16:self.commands.append((f.control_step,f.payload['command']))
                result=super()._advance(event)
                if mode=='death_after' and self.coordinator.runtime_only and self.application and self.application.poll() is None:
                    self.application.kill()
                return result
            def _actions(self,actions):
                for a in actions:
                    if a.kind=='send' and a.peer==3 and a.frame.kind==15 and self.application:
                        key={60:b'd',70:b'i',120:b'l'}.get(a.frame.control_step)
                        if a.frame.control_step==150 and mode in ('leave','death_after','death_before','fault_leave','engine_stop_hang'):
                            if mode=='death_before':
                                # The scenario under test is the SDK application dying
                                # while release is still pending. An application that
                                # already exited did not reach this trigger: the runner
                                # must then prove the startup failure instead of this
                                # branch crediting an early crash as the scenario.
                                self.kill_exit=self.application.poll()
                                if self.kill_exit is None:
                                    self.application.kill();self.kill_triggered=True
                            else: key=b'q'
                        if key:self.application.stdin.write(key);self.application.stdin.flush()
                return super()._actions(actions)
        runner=ObservedRunner(load_bootstrap(d/'bootstrap.json'),d/'bridge.sock',DelayedEngine)
        thread=threading.Thread(target=runner.run);thread.start()
        processes=[];logs=[]
        try:
            for name in ('robot_runtime','application'):
                log=open(d/(name+'.log'),'w+');logs.append(log)
                processes.append(subprocess.Popen([str(build/name),'--config',str(config)],
                    stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT))
                if name=='application':runner.application=processes[-1]
            deadline=time.monotonic()+30
            triggered=False
            pending_since=None
            triggered_at=None
            while runner.coordinator.step<450 and time.monotonic()<deadline:
                if mode in ('physics_hang','signal_physics') and physics_wait.is_set() and not triggered:
                    if mode=='signal_physics': processes[0].send_signal(signal.SIGTERM)
                    else: runner.request_shutdown()
                    triggered=True
                    triggered_at=time.monotonic()
                if mode in ('inference_shutdown','inference_signal') and runner.coordinator.step==120 and runner.coordinator.state=='ApplicationPhase':
                    if pending_since is None: pending_since=time.monotonic()
                    if not triggered and time.monotonic()-pending_since>.03:
                        if mode=='inference_signal': processes[1].send_signal(signal.SIGTERM)
                        else: runner.request_shutdown()
                        triggered=True
                        triggered_at=time.monotonic()
                if runner.coordinator.state in ('Failed','Closed'):break
                if mode=='inference_signal' and processes[1].poll() is not None:break
                if mode=='death_before' and not runner.kill_triggered and any(
                        p.poll() is not None for p in processes):break
                time.sleep(.002)
            for log in logs:log.flush();log.seek(0)
            output='\n'.join(log.read() for log in logs)
            if mode in ('death_before','physics_hang','signal_physics'):
                if mode=='death_before':
                    polls=[p.poll() for p in processes]
                    assert runner.kill_triggered,(
                        'application did not reach the scripted death-before-release kill',
                        runner.coordinator.state,runner.coordinator.step,
                        runner.coordinator.phase,sorted(runner.coordinator.peers),
                        polls,runner.kill_exit,output)
                assert runner.coordinator.state=='Failed',(mode,runner.coordinator.state,
                    runner.coordinator.step,runner.coordinator.phase,
                    sorted(runner.coordinator.peers),
                    [p.poll() for p in processes],runner.kill_exit,output)
                assert not runner.coordinator.stop_proven,output
                assert not runner.coordinator.runtime_only,output
                if mode!='death_before':
                    assert triggered,output
                    assert time.monotonic()-triggered_at<2.5,(mode,'host bound exceeded')
                print('PASS actual SDK bounded stopped-unproven',mode,runner.coordinator.failure)
                return
            if mode=='inference_shutdown':
                assert triggered,output
                thread.join(3)
                assert runner.coordinator.state=='Closed',(runner.coordinator.failure,output)
                assert runner.coordinator.stop_proven
                assert processes[0].wait(timeout=3)==0,output
                assert processes[1].wait(timeout=3)==0,output
                assert not runner.arms,output
                assert all(step<=120 for step,_ in runner.commands),runner.commands
                print('PASS independent shutdown during actual inference wait')
                return
            if mode=='inference_signal':
                assert triggered,output
                assert processes[1].wait(timeout=3)==0,output
                assert runner.coordinator.runtime_only,(runner.coordinator.failure,output)
                assert not runner.arms,output
                assert all(step<=120 for step,_ in runner.commands),runner.commands
                runner.request_shutdown();thread.join(3)
                assert runner.coordinator.state=='Closed',(runner.coordinator.failure,output)
                assert processes[0].wait(timeout=3)==0,output
                print('PASS application signal cancels actual inference and completes leave')
                return
            assert runner.coordinator.step>=450,(runner.coordinator.step,runner.coordinator.state,
                runner.coordinator.deadline,runner.coordinator.host,
                sorted(runner.coordinator.peers),[p.poll() for p in processes],
                thread.is_alive(),runner.worker.is_alive(),runner.coordinator.failure,output)
            assert processes[1].wait(timeout=2)==(-9 if mode=='death_after' else 3 if mode=='fault_leave' else 0),output
            assert processes[0].poll() is None,output
            assert len(engines)==1
            engine=engines[0]
            assert len(engine.releases)==1,engine.releases
            index,before,after=engine.releases[0]
            assert before==after,('release teleported state',before,after)
            # Under the approved overlap the interval [s, s+1) is already in
            # flight when the application phase at s can carry the arm, so the
            # proven arm authorizes the following interval: engine index
            # 2*(s+1). See docs/simulation-protocol.md.
            assert index==(runner.arms[0].control_step+1)*2,(index,runner.arms)
            assert all(not released for step,released in engine.steps if step<index)
            assert all(released for step,released in engine.steps if step>=index)
            assert len(runner.arms)==1,('release missing/duplicate',runner.arms,output)
            assert runner.coordinator.release_state=='Released',runner.coordinator.failure
            assert len(runner.leaves)==1,output
            assert runner.coordinator.runtime_only,output
            # More than 500ms supervisor lease has elapsed without synthetic heartbeats.
            assert runner.coordinator.states[-1]['session_id']==bytes(16),runner.coordinator.states[-1]
            assert runner.coordinator.states[-1]['execution_state']==(4 if mode=='fault_leave' else 2),runner.coordinator.states[-1]
            runner.request_shutdown()
            thread.join(4)
            assert not thread.is_alive(),runner.coordinator.state
            if mode in ('fault_leave','engine_stop_hang'):
                assert runner.coordinator.state=='Failed',runner.coordinator.state
                assert not runner.coordinator.stop_proven
                assert processes[0].wait(timeout=3)!=0,output
                assert engine.released and len(engine.releases)==1
                if mode=='engine_stop_hang': assert physics_wait.is_set()
                print('PASS actual guarded/unreachable-engine shutdown refusal',mode)
                return
            assert runner.coordinator.state=='Closed',(runner.coordinator.failure,
                [p.poll() for p in processes],(d/'robot_runtime.log').read_text(),
                (d/'application.log').read_text())
            assert processes[0].wait(timeout=3)==0,output
            assert runner.coordinator.stop_proven
            print('PASS actual SDK release/LeaveCommitted/runtime-only/expired-session shutdown domain224')
        finally:
            runner.request_stop();unblock.set();thread.join(3)
            if runner.worker: runner.worker.join(2)
            for p in processes:
                if p.poll() is None:p.send_signal(signal.SIGTERM)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            for log in logs:log.close()

if __name__=='__main__':
    build=Path(sys.argv[1]).resolve()
    for mode in ('leave','death_before','death_after','physics_hang','signal_physics','inference_shutdown','inference_signal','fault_leave','engine_stop_hang'):
        run(build,mode)
