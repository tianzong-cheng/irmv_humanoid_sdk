"""Bounded real sockets and an engine worker, no optional engine packages."""
import dataclasses
import hashlib
import pathlib
import socket
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'tools'))
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'support'/'simulation'))
from simulation import protocol as p
from simulation.coordinator import Bootstrap, Action
try:
    from simulation.main import Runner
    from fake_engine import FakeEngine
except ImportError:
    raise AssertionError('Task 6 runner/fake engine missing')


def boot(**kw):
    return Bootstrap(b'r'*16,b't'*16,b'c'*32,b'a'*32,b'e'*32,1000,2000,{},
                     startup_timeout_us=1000000,phase_timeout_us=200000,**kw)


class RunnerTest(unittest.TestCase):
    def test_stop_completion_waits_for_actual_engine_close(self):
        entered=threading.Event()
        unblock=threading.Event()
        class Closing(FakeEngine):
            def close(self):
                entered.set()
                unblock.wait(2)
                super().close()
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),Closing)
            thread=threading.Thread(target=runner._engine_worker,daemon=True)
            thread.start()
            try:
                runner.jobs.put(Action('initialize'));runner.results.get(timeout=1)
                runner.jobs.put(Action('stop_output',torque=(0.,)))
                self.assertTrue(entered.wait(.3), 'stop output never closed engine')
                self.assertTrue(runner.results.empty(), 'reported stop before close completed')
                unblock.set()
                self.assertEqual(runner.results.get(timeout=1).kind,'stopped')
            finally:
                unblock.set();runner.stop.set();thread.join(1)

    def test_worker_release_precedes_step_and_preserves_feedback(self):
        events=[]
        class Observed(FakeEngine):
            def release_base(self):
                before=self.read_feedback()
                events.append(('release',before))
                super().release_base()
                self_test.assertEqual(before,self.read_feedback())
            def step(self,n):
                events.append(('step',self.released))
                return super().step(n)
        self_test=self
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),Observed)
            thread=threading.Thread(target=runner._engine_worker,daemon=True)
            thread.start()
            try:
                runner.jobs.put(Action('initialize'))
                runner.results.get(timeout=1)
                runner.jobs.put(Action('step',torque=(1.,),substeps=2,release=True))
                result=runner.results.get(timeout=1)
                self.assertTrue(result.released)
                self.assertEqual([v[0] for v in events],['release','step'])
                self.assertTrue(events[1][1])
                self.assertEqual(result.feedback.physics_step,2)
            finally: runner.stop.set();thread.join(1)

    def test_shutdown_gate_cancels_queued_release_without_engine_call(self):
        events=[]
        class Observed(FakeEngine):
            def release_base(self): events.append('release');super().release_base()
            def step(self,n): events.append('step');return super().step(n)
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),Observed)
            thread=threading.Thread(target=runner._engine_worker,daemon=True);thread.start()
            try:
                runner.jobs.put(Action('initialize'));runner.results.get(timeout=1)
                runner.request_shutdown()
                runner.jobs.put(Action('step',torque=(1.,),substeps=2,release=True))
                self.assertEqual(runner.results.get(timeout=1).kind,'cancelled')
                self.assertEqual(events,[])
            finally: runner.stop.set();thread.join(1)

    def test_pending_signal_cancels_queued_release_before_control_loop(self):
        calls=[]
        class Observed(FakeEngine):
            def release_base(self): calls.append('release');super().release_base()
            def step(self,n): calls.append('step');return super().step(n)
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),Observed)
            thread=threading.Thread(target=runner._engine_worker,daemon=True);thread.start()
            try:
                runner.jobs.put(Action('initialize'));runner.results.get(timeout=1)
                runner._notify_shutdown(0,None)
                self.assertFalse(runner.shutdown.is_set())
                runner.jobs.put(Action('step',torque=(1.,),substeps=2,release=True))
                self.assertEqual(runner.results.get(timeout=1).kind,'cancelled')
                self.assertEqual(calls,[])
            finally: runner.stop.set();thread.join(1)

    def test_pending_signal_does_not_undo_started_release_or_physics(self):
        entered=threading.Event();finish=threading.Event()
        class InFlight(FakeEngine):
            def step(self,n):
                entered.set()
                if not finish.wait(1): raise RuntimeError('test handoff deadline')
                return super().step(n)
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),InFlight)
            thread=threading.Thread(target=runner._engine_worker,daemon=True);thread.start()
            try:
                runner.jobs.put(Action('initialize'));runner.results.get(timeout=1)
                runner.jobs.put(Action('step',torque=(1.,),substeps=2,release=True))
                self.assertTrue(entered.wait(1))
                runner._notify_shutdown(0,None)
                runner.request_shutdown()
                finish.set()
                result=runner.results.get(timeout=1)
                self.assertEqual(result.kind,'feedback')
                self.assertTrue(result.released)
                self.assertEqual(result.feedback.physics_step,2)
            finally: finish.set();runner.stop.set();thread.join(1)

    def test_timeout_responsive_during_stuck_initialize(self):
        unblock=threading.Event()
        class Stuck(FakeEngine):
            def initialize(self,config):
                unblock.wait(2)
                super().initialize(config)
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),Stuck)
            start=time.monotonic()
            try:
                self.assertEqual(runner.run(),1)
                self.assertLess(time.monotonic()-start,1.5)
                self.assertIn('deadline',runner.coordinator.failure)
                self.assertTrue(runner.worker.is_alive())
            finally: unblock.set(); runner.worker.join(1)

    def test_listener_never_unlinks_existing_path_or_accepts_public_parent(self):
        with tempfile.TemporaryDirectory() as d:
            path=pathlib.Path(d)/'bridge.sock'
            path.write_text('preserve')
            runner=Runner(boot(),str(path),FakeEngine)
            self.assertEqual(runner.run(),1)
            self.assertEqual(path.read_text(),'preserve')
            path.unlink()
            pathlib.Path(d).chmod(0o755)
            runner=Runner(boot(),str(path),FakeEngine)
            self.assertEqual(runner.run(),1)
            self.assertIn('private0700',runner.coordinator.failure)
            self.assertFalse(path.exists())

    def test_engine_exception_is_fatal_without_waiting_for_phase_timeout(self):
        class Broken(FakeEngine):
            def initialize(self,config):raise RuntimeError('engine exploded')
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),Broken)
            start=time.monotonic()
            self.assertEqual(runner.run(),1)
            self.assertLess(time.monotonic()-start,.5)
            self.assertEqual(runner.coordinator.failure,'engine exploded')
            self.assertFalse(runner.worker.is_alive())

    def test_runtime_eof_after_commit_is_not_lost(self):
        self.two_socket_peers(graceful=True)

    def test_two_socket_peers_complete_one_step_then_stop(self):
        self.two_socket_peers(graceful=False)

    def two_socket_peers(self, graceful):
        with tempfile.TemporaryDirectory() as d:
            path=str(pathlib.Path(d)/'bridge.sock')
            committed_closed=threading.Event()
            class ObservedRunner(Runner):
                def _advance(self,event):
                    super()._advance(event)
                    if event.kind=='stopped':
                        committed_closed.wait(1)
            runner=ObservedRunner(boot(),path,FakeEngine)
            result=[]
            thread=threading.Thread(target=lambda:result.append(runner.run()))
            thread.start()
            sockets=[]
            try:
                deadline=time.monotonic()+1
                # bind creates the pathname BEFORE listen; existence is not readiness.
                listening=False
                while time.monotonic()<deadline:
                    try:
                        listening=bool(runner.listener and runner.listener.getsockopt(
                            socket.SOL_SOCKET,socket.SO_ACCEPTCONN))
                    except OSError:pass
                    if listening:break
                    time.sleep(.001)
                self.assertTrue(listening,(runner.coordinator.state,runner.coordinator.failure))
                for role in (2,3):
                    s=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);s.settimeout(1)
                    s.connect(path);sockets.append(s)
                    s.send(p.encode(p.Frame(1,dict(runner.coordinator.hello,role=role),b'r'*16)))
                rt,app=sockets
                def recv(s,kind):
                    f=p.decode(s.recv(65536));self.assertEqual(f.kind,kind);return f
                recv(rt,1);recv(app,1)
                f=recv(rt,2);recv(rt,15)
                m={name:(bytes(16) if field=='Id' else bytes(32) if field=='Digest' else 0)
                   for field,name in p.SCHEMA['MessageIdentity']}
                m.update(kind=5,runtime_id=b't'*16)
                rt.send(p.encode(dataclasses.replace(f,kind=3, payload=dict(write_id=1,
                    accepted=True,state_seq=1,command=m,tau=(2.,)))))
                for sender,consumer,role,phase,disposition in ((rt,app,2,1,2),(app,rt,3,2,1)):
                    messages=[]
                    if role==2:
                        state=dict(m,kind=3,sequence=1,stamp_us=1,execution_state=2,output_action=1)
                        messages=[dict(sender=2,message=state)]
                    else:
                        command=dict(m,kind=5,sequence=1,source_state_seq=1,stamp_us=1)
                        messages=[dict(sender=3,message=command)]
                    for manifest in messages:
                        sender.send(p.encode(dataclasses.replace(f,kind=4,phase=phase,payload=manifest)))
                        recv(consumer,4)
                    seal=dict(sender=role,manifest_count=len(messages),manifest_digest=hashlib.sha256(
                        b''.join(p._pack('Manifest',item) for item in messages)).digest())
                    sf=dataclasses.replace(f,kind=11,phase=phase,payload=seal)
                    sender.send(p.encode(sf));recv(consumer,11)
                    consumer.send(p.encode(dataclasses.replace(sf,kind=13,payload=dict(consumer=5-role,seal=seal))))
                    recv(sender,14)
                    if role==2: recv(app,15)
                    consumer.send(p.encode(dataclasses.replace(sf,kind=12,payload=dict(consumer=5-role,disposition=disposition,seal=seal))))
                fact=recv(rt,10)
                self.assertEqual(fact.payload,dict(write_id=1,physics_step_begin=0,physics_step_end=2))
                f=recv(rt,2);self.assertEqual((f.control_step,f.logical_us),(1,2001))
                self.assertEqual(f.payload['tau'],(2.,))
                self.assertEqual(f.payload['q'],(.000006,))
                self.assertEqual(f.payload['dq'],(.004,))
                if graceful:
                    recv(rt,15)
                    runner.request_shutdown()
                    stop=recv(rt,17);recv(rt,2)
                    rt.send(p.encode(dataclasses.replace(stop,kind=3,payload=dict(
                        write_id=2,accepted=True,state_seq=3,command=m,tau=(0.,)))))
                    state=dict(m,kind=3,sequence=3,stamp_us=2001,
                               execution_state=1,output_action=0)
                    manifest=dict(sender=2,message=state)
                    rt.send(p.encode(dataclasses.replace(stop,kind=4,payload=manifest)))
                    seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
                        p._pack('Manifest',manifest)).digest())
                    rt.send(p.encode(dataclasses.replace(stop,kind=11,payload=seal)))
                    recv(rt,19)
                    rt.close();committed_closed.set()
                else: runner.request_stop()
                thread.join(1)
                self.assertFalse(thread.is_alive());self.assertEqual(result,[0 if graceful else 1])
            finally:
                runner.request_stop()
                for s in sockets:s.close()
                thread.join(2)
            self.assertNotIn('mujoco',sys.modules)
            self.assertNotIn('warp',sys.modules)

if __name__=='__main__':unittest.main()
