"""Bounded real sockets and an engine worker, no optional engine packages."""
import dataclasses
import hashlib
import os
import pathlib
import signal
import socket
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'tools'))
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'support'/'simulation'))
from simulation import protocol as p
from simulation.coordinator import Bootstrap, Action, Event
try:
    from simulation.main import Runner, _fd_identity, _restore_wakeup_fd
    from fake_engine import FakeEngine
except ImportError:
    raise AssertionError('Task 6 runner/fake engine missing')


def boot(**kw):
    return Bootstrap(b'r'*16,b't'*16,b'c'*32,b'a'*32,b'e'*32,1000,2000,{},
                     startup_timeout_us=1000000,phase_timeout_us=200000,**kw)


class RunnerTest(unittest.TestCase):
    def test_evidence_feedback_snapshot_matches_asdict(self):
        from simulation.engine import Feedback
        from simulation.main import feedback_snapshot
        feedback=Feedback(physics_step=7,engine_time_s=.25,q=(.1,.2),dq=(0.,0),tau=(.3,.4),
                          imu_quaternion=(1.,0,0,0),imu_angular_velocity=(0.,0,0),
                          imu_specific_force=(0.,0,9.81),base_position=(0.,0,1.),
                          base_velocity=(0.,)*6,base_quaternion=(1.,0,0,0))
        self.assertEqual(feedback_snapshot(feedback),dataclasses.asdict(feedback))

    def test_socket_reset_after_committed_leave_is_eof_before_leave_fails(self):
        for committed in (False,True):
            with self.subTest(committed=committed), tempfile.TemporaryDirectory() as directory:
                runner=Runner(boot(),str(pathlib.Path(directory)/'bridge.sock'),FakeEngine)
                self.addCleanup(runner.selector.close)
                local,remote=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
                self.addCleanup(local.close)
                runner.connections[local]={'peer':3,'out':__import__('collections').deque()}
                runner.coordinator.runtime_only=committed
                # Unread outbound data makes peer close a real ECONNRESET on Linux.
                local.send(b'unconsumed LeaveCommitted')
                remote.close()
                receive=runner._receive_packet
                if committed:
                    packet,flags=receive(local)
                    self.assertEqual((packet,flags),(b'',0))
                else:
                    with self.assertRaises(ConnectionResetError):receive(local)

    def test_pending_write_to_reset_retired_app_does_not_abort_runtime(self):
        from collections import deque
        for committed in (False,True):
            with self.subTest(committed=committed), tempfile.TemporaryDirectory() as directory:
                runner=Runner(boot(),str(pathlib.Path(directory)/'bridge.sock'),FakeEngine)
                self.addCleanup(runner.selector.close)
                local,remote=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
                self.addCleanup(local.close)
                runner.connections[local]={'peer':3,'out':deque([b'pending commit'])}
                runner.selector.register(local,1)
                runner.peers[3]=local
                runner.coordinator.runtime_only=committed
                runner.coordinator.started=True;runner.coordinator.peers={2,3}
                local.send(b'unconsumed data');remote.close()
                if committed:
                    runner._flush(local)
                    self.assertNotIn(local,runner.connections)
                    self.assertEqual(runner.coordinator.peers,{2})
                    self.assertEqual(runner.coordinator.failure,'')
                else:
                    with self.assertRaises((ConnectionResetError,BrokenPipeError)):runner._flush(local)

    def test_overlapped_completion_is_held_until_both_handshakes(self):
        # The real runner gate, real coordinator and real frame codec, without
        # an engine thread: completed feedback for [t,t+dt) may not be consumed
        # (nor recorded) before the phase-2 completion arrives.
        from collections import deque
        from simulation.engine import Feedback
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(boot(),str(pathlib.Path(d)/'bridge.sock'),FakeEngine)
            self.addCleanup(runner.selector.close)
            local,remote={},{}
            for role in (2,3):
                local[role],remote[role]=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
                self.addCleanup(local[role].close); self.addCleanup(remote[role].close)
                runner.connections[local[role]]={'peer':role,'out':deque(),'mask':1}
                runner.selector.register(local[role],1)
                runner.peers[role]=local[role]
                remote[role].setblocking(False)
            c=runner.coordinator
            def frame(kind,payload,phase=1):
                return p.Frame(p.KINDS.index(kind)+1,payload,b'r'*16,c.step,phase,
                               1+c.step*2000)
            def send(role,kind,payload,phase=1):
                runner._advance(Event('frame',role,frame(kind,payload,phase)))
            def drain(role):
                out=[]
                while True:
                    try:packet=remote[role].recv(65536)
                    except BlockingIOError:return out
                    if not packet:return out
                    out.append(p.decode(packet))
            def identity(kind=5, **values):
                m={name:(bytes(16) if field=='Id' else bytes(32) if field=='Digest' else 0)
                   for field,name in p.SCHEMA['MessageIdentity']}
                m.update(kind=kind,runtime_id=b't'*16);m.update(values)
                return m
            def state_identity(step):
                m=identity(3,sequence=step+1,stamp_us=1+step*2000,
                           execution_state=2,output_action=1,content_digest=b's'*32)
                return m
            def feedback_at(step):
                return Feedback(step,step*.001,(0.,),(0.,),(0.,),(1.,0.,0.,0.),
                                (0.,)*3,(0.,0.,9.81),(0.,)*3,(0.,)*6)
            runner._advance(Event('start'))
            send(2,'Hello',dict(c.hello,role=2),0)
            send(3,'Hello',dict(c.hello,role=3),0)
            runner._advance(Event('feedback',feedback=feedback_at(0)))
            first=drain(2)
            self.assertEqual([(f.kind,f.control_step,f.phase) for f in first],
                             [(1,0,0),(2,0,1),(15,0,1)])
            self.assertEqual([f.kind for f in drain(3)],[1])
            torque=dict(write_id=1,accepted=True,state_seq=1,command=identity(),tau=(2.,))
            send(2,'Torque',torque)
            state=state_identity(0)
            manifest=dict(sender=2,message=state)
            send(2,'Manifest',manifest)
            seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
                p._pack('Manifest',manifest)).digest())
            send(2,'PhaseSeal',seal)
            published=drain(3)
            self.assertIn(p.KINDS.index('PhaseSeal')+1,[f.kind for f in published])
            self.assertEqual([runner.jobs.get_nowait().kind for _ in range(2)],
                             ['initialize','step'])
            send(3,'InventoryReady',dict(consumer=3,seal=seal))
            drain(2);drain(3)
            send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=seal))
            # Application phase inventory and seal (empty inventory is legal).
            empty=dict(sender=3,manifest_count=0,manifest_digest=hashlib.sha256(b'').digest())
            send(3,'PhaseSeal',empty,2)
            self.assertEqual(c.state,'IngressWait')
            self.assertEqual(len(drain(2)),1)
            send(2,'InventoryReady',dict(consumer=2,seal=empty),2)
            drain(3)
            # Engine completion arrives during the handshake: held, not consumed.
            runner._deliver_result(Event('feedback',feedback=feedback_at(2)))
            self.assertIsNotNone(runner.held_feedback)
            self.assertEqual((c.step,c.pending_feedback,c.physics_in_flight),(0,None,True))
            self.assertEqual(drain(2),[])
            # Phase-2 ACK closes the gate; the held completion is consumed and
            # the next tick is emitted exactly once.
            send(2,'PhaseAck',dict(consumer=2,disposition=1,seal=empty),2)
            self.assertIsNone(runner.held_feedback)
            self.assertEqual((c.step,c.pending_feedback,c.physics_in_flight),(1,None,False))
            applied=drain(2)
            self.assertEqual([f.kind for f in applied],
                             [p.KINDS.index('AppliedToStep')+1,2,p.KINDS.index('TickPermit')+1])
            self.assertEqual(applied[0].phase,3)
            self.assertEqual(applied[0].payload,
                             dict(write_id=1,physics_step_begin=0,physics_step_end=2))
            self.assertTrue(runner.jobs.empty())
            # No action is outstanding: a later completion is delivered so the
            # coordinator rejects it instead of holding it silently.
            self.assertTrue(runner._feedback_ready())

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

    def test_wakeup_fd_restore_keeps_previous_identity(self):
        previous=socket.socketpair()
        runner=socket.socketpair()
        try:
            previous[1].setblocking(False)
            runner[1].setblocking(False)
            self.assertEqual(signal.set_wakeup_fd(previous[1].fileno()),-1)
            fd=previous[1].fileno()
            identity=_fd_identity(fd)
            signal.set_wakeup_fd(runner[1].fileno())
            _restore_wakeup_fd(fd,identity)
            self.assertEqual(signal.set_wakeup_fd(-1),fd)
        finally:
            signal.set_wakeup_fd(-1)
            for endpoint in previous+runner: endpoint.close()

    def test_wakeup_fd_restore_disarms_reused_descriptor(self):
        previous=socket.socketpair()
        runner=socket.socketpair()
        replacement=None
        reused_fd=None
        raw_fd=False
        try:
            previous[1].setblocking(False)
            runner[1].setblocking(False)
            self.assertEqual(signal.set_wakeup_fd(previous[1].fileno()),-1)
            fd=previous[1].fileno()
            identity=_fd_identity(fd)
            signal.set_wakeup_fd(runner[1].fileno())
            previous[1].close()
            self.assertIsNone(_fd_identity(fd))
            # Reuse the freed number for a different socket, deterministically.
            replacement=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
            if replacement.fileno()!=fd:
                os.dup2(replacement.fileno(),fd)
                replacement.close()
                replacement=None
                reused_fd=fd
                raw_fd=True
            else:
                reused_fd=fd
            self.assertNotEqual(_fd_identity(reused_fd),identity)
            _restore_wakeup_fd(fd,identity)
            self.assertEqual(signal.set_wakeup_fd(-1),-1)
        finally:
            signal.set_wakeup_fd(-1)
            if replacement is not None: replacement.close()
            if raw_fd and reused_fd is not None:
                try:os.close(reused_fd)
                except OSError:pass
            for endpoint in previous+runner: endpoint.close()

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
                    runtime_commit=recv(rt,19)
                    app_stop=recv(app,17)
                    self.assertEqual(app_stop.payload,dict(role=3))
                    app_commit=recv(app,19)
                    self.assertEqual(app_commit.payload,runtime_commit.payload)
                    self.assertEqual(app_commit.control_step,runtime_commit.control_step)
                    self.assertEqual(app_commit.phase,4)
                    rt.close();app.close();committed_closed.set()
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
