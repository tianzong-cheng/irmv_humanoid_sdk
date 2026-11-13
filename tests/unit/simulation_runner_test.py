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
from simulation.coordinator import Bootstrap
try:
    from simulation.main import Runner
    from fake_engine import FakeEngine
except ImportError:
    raise AssertionError('Task 6 runner/fake engine missing')


def boot(**kw):
    return Bootstrap(b'r'*16,b't'*16,b'c'*32,b'a'*32,b'e'*32,1000,2000,{},
                     startup_timeout_us=1000000,phase_timeout_us=200000,**kw)


class RunnerTest(unittest.TestCase):
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

    def test_two_socket_peers_complete_one_step_then_stop(self):
        with tempfile.TemporaryDirectory() as d:
            path=str(pathlib.Path(d)/'bridge.sock')
            runner=Runner(boot(),path,FakeEngine)
            result=[]
            thread=threading.Thread(target=lambda:result.append(runner.run()))
            thread.start()
            sockets=[]
            try:
                deadline=time.monotonic()+1
                while not pathlib.Path(path).exists() and time.monotonic()<deadline:
                    time.sleep(.001)
                for role in (2,3):
                    s=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);s.settimeout(1)
                    s.connect(path);sockets.append(s)
                    s.send(p.encode(p.Frame(1,dict(runner.coordinator.hello,role=role),b'r'*16)))
                rt,app=sockets
                def recv(s,kind):
                    f=p.decode(s.recv(65536));self.assertEqual(f.kind,kind);return f
                recv(rt,1);recv(app,1)
                f=recv(rt,2);recv(rt,15)
                m=p._default('MessageIdentity');m['runtime_id']=b't'*16
                rt.send(p.encode(dataclasses.replace(f,kind=3, payload=dict(write_id=1,
                    accepted=True,state_seq=1,command=m,tau=(2.,)))))
                for sender,consumer,role,phase,disposition in ((rt,app,2,1,2),(app,rt,3,2,1)):
                    messages=[]
                    if role==2:
                        state=dict(m,kind=3,sequence=1,stamp_us=1)
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
                runner.request_stop()
                thread.join(1)
                self.assertFalse(thread.is_alive());self.assertEqual(result,[1])
            finally:
                runner.request_stop()
                for s in sockets:s.close()
                thread.join(2)
            self.assertNotIn('mujoco',sys.modules)
            self.assertNotIn('warp',sys.modules)

if __name__=='__main__':unittest.main()
