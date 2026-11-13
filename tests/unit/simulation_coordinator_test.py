"""Pure lockstep contract; stdlib only, simulated host deadlines."""
import dataclasses
import hashlib
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'tools'))
from simulation import protocol as p
try:
    from simulation.coordinator import Bootstrap, Event, SimulationCoordinator
except ImportError:
    raise AssertionError('Task 6 coordinator is missing')


def bootstrap(**kw):
    return Bootstrap(run_id=b'r'*16, runtime_id=b't'*16,
                     config_digest=b'c'*32, application_digest=b'a'*32,
                     engine_digest=b'e'*32, physics_dt_us=1000,
                     control_dt_us=2000, engine_config={}, **kw)


def feedback(step=0):
    from simulation.engine import Feedback
    return Feedback(step, step*.001, (0.,), (0.,), (0.,),
                    (1.,0.,0.,0.), (0.,)*3, (0.,0.,9.81), (0.,)*3, (0.,)*6)


class CoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.c = SimulationCoordinator(bootstrap(phase_timeout_us=100, startup_timeout_us=1000))
        self.assertEqual([a.kind for a in self.c.advance(Event('start'), 0)], ['initialize'])

    def frame(self, kind, payload, phase=1):
        return p.Frame(p.KINDS.index(kind)+1, payload, b'r'*16,
                       self.c.step, phase, 1+self.c.step*2000)

    def send(self, role, kind, payload, phase=1, now=None):
        return self.c.advance(Event('frame', role, self.frame(kind,payload,phase)),
                              max(3,self.c.host) if now is None else now)

    def ready(self):
        for role in (2,3):
            self.send(role, 'Hello', dict(self.c.hello, role=role), phase=0, now=1)
        actions = self.c.advance(Event('feedback', feedback=feedback()), 2)
        self.assertEqual([(a.kind,a.peer) for a in actions], [('send',2),('send',2)])
        self.assertEqual(actions[0].frame.kind, 2)
        self.assertEqual(actions[1].frame.payload, {'role':2,'stage':1})

    def inventory(self, role, phase, messages=None):
        if messages is None:
            messages=()
            if role==2:
                m=p._default('MessageIdentity')
                m.update(runtime_id=b't'*16,kind=3,sequence=self.c.step+1,
                         stamp_us=1+self.c.step*2000)
                messages=(m,)
        data = [dict(sender=role, message=m) for m in messages]
        for m in data:
            self.send(role,'Manifest',m,phase)
        seal = dict(sender=role, manifest_count=len(data), manifest_digest=hashlib.sha256(
            b''.join(p._pack('Manifest', m) for m in data)).digest())
        a = self.send(role,'PhaseSeal',seal,phase)
        self.assertEqual(a[-1].peer, 5-role)
        self.assertEqual(a[-1].frame.kind,11)
        return seal

    def finish(self, role, phase, seal):
        consumer=5-role
        a=self.send(consumer,'InventoryReady',dict(consumer=consumer,seal=seal),phase)
        self.assertEqual([(x.peer,x.frame.kind) for x in a],
                         [(role,14),(3,15)] if role==2 else [(role,14)])
        if role==2:
            self.assertEqual((a[1].frame.phase,a[1].frame.payload),(1,{'role':3,'stage':2}))
        return self.send(consumer,'PhaseAck',dict(consumer=consumer,
                    disposition=2 if role==2 else 1,seal=seal),phase)

    def torque(self, value=4.):
        m = p._default('MessageIdentity'); m['runtime_id']=b't'*16
        return dict(write_id=self.c.step+1, accepted=True,state_seq=self.c.step+1,
                    command=m,tau=(value,))

    def test_golden_two_ticks_old_torque_and_empty_handshake(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        s=self.inventory(2,1)
        self.assertEqual(self.c.state,'RuntimeTick')
        a=self.finish(2,1,s)
        self.assertEqual(a,())
        s=self.inventory(3,2)
        self.assertEqual(self.c.state,'IngressWait')
        a=self.finish(3,2,s)
        self.assertEqual([x.kind for x in a],['step'])
        self.assertEqual((a[0].torque,a[0].substeps),((4.,),2))
        self.assertEqual(self.c.step,0)
        a=self.c.advance(Event('feedback',feedback=feedback(2)),90)
        self.assertEqual(self.c.step,1)
        self.assertEqual(a[-1].frame.logical_us,2001)
        self.send(2,'Torque',self.torque(8.))
        s=self.inventory(2,1); self.finish(2,1,s)
        s=self.inventory(3,2); a=self.finish(3,2,s)
        self.assertEqual(a[0].torque,(8.,))

    def test_no_progress_during_inference_and_timeout_not_extended_by_duplicate(self):
        self.ready(); self.send(2,'Torque',self.torque())
        s=self.inventory(2,1); self.finish(2,1,s)
        self.assertEqual(self.c.advance(Event('poll'),90),())
        self.assertEqual(self.c.step,0)
        self.assertEqual(self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=s),1,99),())
        self.c.advance(Event('poll'),103)
        self.assertEqual(self.c.state,'Failed')

    def test_ack_before_installed_is_fatal(self):
        self.ready(); s=self.inventory(2,1)
        self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=s))
        self.assertEqual(self.c.state,'Failed')

    def test_wrong_role_run_phase_digest(self):
        for change in ('role','run','phase','digest'):
            self.setUp(); self.ready()
            if change=='role': self.send(3,'Torque',self.torque())
            elif change=='run':
                f=dataclasses.replace(self.frame('Torque',self.torque()),run_id=b'x'*16)
                self.c.advance(Event('frame',2,f),3)
            elif change=='phase': self.send(2,'Torque',self.torque(),2)
            else: self.send(2,'PhaseSeal',dict(sender=2,manifest_count=0,manifest_digest=bytes(32)))
            self.assertEqual(self.c.state,'Failed',change)

    def test_feedback_step_and_nonfinite_rejected(self):
        for fb in (feedback(1),dataclasses.replace(feedback(),engine_time_s=float('nan'))):
            self.setUp()
            self.c.advance(Event('feedback',feedback=fb),1)
            self.assertEqual(self.c.state,'Failed')

    def test_bootstrap_rejects_overflow_duration_and_bad_ratios(self):
        for kw in ({'control_dt_us':2**64},{'physics_dt_us':700}, {'max_duration_us':3600000001}):
            b=bootstrap()
            with self.assertRaises(ValueError):
                SimulationCoordinator(dataclasses.replace(b,**kw))

    def test_full_inventory_retransmission_is_idempotent_beyond_frame_cache(self):
        self.ready()
        messages=[]
        for i in range(64):
            m=p._default('MessageIdentity')
            m.update(runtime_id=b't'*16,kind=2,sequence=i)
            messages.append(m)
        seal=self.inventory(2,1,messages)
        self.assertEqual(self.send(2,'Manifest',dict(sender=2,message=messages[0])),())
        self.assertEqual(self.c.state,'RuntimeTick')
        self.assertEqual(self.send(2,'PhaseSeal',seal),())

    def test_conflicting_duplicate_and_sixty_fifth_entry_fail(self):
        for conflict in (False,True):
            self.setUp();self.ready()
            m=p._default('MessageIdentity');m.update(runtime_id=b't'*16,kind=2)
            self.send(2,'Manifest',dict(sender=2,message=m))
            if conflict:
                m=dict(m,content_digest=b'z'*32)
                self.send(2,'Manifest',dict(sender=2,message=m))
            else:
                for i in range(1,65):
                    self.send(2,'Manifest',dict(sender=2,message=dict(m,sequence=i)))
            self.assertEqual(self.c.state,'Failed')

    def test_invalid_quaternion_and_engine_time_jump_fail(self):
        for fb in (dataclasses.replace(feedback(),imu_quaternion=(0.,)*4),
                   dataclasses.replace(feedback(),engine_time_s=100.)):
            self.setUp();self.c.advance(Event('feedback',feedback=fb),1)
            self.assertEqual(self.c.state,'Failed')
        self.setUp();self.ready();self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1);self.finish(2,1,seal)
        seal=self.inventory(3,2);self.finish(3,2,seal)
        self.c.advance(Event('feedback',feedback=dataclasses.replace(feedback(2),engine_time_s=100.)),4)
        self.assertEqual(self.c.state,'Failed')

    def test_runtime_state_must_match_tick_torque(self):
        self.ready();self.send(2,'Torque',self.torque())
        m=p._default('MessageIdentity')
        m.update(runtime_id=b't'*16,kind=3,sequence=999,stamp_us=1)
        seal=self.inventory(2,1,(m,))
        self.finish(2,1,seal)
        self.assertEqual(self.c.state,'Failed')

    def test_empty_runtime_inventory_cannot_replace_tick_state(self):
        self.ready();self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1,())
        self.finish(2,1,seal)
        self.assertEqual(self.c.state,'Failed')

    def test_disconnect_and_stop_fail_closed_no_release(self):
        self.ready()
        a=self.c.advance(Event('disconnect',2),4)
        self.assertEqual(self.c.state,'Failed')
        self.assertNotIn('release',[x.kind for x in a])


if __name__=='__main__': unittest.main()
