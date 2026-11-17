"""Pure lockstep contract; stdlib only, simulated host deadlines.

Every normal control step now overlaps the engine handoff for [t,t+dt) with the
application phase handshake for t. These tests pin the gate explicitly: the step
action is issued only after the runtime torque and phase-1 seal, and the next
tick exists only after both phase completions plus completed engine feedback.
"""
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


def feedback(step=0, **kw):
    from simulation.engine import Feedback
    return Feedback(step, step*.001, (0.,), (0.,), (0.,),
                    (1.,0.,0.,0.), (0.,)*3, (0.,0.,9.81), (0.,)*3, (0.,)*6, **kw)


def identity(kind=5, **values):
    # Explicit protocol-neutral test fixture; all unused projections are zero.
    m={name:(bytes(16) if field=='Id' else bytes(32) if field=='Digest' else 0)
       for field,name in p.SCHEMA['MessageIdentity']}
    m.update(kind=kind,runtime_id=b't'*16)
    m.update(values)
    return m


def state_identity(step, command=None, **values):
    m=identity(3,sequence=step+1,stamp_us=1+step*2000,execution_state=2,
               output_action=1,content_digest=b's'*32)
    if command:
        m.update(execution_state=3,output_action=2,source_state_seq=command['sequence'],
                 session_id=command['session_id'],producer_id=command['producer_id'],epoch=command['epoch'])
    m.update(values)
    return m


def seal_for(role, messages):
    data=[dict(sender=role,message=m) for m in messages]
    return data, dict(sender=role, manifest_count=len(data), manifest_digest=hashlib.sha256(
        b''.join(p._pack('Manifest', m) for m in data)).digest())


class CoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.c = SimulationCoordinator(bootstrap(phase_timeout_us=100, startup_timeout_us=1000))
        self.assertEqual([a.kind for a in self.c.advance(Event('start'), 0)], ['initialize'])

    def frame(self, kind, payload, phase=1, step=None):
        step=self.c.step if step is None else step
        return p.Frame(p.KINDS.index(kind)+1, payload, b'r'*16,
                       step, phase, 1+step*2000)

    def send(self, role, kind, payload, phase=1, now=None, step=None):
        # Keep the legacy three-argument call so external test harnesses can
        # supply their own frame builder unchanged.
        built=(self.frame(kind,payload,phase) if step is None
               else self.frame(kind,payload,phase,step))
        return self.c.advance(Event('frame', role, built),
                              max(3,self.c.host) if now is None else now)

    def ready(self):
        for role in (2,3):
            self.send(role, 'Hello', dict(self.c.hello, role=role), phase=0, now=1)
        actions = self.c.advance(Event('feedback', feedback=feedback()), 2)
        self.assertEqual([(a.kind,a.peer) for a in actions], [('send',2),('send',2)])
        self.assertEqual(actions[0].frame.kind, 2)
        self.assertEqual(actions[1].frame.payload, {'role':2,'stage':1})

    def inventory_with_actions(self, role, phase, messages=None):
        """Stream one exact inventory. Returns (seal, actions for the seal)."""
        if messages is None:
            messages=()
            if role==2:
                messages=(state_identity(self.c.step),)
        data=[dict(sender=role,message=m) for m in messages]
        for m in data:
            self.send(role,'Manifest',m,phase)
        # Digest after streaming: the send hook may normalise stamp metadata.
        seal=dict(sender=role, manifest_count=len(data), manifest_digest=hashlib.sha256(
            b''.join(p._pack('Manifest', m) for m in data)).digest())
        return seal, self.send(role,'PhaseSeal',seal,phase)

    def inventory(self, role, phase, messages=None):
        """Compatibility wrapper: seal only, actions via ``inventory_actions``."""
        seal,actions=self.inventory_with_actions(role,phase,messages)
        self.inventory_actions=actions
        return seal

    def finish(self, role, phase, seal):
        """InventoryReady then PhaseAck. Returns (ready_actions, ack_actions)."""
        consumer=5-role
        ready=self.send(consumer,'InventoryReady',dict(consumer=consumer,seal=seal),phase)
        ack=self.send(consumer,'PhaseAck',dict(consumer=consumer,
                    disposition=2 if role==2 else 1,seal=seal),phase)
        return ready,ack

    def torque(self, value=4.):
        m = identity()
        return dict(write_id=self.c.step+1, accepted=True,state_seq=self.c.step+1,
                    command=m,tau=(value,))

    def command(self, **values):
        m=identity(5,session_id=b'u'*16,producer_id=b'p'*16,epoch=7,
                   sequence=9,source_state_seq=1,stamp_us=1,content_digest=b'd'*32)
        m.update(values)
        return m

    def request(self, **values):
        m=identity(1,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                   proposed_epoch=7,operation=3,content_digest=b'r'*32)
        m.update(values)
        return m

    def cycle(self, command=None, messages=(), proofs=(), deliver='late'):
        """One full overlapped control step. The step action must appear at the
        runtime phase-1 seal, never at the application phase."""
        torque=self.torque()
        if command:torque['command']=command
        self.send(2,'Torque',torque)
        seal,actions=self.inventory_with_actions(2,1,(state_identity(self.c.step,command),))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.assertTrue(self.c.physics_in_flight)
        self.finish(2,1,seal)
        self.assertEqual(self.c.state,'ApplicationPhase',self.c.failure)
        for proof in proofs:self.send(3,'FirstCommandProof',proof,phase=2)
        seal,actions=self.inventory_with_actions(3,2,messages)
        self.assertEqual([a.kind for a in actions],['send'],self.c.failure)
        if deliver=='early':
            self.assertEqual(self.c.advance(Event('feedback',
                feedback=feedback((self.c.step+1)*2)),self.c.host+1),())
            self.assertIsNotNone(self.c.pending_feedback)
        ready,ack=self.finish(3,2,seal)
        if deliver=='late':
            self.assertEqual(self.c.state,'PhysicsAdvance',self.c.failure)
            ack=self.c.advance(Event('feedback',
                feedback=feedback((self.c.step+1)*2)),self.c.host+1)
            self.assertEqual([a.kind for a in ack],['send','send','send'],self.c.failure)
        self.assertEqual(self.c.state,'RuntimeTick',self.c.failure)
        self.assertFalse(self.c.physics_in_flight)

    # -- overlap ordering and gates ---------------------------------------

    def test_step_starts_at_runtime_seal_and_next_tick_waits_for_both_proofs(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal,actions=self.inventory_with_actions(2,1)
        # The runtime seal is still forwarded first; physics starts immediately.
        self.assertEqual([(a.kind,a.peer) for a in actions],[('send',3),('step',0)])
        self.assertEqual(actions[0].frame.kind,p.KINDS.index('PhaseSeal')+1)
        self.assertEqual((actions[0].frame.control_step,actions[0].frame.phase),(0,1))
        self.assertEqual((actions[1].torque,actions[1].substeps),((4.,),2))
        self.assertFalse(actions[1].release)
        self.assertEqual((self.c.state,self.c.phase,self.c.step),( 'RuntimeTick',1,0))
        self.assertTrue(self.c.physics_in_flight)
        # Engine completion during phase 1 is held; no proof was skipped.
        self.assertEqual(self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1),())
        self.assertEqual((self.c.pending_feedback.physics_step,self.c.step), (2,0))
        ready,ack=self.finish(2,1,seal)
        self.assertEqual([(a.peer,a.frame.kind) for a in ready],
                         [(2,p.KINDS.index('PublishPermit')+1),
                          (3,p.KINDS.index('TickPermit')+1)])
        self.assertEqual((ack,self.c.state,self.c.phase),((),'ApplicationPhase',2))
        self.assertEqual(self.c.step,0)
        seal2,seal_actions=self.inventory_with_actions(3,2)
        self.assertEqual([(a.kind,a.peer) for a in seal_actions],[('send',2)])
        # App seal alone is not a completion.
        self.assertEqual(self.c.step,0)
        ready2,ack2=self.finish(3,2,seal2)
        self.assertEqual([a.kind for a in ack2],['send','send','send'])
        self.assertEqual([a.frame.kind for a in ack2],
                         [p.KINDS.index('AppliedToStep')+1,2,p.KINDS.index('TickPermit')+1])
        self.assertEqual((ack2[0].frame.control_step,ack2[0].frame.phase),(0,3))
        self.assertEqual(ack2[0].frame.payload,
                         dict(write_id=1,physics_step_begin=0,physics_step_end=2))
        self.assertEqual((ack2[1].frame.control_step,ack2[1].frame.phase),(1,1))
        self.assertEqual(ack2[1].frame.logical_us,2001)
        self.assertEqual((self.c.step,self.c.feedback.physics_step), (1,2))
        self.assertFalse(self.c.physics_in_flight)
        self.assertIsNone(self.c.pending_feedback)

    def test_next_tick_requires_completed_feedback_after_both_proofs(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        seal2=self.inventory(3,2)
        ready,ack=self.finish(3,2,seal2)
        self.assertEqual(ack,())
        self.assertEqual([a for a in ready+ack if a.kind=='step'],[])
        self.assertEqual((self.c.state,self.c.step),('PhysicsAdvance',0))
        self.assertTrue(self.c.physics_in_flight)
        # Feedback arrives only now; the next tick is emitted exactly once.
        actions=self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1)
        self.assertEqual([a.kind for a in actions],['send','send','send'])
        self.assertEqual(self.c.step,1)
        self.assertEqual(self.c.state,'RuntimeTick')

    def test_physics_never_starts_before_torque_or_seal(self):
        self.ready()
        # Seal first, torque absent: no engine action.
        seal,actions=self.inventory_with_actions(2,1)
        self.assertEqual([a.kind for a in actions],['send'],self.c.failure)
        self.assertFalse(self.c.physics_in_flight)
        self.assertEqual(self.c.step,0)
        # Torque now closes the safety gate, still before any app ACK.
        actions=self.send(2,'Torque',self.torque())
        self.assertEqual([a.kind for a in actions],['step'],self.c.failure)
        self.assertTrue(self.c.physics_in_flight)
        self.assertEqual(actions[0].torque,(4.,))
        # Exactly one interval was committed by the redundant phase-1 paths.
        ready,_=self.finish(2,1,seal)
        self.assertNotIn('step',[a.kind for a in ready])
        self.assertFalse(any(a.kind=='step' for a in
                             self.c.advance(Event('poll'),self.c.host+1)))

    def test_torque_alone_does_not_start_physics(self):
        self.ready()
        actions=self.send(2,'Torque',self.torque())
        self.assertEqual(actions,())
        self.assertFalse(self.c.physics_in_flight)
        seal,actions=self.inventory_with_actions(2,1)
        self.assertEqual([a.kind for a in actions],['send','step'])

    def test_feedback_while_in_flight_failure_terminates_and_records_reason(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal,actions=self.inventory_with_actions(2,1)
        self.assertEqual([a.kind for a in actions],['send','step'])
        self.finish(2,1,seal)
        self.assertEqual(self.c.advance(Event('feedback',feedback=feedback(2)),
                                        self.c.host+1),())
        # A conflicting phase-2 proof fails the run while the step is committed.
        bad=identity(5,runtime_id=b'X'*16)
        failed=self.send(3,'Manifest',dict(sender=3,message=bad),2)
        self.assertEqual(self.c.state,'Failed')
        self.assertEqual(self.c.failure,'manifest identity or kind')
        self.assertTrue(self.c.physics_in_flight)
        self.assertEqual(self.c.step,0)
        self.assertNotIn('step',[a.kind for a in failed])
        self.assertNotIn('send',[a.kind for a in failed if a.frame and
                                 a.frame.kind in (p.KINDS.index('AppliedToStep')+1,
                                                 2,p.KINDS.index('TickPermit')+1)])
        self.assertFalse(self.c.stop_proven)
        # No later event can silently complete or tick the failed experiment.
        self.assertEqual(self.c.advance(Event('feedback',feedback=feedback(2)),
                                        self.c.host+1),())
        self.assertEqual(self.c.state,'Failed')

    def test_phase_timeout_while_physics_in_flight_fails_without_hiding_step(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        self.inventory(2,1)
        self.assertTrue(self.c.physics_in_flight)
        actions=self.c.advance(Event('poll'),self.c.deadline)
        self.assertEqual(self.c.state,'Failed')
        self.assertEqual(self.c.failure,'host phase deadline')
        self.assertTrue(self.c.physics_in_flight)
        self.assertEqual(self.c.step,0)
        self.assertFalse(self.c.stop_proven)
        self.assertNotIn('step',[a.kind for a in actions])

    def test_duplicate_ack_cannot_unlock_next_tick_twice(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        seal2=self.inventory(3,2)
        consumer=2
        self.send(consumer,'InventoryReady',dict(consumer=consumer,seal=seal2),2)
        # The engine completion is already held/pending; the ACK closes the gate.
        self.assertEqual(self.c.advance(Event('feedback',feedback=feedback(2)),
                                        self.c.host+1),())
        frame=self.frame('PhaseAck',dict(consumer=consumer,disposition=1,seal=seal2),2)
        wire=p.encode(frame)
        actions=self.c.advance(Event('frame',consumer,p.decode(wire),wire=wire),self.c.host+1)
        self.assertEqual([a.kind for a in actions],['send','send','send'])
        self.assertEqual(self.c.step,1)
        # Exact duplicate: no state change, no second AppliedToStep/tick.
        self.assertEqual(self.c.advance(Event('frame',consumer,p.decode(wire),wire=wire),
                                        self.c.host+1),())
        self.assertEqual(self.c.step,1)
        self.assertFalse(self.c.physics_in_flight)
        self.assertEqual(self.c.state,'RuntimeTick')

    def test_engine_fault_wins_over_in_flight_handshake(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        actions=self.c.advance(Event('engine_error',reason='engine exploded'),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertEqual(self.c.failure,'engine exploded')
        self.assertTrue(self.c.physics_in_flight)
        self.assertNotIn('step',[a.kind for a in actions])
        self.assertEqual(self.c.advance(Event('feedback',feedback=feedback(2)),
                                        self.c.host+1),())
        self.assertEqual(self.c.failure,'engine exploded')
        # A time-fault completion during the same overlap window also wins.
        self.setUp();self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        fault=dataclasses.replace(feedback(2),engine_time_s=100.)
        actions=self.c.advance(Event('feedback',feedback=fault),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('engine time inconsistent',self.c.failure)
        self.assertTrue(self.c.physics_in_flight)
        self.assertNotIn('step',[a.kind for a in actions])

    def test_logical_duration_overflow_never_issues_another_step(self):
        self.c=SimulationCoordinator(bootstrap(max_duration_us=2000,
            phase_timeout_us=1000,startup_timeout_us=10000))
        self.assertEqual([a.kind for a in self.c.advance(Event('start'),0)],['initialize'])
        self.ready()
        self.cycle()
        self.assertEqual(self.c.step,1)
        self.send(2,'Torque',self.torque())
        seal,actions=self.inventory_with_actions(2,1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('logical duration or overflow',self.c.failure)
        self.assertFalse(self.c.physics_in_flight)
        self.assertNotIn('step',[a.kind for a in actions])

    def assert_frozen_stop(self,actions,step,physics_step):
        # The applied evidence keeps the committed engine horizon visible, but
        # the phase-4 identity and feedback stay at the control step the peers
        # were consuming (the runtime rejects a relabelled frozen sample).
        ratio=self.c.b.control_dt_us//self.c.b.physics_dt_us
        self.assertEqual([a.kind if a.frame is None else p.KINDS[a.frame.kind-1]
                          for a in actions],
                         ['AppliedToStep','ShutdownPermit','ShutdownPermit','Feedback'])
        applied=actions[0].frame
        self.assertEqual((applied.control_step,applied.phase),(step,3))
        self.assertEqual(applied.payload['physics_step_begin'],step*ratio)
        self.assertEqual(applied.payload['physics_step_end'],(step+1)*ratio)
        for permit in actions[1:3]:
            self.assertEqual((permit.frame.control_step,permit.frame.phase),(step,4))
        self.assertEqual((actions[3].frame.control_step,actions[3].frame.phase),(step,4))
        self.assertEqual(actions[3].frame.payload['physics_step'],physics_step*ratio)

    def test_stop_during_overlap_freezes_at_consumed_step(self):
        # RuntimeTick: the app has not even been given the step-2 permission.
        self.ready()
        self.send(2,'Torque',self.torque())
        self.inventory(2,1)
        self.assertEqual(self.c.advance(Event('stop'),self.c.host+1),())
        self.assertTrue(self.c.shutdown_pending)
        self.assertTrue(self.c.physics_in_flight)
        self.assertEqual((self.c.state,self.c.step),('RuntimeTick',0))
        actions=self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1)
        self.assertEqual(self.c.step,1)
        self.assertFalse(self.c.physics_in_flight)
        self.assert_frozen_stop(actions,0,0)
        self.assertEqual(self.c.state,'StopTick')
        self.assertFalse(self.c.stop_proven)
        # The runtime answers the frozen phase-4 tick at the same step.
        torque=self.torque(0.);torque['state_seq']=2
        self.send(2,'Torque',torque,4,step=0)
        state=state_identity(0,sequence=2,execution_state=1,output_action=0)
        data,seal=seal_for(2,(state,))
        self.send(2,'Manifest',data[0],4,step=0)
        actions=self.send(2,'PhaseSeal',seal,4,step=0)
        self.assertEqual([a.kind for a in actions],['stop_output'])
        self.assertEqual(self.c.state,'StopEngine')
        actions=self.c.advance(Event('stopped'),self.c.host+1)
        self.assertEqual(self.c.state,'Closed')
        self.assertTrue(self.c.stop_proven)
        self.assertEqual([(a.peer,p.KINDS[a.frame.kind-1]) for a in actions],
                         [(2,'ShutdownCommitted'),(3,'ShutdownCommitted')])
        for committed in actions:
            self.assertEqual((committed.frame.control_step,committed.frame.phase),(0,4))

    def test_stop_with_phase1_ack_pending_freezes_with_or_without_feedback(self):
        for completed in (True,False):
            with self.subTest(completed=completed):
                self.setUp();self.ready()
                self.send(2,'Torque',self.torque())
                seal=self.inventory(2,1)
                ready=self.send(3,'InventoryReady',dict(consumer=3,seal=seal),1)
                self.assertEqual([a.kind for a in ready],['send','send'])
                self.assertEqual((self.c.state,self.c.phase),('ApplicationPhase',1))
                if completed:
                    self.assertEqual(self.c.advance(Event('feedback',
                        feedback=feedback(2)),self.c.host+1),())
                actions=self.c.advance(Event('stop'),self.c.host+1)
                if completed:
                    self.assert_frozen_stop(actions,0,0)
                else:
                    self.assertEqual(actions,())
                    actions=self.c.advance(Event('feedback',
                        feedback=feedback(2)),self.c.host+1)
                    self.assert_frozen_stop(actions,0,0)
                self.assertEqual(self.c.step,1)
                self.assertEqual(self.c.state,'StopTick')
                self.assertFalse(self.c.stop_proven)

    def test_stop_with_phase2_ack_pending_freezes_with_completed_feedback(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        self.assertEqual(self.c.advance(Event('feedback',feedback=feedback(2)),
                                        self.c.host+1),())
        actions=self.c.advance(Event('stop'),self.c.host+1)
        self.assertEqual(self.c.step,1)
        self.assert_frozen_stop(actions,0,0)
        self.assertEqual(self.c.state,'StopTick')
        self.assertFalse(self.c.stop_proven)

    def test_stop_with_phase2_sealed_but_unacked_freezes(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        seal2=self.inventory(3,2)
        self.send(2,'InventoryReady',dict(consumer=2,seal=seal2),2)
        self.assertEqual(self.c.state,'IngressWait')
        self.assertEqual(self.c.advance(Event('stop'),self.c.host+1),())
        actions=self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1)
        self.assert_frozen_stop(actions,0,0)
        self.assertEqual(self.c.step,1)
        self.assertEqual(self.c.state,'StopTick')

    def test_stop_after_both_proofs_advances_and_stops_at_next_step(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal=self.inventory(2,1)
        self.finish(2,1,seal)
        seal2=self.inventory(3,2)
        self.finish(3,2,seal2)
        self.assertEqual(self.c.state,'PhysicsAdvance')
        self.assertEqual(self.c.advance(Event('stop'),self.c.host+1),())
        actions=self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1)
        self.assertEqual(self.c.step,1)
        self.assertEqual([a.kind if a.frame is None else p.KINDS[a.frame.kind-1]
                          for a in actions],
                         ['AppliedToStep','ShutdownPermit','ShutdownPermit','Feedback'])
        self.assertEqual(actions[0].frame.payload['physics_step_end'],2)
        for permit in actions[1:]:
            self.assertEqual((permit.frame.control_step,permit.frame.phase),(1,4))
        self.assertEqual(actions[-1].frame.payload['physics_step'],2)
        self.assertEqual(self.c.state,'StopTick')

    def test_one_millisecond_stop_freezes_and_still_proves_engine_close(self):
        self.c=SimulationCoordinator(dataclasses.replace(bootstrap(),
            control_dt_us=1000,physics_dt_us=1000,
            phase_timeout_us=1000,startup_timeout_us=10000))
        self.assertEqual([a.kind for a in self.c.advance(Event('start'),0)],['initialize'])
        c=self.c
        def frame(kind,payload,phase=1,step=None):
            step=c.step if step is None else step
            return p.Frame(p.KINDS.index(kind)+1,payload,b'r'*16,step,phase,
                           1+step*1000,c.b.physics_dt_us,c.b.control_dt_us)
        def send(role,kind,payload,phase=1,step=None):
            return c.advance(Event('frame',role,frame(kind,payload,phase,step)),
                             max(3,c.host))
        def state(seq,step):
            m=identity(3,sequence=seq,stamp_us=1+step*1000,execution_state=2,
                       output_action=1,content_digest=b's'*32)
            return m
        for role in (2,3):
            send(role,'Hello',dict(c.hello,role=role),0)
        c.advance(Event('feedback',feedback=feedback(0)),max(3,c.host))
        send(2,'Torque',dict(write_id=1,accepted=True,state_seq=1,
                             command=identity(),tau=(4.,)))
        m=state(1,0)
        send(2,'Manifest',dict(sender=2,message=m),1)
        seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
            p._pack('Manifest',dict(sender=2,message=m))).digest())
        actions=send(2,'PhaseSeal',seal,1)
        self.assertEqual([a.kind for a in actions],['send','step'])
        self.assertEqual(actions[1].substeps,1)
        # Stop while the phase-1 permission is still unacknowledged.
        self.assertEqual(c.advance(Event('stop'),c.host+1),())
        actions=c.advance(Event('feedback',feedback=feedback(1)),c.host+1)
        self.assertEqual(c.step,1)
        self.assertEqual([a.kind if a.frame is None else p.KINDS[a.frame.kind-1]
                          for a in actions],
                         ['AppliedToStep','ShutdownPermit','ShutdownPermit','Feedback'])
        for permit in actions[1:]:
            self.assertEqual((permit.frame.control_step,permit.frame.phase,
                              permit.frame.logical_us),(0,4,1))
        self.assertEqual(actions[-1].frame.payload['physics_step'],0)
        torque=dict(write_id=2,accepted=True,state_seq=2,command=identity(),tau=(0.,))
        send(2,'Torque',torque,4,step=0)
        m=identity(3,sequence=2,stamp_us=1,execution_state=1,output_action=0,
                   content_digest=b's'*32)
        data=[dict(sender=2,message=m)]
        send(2,'Manifest',data[0],4,step=0)
        seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
            p._pack('Manifest',data[0])).digest())
        actions=send(2,'PhaseSeal',seal,4,step=0)
        self.assertEqual([a.kind for a in actions],['stop_output'])
        actions=c.advance(Event('stopped'),c.host+1)
        self.assertEqual(c.state,'Closed')
        self.assertTrue(c.stop_proven)
        self.assertEqual([(a.peer,a.frame.control_step,a.frame.phase) for a in actions],
                         [(2,0,4),(3,0,4)])

    def test_cancelled_interval_does_not_advance_logical_time(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        self.inventory(2,1)
        self.c.advance(Event('stop'),self.c.host+1)
        actions=self.c.advance(Event('cancelled'),self.c.host+1)
        self.assertEqual(self.c.step,0)
        self.assertFalse(self.c.physics_in_flight)
        self.assertEqual([a.frame.kind if a.frame else None for a in actions],
                         [p.KINDS.index('ShutdownPermit')+1,
                          p.KINDS.index('ShutdownPermit')+1,2])
        self.assertEqual(self.c.state,'StopTick')
        self.assertFalse(self.c.stop_proven)

    def test_one_step_action_per_control_step(self):
        self.ready()
        self.send(2,'Torque',self.torque())
        seal,actions=self.inventory_with_actions(2,1)
        self.assertEqual(len([a for a in actions if a.kind=='step']),1)
        ready,ack=self.finish(2,1,seal)
        self.assertEqual([a.kind for a in ready+ack if a.kind=='step'],[])
        seal2=self.inventory(3,2)
        self.assertEqual([a.kind for a in self.finish(3,2,seal2)[0] if a.kind=='step'],[])
        # Only the completion event advances and introduces the next interval.
        actions=self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1)
        self.assertEqual([a.kind for a in actions].count('step'),0)
        self.assertEqual(self.c.step,1)

    # -- preserved protocol/evidence behavior -----------------------------

    def test_actions_are_detached_and_supplied_wire_is_authoritative(self):
        # Hello echo: mutating the returned payload cannot rewrite the retained
        # hello, and the second peer still validates against it.
        hello=self.send(2,'Hello',dict(self.c.hello,role=2),phase=0,now=1)[0]
        self.assertEqual(hello.frame.payload,self.c.hello)
        hello.frame.payload['runtime_id']=b'X'*16
        hello.frame.payload['physics_dt_us']=7
        self.assertEqual((self.c.hello['runtime_id'],self.c.hello['physics_dt_us']),
                         (b't'*16,1000))
        self.send(3,'Hello',dict(self.c.hello,role=3),phase=0,now=1)
        tick=self.c.advance(Event('feedback',feedback=feedback()),2)
        self.assertEqual([x.kind for x in tick],['send','send'])
        # Feedback and tick-permit payloads are freshly built, not shared.
        tick[0].frame.payload['q']=(9.,)
        tick[0].frame.payload['valid']=False
        tick[1].frame.payload['stage']=9
        self.assertEqual((self.c.feedback.q,self.c.feedback.base_velocity),
                         ((0.,),(0.,)*6))
        # A supplied packet is authoritative for the dedup key and payload; a
        # later mutation of the decoded frame cannot rewrite retained facts.
        frame=self.frame('Torque',self.torque())
        wire=p.encode(frame)
        decoded=p.decode(wire)
        self.assertEqual(self.c.advance(Event('frame',2,decoded,wire=wire),3),())
        self.assertEqual(self.c.torque['tau'],(4.,))
        decoded.payload['tau']=(8.,)
        decoded.payload['write_id']=99
        self.assertEqual((self.c.torque['tau'],self.c.torque['write_id']),((4.,),1))
        self.assertEqual(self.c.advance(Event('frame',2,p.decode(wire),wire=wire),3),())
        # Manifest and seal echoes stay detached from retained inventory/seal.
        m=state_identity(0)
        echo=self.send(2,'Manifest',dict(sender=2,message=m),1)[0]
        echo.frame.payload['message']['sequence']=777
        echo.frame.payload['sender']=3
        self.assertEqual(self.c.inventory[-1]['sequence'],m['sequence'])
        _,seal=seal_for(2,(m,))
        a=self.send(2,'PhaseSeal',seal,1)
        self.assertEqual([x.kind for x in a],['send','step'])
        self.assertEqual(a[0].frame.payload,seal)
        a[0].frame.payload['manifest_count']=0
        self.assertEqual(self.c.seal['manifest_count'],1)
        a=self.send(3,'InventoryReady',dict(consumer=3,seal=seal),1)
        self.assertEqual([(x.peer,x.frame.kind) for x in a],[(2,14),(3,15)])
        a[0].frame.payload['seal']['manifest_digest']=bytes(32)
        self.assertEqual(self.c.seal['manifest_digest'],seal['manifest_digest'])
        self.assertEqual(self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=seal),1),())
        self.assertEqual(self.c.state,'ApplicationPhase',self.c.failure)
        # A frame whose identity disagrees with its supplied packet fails closed.
        other=p.encode(self.frame('Torque',dict(self.torque(),write_id=99),2))
        self.c.advance(Event('frame',2,self.frame('Hello',dict(self.c.hello,role=2),2),
                             wire=other),4)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('wire identity',self.c.failure)

    def test_nonfinite_torque_frame_fails_before_any_engine_action(self):
        self.ready()
        frame=self.frame('Torque',dict(self.torque(),tau=(float('nan'),)))
        actions=self.c.advance(Event('frame',2,frame),3)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('nonfinite',self.c.failure)
        self.assertNotIn('step',[a.kind for a in actions])

    def test_supplied_wire_is_validated_before_state_affecting_early_returns(self):
        # A malformed or identity-mismatched packet must fail closed even when
        # the frame would otherwise take a cancellation, duplicate or shutdown
        # early return; the old ordering decided those from the frame field.
        # 1. Cancellation: a valid-looking ReleaseCancel frame with junk wire
        #    must not cancel the arm and report success.
        self.setUp()
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        self.assertEqual(self.c.release_state,'Armed')
        actions=self.c.advance(Event('frame',3,
            self.frame('ReleaseCancel',dict(role=3),2),wire=b'x'),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('truncated',self.c.failure)
        self.assertTrue(actions)
        # 2. Cancellation: a well-formed but identity-mismatched packet must
        #    not reach the cancellation branch either.
        self.setUp()
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        torque_wire=p.encode(self.frame('Torque',self.torque(),2))
        self.c.advance(Event('frame',3,
            self.frame('ReleaseCancel',dict(role=3),2),wire=torque_wire),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('frame wire identity',self.c.failure)
        # 3. Duplicate: identical resend is still suppressed, but a frame that
        #    disagrees with the already-seen wire fails before dedup returns.
        self.setUp();self.ready()
        good=self.frame('Torque',self.torque())
        wire=p.encode(good)
        self.assertEqual(self.c.advance(Event('frame',2,p.decode(wire),wire=wire),
                                        self.c.host+1),())
        self.assertEqual(self.c.advance(Event('frame',2,p.decode(wire),wire=wire),
                                        self.c.host+1),())
        self.c.advance(Event('frame',2,
            self.frame('Hello',dict(self.c.hello,role=2)),wire=wire),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('frame wire identity',self.c.failure)
        # 4. Shutdown-pending early return: junk and mismatched packets must
        #    fail instead of being silently discarded as cancelled output.
        self.setUp();self.ready()
        self.c.advance(Event('stop'),3)
        self.assertTrue(self.c.shutdown_pending)
        actions=self.c.advance(Event('frame',2,
            self.frame('Manifest',dict(sender=2,message=state_identity(0)),1),
            wire=b'x'),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('truncated',self.c.failure)
        self.assertTrue(actions)
        self.setUp();self.ready()
        self.c.advance(Event('stop'),3)
        tick_wire=p.encode(self.frame('TickPermit',dict(role=2,stage=1),1))
        self.c.advance(Event('frame',2,
            self.frame('Manifest',dict(sender=2,message=state_identity(0)),1),
            wire=tick_wire),self.c.host+1)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('frame wire identity',self.c.failure)

    def test_no_progress_during_inference_and_timeout_not_extended_by_duplicate(self):
        self.ready(); self.send(2,'Torque',self.torque())
        seal,actions=self.inventory_with_actions(2,1); self.assertEqual([a.kind for a in actions],['send','step'])
        self.finish(2,1,seal)
        self.assertEqual(self.c.advance(Event('poll'),90),())
        self.assertEqual(self.c.step,0)
        self.assertEqual(self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=seal),1,99),())
        self.c.advance(Event('poll'),103)
        self.assertEqual(self.c.state,'Failed')

    def test_ack_before_installed_is_fatal(self):
        self.ready(); seal=self.inventory(2,1)
        self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=seal))
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
        seal,actions=self.inventory_with_actions(2,1,(m,))
        # The safety proof is now checked before the engine action exists.
        self.assertEqual(self.c.state,'Failed')
        self.assertNotIn('step',[a.kind for a in actions])

    def test_empty_runtime_inventory_cannot_replace_tick_state(self):
        self.ready();self.send(2,'Torque',self.torque())
        seal,actions=self.inventory_with_actions(2,1,())
        self.assertEqual(self.c.state,'Failed')
        self.assertNotIn('step',[a.kind for a in actions])

    def test_returned_actions_cannot_mutate_retained_facts(self):
        hello=dict(self.c.hello,role=2)
        a=self.send(2,'Hello',hello,phase=0,now=1)
        a[0].frame.payload['config_digest']=b'x'*32
        self.assertEqual(self.c.hello['config_digest'],b'c'*32)
        hello['engine_digest']=b'x'*32
        self.send(3,'Hello',dict(self.c.hello,role=3),phase=0,now=1)
        self.c.advance(Event('feedback',feedback=feedback()),2)
        self.send(2,'Torque',self.torque())
        m=state_identity(0)
        original=dict(m)
        a=self.send(2,'Manifest',dict(sender=2,message=m))
        a[0].frame.payload['message']['sequence']=999
        m['content_digest']=b'x'*32
        self.assertEqual(self.c.inventory[0],original)
        _,seal=seal_for(2,(original,))
        a=self.send(2,'PhaseSeal',seal)
        a[0].frame.payload['manifest_count']=0
        self.assertEqual(self.c.seal,seal)
        a=self.send(3,'InventoryReady',dict(consumer=3,seal=seal))
        a[0].frame.payload['seal']['manifest_digest']=b'x'*32
        self.assertEqual(self.c.seal,seal)
        self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=seal))
        self.assertEqual(self.c.state,'ApplicationPhase')
        self.assertEqual(self.c.phase,2)

    def test_invented_tracking_command_never_reaches_physics(self):
        self.ready()
        torque=self.torque();torque['command']=self.command()
        self.send(2,'Torque',torque)
        seal,actions=self.inventory_with_actions(2,1,(state_identity(0,torque['command']),))
        self.assertEqual(self.c.state,'Failed')
        self.assertNotIn('step',[a.kind for a in actions])

    def test_changed_ingressed_identity_and_tick_metadata_are_rejected(self):
        for change in (dict(content_digest=b'x'*32),dict(source_state_seq=2),
                       dict(session_id=b'x'*16),dict(epoch=8),dict(sequence=10),dict(kind=1)):
            with self.subTest(change=change):
                self.setUp();self.ready();command=self.command()
                self.cycle(messages=(command,))
                changed=dict(command,**change)
                torque=self.torque();torque['command']=changed
                self.send(2,'Torque',torque)
                if self.c.state!='Failed':
                    self.inventory(2,1,(state_identity(1,changed),))
                self.assertEqual(self.c.state,'Failed')
        for change in (dict(source_state_seq=999),dict(session_id=b'x'*16),
                       dict(producer_id=b'x'*16),dict(epoch=8)):
            with self.subTest(state_change=change):
                self.setUp();self.ready();command=self.command();self.cycle(messages=(command,))
                torque=self.torque();torque['command']=command
                self.send(2,'Torque',torque)
                self.inventory(2,1,(state_identity(1,command,**change),))
                self.assertEqual(self.c.state,'Failed')

    def test_legitimate_held_target_survives_phase_cache_rotation_then_clear(self):
        self.ready();command=self.command();self.cycle(messages=(command,))
        for _ in range(40):self.cycle(command)
        # No-active metadata is valid in Damping and ZeroTorque even if the
        # state retains the historical last-applied sequence.
        for state,output in ((2,1),(1,0),(4,1),(4,0)):
            torque=self.torque();self.send(2,'Torque',torque)
            seal,actions=self.inventory_with_actions(2,1,(state_identity(self.c.step,execution_state=state,
                output_action=output,source_state_seq=9),))
            self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
            self.finish(2,1,seal)
            seal=self.inventory(3,2);self.finish(3,2,seal)
            self.c.advance(Event('feedback',feedback=feedback((self.c.step+1)*2)),self.c.host+1)

        torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        seal,actions=self.inventory_with_actions(2,1,(state_identity(self.c.step,command),))
        self.assertEqual(self.c.state,'Failed','cleared target must need fresh ingress')
        self.assertNotIn('step',[a.kind for a in actions])

    def test_no_active_representation_is_exact(self):
        for change in (dict(content_digest=b'x'*32),dict(session_id=b'u'*16),dict(kind=1)):
            with self.subTest(change=change):
                self.setUp();self.ready();torque=self.torque()
                torque['command'].update(change)
                self.send(2,'Torque',torque)
                self.inventory(2,1)
                self.assertEqual(self.c.state,'Failed')

    def test_nested_first_command_is_auxiliary_not_a_dds_inventory_entry(self):
        self.assertIn('FirstCommandProof',p.KINDS,'typed nested-command proof missing')
        self.ready();command=self.command()
        request=identity(1,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                         proposed_epoch=7,operation=3,content_digest=b'r'*32)
        self.cycle(messages=(request,),proofs=(dict(request=request,command=command),))
        self.cycle(command)

    def test_actual_target_must_match_nested_command_digest(self):
        self.ready();command=self.command()
        request=identity(1,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                         proposed_epoch=7,operation=3,content_digest=b'r'*32)
        self.cycle(messages=(request,),proofs=(dict(request=request,
            command=dict(command,content_digest=b'x'*32)),))
        torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        self.inventory(2,1,(state_identity(self.c.step,command),))
        self.assertEqual(self.c.state,'Failed')

    def test_first_command_proof_conflicts_and_unknown_source_fail(self):
        self.assertIn('FirstCommandProof',p.KINDS,'typed nested-command proof missing')
        for change in ('outer','session','epoch','source','unsealed'):
            with self.subTest(change=change):
                self.setUp();self.ready();self.send(2,'Torque',self.torque())
                seal=self.inventory(2,1);self.finish(2,1,seal)
                command=self.command()
                request=identity(1,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                                 proposed_epoch=7,operation=3,content_digest=b'r'*32)
                declared=dict(request)
                if change=='outer':request=dict(request,content_digest=b'x'*32)
                if change=='session':command=dict(command,session_id=b'x'*16)
                if change=='epoch':command=dict(command,epoch=8)
                if change=='source':command=dict(command,source_state_seq=999)
                self.send(3,'FirstCommandProof',dict(request=request,command=command),phase=2)
                if self.c.state!='Failed':
                    seal=self.inventory(3,2,() if change=='unsealed' else (declared,))
                    self.finish(3,2,seal)
                self.assertEqual(self.c.state,'Failed')

    def test_shutdown_is_distinct_frozen_phase_and_needs_engine_completion(self):
        self.ready()
        actions=self.c.advance(Event('stop'),4)
        self.assertEqual(self.c.state,'StopTick',self.c.failure)
        self.assertEqual(self.c.phase,4)
        self.assertEqual(self.c.step,0)
        self.assertEqual([p.KINDS[a.frame.kind-1] for a in actions],
                         ['ShutdownPermit','ShutdownPermit','Feedback'])
        torque=self.torque(0.);torque['state_seq']=2
        self.send(2,'Torque',torque,4)
        state=state_identity(0,sequence=2,execution_state=1,output_action=0)
        data,seal=seal_for(2,(state,))
        self.send(2,'Manifest',data[0],4)
        actions=self.send(2,'PhaseSeal',seal,4)
        self.assertEqual([a.kind for a in actions],['stop_output'])
        self.assertEqual(self.c.state,'StopEngine')
        self.assertNotEqual(self.c.state,'Closed')
        actions=self.c.advance(Event('stopped'),5)
        self.assertEqual(self.c.state,'Closed')
        self.assertEqual([(a.peer,p.KINDS[a.frame.kind-1]) for a in actions],
                         [(2,'ShutdownCommitted'),(3,'ShutdownCommitted')])

    def test_shutdown_fault_never_bypasses_guard(self):
        self.ready();self.c.advance(Event('stop'),4)
        self.send(2,'Torque',self.torque(),4)
        state=state_identity(0,execution_state=4,output_action=1)
        data,seal=seal_for(2,(state,))
        self.send(2,'Manifest',data[0],4)
        self.send(2,'PhaseSeal',seal,4)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('ZeroTorque',self.c.failure)

    # -- release authorization under overlap ------------------------------

    def armed_phase(self):
        """Land an ingress request and start a Tracking interval armed with an
        exact application proof, leaving the step in flight (overlap)."""
        self.ready();command=self.command();request=self.request()
        self.cycle(messages=(request,),proofs=(dict(request=request,command=command),))
        step=self.c.step
        torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        state=state_identity(step,command)
        result=identity(2,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                        epoch=7,stamp_us=state['stamp_us'],execution_state=3,output_action=2,
                        content_digest=b'z'*32)
        seal,actions=self.inventory_with_actions(2,1,(state,result))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.finish(2,1,seal)
        return dict(tracking_request=request,first_command=command,
                    applied_result=result,tracking_state=state)

    def test_release_armed_during_overlap_applies_to_next_interval_exactly_once(self):
        arm=self.armed_phase()
        self.assertTrue(self.c.physics_in_flight)
        self.send(3,'ReleaseArm',arm,2)
        self.assertEqual(self.c.release_state,'Armed',self.c.failure)
        sealed_step=self.c.step
        # The already-issued interval cannot be rewritten by a later proof.
        seal,seal_actions=self.inventory_with_actions(3,2)
        self.assertEqual([a.kind for a in seal_actions],['send'])
        self.finish(3,2,seal)
        completion=self.c.advance(Event('feedback',feedback=feedback((sealed_step+1)*2)),
                                  self.c.host+1)
        self.assertEqual([a.kind for a in completion],['send','send','send'])
        self.assertFalse(any(a.kind=='step' and a.release for a in completion))
        self.assertEqual(self.c.release_state,'Armed')
        self.assertIsNotNone(self.c.arm)
        # The next interval carries the single release action.
        command=arm['first_command'];torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        seal,actions=self.inventory_with_actions(2,1,(state_identity(self.c.step,command),))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.assertTrue(actions[1].release)
        self.assertEqual(self.c.release_state,'Releasing')
        self.finish(2,1,seal)
        seal=self.inventory(3,2);self.finish(3,2,seal)
        completion=self.c.advance(Event('feedback',feedback=feedback((self.c.step+1)*2),
                                       released=True),self.c.host+1)
        self.assertEqual([a.kind for a in completion],['send','send','send'])
        self.assertEqual(self.c.release_state,'Released')
        self.assertIsNone(self.c.arm)
        self.assertEqual([e[0] for e in self.c.release_events],['Armed','Released'])
        self.assertEqual(self.c.release_events[0][1],sealed_step)
        self.assertEqual(self.c.release_events[1][1],sealed_step+2)

    def test_stale_or_changed_arm_cannot_release(self):
        for field,change in (('applied_result',dict(epoch=6)),
                             ('first_command',dict(content_digest=b'x'*32)),
                             ('tracking_state',dict(sequence=1)),
                             ('tracking_request',dict(request_id=4))):
            with self.subTest(field=field):
                self.setUp();arm=self.armed_phase()
                arm[field]=dict(arm[field],**change)
                self.send(3,'ReleaseArm',arm,2)
                self.assertEqual(self.c.state,'Failed')
                self.assertNotEqual(self.c.release_state,'Released')

    def test_app_cancel_after_arm_prevents_deferred_release(self):
        self.assertIn('ReleaseCancel',p.KINDS)
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        self.send(3,'ReleaseCancel',dict(role=3),2)
        self.assertEqual(self.c.release_state,'Fixed')
        seal=self.inventory(3,2);self.finish(3,2,seal)
        completion=self.c.advance(Event('feedback',feedback=feedback(4)),self.c.host+1)
        self.assertFalse(any(a.kind=='step' for a in completion))
        self.assertFalse(any(a.release for a in completion if a.kind=='step'))
        self.assertIsNone(self.c.arm)

    def test_stop_during_overlap_cancels_armed_release(self):
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        actions=self.c.advance(Event('stop'),self.c.host+1)
        self.assertEqual(actions,())
        self.assertEqual(self.c.release_state,'Fixed')
        self.assertIsNone(self.c.arm)
        completion=self.c.advance(Event('feedback',feedback=feedback(4)),self.c.host+1)
        self.assertNotIn('step',[a.kind for a in completion])
        self.assertFalse(any(a.release for a in completion))
        self.assertEqual(self.c.state,'StopTick')

    def test_stop_inventory_cancels_deferred_arm(self):
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        stop=identity(1,request_id=6,operation=1,session_id=b'u'*16,content_digest=b'x'*32)
        seal,seal_actions=self.inventory_with_actions(3,2,(stop,))
        self.assertEqual([a.kind for a in seal_actions],['send'])
        self.finish(3,2,seal)
        self.assertEqual(self.c.release_state,'Fixed')
        self.assertIsNone(self.c.arm)
        completion=self.c.advance(Event('feedback',feedback=feedback(4)),self.c.host+1)
        self.assertFalse(any(a.kind=='step' for a in completion))
        self.assertEqual(self.c.state,'RuntimeTick')

    def test_later_independent_authorization_can_rearm_after_cancel(self):
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        self.send(3,'ReleaseCancel',dict(role=3),2)
        seal=self.inventory(3,2);self.finish(3,2,seal)
        self.c.advance(Event('feedback',feedback=feedback(4)),self.c.host+1)
        # A fresh command/request cycle at the current step, then a new arm.
        cmd=self.command(epoch=9,sequence=11,source_state_seq=3,stamp_us=4001)
        req=self.request(request_id=7,proposed_epoch=9,content_digest=b'n'*32)
        self.cycle(messages=(req,),proofs=(dict(request=req,command=cmd),))
        step=self.c.step
        torque=self.torque();torque['command']=cmd
        self.send(2,'Torque',torque)
        state=state_identity(step,cmd)
        result=identity(2,request_id=7,session_id=b'u'*16,producer_id=b'p'*16,
                        epoch=9,stamp_us=state['stamp_us'],execution_state=3,output_action=2,
                        content_digest=b'w'*32)
        seal,actions=self.inventory_with_actions(2,1,(state,result))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.finish(2,1,seal)
        self.send(3,'ReleaseArm',dict(tracking_request=req,first_command=cmd,
                                    applied_result=result,tracking_state=state),2)
        self.assertEqual(self.c.release_state,'Armed',self.c.failure)

    def test_fault_leave_needs_actual_output_but_no_fabricated_stop(self):
        self.ready();self.send(2,'Torque',self.torque())
        state=state_identity(0,execution_state=4,output_action=1)
        seal,actions=self.inventory_with_actions(2,1,(state,))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.finish(2,1,seal)
        a=self.send(3,'LeaveReady',dict(stop_request=identity(1),stop_result=identity(2),stopped_state=state),2)
        self.assertTrue(self.c.runtime_only,self.c.failure)
        self.assertEqual(a[0].frame.payload['proof']['stop_request']['content_digest'],bytes(32))
        # The interval was already committed; leave must not author another.
        self.assertNotIn('step',[x.kind for x in a])
        self.assertEqual(self.c.state,'PhysicsAdvance')

    def test_leave_joins_overlapped_interval_without_second_step(self):
        self.ready();self.send(2,'Torque',self.torque())
        state=state_identity(0,execution_state=4,output_action=1)
        seal,actions=self.inventory_with_actions(2,1,(state,))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.finish(2,1,seal)
        a=self.send(3,'LeaveReady',dict(stop_request=identity(1),stop_result=identity(2),
                                        stopped_state=state),2)
        self.assertTrue(self.c.runtime_only,self.c.failure)
        self.assertNotIn('step',[x.kind for x in a])
        self.assertEqual(self.c.state,'PhysicsAdvance')
        # The committed interval closes once; no second physics action exists.
        done=self.c.advance(Event('feedback',feedback=feedback(2)),self.c.host+1)
        self.assertEqual([x.kind for x in done],['send','send','send'])
        self.assertEqual(self.c.step,1)
        self.send(2,'Torque',dict(self.torque(0.),state_seq=3))
        runtime_state=state_identity(1,sequence=3,execution_state=1,output_action=0)
        seal,acts=self.inventory_with_actions(2,1,(runtime_state,))
        self.assertEqual([a.kind for a in acts],['step'],self.c.failure)

    def test_leave_removes_app_without_fabricating_receipt(self):
        self.ready()
        request=identity(1,request_id=8,session_id=b'u'*16,operation=1,
                         content_digest=b'r'*32)
        self.cycle(messages=(request,))
        self.send(2,'Torque',self.torque())
        state=state_identity(1,session_id=b'u'*16)
        result=identity(2,request_id=8,session_id=b'u'*16,stamp_us=2001,
                        execution_state=2,output_action=1,content_digest=b'z'*32)
        seal,actions=self.inventory_with_actions(2,1,(state,result))
        self.assertEqual([a.kind for a in actions],['send','step'],self.c.failure)
        self.finish(2,1,seal)
        a=self.send(3,'LeaveReady',dict(stop_request=request,stop_result=result,
                                     stopped_state=state),2)
        self.assertTrue(self.c.runtime_only,self.c.failure)
        self.assertEqual([p.KINDS[x.frame.kind-1] for x in a if x.kind=='send'],
                         ['LeaveCommitted','RuntimeOnlyPermit'])
        self.assertFalse(a[0].frame.payload['session_closed'])
        self.assertEqual(self.c.advance(Event('disconnect',3),5),())
        self.assertNotEqual(self.c.state,'Failed')

    def test_disconnect_and_stop_fail_closed_no_release(self):
        self.ready()
        a=self.c.advance(Event('disconnect',2),4)
        self.assertEqual(self.c.state,'Failed')
        self.assertNotIn('release',[x.kind for x in a])


if __name__=='__main__': unittest.main()
