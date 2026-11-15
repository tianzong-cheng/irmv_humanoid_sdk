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
                m=state_identity(self.c.step)
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
        m = identity()
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
        seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
            p._pack('Manifest',dict(sender=2,message=original))).digest())
        a=self.send(2,'PhaseSeal',seal)
        a[0].frame.payload['manifest_count']=0
        self.assertEqual(self.c.seal,seal)
        a=self.send(3,'InventoryReady',dict(consumer=3,seal=seal))
        a[0].frame.payload['seal']['manifest_digest']=b'x'*32
        self.assertEqual(self.c.seal,seal)
        self.send(3,'PhaseAck',dict(consumer=3,disposition=2,seal=seal))
        self.assertEqual(self.c.state,'ApplicationPhase')
        self.assertEqual(self.c.phase,2)

    def command(self, **values):
        m=identity(5,session_id=b'u'*16,producer_id=b'p'*16,epoch=7,
                   sequence=9,source_state_seq=1,stamp_us=1,content_digest=b'd'*32)
        m.update(values)
        return m

    def cycle(self, command=None, messages=(), proofs=()):
        torque=self.torque()
        if command:torque['command']=command
        self.send(2,'Torque',torque)
        seal=self.inventory(2,1,(state_identity(self.c.step,command),))
        self.finish(2,1,seal)
        self.assertEqual(self.c.state,'ApplicationPhase',self.c.failure)
        for proof in proofs:self.send(3,'FirstCommandProof',proof,phase=2)
        seal=self.inventory(3,2,messages)
        actions=self.finish(3,2,seal)
        self.assertEqual([a.kind for a in actions],['step'],self.c.failure)
        self.c.advance(Event('feedback',feedback=feedback((self.c.step+1)*2)),self.c.host+1)
        self.assertEqual(self.c.state,'RuntimeTick',self.c.failure)

    def test_invented_tracking_command_never_reaches_physics(self):
        self.ready()
        torque=self.torque();torque['command']=self.command()
        self.send(2,'Torque',torque)
        seal=self.inventory(2,1,(state_identity(0,torque['command']),))
        self.finish(2,1,seal)
        self.assertEqual(self.c.state,'Failed')

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
                    seal=self.inventory(2,1,(state_identity(1,changed),))
                    self.finish(2,1,seal)
                self.assertEqual(self.c.state,'Failed')
        for change in (dict(source_state_seq=999),dict(session_id=b'x'*16),
                       dict(producer_id=b'x'*16),dict(epoch=8)):
            with self.subTest(state_change=change):
                self.setUp();self.ready();command=self.command();self.cycle(messages=(command,))
                torque=self.torque();torque['command']=command
                self.send(2,'Torque',torque)
                seal=self.inventory(2,1,(state_identity(1,command,**change),))
                self.finish(2,1,seal)
                self.assertEqual(self.c.state,'Failed')

    def test_legitimate_held_target_survives_phase_cache_rotation_then_clear(self):
        self.ready();command=self.command();self.cycle(messages=(command,))
        for _ in range(40):self.cycle(command)
        # No-active metadata is valid in Damping and ZeroTorque even if the
        # state retains the historical last-applied sequence.
        for state,output in ((2,1),(1,0),(4,1),(4,0)):
            torque=self.torque();self.send(2,'Torque',torque)
            seal=self.inventory(2,1,(state_identity(self.c.step,execution_state=state,
                output_action=output,source_state_seq=9),))
            self.finish(2,1,seal)
            self.assertEqual(self.c.state,'ApplicationPhase',self.c.failure)
            seal=self.inventory(3,2);self.finish(3,2,seal)
            self.c.advance(Event('feedback',feedback=feedback((self.c.step+1)*2)),self.c.host+1)

        torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        seal=self.inventory(2,1,(state_identity(self.c.step,command),))
        self.finish(2,1,seal)
        self.assertEqual(self.c.state,'Failed','cleared target must need fresh ingress')

    def test_no_active_representation_is_exact(self):
        for change in (dict(content_digest=b'x'*32),dict(session_id=b'u'*16),dict(kind=1)):
            with self.subTest(change=change):
                self.setUp();self.ready();torque=self.torque()
                torque['command'].update(change)
                self.send(2,'Torque',torque)
                seal=self.inventory(2,1);self.finish(2,1,seal)
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
        seal=self.inventory(2,1,(state_identity(self.c.step,command),))
        self.finish(2,1,seal)
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
        self.send(2,'Manifest',dict(sender=2,message=state),4)
        seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
            p._pack('Manifest',dict(sender=2,message=state))).digest())
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
        self.send(2,'Manifest',dict(sender=2,message=state),4)
        seal=dict(sender=2,manifest_count=1,manifest_digest=hashlib.sha256(
            p._pack('Manifest',dict(sender=2,message=state))).digest())
        self.send(2,'PhaseSeal',seal,4)
        self.assertEqual(self.c.state,'Failed')
        self.assertIn('ZeroTorque',self.c.failure)

    def test_release_pairs_exact_policy_proof_and_is_one_worker_action(self):
        self.ready();command=self.command()
        request=identity(1,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                         proposed_epoch=7,operation=3,content_digest=b'r'*32)
        self.cycle(messages=(request,),proofs=(dict(request=request,command=command),))
        torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        state=state_identity(1,command)
        result=identity(2,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                        epoch=7,stamp_us=2001,execution_state=3,output_action=2,
                        content_digest=b'z'*32)
        seal=self.inventory(2,1,(state,result));self.finish(2,1,seal)
        arm=dict(tracking_request=request,first_command=command,
                 applied_result=result,tracking_state=state)
        self.send(3,'ReleaseArm',arm,2)
        self.assertEqual(self.c.release_state,'Armed',self.c.failure)
        seal=self.inventory(3,2);actions=self.finish(3,2,seal)
        self.assertTrue(actions[0].release)
        self.assertEqual([a.kind for a in actions],['step'])

    def armed_phase(self):
        self.ready();command=self.command()
        request=identity(1,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                         proposed_epoch=7,operation=3,content_digest=b'r'*32)
        self.cycle(messages=(request,),proofs=(dict(request=request,command=command),))
        torque=self.torque();torque['command']=command
        self.send(2,'Torque',torque)
        state=state_identity(1,command)
        result=identity(2,request_id=5,session_id=b'u'*16,producer_id=b'p'*16,
                        epoch=7,stamp_us=2001,execution_state=3,output_action=2,
                        content_digest=b'z'*32)
        seal=self.inventory(2,1,(state,result));self.finish(2,1,seal)
        return dict(tracking_request=request,first_command=command,applied_result=result,tracking_state=state)

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

    def test_stop_after_arm_cancels_before_worker_step(self):
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        actions=self.c.advance(Event('stop'),5)
        self.assertEqual(self.c.release_state,'Fixed')
        self.assertIsNone(self.c.arm)
        self.assertNotIn('step',[a.kind for a in actions])
        self.assertEqual(self.c.state,'StopTick')

    def test_stop_inventory_cancels_arm_and_later_authorization_can_rearm(self):
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        stop=identity(1,request_id=6,operation=1,session_id=b'u'*16,content_digest=b'x'*32)
        seal=self.inventory(3,2,(stop,));a=self.finish(3,2,seal)
        self.assertFalse(a[0].release)
        self.assertEqual(self.c.release_state,'Fixed')
        # A later independently proven first policy frame can arm again; no
        # constraint call occurred for the cancelled authorization.
        self.c.advance(Event('feedback',feedback=feedback(4)),6)
        cmd=self.command(epoch=9,sequence=11,source_state_seq=3,stamp_us=4001)
        req=identity(1,request_id=7,session_id=b'u'*16,producer_id=b'p'*16,
                     proposed_epoch=9,operation=3,content_digest=b'n'*32)
        self.cycle(messages=(req,),proofs=(dict(request=req,command=cmd),))
        torque=self.torque();torque['command']=cmd
        self.send(2,'Torque',torque)
        state=state_identity(3,cmd)
        result=identity(2,request_id=7,session_id=b'u'*16,producer_id=b'p'*16,
                        epoch=9,stamp_us=6001,execution_state=3,output_action=2,content_digest=b'w'*32)
        seal=self.inventory(2,1,(state,result));self.finish(2,1,seal)
        self.send(3,'ReleaseArm',dict(tracking_request=req,first_command=cmd,
                                    applied_result=result,tracking_state=state),2)
        self.assertEqual(self.c.release_state,'Armed',self.c.failure)

    def test_app_cancel_between_arm_and_advance_prevents_release(self):
        self.assertIn('ReleaseCancel',p.KINDS)
        arm=self.armed_phase();self.send(3,'ReleaseArm',arm,2)
        self.send(3,'ReleaseCancel',dict(role=3),2)
        seal=self.inventory(3,2);a=self.finish(3,2,seal)
        self.assertFalse(a[0].release)
        self.assertEqual(self.c.release_state,'Fixed')

    def test_fault_leave_needs_actual_output_but_no_fabricated_stop(self):
        self.ready();self.send(2,'Torque',self.torque())
        state=state_identity(0,execution_state=4,output_action=1)
        seal=self.inventory(2,1,(state,));self.finish(2,1,seal)
        a=self.send(3,'LeaveReady',dict(stop_request=identity(1),stop_result=identity(2),stopped_state=state),2)
        self.assertTrue(self.c.runtime_only,self.c.failure)
        self.assertEqual(a[0].frame.payload['proof']['stop_request']['content_digest'],bytes(32))

    def test_leave_removes_app_without_fabricating_receipt(self):
        self.ready()
        request=identity(1,request_id=8,session_id=b'u'*16,operation=1,
                         content_digest=b'r'*32)
        self.cycle(messages=(request,))
        self.send(2,'Torque',self.torque())
        state=state_identity(1,session_id=b'u'*16)
        result=identity(2,request_id=8,session_id=b'u'*16,stamp_us=2001,
                        execution_state=2,output_action=1,content_digest=b'z'*32)
        seal=self.inventory(2,1,(state,result));self.finish(2,1,seal)
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
