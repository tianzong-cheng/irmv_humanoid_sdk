"""Acceptance is fail-closed; fixtures never attest trained-policy reliability."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
try:
    from simulation.evaluate import evaluate, yaw_frame_velocity, scenario_intent
except ImportError:
    raise AssertionError('Task11 streaming acceptance evaluator missing')


class AcceptanceTest(unittest.TestCase):
    def test_absent_evidence_is_failure(self):
        report=evaluate({'schema':'irmv.policy_acceptance','version':1})
        self.assertEqual(report['status'],'fail')
        self.assertTrue(report['reasons'])

    def test_missing_or_nonfinite_bounds_cannot_authorize_motion(self):
        for bounds in ([],[[0,1]],[[0,1],[0,0],[1,-1]],[[0,1],[0,float('nan')],[-1,1]]):
            with self.subTest(bounds=bounds):
                with self.assertRaises(ValueError):scenario_intent('forward',bounds)

    def test_missing_fault_accounting_cannot_be_hidden_by_complete_samples(self):
        from simulation import evaluate as module
        valid=dict(complete=True,truncated=False,corrupt=False,gaps=[],missing_samples=0,
                   faults=dict(complete=True,truncated=False,losses=[],missing_occurrences=0))
        module.require_complete_sdk_summary(valid,'runtime')
        for faults in (None,dict(complete=False),dict(complete=True,truncated=True),
                       dict(complete=True,losses=[{'dropped':1}]),dict(complete=True,missing_occurrences=1)):
            with self.subTest(faults=faults):
                changed=dict(valid,faults=faults)
                with self.assertRaisesRegex(ValueError,'fault'):
                    module.require_complete_sdk_summary(changed,'runtime')

    def test_gpu_skip_and_stale_manifest_files_fail(self):
        from dataclasses import asdict
        from simulation_coordinator_test import bootstrap
        from simulation.evaluate import sha256
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);b=asdict(bootstrap())
            b={k:v.hex() if isinstance(v,bytes) else v for k,v in b.items()}
            files={}
            for key,doc in [('bootstrap',b),('config',{}),('evidence',{})]:
                path=root/(key+'.json');path.write_text(json.dumps(doc))
                files[key]=dict(path=str(path),sha256=sha256(path))
            manifest=dict(schema='irmv.policy_acceptance',version=1,scope='trained_policy',files=files,
                          gpu=dict(executed=False,skipped=True))
            report=evaluate(manifest)
            self.assertEqual(report['status'],'fail');self.assertIn('GPU missing/skipped',report['reasons'][0])
            (root/'config.json').write_text('{"changed":true}')
            self.assertIn('file digest mismatch',evaluate(manifest)['reasons'][0])

    def test_yaw_frame_and_world_angular_velocity(self):
        # At yaw pi/2, world +Y is robot forward; local Z needs full rotation.
        self.assertAlmostEqual(yaw_frame_velocity([math.sqrt(.5),0,0,math.sqrt(.5)],
                                                 [0,1,0,0,0,1])[0],1)
        # Roll pi/2: local +Y is world +Z.
        self.assertAlmostEqual(yaw_frame_velocity([math.sqrt(.5),math.sqrt(.5),0,0],
                                                 [0,0,0,0,1,0])[2],1)

    def test_source_bounds_do_not_turn_zero_into_sideways_success(self):
        bounds=[[0,1.2],[0,0],[-1,1]]
        self.assertIsNone(scenario_intent('lateral',bounds))
        self.assertEqual(scenario_intent('forward',bounds),[1,0,0])
        self.assertEqual(scenario_intent('yaw',bounds),[0,0,1])
        self.assertEqual(scenario_intent('standing',bounds),[0,0,0])




# Adversarial complete lifecycle uses the real recording/coordinator path,
# supplied clocks and captured actions, no sockets or optional GPU packages.

class EvidenceTest(unittest.TestCase):
    def setUp(self):
        import dataclasses
        from simulation import protocol as p
        from simulation.coordinator import Event
        from simulation.main import Runner
        from unittest.mock import patch
        # Explicit supplied host time keeps this core evidence fixture independent
        # of scheduler pauses; no UDS or watchdog wall-clock test is hidden here.
        clock=patch('simulation.main.host_us',return_value=1000)
        clock.start();self.addCleanup(clock.stop)
        from simulation_coordinator_test import CoordinatorTest, bootstrap, feedback, state_identity
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'simulation.jsonl'
        self.b=bootstrap()
        r=Runner(self.b,Path(self.temp.name)/'bridge.sock')
        self.addCleanup(r.selector.close)
        rows=[];header=r.record.header
        class Capture:
            def offer(inner,row):
                rows.append(dict(row,record_type='event',sequence=len(rows)+1))
        r.record=Capture();last=[]
        r._actions=lambda actions:last.extend(actions)
        # Feed Runner its real events, but never execute socket/engine actions.
        class Proxy:
            def __getattr__(inner,key):return getattr(r.coordinator,key)
            def advance(inner,event,host):
                last.clear();r._advance(event);return tuple(last)
        h=CoordinatorTest();h.c=Proxy()
        h.c.advance(Event('start'),0)
        arm=h.armed_phase();h.send(3,'ReleaseArm',arm,2)
        seal=h.inventory(3,2);h.finish(3,2,seal)
        def measured(step):
            return dataclasses.replace(feedback(step),base_position=(0.,0.,.8),base_quaternion=(1.,0.,0.,0.))
        # Bootstrap was captured by ready() with legacy empty quaternion. Supply
        # explicit measured quaternion in this deterministic adapter's snapshot.
        for row in rows:
            if row.get('feedback'):
                row['feedback']['base_quaternion']=[1.,0.,0.,0.]
                row['feedback']['base_position']=[0.,0.,.8]
        h.c.advance(Event('feedback',feedback=measured(4),released=True),4)
        command=arm['first_command']
        for _ in range(2):
            torque=h.torque();torque['command']=command
            h.send(2,'Torque',torque)
            seal=h.inventory(2,1,(state_identity(h.c.step,command),));h.finish(2,1,seal)
            seal=h.inventory(3,2);h.finish(3,2,seal)
            h.c.advance(Event('feedback',feedback=measured((h.c.step+1)*2)),5)
        h.c.advance(Event('stop'),6)
        torque=h.torque();torque['tau']=(0.,)
        h.send(2,'Torque',torque,4)
        state=state_identity(h.c.step,execution_state=1,output_action=0)
        h.send(2,'Manifest',dict(sender=2,message=state),4)
        seal=dict(sender=2,manifest_count=1,manifest_digest=__import__('hashlib').sha256(
            p._pack('Manifest',dict(sender=2,message=state))).digest())
        h.send(2,'PhaseSeal',seal,4)
        h.c.advance(Event('stopped'),7)
        self.assertEqual(h.c.state,'Closed',h.c.failure)
        r._evidence('runner_final',stop_proven=True,failure='')
        self.header=header;self.rows=rows
        self.save()

    def save(self,rows=None):
        rows=self.rows if rows is None else rows
        for i,row in enumerate(rows,1):row['sequence']=i
        footer=dict(record_type='footer',offered=len(rows),written=len(rows),dropped=0,write_failures=0)
        self.path.write_text(''.join(json.dumps(r)+'\n' for r in [self.header,*rows,footer]))

    def audit(self):
        from simulation.evaluate import audit_evidence
        return audit_evidence(self.path,self.b,6000,dict(minimum_height_m=.4),[0,0,0])

    def test_complete_fixture_is_only_protocol_evidence(self):
        metrics,summary=self.audit()
        self.assertEqual(metrics['measured_rollout_steps'],3)
        self.assertFalse(metrics['fall_observed'])
        self.assertEqual(metrics['unexpected_runtime_faults'],0)
        self.assertTrue(summary['measured_feedback_complete'])

    def test_unreleased_missing_nonfinite_fault_fall_extra_cycle_phase_and_digest(self):
        def missing(rows):
            rows.pop(next(i for i,r in enumerate(rows) if r['kind']=='event_feedback' and r['control_step']==3))
        def no_release(rows):
            next(r for r in rows if r.get('released'))['released']=False
        def nonfinite(rows):
            next(r for r in rows if r.get('feedback'))['feedback']['q'][0]=float('nan')
        def fault(rows):rows[-1]['failure']='host phase deadline'
        def fall(rows):
            next(r for r in rows if r.get('released'))['feedback']['base_position'][2]=.3
        def extra(rows):
            i=next(i for i,r in enumerate(rows) if r['kind']=='write_accepted')
            rows.insert(i+1,copy.deepcopy(rows[i]))
        def phase(rows):
            next(r for r in rows if r['kind']=='AppliedToStep')['phase']=2
        def digest(rows):
            next(r for r in rows if r.get('frame',{}).get('name')=='Hello')['frame']['payload']['config_digest']='ff'*32
        def runtime_fault(rows):
            target=next(r for r in rows if r.get('frame',{}).get('name')=='Manifest' and
                        r['frame']['payload']['message']['kind']==3)
            target['frame']['payload']['message']['execution_state']=4
        for mutate in (missing,no_release,nonfinite,fault,fall,extra,phase,digest,runtime_fault):
            with self.subTest(mutation=mutate.__name__):
                rows=copy.deepcopy(self.rows);mutate(rows);self.save(rows)
                with self.assertRaises((ValueError,KeyError)):self.audit()

    def test_missing_footer_is_incomplete_even_with_valid_prefix(self):
        self.path.write_text('\n'.join(self.path.read_text().splitlines()[:-1])+'\n')
        with self.assertRaisesRegex(ValueError,'incomplete'):self.audit()


if __name__=='__main__':unittest.main()
