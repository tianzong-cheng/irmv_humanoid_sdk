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
        dt=getattr(self,'control_dt_us',2000)
        self.b=dataclasses.replace(bootstrap(),control_dt_us=dt,physics_dt_us=dt//2,
                                  engine_config=getattr(self,'engine_config',{}))
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
        def frame(kind,value,phase=1):
            return p.Frame(p.KINDS.index(kind)+1,value,self.b.run_id,h.c.step,
                           phase,1+h.c.step*dt,self.b.physics_dt_us,dt)
        h.frame=frame
        original_state=__import__('simulation_coordinator_test').state_identity
        state_clock=patch('simulation_coordinator_test.state_identity',
            side_effect=lambda step,*a,**kw:dict(original_state(step,*a,**kw),stamp_us=1+step*dt))
        state_clock.start();self.addCleanup(state_clock.stop)
        original_feedback=feedback
        feedback_clock=patch('simulation_coordinator_test.feedback',
            side_effect=lambda step=0:dataclasses.replace(original_feedback(step),engine_time_s=step*dt/2000000))
        feedback_clock.start();self.addCleanup(feedback_clock.stop)
        original_send=h.send
        def send(role,kind,value,phase=1,now=None):
            if kind=='Manifest' and value['message']['kind'] in (2,3):
                value['message']['stamp_us']=1+h.c.step*dt
            return original_send(role,kind,value,phase,now)
        h.send=send
        h.c.advance(Event('start'),0)
        arm=h.armed_phase();h.send(3,'ReleaseArm',arm,2)
        seal=h.inventory(3,2);h.finish(3,2,seal)
        def measured(step):
            return dataclasses.replace(feedback(step),engine_time_s=step*dt/2000000,base_position=(0.,0.,.8),base_quaternion=(1.,0.,0.,0.))
        # Bootstrap was captured by ready() with legacy empty quaternion. Supply
        # explicit measured quaternion in this deterministic adapter's snapshot.
        for row in rows:
            if row.get('feedback'):
                row['feedback']['base_quaternion']=[1.,0.,0.,0.]
                row['feedback']['base_position']=[0.,0.,.8]
        h.c.advance(Event('feedback',feedback=measured(4),released=True),4)
        command=arm['first_command']
        for _ in range(getattr(self,'rollout_steps',3)-1):
            torque=h.torque();torque['command']=command
            h.send(2,'Torque',torque)
            seal=h.inventory(2,1,(state_identity(h.c.step,command),));h.finish(2,1,seal)
            seal=h.inventory(3,2);h.finish(3,2,seal)
            h.c.advance(Event('feedback',feedback=measured((h.c.step+1)*2)),5)
        h.c.advance(Event('stop'),6)
        torque=h.torque();torque['tau']=(0.,)
        h.send(2,'Torque',torque,4)
        state=dict(state_identity(h.c.step,execution_state=1,output_action=0),stamp_us=1+h.c.step*dt)
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


class FullEvaluationTest(unittest.TestCase):
    """Full public evaluator; real coherent synthetic protocol + SDK exports.

    Only config normalization is stubbed at the external authority boundary.
    No audit, read, clock/phase/intent or report metrics are mocked. This is
    deterministic fixture evidence, never a GPU or trained-model attestation.
    """
    def setUp(self):
        from dataclasses import replace,asdict
        from unittest.mock import patch
        from simulation.evaluate import sha256
        from simulation.assets import closure
        self.fixture=EvidenceTest();self.fixture.control_dt_us=1000000
        self.fixture.rollout_steps=60
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.root=Path(self.fixture.temp.name)
        model=self.root/'robot.xml';model.write_text('<mujoco/>')
        self.geometry=dict(initial_base_position=[0,0,.8],initial_base_quaternion=[1,0,0,0],initial_joint_position=[0])
        engine_config=dict(self.geometry,model_path=str(model),base_body='base')
        versions={'mujoco':'3.9.0','mujoco-warp':'3.9.0','warp-lang':'1.15.0'}
        adapter=Path(__file__).resolve().parents[2]/'tools/simulation/mujoco_warp_engine.py'
        binding=dict(versions=versions,adapter_sha256=sha256(adapter))
        engine_digest=__import__('hashlib').sha256(json.dumps(binding,sort_keys=True).encode()).hexdigest()
        # Rebind only the fixture's Hello engine identities/header; all seals,
        # runtime messages and release/physics proofs remain genuine captures.
        self.b=replace(self.fixture.b,engine_digest=bytes.fromhex(engine_digest),engine_config=engine_config)
        header=self.fixture.header
        header['clock'].update(physics_dt_us=500000,control_dt_us=1000000)
        header['provenance'].update(engine_digest=engine_digest,engine_config=engine_config)
        for row in self.fixture.rows:
            if row.get('frame',{}).get('name')=='Hello':row['frame']['payload']['engine_digest']=engine_digest
        self.fixture.save()
        env=self.root/'env.yaml'
        env.write_text('root_height_below_minimum:\n  minimum_height: 0.4\n'+''.join(
            f'{key}: !!python/tuple\n- {low}\n- {high}\n' for key,(low,high) in
            zip(('lin_vel_x','lin_vel_y','ang_vel_z'),[[0,1.2],[0,0],[-1,1]])))
        urdf=self.root/'robot.urdf';urdf.write_text('<robot><link name="base"/></robot>')
        policy=self.root/'fixture.onnx';policy.write_bytes(b'explicit non-executed contract fixture')
        deployment=dict(source=dict(env_config_sha256=sha256(env),asset_revision=sha256(urdf)))
        asset=dict(prepared_closure=closure(model),policy_sha256=sha256(policy),
                   policy_scope='backend_integration_fixture',initial_geometry=self.geometry,source_urdf_sha256=sha256(urdf))
        self.cfg=dict(simulation=dict(engine_sha256=engine_digest,asset_sha256=asset['prepared_closure']['digest']),
            robot=dict(joints=[dict(name='joint')]),runtime=dict(control_hz=1,safety=dict(torque_limits=dict(joint=20))),
            telemetry=dict(recorder=dict(sample_hz=1)))
        bootstrap=asdict(self.b)
        bootstrap={k:v.hex() if isinstance(v,bytes) else v for k,v in bootstrap.items()}
        self.owner=dict(run_id=self.b.run_id.hex(),runtime_id=self.b.runtime_id.hex(),state='Closed',
            stop_proven=True,coordinator_result=0,final_child_codes=[0,0],release_state='Released',
            release_event_drops=0,failure='',engine_ready_before_sdk=True)
        self.paths=dict(sidecar=self.fixture.path,adapter=adapter,policy=policy,training_env=env,source_urdf=urdf)
        for key,doc in dict(bootstrap=bootstrap,config=self.cfg,evidence=self.owner,asset_manifest=asset,deployment=deployment).items():
            path=self.root/(key+'.json');path.write_text(json.dumps(doc));self.paths[key]=path
        normalizer=self.root/'normalizer';normalizer.write_bytes(b'unit-test normalization authority stub')
        self.paths['normalizer']=normalizer
        self.normalized={k:bootstrap[k] for k in ('config_digest','application_digest','engine_digest','engine_config',
                                                   'physics_dt_us','control_dt_us','startup_timeout_us','phase_timeout_us')}
        authority=patch('simulation.evaluate.subprocess.check_output',return_value=json.dumps(self.normalized))
        authority.start();self.addCleanup(authority.stop)
        self.runtime=[];self.application=[]
        for step in range(1,61):
            self.runtime.append(dict(record_type='sample',stamp_us=1+step*1000000,state_seq=step+1,
                execution_state=3,fault_id=0,stop_reason=0,send_accepted=True,feedback_valid=True,output_valid=True,
                joint_valid=[True],q=[0],measured_q=[0],output_torque=[4]))
            self.application.append(dict(record_type='velocity',stamp_us=1+step*1000000,sample_seq=step,
                latest_state_seq=step+1,runtime_id=self.b.runtime_id.hex(),session_id=(b'u'*16).hex(),
                identity_known=True,forward=0,lateral=0,yaw=0))
        self.application.append(dict(record_type='policy_session',model=dict(sha256=sha256(policy),
            contract_sha256=sha256(self.paths['deployment'])),runtime_id=self.b.runtime_id.hex(),
            session_id=(b'u'*16).hex(),source='local',logical_us=1000001))
        self.summary=dict(complete=True,truncated=False,clock=header['clock'],config_id=self.b.config_digest.hex(),
            application_config_id=self.b.application_digest.hex(),sample_hz=1,loop_period_us=1000000,
            faults=dict(complete=True,truncated=False,losses=[],missing_occurrences=0))
        self.manifest=dict(schema='irmv.policy_acceptance',version=1,scope='backend_integration_fixture',
            gpu=dict(executed=True,skipped=False,engine_digest=engine_digest,**binding),
            scenario='standing',intent_bounds=[[0,1.2],[0,0],[-1,1]],intent=[0,0,0],duration_us=60000000,
            fall=dict(frame='world',body='base',origin_offset_m=[0,0,0],minimum_height_m=.4),
            source_env_sha256=sha256(env),configured_limit_provenance='synthetic configured limit; no motor claim',files={})
        self.save_exports()

    def save_exports(self):
        # Summaries remain internally complete even for intentionally shortened
        # intent streams. Required external coverage cannot come from these flags.
        for role,rows in [('runtime',self.runtime),('application',self.application)]:
            selected=[r for r in rows if r['record_type'] in ('sample','velocity')]
            summary=dict(self.summary,record_kind=role,rows_in_range=len(selected),
                first_stamp_us=selected[0]['stamp_us'],last_stamp_us=selected[-1]['stamp_us'])
            if role=='application':summary.update(first_sample_seq=selected[0]['sample_seq'],last_sample_seq=selected[-1]['sample_seq'])
            path=self.root/(role+'.jsonl')
            path.write_text(''.join(json.dumps(r)+'\n' for r in [*rows,summary]));self.paths[role+'_export']=path
        from simulation.evaluate import sha256
        self.manifest['files']={key:dict(path=str(path),sha256=sha256(path)) for key,path in self.paths.items()}

    def test_positive_full_public_evaluator_fixture(self):
        report=evaluate(self.manifest)
        self.assertEqual(report['status'],'pass',report)
        self.assertFalse(report['trained_policy_accepted'])
        self.assertEqual(report['metrics']['measured_rollout_steps'],60)

    def test_missing_intent_prefix_suffix_or_interior_fails_complete_export(self):
        original=copy.deepcopy(self.application)
        for removed in (range(0,10),range(50,60),range(25,35)):
            with self.subTest(removed=removed):
                self.application=[copy.deepcopy(row) for i,row in enumerate(original) if i not in removed]
                # Even a consistently renumbered subset must not pass the horizon.
                for i,row in enumerate(self.application):
                    if row['record_type']=='velocity':row['sample_seq']=i+1
                self.save_exports();report=evaluate(self.manifest)
                self.assertEqual(report['status'],'fail',report)
                self.assertIsNone(report['metrics'].get('velocity_rmse_yaw_xy_world_yaw_rad_s'))

    def test_intent_state_and_session_must_match_consumed_phase(self):
        original=copy.deepcopy(self.application)
        for key,value in [('latest_state_seq',999),('latest_state_seq',1),('sample_seq',999),('session_id','ff'*16)]:
            with self.subTest(key=key,value=value):
                self.application=copy.deepcopy(original);self.application[20][key]=value
                self.save_exports();report=evaluate(self.manifest)
                self.assertEqual(report['status'],'fail',report)

    def test_failed_owner_retains_observed_release_and_torque(self):
        self.owner['final_child_codes']=[0,3]
        self.paths['evidence'].write_text(json.dumps(self.owner));self.save_exports()
        report=evaluate(self.manifest)
        self.assertEqual(report['status'],'fail')
        self.assertFalse(report['trained_policy_accepted'])
        self.assertEqual(report['metrics'].get('release_logical_us'),1000001)
        self.assertGreater(report['metrics'].get('completed_applied_intervals',0),0)
        self.assertEqual(report['metrics'].get('measured_rollout_steps'),60)

    def test_offline_four_report_regeneration_preserves_originals(self):
        import runpy
        from simulation.evaluate import sha256
        module=runpy.run_path(str(Path(__file__).resolve().parents[1]/'integration/simulation_policy_acceptance_test.py'))
        self.owner.update(final_child_codes=[0,3],failure_observation=dict(
            application_failure_lines=['behavior=Fault cause=InitializationFailed'],
            max_initialization_target_error_rad=.0248))
        self.paths['evidence'].write_text(json.dumps(self.owner));self.save_exports()
        old=self.root/'old-reports';old.mkdir()
        manifest_path=self.root/'manifest.json';manifest_path.write_text(json.dumps(self.manifest))
        reports=[]
        for scenario,status in [('standing','fail'),('forward','fail'),('lateral','unsupported'),('yaw','fail')]:
            directory=old/scenario;directory.mkdir()
            report=dict(scenario=scenario,status=status,scope='backend_integration_fixture',
                reasons=['original startup prerequisite' if status=='fail' else 'unsupported lateral'],metrics={})
            if scenario=='standing':report['manifest']=dict(path=str(manifest_path),sha256=sha256(manifest_path))
            elif status=='fail':report['prerequisite']=dict(reason='original startup prerequisite',files=[])
            (directory/'report.json').write_text(json.dumps(report));reports.append(report)
        (old/'reports.json').write_text(json.dumps(reports))
        before={str(path):sha256(path) for path in old.rglob('*.json')}
        output=self.root/'derived-reports'
        import contextlib,io
        printed=io.StringIO()
        with contextlib.redirect_stdout(printed):
            self.assertEqual(module['reevaluate_reports'](old,output),1)
        self.assertEqual(len(json.loads(printed.getvalue())),4)
        self.assertEqual(before,{str(path):sha256(path) for path in old.rglob('*.json')})
        derived=json.loads((output/'reports.json').read_text())
        self.assertEqual([r['status'] for r in derived],['fail','fail','unsupported','fail'])
        self.assertGreater(derived[0]['metrics']['completed_applied_intervals'],0)
        self.assertIs(derived[0]['metrics']['application_fault_observed'],True)
        for report in derived[1:]:
            self.assertFalse(report['actual_current_attempt_executed'])
            self.assertIsNone(report['metrics']['completed_applied_intervals'])
            self.assertFalse(report['trained_policy_accepted'])

    def test_recorded_protection_retains_observation_on_failure(self):
        rows=copy.deepcopy(self.fixture.rows)
        target=next(r for r in rows if r.get('frame',{}).get('name')=='Manifest' and
                    r['frame']['payload']['message']['kind']==2)
        target['frame']['payload']['message']['status']=1
        self.fixture.save(rows);self.save_exports();report=evaluate(self.manifest)
        self.assertEqual(report['status'],'fail',report)
        self.assertIs(report['metrics']['protection_observed'],True)
        self.assertGreater(report['metrics']['unexpected_protections'],0)
        self.assertTrue(any('protection' in reason for reason in report['reasons']))

    def test_foreign_run_never_labels_its_prefix_as_current_observations(self):
        self.fixture.header['clock']['clock_run_id']='ff'*16
        self.fixture.save();self.save_exports();report=evaluate(self.manifest)
        self.assertEqual(report['status'],'fail',report)
        self.assertIsNone(report['metrics']['max_abs_measured_torque_Nm'])
        self.assertIsNone(report['metrics']['completed_applied_intervals'])

    def test_absent_intent_export_keeps_bound_engine_observations(self):
        self.paths['application_export'].unlink()
        report=evaluate(self.manifest);metrics=report['metrics']
        self.assertEqual(report['status'],'fail',report)
        self.assertEqual(metrics['release_logical_us'],1000001)
        self.assertGreater(metrics['completed_applied_intervals'],0)
        self.assertIsNone(metrics['intent_samples'])
        self.assertIsNone(metrics['velocity_rmse_yaw_xy_world_yaw_rad_s'])

    def test_fall_height_survives_unavailable_velocity_orientation(self):
        rows=copy.deepcopy(self.fixture.rows)
        row=next(r for r in rows if r.get('feedback') and r['control_step']==20)
        row['feedback'].update(base_position=[0,0,.3],base_quaternion=[])
        self.fixture.save(rows);self.save_exports();report=evaluate(self.manifest)
        self.assertEqual(report['status'],'fail',report)
        self.assertIs(report['metrics']['fall_observed'],True)
        self.assertEqual(report['metrics']['min_base_height_m'],.3)
        self.assertIsNone(report['metrics']['velocity_rmse_yaw_xy_world_yaw_rad_s'])

    def test_overflowed_numeric_metric_is_unavailable_not_invalid_json(self):
        self.runtime[0].update(q=[1e308],measured_q=[-1e308])
        self.save_exports();report=evaluate(self.manifest)
        self.assertEqual(report['status'],'fail',report)
        self.assertIsNone(report['metrics']['joint_rmse_rad'])
        self.assertGreater(report['metrics']['completed_applied_intervals'],0)
        json.dumps(report,allow_nan=False)

    def test_partial_fall_and_missing_tail_keep_unknown_distinct_from_zero(self):
        rows=copy.deepcopy(self.fixture.rows)
        cut=next(i for i,r in enumerate(rows) if r.get('feedback') and r['control_step']==20)
        rows[cut]['feedback']['base_position'][2]=.3
        self.fixture.save(rows[:cut+1]);self.save_exports()
        report=evaluate(self.manifest);metrics=report['metrics']
        self.assertEqual(report['status'],'fail',report)
        self.assertIs(metrics['fall_observed'],True)
        self.assertGreater(metrics['measured_rollout_steps'],0)
        self.assertLess(metrics['measured_rollout_steps'],60)
        self.assertIsNone(metrics['unexpected_runtime_faults'])
        self.assertIsNone(metrics['velocity_rmse_yaw_xy_world_yaw_rad_s'])
        self.assertFalse(metrics['rollout_measurement_coverage_complete'])
        self.assertEqual(metrics['max_abs_measured_torque_Nm'],0)  # measured zero
        self.assertEqual(metrics['min_base_height_m'],.3)

    def test_missing_evidence_metrics_are_unavailable(self):
        self.paths['sidecar'].unlink()
        report=evaluate(self.manifest)
        self.assertEqual(report['status'],'fail',report)
        self.assertIsNone(report['metrics']['max_abs_measured_torque_Nm'])
        self.assertIsNone(report['metrics']['fall_observed'])
        self.assertIsNone(report['metrics']['completed_applied_intervals'])

    def test_fall_and_fault_failures_retain_partial_metrics(self):
        original=copy.deepcopy(self.fixture.rows)
        for kind in ('fall','fault'):
            with self.subTest(kind=kind):
                rows=copy.deepcopy(original)
                if kind=='fall':
                    next(r for r in rows if r.get('feedback') and r['control_step']==20)['feedback']['base_position'][2]=.3
                else:
                    target=next(r for r in rows if r.get('frame',{}).get('name')=='Manifest' and
                        r['control_step']==20 and r['frame']['payload']['message']['kind']==3)
                    target['frame']['payload']['message']['execution_state']=4
                self.fixture.save(rows);self.save_exports();report=evaluate(self.manifest)
                self.assertEqual(report['status'],'fail',report)
                self.assertFalse(report['trained_policy_accepted'])
                metrics=report['metrics']
                self.assertEqual(metrics.get('release_logical_us'),1000001)
                self.assertGreater(metrics.get('completed_applied_intervals',0),0)
                if kind=='fall':self.assertIs(metrics.get('fall_observed'),True)
                else:self.assertGreater(metrics.get('unexpected_runtime_faults',0),0)


if __name__=='__main__':unittest.main()
