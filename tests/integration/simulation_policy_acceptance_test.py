"""Four independent actual-policy experiments; no reset, tuning, or skip-pass.

An explicit specification binds config, saved training bounds/fall origin and
asset provenance. A known failed prerequisite produces four honest reports
without fruitlessly replaying the unchanged failed startup experiment.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from simulation.evaluate import evaluate,sha256,scenario_intent,empty_metrics


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def run_scenario(build,spec,scenario,directory):
    from sim_launch import Experiment
    from simulation.main import Runner
    from simulation.assets import engine_identity
    intent=scenario_intent(scenario,spec['intent_bounds'])
    report=dict(status='incomplete',scenario=scenario,scope=spec['scope'],
                trained_policy_accepted=False,physical_safety='not evaluated',reasons=[],metrics=empty_metrics(),
                actual_current_attempt_executed=False,actual_current_rollout_executed=False)
    directory.mkdir()
    report['prepared_config']=dict(path=spec['config'],sha256=sha256(spec['config']))
    report['configured_limit_provenance']=spec['configured_limit_provenance']
    if intent is None:
        report.update(status='unsupported',reasons=['nonzero requested axis outside source training bounds'])
        write_json(directory/'report.json',report);return report
    if spec.get('known_failed_prerequisite'):
        prior=spec['known_failed_prerequisite']
        for item in prior['files']:
            if sha256(item['path'])!=item['sha256']:raise ValueError('historical prerequisite evidence changed')
        report.update(status='fail',reasons=[prior['reason']],
            prerequisite=prior,actual_current_rollout_executed=False)
        write_json(directory/'report.json',report);return report
    cfg=json.loads(Path(spec['config']).read_text())
    cfg['dds']['domain']=227  # Reserved for this test; fresh cases run sequentially.
    # Separate run artifacts; normalization digests the actual final config.
    cfg['simulation']['socket_directory']=str(directory/'run')
    cfg['telemetry']['directory']=str(directory/'logs')
    (directory/'logs').mkdir()
    cfg['telemetry']['recorder'].update(enabled=True,directory=str(directory),sample_hz=cfg['runtime']['control_hz'])
    config=directory/'sdk.json';write_json(config,cfg)
    keys=cfg['input']['bindings'];sent=set();released=None;leave=None;attempt_offset=0;last_intent=-1;failure='';failure_observation={}
    owner=None
    class Inputs(Runner):
        def _actions(self,actions):
            nonlocal released,leave,attempt_offset,last_intent,failure,failure_observation
            c=self.coordinator
            if owner and len(owner.children)==2:
                output=(owner.directory/'application.log').read_text()
                def send(key):
                    owner.children[1].stdin.write(key.encode());owner.children[1].stdin.flush()
                if not failure:
                    if 'behavior=Fault' in output or 'init=refused' in output:
                        failure='actual startup/SDK constraint incompatibility'
                    if keys['start_locomotion'] in sent and c.release_state!='Released' and any(
                        v in output[attempt_offset:] for v in ('reject=Discontinuity','stop=CommandTimeout')):
                        failure='actual Locomotion handoff rejected by SDK constraints'
                if failure:
                    if not failure_observation:
                        from dataclasses import asdict
                        f=c.feedback
                        goals=[cfg['application']['initialization']['target_position'][j['name']] for j in cfg['robot']['joints']]
                        failure_observation=dict(observed_control_step=c.step,phase=c.phase,coordinator_state=c.state,
                            logical_us=1+c.step*c.b.control_dt_us,feedback=asdict(f),
                            max_initialization_target_error_rad=max(abs(q-target) for q,target in zip(f.q,goals)),
                            application_failure_lines=[line for line in output.splitlines() if 'behavior=Fault' in line or 'init=refused' in line])
                    self.request_shutdown()
                else:
                    for behavior,action in [('Standby','damping'),('Damping','initialize'),('Ready','start_locomotion')]:
                        key=keys[action]
                        if ('behavior='+behavior) in output and key not in sent:
                            if action=='initialize':
                                f=c.feedback;names=[j['name'] for j in cfg['robot']['joints']]
                                init=cfg['application']['initialization']
                                admissible=all(j['position_min']<=q<=j['position_max'] and
                                    abs(dq)<=cfg['runtime']['continuity']['velocity_delta'][n] and
                                    abs((init['gains'][n]['kd']-cfg['runtime']['damping']['kd'][n])*dq)<=cfg['runtime']['continuity']['torque_delta'][n] and
                                    abs(init['target_position'][n]-q)*3.141592653589793/(2*init['trajectory']['duration_us']/1e6)<=cfg['runtime']['safety']['target_velocity_limits'][n]
                                    for n,j,q,dq in zip(names,cfg['robot']['joints'],f.q,f.dq))
                                if not admissible:continue
                            if action=='start_locomotion':
                                # Actual unit intent precedes the first policy tick;
                                # no zero-intent interval is relabeled as forward.
                                velocity={'forward':'velocity_forward','lateral':'velocity_left','yaw':'velocity_yaw_left'}.get(scenario)
                                if velocity:send(keys[velocity])
                                last_intent=c.step*c.b.control_dt_us
                                attempt_offset=len(output)
                            send(key);sent.add(key)
                    if c.release_state=='Released':
                        if released is None:released=c.release_events[-1][1]-1
                        now=c.step*c.b.control_dt_us
                        if now-last_intent>=cfg['input']['intent_timeout_us']//2:
                            action={'forward':'velocity_forward','lateral':'velocity_left','yaw':'velocity_yaw_left'}.get(scenario)
                            if action:send(keys[action])
                            last_intent=now
                        if (c.step-released)*c.b.control_dt_us>=60000000 and keys['exit'] not in sent:
                            send(keys['exit']);sent.add(keys['exit'])
                        if c.feedback.base_position[2]<spec['fall']['minimum_height_m']:
                            failure='source-derived fall event';self.request_shutdown()
                    if c.runtime_only:
                        if leave is None:leave=c.step
                        if c.step>=leave+2:self.request_shutdown()
            return super()._actions(actions)
    owner=Experiment(config,build/'robot_runtime',build/'application',runner_type=Inputs,
                     application_stdin=subprocess.PIPE)
    try:
        owner.start()
        deadline=time.monotonic()+spec.get('host_timeout_s',180)
        while owner.thread.is_alive() and time.monotonic()<deadline:
            if owner.runner.coordinator.step*owner.runner.coordinator.b.control_dt_us>4000000 and not released and not failure:
                failure='startup/release incomplete within 4 logical seconds';owner.runner.request_shutdown()
            time.sleep(.01)
        if owner.thread.is_alive():
            failure='host deadline; incomplete experiment';owner.runner.request_stop()
        else:owner.wait()
    except (OSError,ValueError,RuntimeError,AssertionError,subprocess.SubprocessError) as exc:
        failure=str(exc)
    finally:
        owner.evidence.update(acceptance_scope=spec['scope'],failure_observation=failure_observation,
            acceptance_failure=failure,requested_scenario=scenario)
        owner.close()
    manifest=dict(schema='irmv.policy_acceptance',version=1,scope=spec['scope'],scenario=scenario,
        duration_us=60000000,intent=intent,intent_bounds=spec['intent_bounds'],fall=spec['fall'],
        source_env_sha256=sha256(spec['training_env']),
        configured_limit_provenance=spec['configured_limit_provenance'],files={})
    identity,binding=engine_identity()
    manifest['gpu']=dict(executed=owner.runner is not None and owner.runner.coordinator.feedback is not None,
        skipped=False,engine_digest=identity,**binding)
    files=dict(config=config,normalizer=build/'simulation_normalize',bootstrap=directory/'run/bootstrap.json',sidecar=directory/'run/simulation.jsonl',
        evidence=directory/'run/evidence.json',adapter=ROOT/'tools/simulation/mujoco_warp_engine.py',
        policy=Path(cfg['application']['policy']['model_path']),training_env=Path(spec['training_env']),
        source_urdf=Path(spec['source_urdf']),deployment=Path(spec['deployment']),asset_manifest=Path(cfg['simulation']['model_path']).with_suffix('.manifest.json'))
    for role in ('runtime','application'):
        candidates=[]
        for meta in directory.glob('*/meta.json'):
            doc=json.loads(meta.read_text())
            if (role=='runtime')==('schema_version' in doc):candidates.append(meta.parent)
        if len(candidates)==1:
            out=directory/(role+'.jsonl')
            with out.open('w') as stream:
                subprocess.run([str(build/'read_record'),'--run',str(candidates[0])],stdout=stream,check=True,timeout=60)
            files[role+'_export']=out
    for key,path in files.items():
        if path.is_file():manifest['files'][key]=dict(path=str(path),sha256=sha256(path))
    write_json(directory/'manifest.json',manifest)
    report=assemble_report(manifest,directory/'manifest.json')
    if failure and failure not in report['reasons']:
        report['status']='fail';report['trained_policy_accepted']=False;report['reasons'].insert(0,failure)
    write_json(directory/'report.json',report)
    return report


def assemble_report(manifest,manifest_path):
    """Same assembly for live completion and offline re-evaluation."""
    report=evaluate(manifest);metrics=report['metrics']
    report.update(scenario=manifest['scenario'],
        actual_current_attempt_executed=(metrics['observed_feedback_steps'] or 0)>0,
        actual_current_rollout_executed=metrics['released_control_step'] is not None,
        manifest=dict(path=str(manifest_path),sha256=sha256(manifest_path)))
    return report


def reevaluate_reports(source,output):
    """Derive reports from immutable artifacts; never call an engine or launcher."""
    source=Path(source).resolve();output=Path(output).resolve()
    originals=json.loads((source/'reports.json').read_text())
    if [r['scenario'] for r in originals]!=['standing','forward','lateral','yaw']:
        raise ValueError('ordered four-scenario artifact group required')
    output.mkdir()  # Exclusive: never overwrite original or earlier fix evidence.
    reports=[];standing=None
    for original in originals:
        scenario=original['scenario'];directory=output/scenario;directory.mkdir()
        original_path=source/scenario/'report.json'
        if scenario=='standing':
            descriptor=original['manifest'];path=Path(descriptor['path'])
            if sha256(path)!=descriptor['sha256']:raise ValueError('original manifest changed')
            report=assemble_report(json.loads(path.read_text()),path)
        else:
            prerequisite=original.get('prerequisite')
            if prerequisite:
                for item in prerequisite['files']:
                    if sha256(item['path'])!=item['sha256']:raise ValueError('prerequisite evidence changed')
            report=dict(original,metrics=empty_metrics(),trained_policy_accepted=False,
                actual_current_attempt_executed=False,actual_current_rollout_executed=False)
            if prerequisite and standing:
                report['prerequisite']=dict(prerequisite,current_evaluation=dict(
                    path=str(standing),sha256=sha256(standing)))
        report['derived_from']=dict(path=str(original_path),sha256=sha256(original_path))
        write_json(directory/'report.json',report)
        if scenario=='standing':standing=directory/'report.json'
        reports.append(report)
    write_json(output/'reports.json',reports)
    print(json.dumps(reports,indent=2,allow_nan=False))
    return 0 if all(r['status']=='pass' for r in reports) else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build',type=Path,nargs='?');parser.add_argument('--spec',type=Path)
    parser.add_argument('--reevaluate',type=Path,help='existing four-report artifact group; offline only')
    parser.add_argument('--output',type=Path,help='new exclusive derived-report directory')
    args=parser.parse_args()
    if args.reevaluate:
        if args.build or args.spec or not args.output:parser.error('--reevaluate requires --output and no build/spec')
        return reevaluate_reports(args.reevaluate,args.output)
    if not args.build or not args.spec or args.output:parser.error('experiment mode requires build and --spec')
    spec=json.loads(args.spec.read_text())
    output=Path(spec['output']).resolve();output.mkdir()  # Exclusive: preserve all prior evidence.
    reports=[]
    for scenario in ('standing','forward','lateral','yaw'):
        try:report=run_scenario(args.build.resolve(),spec,scenario,output/scenario)
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
            report=dict(status='incomplete',scenario=scenario,trained_policy_accepted=False,reasons=[str(exc)],metrics=empty_metrics(),
                        actual_current_attempt_executed=False,actual_current_rollout_executed=False)
            (output/scenario).mkdir(exist_ok=True);write_json(output/scenario/'report.json',report)
        reports.append(report)
        if (scenario=='standing' and report.get('actual_current_attempt_executed') and
                report.get('failure_observation') and not report.get('actual_current_rollout_executed')):
            files=[output/scenario/name for name in ('report.json','manifest.json','run/evidence.json','run/simulation.jsonl','run/application.log')]
            spec=dict(spec,known_failed_prerequisite=dict(reason='Blocked by current standing startup/SDK constraint incompatibility; no repeated identical startup experiment',
                files=[dict(path=str(path),sha256=sha256(path)) for path in files]))
    write_json(output/'reports.json',reports)
    print(json.dumps(reports,indent=2))
    return 0 if all(r['status']=='pass' for r in reports) else 1

if __name__=='__main__':raise SystemExit(main())
