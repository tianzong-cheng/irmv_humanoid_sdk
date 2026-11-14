"""Fail-closed offline acceptance. No physical safety inference or GPU fallback.

The caller supplies a byte-bound run manifest. Evidence is streamed, not loaded
with record.read's deliberately capped convenience API. Protocol events are
replayed through the clock-free coordinator to validate first-command/release,
DDS phase seals, step ordering and actual applied-write identities.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import subprocess
from pathlib import Path

from . import protocol as p
from .coordinator import Bootstrap, Event, SimulationCoordinator
from .engine import Feedback
from .record import MAX_LINE, read


def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def bound_file(manifest,key):
    item=manifest['files'][key]
    path=Path(item['path'])
    if sha256(path)!=item['sha256']:raise ValueError('file digest mismatch: '+key)
    return path


def load(path):
    return json.loads(Path(path).read_text())


def finite(value):
    if isinstance(value,dict):
        for v in value.values():finite(v)
    elif isinstance(value,(list,tuple)):
        for v in value:finite(v)
    elif isinstance(value,float) and not math.isfinite(value):
        raise ValueError('nonfinite metric/evidence')


def scenario_intent(scenario,bounds):
    if (len(bounds)!=3 or any(len(pair)!=2 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in pair) or
            pair[0]>pair[1] for pair in bounds)):
        raise ValueError('three finite ordered source intent bounds required')
    intent={'standing':[0,0,0],'forward':[1,0,0],
            'lateral':[0,1,0],'yaw':[0,0,1]}[scenario]
    return intent if all(low<=v<=high for v,(low,high) in zip(intent,bounds)) else None


def yaw_frame_velocity(quaternion,velocity):
    w,x,y,z=quaternion
    if abs(sum(v*v for v in quaternion)-1)>1e-6:raise ValueError('nonunit base orientation')
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    vx,vy,_,wx,wy,wz=velocity
    return [math.cos(yaw)*vx+math.sin(yaw)*vy,
            -math.sin(yaw)*vx+math.cos(yaw)*vy,
            2*(x*z-w*y)*wx+2*(y*z+w*x)*wy+(1-2*(x*x+y*y))*wz]


def payload(kind,value):
    if kind in ('Id','Digest','Text256'):return bytes.fromhex(value)
    if kind in p.SCHEMA:return {n:payload(t,value[n]) for t,n in p.SCHEMA[kind]}
    return value


def wire_frame(row,b):
    frame=row['frame'];name=frame['name']
    return p.decode(p.encode(p.Frame(p.KINDS.index(name)+1,payload(name,frame['payload']),
        b.run_id,row['control_step'],row['phase'],row['logical_us'],b.physics_dt_us,b.control_dt_us)))


def stream_json(path):
    with Path(path).open('rb') as stream:
        while True:
            raw=stream.readline(MAX_LINE+2)
            if not raw:break
            if len(raw)>MAX_LINE or not raw.endswith(b'\n'):raise ValueError('invalid SDK export line')
            row=json.loads(raw);finite(row);yield row


def require_complete_sdk_summary(summary,role):
    if (not summary or not summary.get('complete') or summary.get('truncated') or
            summary.get('corrupt') or summary.get('gaps') or summary.get('losses') or
            summary.get('policy_session_truncated') or summary.get('missing_samples',0)):
        raise ValueError('incomplete '+role+' SDK record')
    if role=='runtime':
        faults=summary.get('faults')
        if (not faults or not faults.get('complete') or faults.get('truncated') or
                faults.get('losses') or faults.get('missing_occurrences',0)):
            raise ValueError('incomplete fault accounting')


def evaluate(manifest):
    report=dict(status='fail',scope=manifest.get('scope','unknown'),reasons=[],metrics={},
                physical_safety='not evaluated',trained_policy_accepted=False)
    try:
        _evaluate(manifest,report)
        report['status']='pass'
        report['trained_policy_accepted']=manifest['scope']=='trained_policy'
    except (OSError,ValueError,KeyError,TypeError,IndexError,AttributeError,ZeroDivisionError,subprocess.SubprocessError) as exc:
        report['reasons'].append(str(exc))
    return report


def audit_evidence(path,b,duration_us,fall,intent):
    """Streaming protocol/measurement audit; duration supplied by experiment owner."""
    report={'metrics':{}}
    c=SimulationCoordinator(b);pending=deque();release=None;measured=0;applied=0
    sum_velocity=[0.,0.,0.];max_tau=0.;minimum_height=None;host_max=0;host_total=0
    previous_host=-1;arm=None
    def visitor(row):
        nonlocal release,measured,applied,max_tau,minimum_height,host_max,host_total,previous_host,arm
        finite(row)
        if row.get('failure'):raise ValueError('protection/runner failure: '+row['failure'])
        host=row['host_observed_us']
        if type(host) is not int or host<previous_host:raise ValueError('host clock order')
        previous_host=host
        host_max=max(host_max,row.get('host_operation_duration_us',0));host_total+=row.get('host_operation_duration_us',0)
        kind=row['kind']
        if kind.startswith('event_'):
            if pending:raise ValueError('missing accepted/applied action evidence')
            event_kind=kind[6:]
            f=wire_frame(row,b) if row.get('frame') else None
            fb=row.get('feedback')
            feedback=Feedback(**{k:tuple(v) if isinstance(v,list) else v for k,v in fb.items()}) if fb else None
            if f and p.KINDS[f.kind-1]=='Manifest':
                message=f.payload['message']
                if message['kind']==3 and (message['execution_state']==4 or message['fault_id']):
                    raise ValueError('unexpected runtime fault')
                if message['kind']==2 and message['status']!=0:
                    raise ValueError('unexpected request rejection/protection')
            old=c.torque
            actions=c.advance(Event(event_kind,peer=row.get('peer',0),frame=f,feedback=feedback,
                reason=row.get('reason',''),released=row.get('released',False)),host)
            if c.state=='Failed':raise ValueError('protocol replay: '+c.failure)
            if (row['control_step']!= (f.control_step if f else c.step) or row['phase']!=(f.phase if f else c.phase)
                    or row['coordinator_state']!=c.state):raise ValueError('step/phase/state mismatch')
            if f and p.KINDS[f.kind-1]=='Torque' and c.torque is not old:
                pending.append(('write_accepted',f))
            for action in actions:
                if action.frame and p.KINDS[action.frame.kind-1] in ('AppliedToStep','ShutdownCommitted','LeaveCommitted'):
                    pending.append((p.KINDS[action.frame.kind-1],action.frame))
            if f and p.KINDS[f.kind-1]=='ReleaseArm':arm=f.payload
            if row.get('released'):
                if release is not None or arm is None:raise ValueError('release lacks actual application confirmation')
                release=c.step-1
            if feedback:
                if row.get('feedback_validated') is not True:raise ValueError('unvalidated measurement')
                if release is not None and c.step>release:
                    if c.step-release<=duration_us//b.control_dt_us:
                        values=yaw_frame_velocity(feedback.base_quaternion,feedback.base_velocity)
                        for i in range(3):sum_velocity[i]+=(values[i]-intent[i])**2
                        measured+=1
                        height=feedback.base_position[2]
                        minimum_height=height if minimum_height is None else min(minimum_height,height)
                        if height<fall['minimum_height_m']:raise ValueError('fall observed')
                        max_tau=max(max_tau,*map(abs,feedback.tau))
        elif kind in ('write_accepted','AppliedToStep','ShutdownCommitted','LeaveCommitted'):
            if not pending:raise ValueError('extra command/application cycle')
            expected,frame=pending.popleft()
            if kind!=expected or wire_frame(row,b)!=frame:raise ValueError('write/application identity mismatch')
            if kind=='AppliedToStep':applied+=1
        elif kind=='runner_final':
            if pending or c.state!='Closed' or not c.stop_proven:raise ValueError('incomplete final lifecycle')
        else:raise ValueError('unknown evidence kind: '+kind)
    _,summary=read(path,visitor=visitor)
    report['metrics'].update(released_control_step=release,release_logical_us=None if release is None else 1+release*b.control_dt_us,
        completed_applied_intervals=applied,measured_rollout_steps=measured,
        fall_observed=False,unexpected_runtime_faults=0,
        min_base_height_m=minimum_height,max_abs_measured_torque_Nm=max_tau,
        host_engine_operation_max_us=host_max,host_engine_operation_total_us=host_total)
    provenance=summary['provenance']
    for key in ('runtime_id','config_digest','application_digest','engine_digest'):
        if provenance[key]!=getattr(b,key).hex():raise ValueError('record identity/digest mismatch: '+key)
    if provenance['engine_config']!=b.engine_config:raise ValueError('record engine config mismatch')
    if summary['clock']['clock_run_id']!=b.run_id.hex():raise ValueError('record run mismatch')
    if not summary['evidence_complete'] or not summary['measured_feedback_complete']:
        raise ValueError('incomplete file/measured evidence')
    if pending or summary['final']!=dict(coordinator_state='Closed',stop_proven=True,failure=''):
        raise ValueError('incomplete final evidence')
    if release is None:raise ValueError('base never released')
    if measured*b.control_dt_us!=duration_us:raise ValueError('missing rollout steps')
    report['metrics']['velocity_rmse_yaw_xy_world_yaw_rad_s']=[math.sqrt(v/measured) for v in sum_velocity]
    return report['metrics'],summary


def _evaluate(m,report):
    if m.get('schema')!='irmv.policy_acceptance' or m.get('version')!=1:
        raise ValueError('explicit versioned run manifest required')
    finite(m)
    if m['scope'] not in ('trained_policy','backend_integration_fixture'):
        raise ValueError('explicit policy/fixture scope required')
    # Every file is attested, including failed-attempt logs and original evidence.
    paths={key:bound_file(m,key) for key in m['files']}
    bdoc=load(paths['bootstrap']);cfg=load(paths['config']);evidence=load(paths['evidence'])
    b=Bootstrap(**{k:(bytes.fromhex(v) if k in ('run_id','runtime_id','config_digest',
        'application_digest','engine_digest') else v) for k,v in bdoc.items()})
    if m['gpu'].get('executed') is not True or m['gpu'].get('skipped') is not False:
        raise ValueError('GPU missing/skipped; cannot accept')
    if m['gpu']['engine_digest']!=b.engine_digest.hex():raise ValueError('GPU binding mismatch')
    if sha256(paths['adapter'])!=m['gpu']['adapter_sha256']:raise ValueError('adapter binding mismatch')
    engine_binding=dict(versions=m['gpu']['versions'],adapter_sha256=m['gpu']['adapter_sha256'])
    current=hashlib.sha256(json.dumps(engine_binding,sort_keys=True).encode()).hexdigest()
    if current!=b.engine_digest.hex() or cfg['simulation']['engine_sha256']!=current:
        raise ValueError('stale engine binding')
    normalized=json.loads(subprocess.check_output([str(paths['normalizer']),str(paths['config'])],text=True,timeout=30))
    for key in ('config_digest','application_digest','engine_digest'):
        if normalized[key]!=getattr(b,key).hex():raise ValueError('actual config normalization mismatch: '+key)
    for key in ('engine_config','physics_dt_us','control_dt_us','startup_timeout_us','phase_timeout_us'):
        if normalized[key]!=getattr(b,key):raise ValueError('actual config normalization mismatch: '+key)
    if m['gpu']['versions']!={'mujoco':'3.9.0','mujoco-warp':'3.9.0','warp-lang':'1.15.0'}:
        raise ValueError('unvalidated GPU dependencies')
    asset=load(paths['asset_manifest'])
    if asset.get('policy_scope','trained_policy')!=m['scope']:
        raise ValueError('fixture cannot upgrade trained-policy acceptance')
    from .assets import admit_initial_geometry
    admit_initial_geometry(b.engine_config,asset)
    from .assets import closure
    if closure(b.engine_config['model_path'])!=asset['prepared_closure']:
        raise ValueError('asset closure mismatch')
    if cfg['simulation']['asset_sha256']!=asset['prepared_closure']['digest']:
        raise ValueError('config asset mismatch')
    if sha256(paths['policy'])!=asset['policy_sha256']:raise ValueError('policy binding mismatch')
    if any(evidence.get(k)!=v for k,v in dict(run_id=b.run_id.hex(),runtime_id=b.runtime_id.hex(),
        state='Closed',stop_proven=True,coordinator_result=0,final_child_codes=[0,0],release_state='Released',
        release_event_drops=0).items()):raise ValueError('incomplete/failed actual experiment')
    if evidence.get('failure'):raise ValueError('experiment failure')
    if evidence.get('engine_ready_before_sdk') is not True:raise ValueError('actual SDK startup absent')
    bounds=m['intent_bounds'];intent=scenario_intent(m['scenario'],bounds)
    if intent is None:raise ValueError('scenario unsupported within source bounds')
    if m['intent']!=intent:raise ValueError('intent does not match configured InputRouter unit keys')
    if m['duration_us']!=60000000:raise ValueError('required 60 logical seconds after release')
    fall=m['fall']
    if fall['frame']!='world' or fall['body']!=b.engine_config['base_body'] or fall['origin_offset_m']!=[0,0,0]:
        raise ValueError('fall origin/frame not bound to measured base')
    if fall['minimum_height_m']<=0:raise ValueError('invalid source fall threshold')
    source=paths['training_env'].read_text()
    # Saved tagged YAML is inspected as text, never executed/deserialized.
    import re
    for name,limits in zip(('lin_vel_x','lin_vel_y','ang_vel_z'),bounds):
        match=re.search(r'\b'+name+r':\s*!!python/tuple\s*\n\s*- ([^\n]+)\n\s*- ([^\n]+)',source)
        if not match or list(map(float,match.groups()))!=limits:raise ValueError('source intent bounds mismatch')
    match=re.search(r'root_height_below_minimum:.*?minimum_height:\s*([0-9.]+)',source,re.S)
    if not match or float(match[1])!=fall['minimum_height_m']:raise ValueError('source fall threshold mismatch')
    contract=load(paths['deployment'])
    if contract['source']['asset_revision']!=asset['source_urdf_sha256']:
        raise ValueError('deployment source asset mismatch')
    import xml.etree.ElementTree as ET
    urdf=ET.parse(paths['source_urdf'])
    roots={n.get('name') for n in urdf.findall('link')}-{n.get('link') for n in urdf.findall('joint/child')}
    if roots!={fall['body']} or sha256(paths['source_urdf'])!=asset['source_urdf_sha256']:
        raise ValueError('fall actor origin not bound to URDF root')
    if contract['source']['env_config_sha256']!=sha256(paths['training_env']):
        raise ValueError('model deployment training source mismatch')
    if m['source_env_sha256']!=sha256(paths['training_env']):raise ValueError('training source digest mismatch')
    metrics,summary=audit_evidence(paths['sidecar'],b,m['duration_us'],fall,intent)
    report['metrics'].update(metrics)
    if summary['provenance'].get('engine_factory')!='simulation.main.warp_engine':
        raise ValueError('actual GPU engine factory evidence absent')
    release=metrics['released_control_step']
    # SDK binary-reader exports retain numeric target/feedback coherence and
    # application intent/policy-session identity. Do not fabricate targets from q.
    joint_sum=0.;joint_n=0;saturated=0;torque_n=0;runtime_summary=None;app_summary=None;policy_seen=False;velocities=0
    start=1+release*b.control_dt_us;end=start+m['duration_us']
    names=[j['name'] for j in cfg['robot']['joints']]
    limits=[cfg['runtime']['safety']['torque_limits'][n] for n in names]
    sample_dt=1000000//cfg['telemetry']['recorder']['sample_hz']
    next_sample=1+((start-1+sample_dt-1)//sample_dt)*sample_dt
    last_velocity_stamp=None
    for row in stream_json(paths['runtime_export']):
        kind=row.get('record_type')
        if kind=='sample' and start<=row['stamp_us']<end:
            if row['stamp_us']!=next_sample:raise ValueError('missing/duplicate numeric joint metric sample')
            next_sample+=sample_dt
            if (row['execution_state']!=3 or row['fault_id'] or row['stop_reason'] or not row['send_accepted']
                    or not row['feedback_valid'] or not row['output_valid'] or not all(row['joint_valid'])):
                raise ValueError('unexpected runtime fault/protection/missing output')
            if any(len(row[k])!=len(names) for k in ('q','measured_q','output_torque','joint_valid')):raise ValueError('joint metric shape')
            for q,actual,tau,limit in zip(row['q'],row['measured_q'],row['output_torque'],limits):
                joint_sum+=(q-actual)**2;joint_n+=1;saturated+=abs(tau)>=limit;torque_n+=1
        elif kind=='fault':raise ValueError('unexpected fault event')
        elif row.get('record_kind')=='runtime':runtime_summary=row
    for row in stream_json(paths['application_export']):
        kind=row.get('record_type')
        if kind=='velocity' and start<=row['stamp_us']<end:
            if last_velocity_stamp is not None and row['stamp_us']<=last_velocity_stamp:
                raise ValueError('duplicate/unordered application intent sample')
            last_velocity_stamp=row['stamp_us']
            if row['runtime_id']!=b.runtime_id.hex() or not row['identity_known']:
                raise ValueError('application intent identity missing')
            if [row[k] for k in ('forward','lateral','yaw')]!=intent:raise ValueError('recorded intent mismatch')
            velocities+=1
        elif kind=='policy_session':
            if (row.get('model',{}).get('sha256')==sha256(paths['policy']) and
                    row['model'].get('contract_sha256')==sha256(paths['deployment']) and
                    row.get('runtime_id')==b.runtime_id.hex() and row.get('source')=='local' and
                    type(row.get('logical_us')) is int):policy_seen=True
        elif row.get('record_kind')=='application':app_summary=row
    for label,s in (('runtime',runtime_summary),('application',app_summary)):
        require_complete_sdk_summary(s,label)
        if s['clock']['clock_run_id']!=b.run_id.hex() or s['clock']['clock_domain']!='simulation':
            raise ValueError('SDK clock/run mismatch')
        if s.get('config_id')!=b.config_digest.hex():raise ValueError('SDK config digest mismatch')
        if label=='application' and s.get('application_config_id')!=b.application_digest.hex():
            raise ValueError('SDK application digest mismatch')
    if next_sample<end:raise ValueError('incomplete numeric joint metric horizon')
    if not joint_n or not velocities or not policy_seen:raise ValueError('missing joint/intent/actual-policy records')
    report['metrics']['saturation_limit_provenance']=m['configured_limit_provenance']
    report['metrics']['configured_torque_limits_Nm']=limits
    report['metrics'].update(joint_rmse_rad=math.sqrt(joint_sum/joint_n),
        torque_saturation_fraction=saturated/torque_n,joint_metric_values=joint_n,intent_samples=velocities,
        baseline_command_application_control_cycles=1)
    finite(report)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();report=evaluate(load(args.manifest))
    result=json.dumps(report,indent=2,allow_nan=False)
    if args.output:args.output.write_text(result+'\n')
    print(result)
    return 0 if report['status']=='pass' else 1

if __name__=='__main__':raise SystemExit(main())
