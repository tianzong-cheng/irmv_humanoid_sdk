"""Task9 real GPU semantics and real-policy lifecycle. No skip/CPU fallback."""
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))


def semantics():
    from simulation.mujoco_warp_engine import MujocoWarpEngine
    engine=MujocoWarpEngine()
    angle=.4
    quat=[math.cos(angle/2),math.sin(angle/2),0,0]
    cfg=dict(model_path=str(ROOT/'tests/support/simulation/free_base.xml'),
        joint_names=['joint'],actuator_names=['torque'],base_body='base',weld_name='support',imu_site='imu',
        initial_base_position=[0,0,1.2],initial_base_quaternion=quat,initial_joint_position=[.3])
    try:
        engine.initialize(cfg)
        first=engine.read_feedback()
        assert abs(first.q[0]-.3)<1e-6,('configured initial q ignored',first)
        assert math.dist(first.base_position,[0,0,1.2])<1e-6,first
        assert math.dist(first.imu_quaternion,quat)<1e-6,first
        held=engine.step(20)
        expected=[0,9.81*math.sin(angle),9.81*math.cos(angle)]
        assert math.dist(held.imu_specific_force,expected)<.03,(held,expected)
        engine.apply_torque([.01]);driven=engine.step(1)
        assert 9.8<(driven.dq[0]-held.dq[0])/.001<10.2
        assert abs(driven.tau[0]-.01)<1e-6
        engine.apply_torque([0])
        before=engine.read_feedback();engine.release_base();after=engine.read_feedback()
        assert before==after,'release must preserve all feedback exactly'
        fallen=engine.step(20)
        assert max(abs(x) for x in fallen.imu_specific_force)<.03,fallen
        axis_checks={}
        for axis in (1,2):
            engine.close()
            rotated=[math.cos(angle/2),0,0,0];rotated[axis+1]=math.sin(angle/2)
            cfg['initial_base_quaternion']=rotated
            engine.initialize(cfg)
            check=engine.step(20)
            w,x,y,z=rotated
            gravity=[2*(x*z-w*y)*9.81,2*(y*z+w*x)*9.81,(1-2*(x*x+y*y))*9.81]
            assert math.dist(check.imu_quaternion,rotated)<1e-4,check
            assert math.dist(check.imu_specific_force,gravity)<.03,(check,gravity)
            axis_checks[str(axis)]=dict(quaternion=check.imu_quaternion,specific_force=check.imu_specific_force)
        # Known body angular velocity at startup; only GPU forward computes sensors.
        engine.close()
        cfg['initial_base_velocity']=[0,0,0,.2,.3,.4]
        engine.initialize(cfg)
        moving=engine.read_feedback()
        assert math.dist(moving.imu_angular_velocity,[.2,.3,.4])<1e-6,moving
        return dict(rotated_specific_force=held.imu_specific_force,gyro=moving.imu_angular_velocity,
                    tau_est=driven.tau,release_continuous=True,other_axes=axis_checks)
    finally:engine.close()


def check_locomotion_rejection(output, *, attempted, released):
    """Fail on observed admission/command protection, not after the host deadline."""
    if attempted and not released:
        for line in output.splitlines():
            if 'reject=Discontinuity' in line or 'stop=CommandTimeout' in line:
                raise AssertionError('Locomotion protection: '+line)


def lifecycle(build,config):
    import json,subprocess,time
    from sim_launch import Experiment
    from simulation.mujoco_warp_engine import MujocoWarpEngine
    config=Path(config).resolve()
    cfg=json.loads(config.read_text())
    assert cfg['dds']['domain']==225,'GPU test requires reserved serial domain225'
    engines=[]
    class ObservedEngine(MujocoWarpEngine):
        def initialize(self,c):
            self.releases=[];self.samples=[];self.closes=0;self.tracking_diagnostic=[]
            super().initialize(c);engines.append(self)
        def release_base(self):
            before=self.read_feedback();super().release_base()
            after=self.read_feedback()
            assert before==after,'real robot release discontinuity'
            self.releases.append(before)
        def step(self,n):
            value=super().step(n);self.samples.append(value)
            if owner.runner.coordinator.states and owner.runner.coordinator.states[-1]['execution_state']==3:
                targets=cfg['application']['initialization']['target_position']
                names=[j['name'] for j in cfg['robot']['joints']]
                self.tracking_diagnostic.append(dict(physics_step=value.physics_step,q=value.q,dq=value.dq,
                    max_target_error=max(abs(q-targets[name]) for name,q in zip(names,value.q)),
                    max_abs_dq=max(abs(v) for v in value.dq),tau=value.tau))
            return value
        def close(self):
            self.closes+=1;super().close()
    owner=Experiment(config,build/'robot_runtime',build/'application',
        engine_factory=ObservedEngine,application_stdin=subprocess.PIPE)
    seen=set();sent=set();released_step=None;runtime_only_step=None;admission=None;admissible_since=None
    try:
        owner.start()
        deadline=time.monotonic()+420
        while time.monotonic()<deadline:
            c=owner.runner.coordinator
            assert owner.result is None,(c.state,c.failure)
            seen.update(s['execution_state'] for s in c.states)
            output=(owner.directory/'application.log').read_text()
            f=c.feedback
            names=[j['name'] for j in cfg['robot']['joints']]
            init=cfg['application']['initialization']
            runnable=all(
                joint['position_min']<=q<=joint['position_max']
                and abs(dq)<=.8*cfg['runtime']['continuity']['velocity_delta'][name]
                and abs((init['gains'][name]['kd']-cfg['runtime']['damping']['kd'][name])*dq)<=.8*cfg['runtime']['continuity']['torque_delta'][name]
                and abs(init['target_position'][name]-q)*math.pi/(2*init['trajectory']['duration_us']/1e6)<=.8*cfg['runtime']['safety']['target_velocity_limits'][name]
                for name,joint,q,dq in zip(names,cfg['robot']['joints'],f.q,f.dq))
            if runnable:
                if admissible_since is None:admissible_since=c.step
            else:admissible_since=None
            if b'i' not in sent and c.step*c.b.control_dt_us>2000000:
                raise AssertionError(('initialization never admissible within 2 logical seconds',c.step,f))
            for condition,key in [("behavior=Standby",b'd'),("behavior=Damping",b'i'),("behavior=Ready",b'l')]:
                if condition in output and key not in sent:
                    if key==b'i' and (admissible_since is None or (c.step-admissible_since)*c.b.control_dt_us<20000):continue
                    if key==b'i':
                        f=c.feedback
                        admission=dict(step=c.step,q=f.q,dq=f.dq,
                            continuity_velocity_limit=cfg['runtime']['continuity']['velocity_delta'],
                            position_limits=cfg['robot']['joints'])
                        print('initialization admission',json.dumps(admission),flush=True)
                    owner.children[1].stdin.write(key);owner.children[1].stdin.flush();sent.add(key)
            check_locomotion_rejection(output,attempted=b'l' in sent,
                released=c.release_state=='Released')
            if 'behavior=Fault' in output:
                raise AssertionError(('actual application Fault',c.step,c.state,c.phase,
                    engines[0].tracking_diagnostic[-1] if engines[0].tracking_diagnostic else None,output))
            if 'init=refused' in output:
                raise AssertionError(('initialization refused; no retry or limit change',admission,output))
            if c.release_state=='Released':
                if released_step is None:released_step=c.step
                if c.step>=released_step+100 and b'x' not in sent:
                    owner.children[1].stdin.write(b'x');owner.children[1].stdin.flush();sent.add(b'x')
            if c.runtime_only:
                assert owner.children[1].wait(timeout=3)==0,output
                assert owner.children[0].poll() is None
                assert c.states[-1]['execution_state']==2,c.states[-1]
                if runtime_only_step is None:runtime_only_step=c.step
                if c.step>=runtime_only_step+10:break
            if owner.children[1].poll() is not None and not c.runtime_only:
                raise AssertionError(('app failed',output,c.failure))
            time.sleep(.002)
        else:raise AssertionError(('GPU lifecycle host deadline',sent,c.step,output))
        assert {0,1,2,3}<=seen,seen
        assert b'l' in sent and 'behavior=Ready' in output,output
        assert len(engines[0].releases)==1
        assert c.release_event_drops==0
        owner.notify_shutdown()
        assert owner.wait()==0,(c.failure,(owner.directory/'runtime.log').read_text(),output)
        engine=engines[0]
        assert engine.closes==1 and c.stop_proven
        samples=engine.samples
        metrics=dict(min_base_height_m=min(v.base_position[2] for v in samples),
            max_abs_dq_rad_s=max(abs(x) for v in samples for x in v.dq),
            max_abs_tau_est_Nm=max(abs(x) for v in samples for x in v.tau),
            completed_physics_steps=samples[-1].physics_step,
            released_control_step=released_step,policy_rollout_us=100*c.b.control_dt_us,
            numerical_tracking_acceptance='reported metrics only; Task11 owns reliability')
        owner.evidence.update(real_policy_lifecycle=True,metrics=metrics,runtime_states_seen=sorted(seen),
            behavior_ready_observed=True,real_robot_release_continuity=True)
        print('PASS real policy lifecycle',json.dumps(metrics))
    finally:
        owner.evidence.update(initialization_admission=admission,runtime_states_seen=sorted(seen),
            final_control_step=owner.runner.coordinator.step if owner.runner else None,
            final_coordinator_phase=owner.runner.coordinator.phase if owner.runner else None,
            final_coordinator_state=owner.runner.coordinator.state if owner.runner else None,
            tracking_diagnostic=engines[0].tracking_diagnostic[-5:] if engines else [])
        if engines and engines[0].samples:
            f=engines[0].samples[-1]
            owner.evidence['last_feedback']=dict(physics_step=f.physics_step,q=f.q,dq=f.dq,base_position=f.base_position)
        owner.close()

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('build',type=Path)
    parser.add_argument('--config',required=True)
    parser.add_argument('--semantics-only',action='store_true')
    parser.add_argument('--lifecycle-only',action='store_true')
    args=parser.parse_args()
    if not args.lifecycle_only:print('PASS GPU semantics',semantics())
    if not args.semantics_only:
        if not args.config:raise ValueError('explicit prepared real-policy config required')
        lifecycle(args.build.resolve(),args.config)
