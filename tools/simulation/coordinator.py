"""Engine-independent lockstep state machine. No clocks, sockets or engine imports.

Only advance mutates state. Each event is consumed atomically; actions are ordered.
The runner owns host time and executes all actions outside this object.
"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import math

from . import protocol as p


@dataclass(frozen=True)
class Bootstrap:
    run_id: bytes
    runtime_id: bytes
    config_digest: bytes
    application_digest: bytes
    engine_digest: bytes
    physics_dt_us: int
    control_dt_us: int
    engine_config: dict
    startup_timeout_us: int = 120000000
    phase_timeout_us: int = 30000000
    max_duration_us: int = 3600000000


@dataclass(frozen=True)
class Event:
    kind: str
    peer: int = 0
    frame: p.Frame | None = None
    feedback: object = None
    reason: str = ''


@dataclass(frozen=True)
class Action:
    kind: str
    peer: int = 0
    frame: p.Frame | None = None
    torque: tuple = ()
    substeps: int = 0
    reason: str = ''


class SimulationCoordinator:
    def __init__(self, bootstrap: Bootstrap):
        self.b = bootstrap
        if (type(bootstrap.physics_dt_us) is not int or type(bootstrap.control_dt_us) is not int
                or not 0 < bootstrap.physics_dt_us <= bootstrap.control_dt_us < 2**64
                or bootstrap.control_dt_us % bootstrap.physics_dt_us
                or not 0 < bootstrap.max_duration_us <= 3600000000
                or bootstrap.control_dt_us > bootstrap.max_duration_us
                or not 0 < bootstrap.phase_timeout_us < 2**63
                or not 0 < bootstrap.startup_timeout_us < 2**63):
            raise ValueError('invalid bootstrap time bounds')
        for name, n in (('run_id',16), ('runtime_id',16), ('config_digest',32),
                        ('application_digest',32), ('engine_digest',32)):
            value = getattr(bootstrap,name)
            if not isinstance(value,bytes) or len(value)!=n or not any(value):
                raise ValueError('invalid bootstrap '+name)
        self.hello = {name:getattr(bootstrap,name) for name in (
            'runtime_id','config_digest','application_digest','engine_digest',
            'physics_dt_us','control_dt_us')}
        self.hello['role']=1
        self.state='Handshake'
        self.step=0
        self.phase=0
        self.deadline=None
        self.host=-1
        self.started=False
        self.peers=set()
        self.feedback=None
        self.torque=None
        self.last_write_id=0
        self.inventory=[]
        self.first_commands=[]
        self.seal=None
        self.permitted=False
        self.recent=deque(maxlen=32)  # Completed phase caches, not just 32 frames.
        self.seen=set()
        self.failure=''
        # Metadata evidence, independent of replay/dedup phase cache rotation.
        self.states=deque(maxlen=8192)  # Runtime's maximum source-state window.
        self.candidates=deque(maxlen=32)  # <=128 targets per completed app phase.
        self.current_target=None  # Pinned (command, actual source state) pair.

    def frame(self, name, payload, phase=None):
        return p.Frame(p.KINDS.index(name)+1,payload,self.b.run_id,self.step,
                       self.phase if phase is None else phase,
                       1+self.step*self.b.control_dt_us,
                       self.b.physics_dt_us,self.b.control_dt_us)

    def send(self, peer, name, payload, phase=None):
        return Action('send',peer,self.frame(name,payload,phase))

    def fail(self, reason):
        self.state='Failed'
        self.failure=reason
        return tuple(self.send(peer,'Abort',dict(code=1,reason=reason.encode()[:256]))
                     for peer in sorted(self.peers))+(Action('cancel',reason=reason),)

    def enter(self, state, phase):
        if self.seen:
            self.recent.append(self.seen)
        self.seen=set()
        self.state=state
        self.phase=phase
        self.deadline=self.host+self.b.phase_timeout_us
        self.inventory=[]
        self.first_commands=[]
        self.seal=None
        self.permitted=False

    def tick(self):
        self.enter('RuntimeTick',1)
        self.torque=None
        payload=asdict(self.feedback)
        payload.update(valid=True,imu_valid=True)
        return (self.send(2,'Feedback',payload),
                self.send(2,'TickPermit',dict(role=2,stage=1)))

    def check_feedback(self, feedback, expected):
        if feedback.physics_step!=expected:
            raise ValueError('unexpected completed physics step')
        values=asdict(feedback)
        for name,value in values.items():
            if name=='physics_step': continue
            if not all(math.isfinite(x) for x in (value if isinstance(value,tuple) else (value,))):
                raise ValueError('nonfinite feedback')
        if not 0 < len(feedback.q) <= 64 or not len(feedback.q)==len(feedback.dq)==len(feedback.tau):
            raise ValueError('feedback joints')
        if any(len(getattr(feedback,k))!=n for k,n in (
                ('imu_quaternion',4),('imu_angular_velocity',3),('imu_specific_force',3),
                ('base_position',3),('base_velocity',6))):
            raise ValueError('feedback shape')
        if expected==0 and feedback.engine_time_s!=0:
            raise ValueError('initial engine time')
        if self.feedback is not None and feedback.engine_time_s<=self.feedback.engine_time_s:
            raise ValueError('engine time did not advance')
        if abs(sum(x*x for x in feedback.imu_quaternion)-1)>1e-6:
            raise ValueError('invalid IMU quaternion')
        if self.feedback is not None:
            count=expected-self.feedback.physics_step
            dt=self.b.physics_dt_us/1000000
            upper=self.feedback.engine_time_s+count*dt
            ulp=2.0**max(-149, math.frexp(upper)[1]-24)
            # Local handoff bound: each float32 addition rounds by <= half ULP.
            # This is only a sanity diagnostic, never execution evidence.
            # The adapter must verify device completion at every physical substep.
            tolerance=(count+2)*ulp + count*dt*2**-23
            if abs(feedback.engine_time_s-upper)>tolerance:
                raise ValueError('engine time inconsistent with physics handoff')
        # EngineAdapter validates incremental float32 accumulation at each substep.
        # Integer physics_step is authoritative, engine time is diagnostic only.

    def source_state(self, command):
        for state in reversed(self.states):
            if state['sequence']==command['source_state_seq']:
                return state
        if self.current_target and self.current_target[1]['sequence']==command['source_state_seq']:
            return self.current_target[1]
        return None

    def commit_ingress(self):
        # A request's nested first command is not a separate DDS sample.
        requests=[m for m in self.inventory if m['kind']==1 and m['operation']==3]
        if len(requests)!=len(self.first_commands):
            raise ValueError('Tracking request requires exact first-command proof')
        targets=[(m,self.source_state(m)) for m in self.inventory if m['kind']==5]
        for proof in self.first_commands:
            request,command=proof['request'],proof['command']
            if request not in requests:
                raise ValueError('first-command proof is not in sealed request inventory')
            if (request['runtime_id']!=command['runtime_id'] or
                    request['session_id']!=command['session_id'] or
                    request['producer_id']!=command['producer_id'] or
                    request['proposed_epoch']!=command['epoch']):
                raise ValueError('nested first-command context mismatch')
            source=self.source_state(command)
            if source is None:
                raise ValueError('nested first-command source state was never observed')
            targets.append((command,source))
        self.candidates.append(tuple(targets))

    def bind_tick_target(self, state):
        command=self.torque['command']
        if state['output_action']!=2:
            # Exact local sentinel, not Identify(default JointCommand): no DDS
            # sample/content digest exists for a tick with no active output.
            empty={name:(bytes(16) if field=='Id' else bytes(32) if field=='Digest' else 0)
                   for field,name in p.SCHEMA['MessageIdentity']}
            empty.update(kind=5,runtime_id=self.b.runtime_id)
            if command!=empty:
                raise ValueError('non-Tracking output must name exact no-active-command metadata')
            self.current_target=None
            # Revoked targets cannot be resurrected from an old evidence cache.
            self.candidates.clear()
            return
        if state['execution_state']!=3 or command['kind']!=5:
            raise ValueError('Tracking output requires an actual Command target')
        evidence=[self.current_target] if self.current_target else []
        evidence.extend(target for phase in self.candidates for target in phase)
        target=next((target for target in evidence if target[0]==command),None)
        if target is None or target[1] is None:
            raise ValueError('tick target has no completed ingress/source-state provenance')
        if (any(state[k]!=command[k] for k in ('runtime_id','session_id','producer_id','epoch'))
                or state['source_state_seq']!=command['sequence']):
            raise ValueError('tick state and actual current_target metadata disagree')
        self.current_target=target

    def advance(self, event: Event, host_now_us: int) -> tuple[Action,...]:
        if self.state in ('Failed','Closed'):
            return ()
        if type(host_now_us) is not int or host_now_us < self.host:
            return self.fail('host clock reversed')
        self.host=host_now_us
        if self.deadline is not None and self.host>=self.deadline:
            return self.fail('host phase deadline')
        try:
            # Actions are caller-owned, including every nested payload dictionary.
            return deepcopy(self._advance(event))
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
            return self.fail(str(exc))

    def _advance(self, e):
        if e.kind=='start':
            if self.started: raise ValueError('duplicate start')
            self.started=True
            self.deadline=self.host+self.b.startup_timeout_us
            return (Action('initialize'),)
        if not self.started: raise ValueError('not started')
        if e.kind=='poll': return ()
        if e.kind in ('disconnect','engine_error','stop'):
            # Task 8 will add guarded stop/leave proofs. Fail closed today; never
            # claim ZeroTorque, physical stop, release or a successful GPU cancel.
            self.state='Stopping'
            return self.fail(e.reason or e.kind)
        if e.kind=='feedback':
            if self.state=='Handshake':
                if self.feedback is not None: raise ValueError('duplicate initial feedback')
                self.check_feedback(e.feedback,0)
                self.feedback=e.feedback
                return self.tick() if self.peers=={2,3} else ()
            if self.state!='PhysicsAdvance': raise ValueError('unsolicited feedback')
            expected=(self.step+1)*(self.b.control_dt_us//self.b.physics_dt_us)
            self.check_feedback(e.feedback,expected)
            applied=self.send(2,'AppliedToStep',dict(write_id=self.torque['write_id'],
                physics_step_begin=self.step*(self.b.control_dt_us//self.b.physics_dt_us),
                physics_step_end=expected))
            self.feedback=e.feedback
            self.step+=1
            return (applied,)+self.tick()
        if e.kind!='frame' or e.peer not in (2,3): raise ValueError('event or role')
        f=e.frame
        wire=p.encode(f)
        if (f.run_id!=self.b.run_id or f.physics_dt_us!=self.b.physics_dt_us
                or f.control_dt_us!=self.b.control_dt_us): raise ValueError('run or dt')
        key=(e.peer,hashlib.sha256(wire).digest())
        if key in self.seen or any(key in phase for phase in self.recent): return ()
        if (f.control_step!=self.step or f.logical_us!=1+self.step*self.b.control_dt_us
                or f.phase!=self.phase): raise ValueError('wrong phase')
        if len(self.seen)>=(134 if self.phase==2 else 70): raise ValueError('phase frame capacity')
        self.seen.add(key)
        # Detach caller-owned dictionaries before retaining protocol facts.
        f=p.decode(wire)
        v=f.payload
        name=p.KINDS[f.kind-1]
        actions=()
        if name=='Hello':
            if self.state!='Handshake' or e.peer in self.peers or v['role']!=e.peer or not p.matches_hello(v,self.hello):
                raise ValueError('hello identity')
            self.peers.add(e.peer)
            actions=(self.send(e.peer,'Hello',self.hello),)
            if self.peers=={2,3} and self.feedback is not None: actions+=self.tick()
        elif e.peer not in self.peers: raise ValueError('peer not ready')
        elif name=='Abort': raise ValueError('peer aborted')
        elif name=='Torque':
            if self.state!='RuntimeTick' or e.peer!=2 or self.torque is not None or not v['accepted']:
                raise ValueError('unexpected torque')
            if (len(v['tau'])!=len(self.feedback.q) or v['write_id']<=self.last_write_id
                    or v['command']['runtime_id']!=self.b.runtime_id):
                raise ValueError('torque identity or shape')
            self.torque=v
            self.last_write_id=v['write_id']
        elif name=='FirstCommandProof':
            if e.peer!=3 or self.phase!=2 or self.state!='ApplicationPhase' or self.seal is not None:
                raise ValueError('first-command proof outside application inventory')
            if len(self.first_commands)==64 or any(proof['request']==v['request'] for proof in self.first_commands):
                raise ValueError('conflicting or excess first-command proof')
            self.first_commands.append(v)
        elif name in ('Manifest','PhaseSeal','InventoryReady','PhaseAck'):
            if self.state not in ('RuntimeTick','ApplicationPhase','IngressWait'):
                raise ValueError('inventory outside phase')
            sender=2 if self.phase==1 else 3
            consumer=5-sender
            if name=='Manifest':
                if e.peer!=sender or v['sender']!=sender or self.seal is not None:
                    raise ValueError('manifest role or sealed')
                m=v['message']
                if m['runtime_id']!=self.b.runtime_id or m['kind'] not in ((2,3) if sender==2 else (1,4,5)):
                    raise ValueError('manifest identity or kind')
                projection={k:x for k,x in m.items() if k!='content_digest'}
                if any({k:x for k,x in old.items() if k!='content_digest'}==projection for old in self.inventory):
                    raise ValueError('conflicting manifest')
                if len(self.inventory)==64: raise ValueError('manifest capacity')
                self.inventory.append(m)
                actions=(Action('send',consumer,f),)
            elif name=='PhaseSeal':
                digest=hashlib.sha256(b''.join(p._pack('Manifest',dict(sender=sender,message=m))
                                             for m in self.inventory)).digest()
                if e.peer!=sender or v!=dict(sender=sender,manifest_count=len(self.inventory),manifest_digest=digest) or self.seal is not None:
                    raise ValueError('seal mismatch')
                self.seal=v
                if self.phase==2: self.state='IngressWait'
                actions=(Action('send',consumer,f),)
            else:
                if e.peer!=consumer or v['consumer']!=consumer or self.seal is None or v['seal']!=self.seal:
                    raise ValueError('consumer seal mismatch')
                if name=='InventoryReady':
                    if self.permitted: raise ValueError('conflicting readiness')
                    self.permitted=True
                    actions=(self.send(sender,'PublishPermit',dict(seal=self.seal)),)
                    if self.phase==1:
                        self.state='ApplicationPhase'
                        actions+=(self.send(3,'TickPermit',dict(role=3,stage=2)),)
                else:
                    if not self.permitted or v['disposition']!=(2 if sender==2 else 1):
                        raise ValueError('completion before install or wrong disposition')
                    if self.phase==1:
                        if self.torque is None: raise ValueError('missing tick torque')
                        states=[m for m in self.inventory if m['kind']==3]
                        if (len(states)!=1 or states[0]['sequence']!=self.torque['state_seq']
                                or states[0]['stamp_us']!=1+self.step*self.b.control_dt_us):
                            raise ValueError('tick state does not match torque')
                        self.bind_tick_target(states[0])
                        self.states.append(states[0])
                        self.enter('ApplicationPhase',2)
                        actions=()
                    else:
                        next_us=(self.step+1)*self.b.control_dt_us
                        if next_us>self.b.max_duration_us or next_us>=2**64-1:
                            raise ValueError('logical duration or overflow')
                        self.commit_ingress()
                        self.enter('PhysicsAdvance',3)
                        actions=(Action('step',torque=tuple(self.torque['tau']),
                            substeps=self.b.control_dt_us//self.b.physics_dt_us),)
        else: raise ValueError('unsupported control frame')
        return actions
