"""Local coordinator runner. Invoke: python3 -m tools.simulation.main --help.

No SDK processes are launched here. The normalized bootstrap is produced before
process startup, with independent fresh run/runtime IDs. Configuration canonical
hashing belongs to the C++ loader, not this module.
"""
import argparse
from dataclasses import asdict
import platform
import importlib.metadata
from collections import deque
import json
import os
from pathlib import Path
import queue
import selectors
import signal
import socket
import stat
import struct
import threading
import time

from . import protocol as p
from .coordinator import Bootstrap, Event, SimulationCoordinator
from .record import Writer


def host_us():
    return time.monotonic_ns()//1000


def load_bootstrap(path):
    data=json.loads(Path(path).read_text())
    for name in ('run_id','runtime_id','config_digest','application_digest','engine_digest'):
        data[name]=bytes.fromhex(data[name])
    b=Bootstrap(**data)
    SimulationCoordinator(b)  # Validate before binding sockets or touching an engine.
    return b


def warp_engine():
    # Selected only by main; fake tests never import optional dependencies.
    from .mujoco_warp_engine import MujocoWarpEngine
    return MujocoWarpEngine()


class Runner:
    def __init__(self, bootstrap, socket_path, engine_factory=warp_engine):
        self.coordinator=SimulationCoordinator(bootstrap)
        self.path=Path(socket_path).absolute()
        self.engine_factory=engine_factory
        self.stop=threading.Event()
        self.shutdown=threading.Event()
        self._shutdown_notified=False
        self.engine_gate=threading.Lock()
        self.release_cancel=threading.Event()
        self.step_started=False
        self.jobs=queue.Queue(maxsize=32)
        self.results=queue.Queue(maxsize=32)
        self.worker=None
        self.viewer=None
        self.selector=selectors.DefaultSelector()
        self.connections={}
        self.peers={}
        self.listener=None
        self.directory=None
        self.socket_inode=None
        b=bootstrap
        versions={}
        for name in ('mujoco','mujoco-warp','warp-lang'):
            try:versions[name]=importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:versions[name]='unavailable'
        self.record=Writer(self.path.parent/'simulation.jsonl',dict(
            clock='simulation',clock_domain='simulation',available=True,
            clock_run_id=b.run_id.hex(),logical_origin_us=1,
            physics_dt_us=b.physics_dt_us,control_dt_us=b.control_dt_us),provenance=dict(
                runtime_id=b.runtime_id.hex(),config_digest=b.config_digest.hex(),
                application_digest=b.application_digest.hex(),engine_digest=b.engine_digest.hex(),
                engine_config=b.engine_config,engine_factory=engine_factory.__module__+'.'+engine_factory.__qualname__,
                python=platform.python_version(),dependencies=versions))

    def request_stop(self):
        self.stop.set()

    def _notify_shutdown(self,signum,frame):
        # Signal context: assignment only, even if this thread owns engine_gate.
        self._shutdown_notified=True

    def request_shutdown(self):
        # Independent owner capability. Application never signals runtime.
        with self.engine_gate:
            self.shutdown.set()

    def _engine_worker(self):
        engine=None
        visual_released=False
        try:
            while not self.stop.is_set():
                try: action=self.jobs.get(timeout=.01)
                except queue.Empty: continue
                released=False
                operation_start=host_us()
                release_duration=0
                if action.kind=='initialize':
                    engine=self.engine_factory()
                    engine.initialize(self.coordinator.b.engine_config)
                    if abs(engine.timestep_s*1000000-self.coordinator.b.physics_dt_us)>1e-6:
                        raise ValueError('engine timestep differs from bootstrap')
                    feedback=engine.read_feedback()
                elif action.kind=='step':
                    with self.engine_gate:
                        cancelled=(self._shutdown_notified or
                                   self.shutdown.is_set() or self.stop.is_set())
                        release=action.release and not self.release_cancel.is_set()
                        if not cancelled: self.step_started=True
                    if cancelled:
                        self.results.put_nowait(Event('cancelled'))
                        continue
                    # Passing the gate commits this worker operation. Cancellation
                    # cannot undo an already executing GPU call or release.
                    if release:
                        release_start=host_us()
                        engine.release_base()
                        release_duration=host_us()-release_start
                        released=True
                    engine.apply_torque(action.torque)
                    feedback=engine.step(action.substeps)
                elif action.kind=='stop_output':
                    engine.apply_torque(action.torque)
                    # Completion includes resource close, not just zero-output submission.
                    engine.close()
                    engine=None
                    self.results.put_nowait(Event('stopped',host_operation_start_us=operation_start,host_operation_duration_us=host_us()-operation_start))
                    continue
                else: raise ValueError('invalid engine action')
                visual_released=visual_released or released
                if self.viewer is not None:
                    # One latest display slot; the independent sender paces UI.
                    try:
                        self.viewer.publish(engine.visual_qpos(),feedback.physics_step,visual_released)
                    except Exception as exc:
                        # A display failure cannot become a physics failure.
                        self.viewer.state='error'
                        self.viewer.error=str(exc)
                if not self.stop.is_set():
                    self.results.put_nowait(Event('feedback',feedback=feedback,released=released,host_operation_start_us=operation_start,host_operation_duration_us=host_us()-operation_start,host_release_duration_us=release_duration))
        except Exception as exc:
            if not self.stop.is_set():
                try:self.results.put_nowait(Event('engine_error',reason=str(exc)))
                except queue.Full:self.stop.set()
        finally:
            # Only this worker ever touches the engine, including close. If GPU
            # code is stuck this daemon can outlive run(); Python cannot kill it.
            if engine is not None:
                try:engine.close()
                except Exception:pass

    def _listen(self):
        self.directory=os.open(self.path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
        info=os.fstat(self.directory)
        if info.st_uid!=os.getuid() or stat.S_IMODE(info.st_mode)!=0o700:
            raise ValueError('socket parent must be same-user private0700')
        self.listener=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.listener.setblocking(False)
        # Pin the validated parent through the descriptor (no unlink on bind).
        self.listener.bind(f'/proc/self/fd/{self.directory}/{self.path.name}')
        self.socket_inode=os.stat(self.path.name,dir_fd=self.directory,follow_symlinks=False).st_ino
        self.listener.listen(2)
        self.selector.register(self.listener,selectors.EVENT_READ)

    def _interest(self,conn):
        self.selector.modify(conn,selectors.EVENT_READ | (
            selectors.EVENT_WRITE if self.connections[conn]['out'] else 0))

    def _retired_application(self,conn):
        return (self.coordinator.runtime_only and
                self.connections[conn]['peer']==3)

    def _receive_packet(self,conn):
        try:
            packet,_,flags,_=conn.recvmsg(p.MAX_PACKET_BYTES)
            return packet,flags
        except ConnectionResetError:
            if not self._retired_application(conn):raise
            # Membership was already revoked by a validated LeaveReady. Linux
            # reports reset rather than EOF when a killed peer had unread data.
            return b'',0

    def _disconnect(self,conn):
        peer=self.connections[conn]['peer']
        self.selector.unregister(conn)
        del self.connections[conn]
        self.peers.pop(peer,None)
        conn.close()
        self._advance(Event('disconnect',peer,reason='peer disconnected'))

    def _flush(self,conn):
        outgoing=self.connections[conn]['out']
        while outgoing:
            try:n=conn.send(outgoing[0])
            except BlockingIOError:break
            except (BrokenPipeError,ConnectionResetError):
                if not self._retired_application(conn):raise
                self._disconnect(conn)
                return
            if n!=len(outgoing[0]):raise ValueError('partial seqpacket send')
            outgoing.popleft()
        self._interest(conn)

    def _actions(self,actions):
        for a in actions:
            if a.kind in ('initialize','step','stop_output'):
                self.jobs.put_nowait(a)
            elif a.kind=='send':
                conn=self.peers.get(a.peer)
                if conn is None:raise ValueError('missing action peer')
                outgoing=self.connections[conn]['out']
                if len(outgoing)>=32:raise ValueError('socket output queue capacity')
                outgoing.append(p.encode(a.frame))
                self._flush(conn)
            elif a.kind=='cancel':self.stop.set()
            else:raise ValueError('unknown action')

    def _evidence(self,kind,frame=None,**facts):
        c=self.coordinator
        row=dict(kind=kind,control_step=c.step,phase=c.phase,
                 logical_us=1+c.step*c.b.control_dt_us,coordinator_state=c.state,
                 host_observed_us=host_us(),**facts)
        if frame:
            row.update(control_step=frame.control_step,phase=frame.phase,logical_us=frame.logical_us)
            row['frame']=dict(name=p.KINDS[frame.kind-1],payload=frame.payload)
        def encode(value):
            if isinstance(value,bytes):return value.hex()
            raise TypeError('unsupported evidence value')
        self.record.offer(json.loads(json.dumps(row,default=encode)))

    def _advance(self,event):
        if event.kind in ('feedback','cancelled'):
            with self.engine_gate:
                self.step_started=False
                self.release_cancel.clear()
        if event.frame and event.frame.kind==p.KINDS.index('ReleaseCancel')+1:
            with self.engine_gate:
                started=self.step_started
                if event.frame.control_step==self.coordinator.step and not started:
                    self.release_cancel.set()
            if started:
                from dataclasses import replace
                event=replace(event,reason='operation_started')
        old_torque=self.coordinator.torque
        actions=self.coordinator.advance(event,host_us())
        c=self.coordinator
        if event.kind!='poll':
            self._evidence('event_'+event.kind,event.frame,peer=event.peer,
                reason=event.reason,released=event.released,
                host_operation_start_us=event.host_operation_start_us,
                host_operation_duration_us=event.host_operation_duration_us,
                host_release_duration_us=event.host_release_duration_us,
                host_stop_duration_us=event.host_operation_duration_us if event.kind=='stopped' else 0,
                engine_time_s=event.feedback.engine_time_s if event.feedback else None,
                completed_physics_step=event.feedback.physics_step if event.feedback else None,
                feedback=asdict(event.feedback) if event.feedback else None,
                feedback_validated=event.feedback is not None and event.feedback is c.feedback,
                stop_proven=c.stop_proven,release_state=c.release_state,failure=c.failure)
        if event.frame and p.KINDS[event.frame.kind-1]=='Torque' and c.torque is not None and c.torque is not old_torque:
            self._evidence('write_accepted',event.frame)
        for action in actions:
            if action.frame and p.KINDS[action.frame.kind-1] in (
                    'AppliedToStep','ShutdownCommitted','LeaveCommitted'):
                self._evidence(p.KINDS[action.frame.kind-1],action.frame)
        self._actions(actions)

    def run(self):
        try:
            self.record.start()
            self._listen()
            self.worker=threading.Thread(target=self._engine_worker,daemon=True,name='simulation-engine')
            self.worker.start()
            self._advance(Event('start'))
            while self.coordinator.state not in ('Failed','Closed'):
                if self._shutdown_notified and not self.shutdown.is_set():
                    self.request_shutdown()
                if self.shutdown.is_set() and not self.coordinator.shutdown_pending:
                    self._advance(Event('stop',reason='independent experiment shutdown'))
                if self.stop.is_set():
                    self._actions(self.coordinator.fail('forced stop; stopped-unproven'))
                    break
                for _ in range(32):
                    try:event=self.results.get_nowait()
                    except queue.Empty:break
                    self._advance(event)
                    if self.coordinator.state in ('Failed','Closed'):break
                if self.coordinator.state in ('Failed','Closed'):break
                self._advance(Event('poll'))
                if self.coordinator.state in ('Failed','Closed'):break
                wait=max(0,min(.01,(self.coordinator.deadline-host_us())/1000000))
                for key,mask in self.selector.select(wait):
                    conn=key.fileobj
                    if conn is self.listener:
                        client,_=self.listener.accept();client.setblocking(False)
                        credentials=client.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12)
                        _,uid,_=struct.unpack('3i',credentials)
                        if uid!=os.getuid() or len(self.connections)>=2:
                            client.close();raise ValueError('peer uid or connection capacity')
                        self.connections[client]={'peer':0,'out':deque()}
                        self.selector.register(client,selectors.EVENT_READ)
                    else:
                        if mask & selectors.EVENT_WRITE:self._flush(conn)
                        if conn not in self.connections:continue
                        if mask & selectors.EVENT_READ:
                            packet,flags=self._receive_packet(conn)
                            if not packet:
                                self._disconnect(conn)
                                continue
                            if flags & socket.MSG_TRUNC:raise ValueError('oversized packet')
                            f=p.decode(packet)
                            entry=self.connections[conn]
                            if not entry['peer']:
                                if f.kind!=1 or f.payload['role'] not in (2,3):
                                    raise ValueError('first frame must identify SDK role')
                                role=f.payload['role']
                                if role in self.peers:raise ValueError('duplicate peer role')
                                entry['peer']=role;self.peers[role]=conn
                            self._advance(Event('frame',entry['peer'],f))
                    if self.coordinator.state in ('Failed','Closed'):break
            if self.coordinator.state=='Closed':
                # Keep both authenticated endpoints alive until their owners
                # consume the proof and close. Never use a fixed delay as proof.
                end=host_us()+self.coordinator.b.phase_timeout_us
                required={2} if self.coordinator.runtime_only else {2,3}
                while required and host_us()<end:
                    for key,mask in self.selector.select(.01):
                        conn=key.fileobj
                        if conn is self.listener:continue
                        if mask & selectors.EVENT_WRITE:self._flush(conn)
                        if conn not in self.connections:continue
                        if mask & selectors.EVENT_READ:
                            packet,_=self._receive_packet(conn)
                            if packet:
                                f=p.decode(packet)
                                # Normal-phase output already in flight when
                                # cancellation arrived is never a stop proof.
                                if (f.run_id!=self.coordinator.b.run_id or
                                    f.control_step>self.coordinator.step or f.phase>=4):
                                    raise ValueError('unexpected output after shutdown commit')
                                continue
                            peer=self.connections[conn]['peer']
                            if self.connections[conn]['out']:
                                raise ValueError('peer disconnected before proof delivery')
                            self.selector.unregister(conn)
                            del self.connections[conn]
                            self.peers.pop(peer,None)
                            conn.close()
                            required.discard(peer)
                if required:
                    self.coordinator.fail('stopped-unproven: SDK close deadline')
            return 0 if self.coordinator.state=='Closed' else 1
        except (OSError,ValueError,queue.Full) as exc:
            actions=self.coordinator.fail(str(exc))
            try:self._actions(actions)
            except (OSError,ValueError,queue.Full):pass
            return 1
        finally:
            self._evidence('runner_final',stop_proven=self.coordinator.stop_proven,
                           failure=self.coordinator.failure)
            self.record.close()
            self.stop.set()
            # Bounded logical cancellation, never a physical-stop assertion.
            if self.worker is not None:self.worker.join(.05)
            for conn in self.connections:conn.close()
            if self.listener is not None:self.listener.close()
            self.selector.close()
            if self.directory is not None:
                try:
                    info=os.stat(self.path.name,dir_fd=self.directory,follow_symlinks=False)
                    if info.st_ino==self.socket_inode and stat.S_ISSOCK(info.st_mode):
                        os.unlink(self.path.name,dir_fd=self.directory)
                except FileNotFoundError:pass
                finally:os.close(self.directory)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bootstrap',required=True,help='normalized JSON from independent launcher')
    parser.add_argument('--socket',required=True,help='path inside existing private0700 directory')
    args=parser.parse_args()
    runner=Runner(load_bootstrap(args.bootstrap),args.socket)
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,runner._notify_shutdown)
    result=runner.run()
    if result:print('simulation failed: '+runner.coordinator.failure)
    return result


if __name__=='__main__':raise SystemExit(main())
