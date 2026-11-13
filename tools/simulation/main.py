"""Local coordinator runner. Invoke: python3 -m tools.simulation.main --help.

No SDK processes are launched here. The normalized bootstrap is produced before
process startup, with independent fresh run/runtime IDs. Configuration canonical
hashing belongs to the C++ loader, not this module.
"""
import argparse
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
        self.engine_gate=threading.Lock()
        self.release_cancel=threading.Event()
        self.step_started=False
        self.jobs=queue.Queue(maxsize=32)
        self.results=queue.Queue(maxsize=32)
        self.worker=None
        self.selector=selectors.DefaultSelector()
        self.connections={}
        self.peers={}
        self.listener=None
        self.directory=None
        self.socket_inode=None

    def request_stop(self):
        self.stop.set()

    def request_shutdown(self):
        # Independent owner capability. Application never signals runtime.
        with self.engine_gate:
            self.shutdown.set()

    def _engine_worker(self):
        engine=None
        try:
            while not self.stop.is_set():
                try: action=self.jobs.get(timeout=.01)
                except queue.Empty: continue
                released=False
                if action.kind=='initialize':
                    engine=self.engine_factory()
                    engine.initialize(self.coordinator.b.engine_config)
                    if abs(engine.timestep_s*1000000-self.coordinator.b.physics_dt_us)>1e-6:
                        raise ValueError('engine timestep differs from bootstrap')
                    feedback=engine.read_feedback()
                elif action.kind=='step':
                    with self.engine_gate:
                        cancelled=self.shutdown.is_set() or self.stop.is_set()
                        release=action.release and not self.release_cancel.is_set()
                        if not cancelled: self.step_started=True
                    if cancelled:
                        self.results.put_nowait(Event('cancelled'))
                        continue
                    # Passing the gate commits this worker operation. Cancellation
                    # cannot undo an already executing GPU call or release.
                    if release:
                        engine.release_base()
                        released=True
                    engine.apply_torque(action.torque)
                    feedback=engine.step(action.substeps)
                elif action.kind=='stop_output':
                    engine.apply_torque(action.torque)
                    # Completion includes resource close, not just zero-output submission.
                    engine.close()
                    engine=None
                    self.results.put_nowait(Event('stopped'))
                    continue
                else: raise ValueError('invalid engine action')
                if not self.stop.is_set():
                    self.results.put_nowait(Event('feedback',feedback=feedback,released=released))
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

    def _flush(self,conn):
        outgoing=self.connections[conn]['out']
        while outgoing:
            try:n=conn.send(outgoing[0])
            except BlockingIOError:break
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
        self._actions(self.coordinator.advance(event,host_us()))

    def run(self):
        try:
            self._listen()
            self.worker=threading.Thread(target=self._engine_worker,daemon=True,name='simulation-engine')
            self.worker.start()
            self._advance(Event('start'))
            while self.coordinator.state not in ('Failed','Closed'):
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
                        if mask & selectors.EVENT_READ:
                            packet,_,flags,_=conn.recvmsg(p.MAX_PACKET_BYTES)
                            if not packet:
                                peer=self.connections[conn]['peer']
                                self.selector.unregister(conn)
                                del self.connections[conn]
                                self.peers.pop(peer,None)
                                conn.close()
                                self._advance(Event('disconnect',peer,reason='peer disconnected'))
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
                # The engine has acknowledged zero output. Keep the socket alive
                # until runtime consumes ShutdownCommitted and closes its own
                # driver; a fixed sleep could race endpoint delivery under load.
                end=host_us()+self.coordinator.b.phase_timeout_us
                runtime_closed=False
                while not runtime_closed and host_us()<end:
                    for key,mask in self.selector.select(.01):
                        conn=key.fileobj
                        if conn is self.listener: continue
                        if mask & selectors.EVENT_WRITE: self._flush(conn)
                        if mask & selectors.EVENT_READ:
                            packet=conn.recv(p.MAX_PACKET_BYTES)
                            if packet: raise ValueError('unexpected output after shutdown commit')
                            peer=self.connections[conn]['peer']
                            self.selector.unregister(conn)
                            del self.connections[conn]
                            self.peers.pop(peer,None)
                            conn.close()
                            if peer==2: runtime_closed=True
                            elif not self.coordinator.runtime_only:
                                raise ValueError('required application died before leave commit')
                if not runtime_closed:
                    self.coordinator.fail('stopped-unproven: runtime close deadline')
            return 0 if self.coordinator.state=='Closed' else 1
        except (OSError,ValueError,queue.Full) as exc:
            actions=self.coordinator.fail(str(exc))
            try:self._actions(actions)
            except (OSError,ValueError,queue.Full):pass
            return 1
        finally:
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
        signal.signal(sig,lambda *_:runner.request_shutdown())
    result=runner.run()
    if result:print('simulation failed: '+runner.coordinator.failure)
    return result


if __name__=='__main__':raise SystemExit(main())
