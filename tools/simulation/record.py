"""Version 1 simulation evidence. Host monotonic durations are never logical time.

Single coordinator producer, single file worker; bounded offers may lose facts.
Missing footer, sequence holes or any loss cannot prove a complete lifecycle.
"""
import json
import math
from pathlib import Path
import queue
import threading

MAX_LINE=65536
SCHEMA='irmv.simulation_evidence'


class Writer:
    def __init__(self,path,clock,capacity=256,provenance=None):
        self.path=Path(path)
        self.header=dict(schema=SCHEMA,version=1,record_type='header',clock=clock,
            provenance=provenance or {},host_clock='linux_clock_monotonic',
            host_origin='kernel boot; host_*_duration_us are elapsed host time',
            logical_origin='logical_us = 1 + control_step * control_dt_us; phase4 freezes time',
            physics_origin='physics_step starts at 0; completion requires worker feedback',
            acceptance='write_accepted is host handoff; AppliedToStep is completed engine evidence',
            feedback_units=dict(q='rad',dq='rad/s',tau='Nm actuator generalized force',
                imu_quaternion='wxyz IMU site to world',imu_angular_velocity='rad/s local IMU site',
                imu_specific_force='m/s^2 local IMU site; includes gravity at rest',
                base_position='m world',base_velocity='free-joint qvel: world linear m/s, local angular rad/s',
                base_quaternion='wxyz base to world; empty means unavailable',engine_time_s='engine seconds diagnostic'))
        self.queue=queue.Queue(maxsize=capacity)
        self.done=threading.Event()
        self.worker=None
        self.sequence=0
        self.dropped=0
        self.write_failures=0
        self.written=0
        self.finished=False

    def offer(self,row):
        self.sequence+=1
        row=dict(row,record_type='event',sequence=self.sequence)
        try:
            line=json.dumps(row,separators=(',',':'),allow_nan=False)
            if len(line.encode())>MAX_LINE:raise ValueError('oversized evidence')
            if self.done.is_set():raise ValueError('closed writer')
            self.queue.put_nowait(line)
            return True
        except (ValueError,TypeError,queue.Full):
            self.dropped+=1
            return False

    def start(self):
        self.worker=threading.Thread(target=self._run,name='simulation-record',daemon=True)
        self.worker.start()

    def _run(self):
        try:
            with self.path.open('x') as f:
                f.write(json.dumps(self.header,separators=(',',':'))+'\n')
                while not self.done.is_set() or not self.queue.empty():
                    try:line=self.queue.get(timeout=.01)
                    except queue.Empty:continue
                    try:
                        f.write(line+'\n');f.flush()
                        self.written+=1
                    except OSError:self.write_failures+=1
                f.write(json.dumps(dict(record_type='footer',offered=self.sequence,
                    written=self.written,dropped=self.dropped,write_failures=self.write_failures))+'\n')
                f.flush()
            self.finished=True
        except (OSError,ValueError):self.write_failures+=1

    def close(self):
        self.done.set()
        if self.worker:self.worker.join(.5)
        return self.finished


def read(path, visitor=None):
    """Bounded line validation, streaming when visitor supplied; never infer success."""
    rows=[]
    header=None
    footer=None
    last=0
    holes=0
    count=0
    measured_complete=True
    measured_count=0
    final={}
    with Path(path).open('rb') as f:
        while True:
            raw=f.readline(MAX_LINE+2)
            if not raw:break
            if len(raw)>MAX_LINE+1:raise ValueError('oversized simulation evidence line')
            if not raw.endswith(b'\n'):break  # Interrupted tail: footer remains absent.
            try:row=json.loads(raw)
            except (ValueError,UnicodeError) as e:raise ValueError('invalid simulation evidence JSON') from e
            if not isinstance(row,dict):raise ValueError('simulation evidence object required')
            if header is None:
                if row.get('schema')!=SCHEMA or row.get('version')!=1 or row.get('record_type')!='header':
                    raise ValueError('unknown simulation evidence schema/version')
                c=row.get('clock',{})
                run=c.get('clock_run_id','')
                if (c.get('clock_domain')!='simulation' or c.get('clock')!='simulation'
                        or not c.get('available') or len(run)!=32
                        or any(ch not in '0123456789abcdef' for ch in run) or set(run)=={'0'}
                        or c.get('logical_origin_us')!=1
                        or type(c.get('physics_dt_us')) is not int or c['physics_dt_us']<=0
                        or type(c.get('control_dt_us')) is not int or c['control_dt_us']<c['physics_dt_us']
                        or c['control_dt_us']%c['physics_dt_us']):
                    raise ValueError('invalid simulation evidence clock')
                header=row
                continue
            if footer is not None:raise ValueError('data after simulation footer')
            if row.get('record_type')=='footer':
                for key in ('offered','written','dropped','write_failures'):
                    if type(row.get(key)) is not int or row[key]<0:raise ValueError('invalid footer count')
                footer=row
                continue
            seq=row.get('sequence')
            if (row.get('record_type')!='event' or type(seq) is not int or seq<=last
                    or type(row.get('logical_us')) is not int
                    or type(row.get('control_step')) is not int
                    or row['control_step']<0 or row.get('phase') not in range(5)
                    or row['logical_us']!=1+row['control_step']*header['clock']['control_dt_us']):
                raise ValueError('invalid simulation event identity')
            if row.get('kind')=='event_feedback':
                feedback=row.get('feedback')
                if row.get('feedback_validated'):
                    if row['control_step']!=measured_count:measured_complete=False
                    measured_count+=1
                    if not isinstance(feedback,dict):raise ValueError('missing measured feedback')
                    shapes=dict(imu_quaternion=4,imu_angular_velocity=3,imu_specific_force=3,
                                base_position=3,base_velocity=6)
                    n=len(feedback.get('q',[]));shapes.update(q=n,dq=n,tau=n)
                    if not 0<n<=64:raise ValueError('invalid measured joints')
                    for key,size in shapes.items():
                        values=feedback.get(key)
                        if (not isinstance(values,list) or len(values)!=size or
                                any(type(x) not in (float,int) or not math.isfinite(x) for x in values)):
                            raise ValueError('invalid measured '+key)
                    quaternion=feedback.get('base_quaternion',[])
                    if not quaternion:measured_complete=False
                    elif (len(quaternion)!=4 or any(type(x) not in (float,int) or not math.isfinite(x) for x in quaternion)):
                        raise ValueError('invalid measured base quaternion')
                    if (type(feedback.get('physics_step')) is not int or
                        feedback['physics_step']!=row['control_step']*(header['clock']['control_dt_us']//header['clock']['physics_dt_us'])):
                        raise ValueError('invalid measured step identity')
                else:measured_complete=False
            if row.get('kind')=='runner_final':
                final={key:row.get(key) for key in ('coordinator_state','stop_proven','failure')}
            holes+=seq-last-1;last=seq;count+=1
            if visitor:visitor(row)
            else:
                # Convenience consumer also has a hard memory bound.
                if len(rows)>=65536:raise ValueError('use streaming visitor for large simulation evidence')
                rows.append(row)
    if header is None:raise ValueError('missing simulation evidence header')
    summary=dict(clock=header['clock'],provenance=header.get('provenance',{}),
                 sequence_gaps=holes,rows=count,footer_present=footer is not None,
                 dropped=(footer or {}).get('dropped',0),
                 write_failures=(footer or {}).get('write_failures',0))
    summary['final']=final
    summary['feedback_units']=header.get('feedback_units',{})
    summary['evidence_complete']=bool(footer and footer['written']==count and
        footer['offered']==count and not footer['dropped'] and not footer['write_failures'] and not holes)
    summary['measured_feedback_complete']=bool(measured_count and measured_complete and summary['evidence_complete'])
    return rows,summary
