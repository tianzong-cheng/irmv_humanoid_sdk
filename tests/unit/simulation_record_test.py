"""Bounded evidence writer: overflow and incomplete files never imply success."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))

class RecordTest(unittest.TestCase):
    def test_stream_and_loss(self):
        from simulation.record import Writer, read
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'simulation.jsonl'
            w=Writer(path,dict(clock='simulation',available=True,clock_domain='simulation',clock_run_id='12'*16,
                logical_origin_us=1,physics_dt_us=1000,control_dt_us=2000),capacity=1)
            # Before starting the worker the first bounded slot is deterministic.
            self.assertTrue(w.offer(dict(kind='write_accepted',logical_us=1,control_step=0,phase=1)))
            self.assertFalse(w.offer(dict(kind='lost',logical_us=1,control_step=0,phase=1)))
            w.start();self.assertTrue(w.close())
            rows,summary=read(path)
            self.assertEqual([r['kind'] for r in rows],['write_accepted'])
            self.assertEqual(summary['dropped'],1)
            self.assertFalse(summary['evidence_complete'])
            self.assertFalse(summary['measured_feedback_complete'])
            self.assertEqual(summary['clock']['logical_origin_us'],1)
            path.write_text(path.read_text().rsplit('\n',2)[0]+'\n')
            self.assertFalse(read(path)[1]['evidence_complete'])

    def test_missing_measured_feedback_is_not_complete(self):
        from simulation.record import Writer, read
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'simulation.jsonl'
            w=Writer(path,dict(clock='simulation',available=True,clock_domain='simulation',
                clock_run_id='12'*16,logical_origin_us=1,physics_dt_us=1000,control_dt_us=2000))
            w.offer(dict(kind='event_feedback',logical_us=1,control_step=0,phase=1,
                         feedback_validated=True,feedback=None))
            w.start();self.assertTrue(w.close())
            with self.assertRaises(ValueError):read(path)

    def write_measurements(self, path, last_measured, marker):
        from simulation.record import Writer
        clock=dict(clock='simulation',available=True,clock_domain='simulation',
                   clock_run_id='12'*16,logical_origin_us=1,physics_dt_us=1000,control_dt_us=2000)
        header=Writer(path,clock).header
        rows=[]
        for step in range(last_measured+1):
            rows.append(dict(kind='event_feedback',control_step=step,phase=1,
                logical_us=1+step*2000,feedback_validated=True,
                feedback=dict(physics_step=step*2,q=[0.],dq=[0.],tau=[0.],
                    imu_quaternion=[1.,0.,0.,0.],imu_angular_velocity=[0.]*3,
                    imu_specific_force=[0.,0.,9.81],base_position=[0.]*3,
                    base_velocity=[0.]*6,base_quaternion=[1.,0.,0.,0.])))
        if marker=='runner_final':
            rows.append(dict(kind=marker,control_step=3,phase=4,logical_us=6001,
                             coordinator_state='Failed',stop_proven=False,failure='fixture failure'))
        elif marker=='AppliedToStep':
            rows.append(dict(kind=marker,control_step=2,phase=3,logical_us=4001,
                frame=dict(name='AppliedToStep',payload=dict(write_id=17,
                    physics_step_begin=4,physics_step_end=6))))
        for i,row in enumerate(rows,1):row.update(record_type='event',sequence=i)
        footer=dict(record_type='footer',offered=len(rows),written=len(rows),dropped=0,write_failures=0)
        path.write_text(''.join(json.dumps(row)+'\n' for row in [header,*rows,footer]))

    def test_completed_horizon_requires_all_step_measurements(self):
        from simulation.record import read
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'simulation.jsonl'
            for marker in ('runner_final','AppliedToStep'):
                for last in (0,2,3):
                    with self.subTest(marker=marker,last_measured=last):
                        self.write_measurements(path,last,marker)
                        _,summary=read(path)
                        self.assertTrue(summary['evidence_complete'])
                        self.assertEqual(summary['measured_feedback_complete'],last==3)
                        self.assertFalse(summary['final'].get('stop_proven',False))

    def test_partial_data_after_footer_is_rejected(self):
        from simulation.record import read
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'simulation.jsonl'
            self.write_measurements(path,0,None)
            self.assertTrue(read(path)[1]['measured_feedback_complete'])
            clean=path.read_bytes()
            for suffix in (b'{',b' ',b'{}\n'):
                with self.subTest(suffix=suffix):
                    path.write_bytes(clean+suffix)
                    with self.assertRaisesRegex(ValueError,'after simulation footer'):read(path)
            # An interrupted pre-footer write is incomplete, not a clean file.
            path.write_bytes(clean.rsplit(b'\n',2)[0]+b'\n{"record_type":"footer"')
            summary=read(path)[1]
            self.assertFalse(summary['evidence_complete'])
            self.assertFalse(summary['measured_feedback_complete'])

    def test_bounded_validation(self):
        from simulation.record import read
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'simulation.jsonl'
            path.write_text('x'*65537+'\n')
            with self.assertRaises(ValueError):read(path)

if __name__=='__main__':unittest.main()
