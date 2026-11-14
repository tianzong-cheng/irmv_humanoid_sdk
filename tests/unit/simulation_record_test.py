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

    def test_bounded_validation(self):
        from simulation.record import read
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'simulation.jsonl'
            path.write_text('x'*65537+'\n')
            with self.assertRaises(ValueError):read(path)

if __name__=='__main__':unittest.main()
