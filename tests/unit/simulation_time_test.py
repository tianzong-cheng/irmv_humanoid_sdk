"""Incremental engine diagnostic, never an SDK clock; stdlib only."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'tools'))
from simulation import engine

class TimeTest(unittest.TestCase):
    def test_incremental_rounding_does_not_accept_loose_elapsed_time(self):
        self.assertTrue(hasattr(engine,'EngineTimeDiagnostic'),'incremental time diagnostic missing')
        d=engine.EngineTimeDiagnostic(1000)
        for _ in range(100000): d.advance()
        self.assertAlmostEqual(d.expected_s,99.95669555664062,places=7)
        d.validate(99.95669555664062)
        with self.assertRaises(RuntimeError):d.validate(100.)
        with self.assertRaises(RuntimeError):d.validate(float('nan'))

    def test_initial_diagnostic_cannot_hide_an_engine_time_offset(self):
        d=engine.EngineTimeDiagnostic(1000)
        d.validate(0.)
        with self.assertRaises(RuntimeError):d.validate(1e-8)

    def test_one_hour_duration_guard(self):
        self.assertTrue(hasattr(engine,'EngineTimeDiagnostic'),'duration guard missing')
        d=engine.EngineTimeDiagnostic(1000000)
        for _ in range(3600):d.advance()
        d.validate(3600.)
        with self.assertRaises(ValueError):d.advance()

if __name__=='__main__':unittest.main()
