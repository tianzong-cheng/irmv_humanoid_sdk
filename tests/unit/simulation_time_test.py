"""Incremental engine diagnostic, never an SDK clock; stdlib only."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from simulation import engine


class TimeTest(unittest.TestCase):
    def test_incremental_rounding_does_not_accept_loose_elapsed_time(self):
        self.assertTrue(
            hasattr(engine, "EngineTimeDiagnostic"),
            "incremental time diagnostic missing",
        )
        d = engine.EngineTimeDiagnostic(1000)
        for _ in range(100000):
            d.advance()
        self.assertAlmostEqual(d.expected_s, 99.95669555664062, places=7)
        d.validate(99.95669555664062)
        with self.assertRaises(RuntimeError):
            d.validate(100.0)
        with self.assertRaises(RuntimeError):
            d.validate(float("nan"))

    def test_initial_diagnostic_cannot_hide_an_engine_time_offset(self):
        d = engine.EngineTimeDiagnostic(1000)
        d.validate(0.0)
        with self.assertRaises(RuntimeError):
            d.validate(1e-8)

    def test_review_one_ulp_missing_increment_is_rejected(self):
        # Arithmetic probe only: float32(1024 + float32(100us)) is one ULP.
        import struct

        f32 = lambda x: struct.unpack("<f", struct.pack("<f", x))[0]
        expected = f32(1024.0 + f32(0.0001))
        self.assertEqual(expected, 1024.0001220703125)
        d = engine.EngineTimeDiagnostic(1000)
        d.expected_s = expected
        with self.assertRaises(RuntimeError):
            d.validate(1024.0)
        d.validate(1024.0001220703125)

    def test_hour_range_rejects_timesteps_that_can_stall(self):
        for dt in (1, 10, 100, 999):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                engine.EngineTimeDiagnostic(dt)

    def test_supported_long_checkpoint_rejects_single_missing_increment(self):
        d = engine.EngineTimeDiagnostic(1000)
        # Known checkpoint; validates addition arithmetic, not a GPU rollout.
        d.expected_s = 2048.0
        d.advance()
        self.assertEqual(d.expected_s, 2048.0009765625)
        d.validate(2048.0009765625)
        with self.assertRaises(RuntimeError):
            d.validate(2048.0)

    def test_one_hour_duration_guard(self):
        self.assertTrue(
            hasattr(engine, "EngineTimeDiagnostic"), "duration guard missing"
        )
        d = engine.EngineTimeDiagnostic(1000000)
        for _ in range(3600):
            d.advance()
        d.validate(3600.0)
        with self.assertRaises(ValueError):
            d.advance()


if __name__ == "__main__":
    unittest.main()
