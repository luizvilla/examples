# SPDX-License-Identifier: LGPL-2.1
"""design/loop.py: hand calculations, the gains in control.cpp, and the switched model."""
import math
import pathlib
import shutil
import sys
import tempfile
import unittest

MISSION = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(MISSION / "design"), str(MISSION / "plant")]
import loop  # noqa: E402
from twist_buck import load_params  # noqa: E402


class Loop(unittest.TestCase):
    def setUp(self):
        self.q = load_params()

    def test_dc_gain_is_v_in(self):
        g = loop.plant(1j * 2 * math.pi * 0.01, self.q, 40.0, 12.0)
        self.assertAlmostEqual(abs(g), 40.0, places=2)

    def test_integrator_crossover_by_hand(self):
        # A slow PI is an integrator Kp/(Ti s) times V_in: crossover Kp V_in / (2 pi Ti)
        kp, ti = 0.000215, 7.5175e-5
        m = loop.analyse(self.q, kp, ti)["margins"]
        self.assertAlmostEqual(m["crossover_Hz"], kp * 40 / (2 * math.pi * ti), delta=0.5)
        self.assertGreater(m["phase_margin_deg"], 85)

    def test_formula_matches_the_switched_model(self):
        r = loop.check_against_pulsim(self.q, [100, 1000, 3000, 6000, 10000])
        for f, a, b, pa, pb in zip(r["f_Hz"], r["pulsim_dB"], r["formula_dB"], r["pulsim_deg"], r["formula_deg"]):
            self.assertLess(abs(a - b), 0.5, f"{f} Hz magnitude")
            self.assertLess(abs(pa - pb), 15.0, f"{f} Hz phase")

    def test_gains_round_trip_and_nothing_else_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            copy = pathlib.Path(tmp) / "control.cpp"
            shutil.copy(loop.CONTROL_CPP, copy)
            before = copy.read_text()
            loop.write_gains(0.002, 2e-5, copy)
            self.assertEqual(loop.read_gains(copy), {"kp": 0.002, "ti": 2e-5})
            changed = [(a, b) for a, b in zip(before.splitlines(), copy.read_text().splitlines()) if a != b]
            self.assertEqual(len(changed), 2)
            with self.assertRaises(ValueError):
                loop.write_gains(-1, 1, copy)


if __name__ == "__main__":
    unittest.main()
