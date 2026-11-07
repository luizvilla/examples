# SPDX-License-Identifier: LGPL-2.1
"""design/sizing.py against hand calculations."""
import math
import pathlib
import sys
import unittest

import yaml

MISSION = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(MISSION / "design"), str(MISSION / "plant")]
import sizing  # noqa: E402
from twist_buck import load_params  # noqa: E402


class Formulas(unittest.TestCase):
    def test_passive_dip(self):
        # 1 A into sqrt(33 uH / 33 uF) = 1 ohm gives 1 V
        self.assertAlmostEqual(sizing.undershoot_lc(1.0, 33e-6, 33e-6), 1.0)

    def test_capacitance_for_undershoot_inverts_the_dip(self):
        c = sizing.capacitance_for_undershoot(1.0, 0.6, 33e-6)
        self.assertAlmostEqual(sizing.undershoot_lc(1.0, 33e-6, c), 0.6)

    def test_loop_dip_at_resonance_equals_passive_dip(self):
        L, C = 33e-6, 61.1e-6
        f0 = sizing.resonance(L, C)
        self.assertAlmostEqual(sizing.undershoot_with_loop(1.0, C, f0), sizing.undershoot_lc(1.0, L, C))


class MissionSizing(unittest.TestCase):
    def setUp(self):
        reqs = yaml.safe_load((MISSION / "requirements.yaml").read_text())
        self.r = {x["id"]: x for x in sizing.size(reqs, load_params())["results"]}

    def test_inductance_by_hand(self):
        # (40 - 12) * 0.3 / (0.4 * 1.5 A * 200 kHz) = 70 uH
        self.assertAlmostEqual(self.r["L"]["value"], 28 * 0.3 / (0.4 * 1.5 * 200e3))

    def test_ripple_is_met_by_the_ceramics(self):
        self.assertTrue(self.r["C_ripple"]["meets"])

    def test_the_twist_filter_alone_holds_the_half_step(self):
        # 33 uH * (0.5 A / 0.6 V)^2 = 22.9 uF < 61.1 uF; the passive dip 0.5 A * sqrt(L/C) = 0.37 V < 0.6 V
        self.assertAlmostEqual(self.r["C_dip"]["value"], 33e-6 * (0.5 / 0.6) ** 2)
        self.assertTrue(self.r["C_dip"]["meets"])
        self.assertTrue(self.r["dip_lc"]["meets"])
        self.assertAlmostEqual(self.r["f0"]["value"], 1 / (2 * math.pi * math.sqrt(33e-6 * 61.1e-6)))

    def test_no_control_period_is_needed_when_the_filter_holds_the_dip(self):
        f_c = 0.5 / (2 * math.pi * 61.1e-6 * 0.6)
        self.assertAlmostEqual(self.r["period_for_req"]["value"], 1 / (10 * f_c))
        self.assertTrue(self.r["period_for_req"]["meets"])
        self.assertIn("Not needed", self.r["period_for_req"]["note"])


class TheFullStep(unittest.TestCase):
    """The 1 A -> 2 A step of the mission before #11: the filter alone does not hold it."""
    def setUp(self):
        reqs = yaml.safe_load((MISSION / "requirements.yaml").read_text())
        for r in reqs["requirements"]:
            if "load_step_A" in r.get("conditions", {}):
                r["conditions"] = dict(r["conditions"], load_step_A=[1.0, 2.0])
        self.r = {x["id"]: x for x in sizing.size(reqs, load_params())["results"]}

    def test_undershoot_needs_more_than_the_twist_has(self):
        # 33 uH * (1 A / 0.6 V)^2 = 91.7 uF > 61.1 uF
        self.assertAlmostEqual(self.r["C_dip"]["value"], 33e-6 / 0.36)
        self.assertFalse(self.r["C_dip"]["meets"])

    def test_control_period_for_the_undershoot(self):
        f_c = 1.0 / (2 * math.pi * 61.1e-6 * 0.6)
        self.assertAlmostEqual(self.r["period_for_req"]["value"], 1 / (10 * f_c))
        self.assertFalse(self.r["period_for_req"]["meets"])


if __name__ == "__main__":
    unittest.main()
