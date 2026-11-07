# SPDX-License-Identifier: LGPL-2.1
"""BENCH-MOD-01 with a fake board and scripted operator answers."""
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE / "bench"), str(HERE.parent / "plant")]
import bench_mod_01 as bench  # noqa: E402


class FakeLink:
    def __init__(self, v1=12.0):
        self.mode, self.ref, self.v1, self.writes = 0, 12.0, v1, []

    def get(self, path):
        if path == "Converter":
            return {"Mode": self.mode, "wVoltageRef_V": self.ref}
        on = self.mode == 1
        return {"rV1Low_V": self.v1 if on else 0.2, "rV2Low_V": 1.0, "rVHigh_V": 29.3, "rI1Low_A": 0.5,
                "rI2Low_A": 0.4, "rIHigh_A": 0.2, "rTemp1_degC": 25.0, "rTemp2_degC": 24.0,
                "rDuty": self.ref / 30.0 if on else 0.0}

    def update(self, group, values):
        self.writes.append(values)
        self.mode = values.get("Mode", self.mode)
        self.ref = values.get("wVoltageRef_V", self.ref)


def answers(ref_meter_ok=True):
    queue = []
    for ref in bench.REFERENCES_V:
        v = ref if ref_meter_ok else ref * 1.1
        queue += [v, v, 2 * v * v / 33 / 30, 30.0]
    return lambda question: queue.pop(0)


def fake_model(v_in, v_out, load):
    return v_out / v_in, 2 * v_out ** 2 / load / v_in


class BenchTest(unittest.TestCase):
    def bench(self, link, ask, confirm=lambda q: True):
        return bench.Bench(link, ask, confirm, log=lambda s: None, sleep=lambda s: None, model=fake_model)

    def test_cases_and_operator_values(self):
        link = FakeLink()
        cases, operator = self.bench(link, answers()).run()
        self.assertEqual(len(cases), 4 * len(bench.REFERENCES_V))
        verdicts = {c["id"]: c["verdict"] if "verdict" in c else c["passed"] for c in cases}
        self.assertTrue(verdicts["BENCH-MOD-01.a12"])
        self.assertEqual(verdicts["BENCH-MOD-01.d5"], "INFO")
        self.assertEqual(operator["V_LOW1_meter_8V"], 8.0)
        self.assertEqual(link.mode, 0)

    def test_meter_off_fails_case_a(self):
        cases, _ = self.bench(FakeLink(), answers(ref_meter_ok=False)).run()
        self.assertFalse(next(c for c in cases if c["id"] == "BENCH-MOD-01.a5")["passed"])

    def test_no_confirmation_never_switches_on(self):
        link = FakeLink()
        with self.assertRaises(bench.Abort):
            self.bench(link, answers(), confirm=lambda q: False).run()
        self.assertNotIn({"Mode": 1}, link.writes)

    def test_over_voltage_switches_off(self):
        link = FakeLink(v1=17.0)
        with self.assertRaises(bench.Abort):
            self.bench(link, answers()).run()
        self.assertEqual((link.mode, link.writes[-1]), (0, {"Mode": 0}))

    def test_plant_model_duty_is_close_to_the_ideal(self):
        duty, i_in = bench.model_duty(30.0, 12.0, 33.0)
        self.assertAlmostEqual(duty, 0.40, delta=0.03)
        self.assertAlmostEqual(i_in, 2 * 144 / 33 / 30, places=6)


if __name__ == "__main__":
    unittest.main()
