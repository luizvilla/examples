#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""Model tests of the Twist buck plant: does the switched model match the analytic buck?

These checks come before any test of the firmware against the plant (L1):
a plant that disagrees with first principles would make every L1 verdict
meaningless. Comparing the model with the real board on the bench is a
separate, later step.

Usage: plant/model_tests/run_model_tests.py [out-dir]   (default: <new campaign>/model-tests)
"""
import pathlib
import sys

import numpy as np
import pulsim

HERE = pathlib.Path(__file__).resolve().parent
PLANT = HERE.parent
MISSION = PLANT.parent
sys.path[:0] = [str(PLANT), str(MISSION / "design"), str(MISSION / "tests")]

import record  # noqa: E402
import sizing  # noqa: E402
from twist_buck import build, load_params  # noqa: E402


def period_mean(trace, f_sw):
    """Average each quantity over one switching period, removing the ripple."""
    n = int(round(1.0 / f_sw / (trace.t[1] - trace.t[0])))
    kernel = np.ones(n) / n
    return np.convolve(trace.v_low, kernel, mode="valid"), trace.t[n - 1:]


def case(cid, title, measured, expected, tolerance, unit=""):
    error = abs(measured - expected) / abs(expected)
    return {"id": cid, "title": title, "measured": float(measured), "expected": float(expected),
            "unit": unit, "criterion": f"within {tolerance:.0%}", "relative_error": float(error),
            "passed": bool(error <= tolerance)}


def main(out):
    q = load_params()
    f, vin, L, C = q["switching_freq_Hz"], q["port_high_V"], q["inductance_H"], q["ceramic_capacitance_F"]
    cases = []

    # PM-01: steady-state output voltage at two duty cycles.
    for cid, duty in (("PM-01a", 0.3), ("PM-01b", 0.5)):
        tr = build(q).run_open_loop(duty, 4e-3)
        steady = tr.t > 3e-3
        expected = sizing.output_voltage(vin, sizing.effective_duty(duty, q["dead_time_s"], f))
        cases.append(case(cid, f"mean LOW1 voltage at duty {duty}", tr.v_low[steady].mean(), expected, 0.02, "V"))
        if duty == 0.3:
            v_out = tr.v_low[steady].mean()
            di = tr.i_l[steady].max() - tr.i_l[steady].min()
            dv = tr.v_low[steady].max() - tr.v_low[steady].min()
            # PM-02: inductor current ripple.
            cases.append(case("PM-02", "inductor current ripple, peak to peak",
                              di, sizing.inductor_ripple(vin, v_out, L, f), 0.10, "A"))
            # PM-03: output voltage ripple, from the measured current ripple. At 200 kHz
            # the ceramics carry the ripple: the electrolytic branch is 0.9 ohm against 0.06 ohm.
            cases.append(case("PM-03", "LOW1 voltage ripple, peak to peak",
                              dv, sizing.output_ripple(di, C, f), 0.15, "V"))

    # PM-04: LC resonance, from the start-up ringing at light load, with the
    # electrolytic switched out (its ESR damps the ringing, and E16's AC sweep
    # checks the plant with it).
    tr = build(q, load_ohm=60.0, electrolytic=False).run_open_loop(0.3, 2e-3)
    v, t = period_mean(tr, f)
    final = v[t > 1.6e-3].mean()
    rising = np.where((v[:-1] < final) & (v[1:] >= final))[0]
    measured_f0 = 1.0 / np.mean(np.diff(t[rising]))
    cases.append(case("PM-04", "output filter resonance from start-up ringing",
                      measured_f0, sizing.resonance(L, C), 0.05, "Hz"))

    for c in cases:
        print(f"{'PASS' if c['passed'] else 'FAIL'} {c['id']}: {c['title']}: "
              f"{c['measured']:.4g} vs {c['expected']:.4g} {c['unit']} ({c['criterion']})")
    passed = record.write(out, "Twist buck plant model tests", "model", "simulator", cases, MISSION,
                          {"plant/params.yaml": PLANT / "params.yaml", "plant/twist_buck.py": PLANT / "twist_buck.py",
                           "design/sizing.py": MISSION / "design" / "sizing.py"},
                          {"pulsim": pulsim.__version__, "numpy": np.__version__})
    print(f"record: {out / 'record.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main(record.out_dir(sys.argv[1:], "model-tests")))
