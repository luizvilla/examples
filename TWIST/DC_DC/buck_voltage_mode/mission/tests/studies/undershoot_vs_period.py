#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""Study: the best undershoot a PI can reach on the Twist, against the control period.

Exploratory (Core2's "study"): it informs REQ-UNDERSHOOT and never sets a verdict.
Same plant and load step as BUCK-STEP-01 (40 V in, 12 V, 1 A -> 2 A); the PI is
written here, swept over Kp and Ki, at a 100 us and a 20 us control period.
A run counts only if it holds 12 V before the step and settles after it.

Usage: tests/studies/undershoot_vs_period.py [out-dir]   (default: records/study_undershoot)
"""
import itertools
import json
import pathlib
import sys

import numpy as np
import pulsim as p

MISSION = pathlib.Path(__file__).resolve().parents[2]
sys.path[:0] = [str(MISSION / "plant")]
from twist_buck import build, load_params  # noqa: E402

REF, V_IN, STEP_AT, END = 12.0, 40.0, 8e-3, 14e-3
PERIODS = [100e-6, 20e-6]
KP = [0.0, 0.001, 0.002, 0.004, 0.007, 0.01]
KI = [10.0, 30.0, 60.0, 100.0, 200.0]


def run(period_s, kp, ki, electrolytic=True):
    q = load_params()
    b = build(q, load_ohm=12.0, electrolytic=electrolytic).builder
    b.add_resistor("R_step", "low1", "step", 12.0)
    b.add_switch("S_step", "step", "0", 1e3, 1e-9)
    iv = list(b.state_var_names()).index("V(low1)")
    hs, ls, st = b.switch_index_of("S_hs"), b.switch_index_of("S_ls"), b.switch_index_of("S_step")
    pwm, dead = 1 / q["switching_freq_Hz"], q["dead_time_s"]
    s = {"applied": REF / V_IN, "command": REF / V_IN, "k": -1, "next": 0.0, "integral": REF / V_IN}
    mask = p.SwitchStateMask(b.graph.num_switches)

    # PWM edges by step index, as tests/sim_harness.py (the duty moves in 0.5 % quanta).
    dt = q["step_s"]
    steps, dead_steps = int(round(pwm / dt)), int(round(dead / dt))
    s.update({"on": 0})

    def switch_fn(t):
        k = int(np.floor(t / pwm + 1e-6))
        if k != s["k"]:
            s["on"] = min(max(int(round(s["command"] * steps)), 0), steps)
            s["k"], s["applied"] = k, s["command"]
        step = int(round((t - k * pwm) / dt))
        mask.set(hs, dead_steps <= step < s["on"])
        mask.set(ls, s["on"] + dead_steps <= step < steps)
        mask.set(st, t >= STEP_AT)
        return mask

    def observer(t, x):
        if t >= s["next"] - 1e-12:
            s["next"] += period_s
            e = REF - x[iv]
            s["integral"] = min(max(s["integral"] + ki * period_s * e, 0.0), 1.0)
            s["command"] = min(max(kp * e + s["integral"], 0.0), 1.0)

    r = p.simulate(b, t_end=END, dt=q["step_s"], engine="pwl", switch_fn=switch_fn, step_observer=observer, store_every=4)
    t, v = np.asarray(r.times), np.asarray(r.v("low1"))
    n = int(round(pwm / (t[1] - t[0])))
    vm, t = np.convolve(v, np.ones(n) / n, "valid"), t[n - 1:]
    before, after, tail = vm[(t > STEP_AT - 1e-3) & (t < STEP_AT)], vm[t >= STEP_AT], vm[t > END - 1e-3]
    held = abs(before.mean() - REF) < 0.3 and (before.max() - before.min()) < 0.03 * REF
    settled = np.abs(tail - REF).max() < 0.02 * REF
    return {"period_us": period_s * 1e6, "kp": kp, "ki": ki, "valid": bool(held and settled),
            "undershoot_pct": float((REF - after.min()) / REF * 100)}


def main(out: pathlib.Path):
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    for period, kp, ki in itertools.product(PERIODS, KP, KI):
        r = run(period, kp, ki)
        runs.append(r)
        print(json.dumps(r), flush=True)
    lines = ["# Study: best undershoot against the control period", "",
             "Exploratory; never sets a verdict. BUCK-STEP-01's load step on the plant model, PI swept over "
             f"Kp {KP} and Ki {KI}.", "", "| Control period | Best undershoot | Kp | Ki | Valid runs |", "| --- | --- | --- | --- | --- |"]
    best = {}
    for period in PERIODS:
        valid = [r for r in runs if r["valid"] and abs(r["period_us"] - period * 1e6) < 1e-6]
        if valid:
            b = min(valid, key=lambda r: r["undershoot_pct"])
            best[f"{period * 1e6:g} us"] = b
            lines.append(f"| {period * 1e6:g} µs | {b['undershoot_pct']:.2f} % | {b['kp']} | {b['ki']} | {len(valid)} |")
        else:
            lines.append(f"| {period * 1e6:g} µs | none valid | | | 0 |")
    (out / "study.json").write_text(json.dumps({"runs": runs, "best": best}, indent=1) + "\n")
    (out / "study.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main(pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else MISSION / "records" / "study_undershoot")
