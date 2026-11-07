# SPDX-License-Identifier: LGPL-2.1
"""BUCK-REF-01: the reference steps from 10 V to 12 V; steady state at both, on either target.

The same procedure on the simulator (L1) and on the board (L2):
  python -m ownstudio_lab.procedure . tests/procedures/reference_step.py --target simulator
  python -m ownstudio_lab.procedure . tests/procedures/reference_step.py --target board --allow-power

On the board it switches the converter on: OwnStudio asks for confirmation each
time, and the operator states the bench (supply limited, loads connected). The
board's own LOW1 reading is used, so its sensor calibration matters: compare with
BENCH-MOD-01. The settling time needs a waveform, so on the board it is blocked
until the firmware scope (step 4).
"""
import statistics

import numpy as np

PROCEDURE = {"title": "BUCK-REF-01 reference step 10 V -> 12 V", "level": "L1/L2", "targets": ["simulator", "board"],
             "tests": ["BUCK-REF-01a", "BUCK-REF-01b", "BUCK-REF-01c"],
             "power_on": {"group": "Converter", "name": "Mode", "value": 1}}

LEVELS_V = (10.0, 12.0)
TOLERANCE = 0.02
# Seconds at each level. The first includes start-up; the example PI settles in about 4 x 8.7 ms.
HOLD_S = {"simulator": (0.08, 0.05), "board": (3.0, 3.0)}
V_LOW_MAX = 16.0


def steady(target, ctx, seconds):
    """Mean LOW1 over the second half of the hold."""
    if target.kind == "simulator":
        target.wait(seconds)
        wave = target.capture()
        t, v = wave["t"], wave["v_low1"]
        return float(v[t > t[-1] - seconds / 2].mean()), wave
    target.wait(seconds / 2)
    readings = []
    for _ in range(10):
        m = target.get("Measurements")
        if m["rV1Low_V"] > V_LOW_MAX:
            raise RuntimeError(f"LOW1 reads {m['rV1Low_V']:.2f} V, above {V_LOW_MAX} V: stopping")
        readings.append(m["rV1Low_V"])
        target.wait(seconds / 20)
    return statistics.fmean(readings), None


def run(target, ctx):
    if target.kind == "board" and not ctx.confirm(
            "The converter will be switched on, with LOW1 at 10 V then 12 V. Is the supply current-limited "
            "and the load connected?"):
        raise RuntimeError("the operator did not confirm the bench")
    hold = HOLD_S[target.kind]
    cases, waves = [], []
    target.update("Converter", {"wVoltageRef_V": LEVELS_V[0]})
    target.update("Converter", {"Mode": 1})
    for letter, level, seconds in zip("ab", LEVELS_V, hold):
        target.update("Converter", {"wVoltageRef_V": level})
        mean, wave = steady(target, ctx, seconds)
        waves.append(wave)
        ctx.log(f"reference {level:g} V: LOW1 {mean:.3f} V")
        cases.append({"id": f"BUCK-REF-01{letter}", "title": f"Steady LOW1 at a {level:g} V reference",
                      "measured": mean, "expected": level, "unit": "V", "criterion": f"within {TOLERANCE:.0%}",
                      "passed": abs(mean - level) <= TOLERANCE * level})
    target.update("Converter", {"Mode": 0})
    if target.kind == "simulator":
        t, v = waves[1]["t"], waves[1]["v_low1"]
        n = max(1, int(round(5e-6 / (t[1] - t[0]))))
        vm, tm = np.convolve(v, np.ones(n) / n, "valid"), t[n - 1:]
        outside = np.where(np.abs(vm - LEVELS_V[1]) > TOLERANCE * LEVELS_V[1])[0]
        settling = float(tm[outside[-1]] - t[0]) if len(outside) else 0.0
        cases.append({"id": "BUCK-REF-01c", "title": "Settling into ±2 % after the 10 V → 12 V step",
                      "measured": settling * 1e3, "unit": "ms", "criterion": "information only", "verdict": "INFO"})
    else:
        cases.append({"id": "BUCK-REF-01c", "title": "Settling into ±2 % after the 10 V → 12 V step",
                      "verdict": "BLOCKED", "reason": "needs a waveform: the firmware scope (step 4)"})
    ctx.record(f"buck-ref-01-{target.kind}", "BUCK-REF-01 reference step", "L1" if target.kind == "simulator" else "L2", cases)
