# SPDX-License-Identifier: LGPL-2.1
"""BUCK-REF-01: the reference steps from 10 V to 12 V; LOW1 at both levels, and the settling between, on either target.

The same procedure on the simulator (L1) and on the board (L2):
  python -m ownstudio_lab.procedure . tests/procedures/reference_step.py --target simulator
  python -m ownstudio_lab.procedure . tests/procedures/reference_step.py --target board --allow-power --twin

Both levels come from LOW1 readings, taken the same way on both targets. The
settling comes from a waveform: the capture is armed just before the step, so
on the board the firmware scope keeps 20 ms before it and 82 ms after.

On the board it switches the converter on: OwnStudio asks for confirmation each
time, and the operator confirms the bench and states the load. The record keeps
the conditions (HIGH voltage as measured, load on LOW1); with --twin the runner
simulates the same run at those conditions and the Lab overlays the two. The
board's own LOW1 reading is used, so its calibration matters: compare with
BENCH-MOD-01.

Option hold_s (seconds at each level) shortens the run, for tests.
"""
import statistics

import numpy as np

PROCEDURE = {"title": "BUCK-REF-01 reference step 10 V -> 12 V", "level": "L1/L2", "targets": ["simulator", "board"],
             "tests": ["BUCK-REF-01a", "BUCK-REF-01b", "BUCK-REF-01c"],
             "power_on": {"group": "Converter", "name": "Mode", "value": 1},
             "twin": {"v_in": "v_high_V", "load_ohm": "load_ohm"}}

LEVELS_V = (10.0, 12.0)
TOLERANCE = 0.02
# Seconds at each level. The first includes start-up; the example PI settles in about 4 x 8.7 ms.
HOLD_S = {"simulator": (0.08, 0.05), "board": (3.0, 3.0)}
V_LOW_MAX = 16.0
BEFORE_S = 0.02             # waveform kept before the step
CAPTURE_AFTER_S = 0.1       # the 12 V hold lasts at least this: the board's scope records 82 ms after the step
RIPPLE_WINDOW_S = 5e-6      # the simulator's switching ripple is averaged out over this; the board samples once a period
POINTS = 2000               # at most, in waveform.csv


def steady(target, seconds):
    """Mean of ten LOW1 readings over the second half of the hold."""
    target.wait(seconds / 2)
    readings = []
    for _ in range(10):
        m = target.get("Measurements")
        if m["rV1Low_V"] > V_LOW_MAX:
            raise RuntimeError(f"LOW1 reads {m['rV1Low_V']:.2f} V, above {V_LOW_MAX} V: stopping")
        readings.append(m["rV1Low_V"])
        target.wait(seconds / 20)
    return statistics.fmean(readings)


def settling_time(t, v, level, tolerance):
    """Seconds from the step (t = 0) until LOW1 stays within ±tolerance of level; None if it never does."""
    after = t >= 0
    t, v = t[after], v[after]
    n = max(1, int(round(RIPPLE_WINDOW_S / np.median(np.diff(t)))))
    if n > 1:
        v, t = np.convolve(v, np.ones(n) / n, "valid"), t[n - 1:]
    outside = np.where(np.abs(v - level) > tolerance * level)[0]
    if len(outside) == 0:
        return 0.0
    if outside[-1] == len(v) - 1:
        return None
    return float(t[outside[-1] + 1])


def run(target, ctx):
    hold = (ctx.options["hold_s"],) * 2 if "hold_s" in ctx.options else HOLD_S[target.kind]
    if target.kind == "board":
        if not ctx.confirm("The converter will be switched on, with LOW1 at 10 V then 12 V. Is the supply "
                           "current-limited and the load connected?"):
            raise RuntimeError("the operator did not confirm the bench")
        load = ctx.ask_number("Load resistance on LOW1, in ohms?")
    else:
        load = target.harness.load_ohm
    v_high = target.get("Measurements")["rVHigh_V"]
    ctx.log(f"HIGH {v_high:.2f} V, load {load:g} ohm on LOW1")

    target.update("Converter", {"wVoltageRef_V": LEVELS_V[0]})
    target.update("Converter", {"Mode": 1})
    levels = [steady(target, hold[0])]
    target.arm()
    target.update("Converter", {"wVoltageRef_V": LEVELS_V[1]})
    levels.append(steady(target, max(hold[1], CAPTURE_AFTER_S)))
    target.update("Converter", {"Mode": 0})
    wave = target.capture()

    t = np.asarray(wave["t"]) - wave["t_trigger"]
    keep = t >= -BEFORE_S
    t, v, i = t[keep], np.asarray(wave["rV1Low_V"])[keep], np.asarray(wave["rI1Low_A"])[keep]

    # window_s: the part of waveform.csv each level belongs to, for the requirement band in the Lab
    cases = []
    for letter, level, mean, window in zip("ab", LEVELS_V, levels, ([float(t[0]), 0.0], [0.0, float(t[-1])])):
        ctx.log(f"reference {level:g} V: LOW1 {mean:.3f} V")
        cases.append({"id": f"BUCK-REF-01{letter}", "title": f"Steady LOW1 at a {level:g} V reference",
                      "measured": mean, "expected": level, "unit": "V", "criterion": f"within {TOLERANCE:.0%}",
                      "passed": abs(mean - level) <= TOLERANCE * level, "window_s": window})
    settling = settling_time(t, v, LEVELS_V[1], TOLERANCE)
    title = "Settling into ±2 % after the 10 V → 12 V step"
    if settling is None:
        cases.append({"id": "BUCK-REF-01c", "title": title, "criterion": "information only", "verdict": "INFO",
                      "reason": f"not settled within the {t[-1] * 1e3:.0f} ms recorded after the step"})
        ctx.log("settling: not within the waveform")
    else:
        cases.append({"id": "BUCK-REF-01c", "title": title, "measured": settling * 1e3, "unit": "ms",
                      "criterion": "information only", "verdict": "INFO"})
        ctx.log(f"settling: {settling * 1e3:.2f} ms ({wave.get('source', target.kind)})")

    step = max(1, len(t) // POINTS)
    csv = "t_s,rV1Low_V,rI1Low_A\n" + "".join(f"{a:.7g},{b:.7g},{c:.7g}\n" for a, b, c in zip(t[::step], v[::step], i[::step]))
    ctx.record(f"buck-ref-01-{target.kind}", "BUCK-REF-01 reference step", "L1" if target.kind == "simulator" else "L2",
               cases, conditions={"v_high_V": round(float(v_high), 2), "load_ohm": float(load)},
               files={"waveform.csv": csv})
