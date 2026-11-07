# SPDX-License-Identifier: LGPL-2.1
"""BUCK-STEP-01: the 1 A -> 1.5 A load step at 12 V, with control.cpp closed loop (L1 on the simulator).

A procedure: python -m ownstudio_lab.procedure . tests/procedures/step_response.py --target simulator
(or from OwnStudio's Lab). It switches the converter on at 12 V, waits 60 ms,
switches the second load in, waits 40 ms, and checks the four BUCK-STEP-01
criteria on the captured LOW1 voltage. Pass criteria come from requirements.yaml.

On the board this needs a switched load and the firmware scope (BUCK-STEP-02,
step 4), so this procedure runs on the simulator only.
"""
import pathlib

import numpy as np
import yaml

MISSION = pathlib.Path(__file__).resolve().parents[2]

PROCEDURE = {"title": "BUCK-STEP-01 load step", "level": "L1", "targets": ["simulator"],
             "tests": ["BUCK-STEP-01a", "BUCK-STEP-01b", "BUCK-STEP-01c", "BUCK-STEP-01d"],
             "power_on": {"group": "Converter", "name": "Mode", "value": 1}}

REQUIREMENTS = {r["id"]: r for r in yaml.safe_load((MISSION / "requirements.yaml").read_text())["requirements"]}

# Scenario of BUCK-STEP-01 (the nominal conditions of requirements.yaml)
REFERENCE_V = float(REQUIREMENTS["REQ-VOUT"]["value"])
# The load step of REQ-UNDERSHOOT (1 A -> 1.5 A since Luiz's decision on #11): 12 ohm, then 24 ohm in parallel
STEP_A = [float(a) for a in REQUIREMENTS["REQ-UNDERSHOOT"]["conditions"]["load_step_A"]]
LOAD_BEFORE_OHM = REFERENCE_V / STEP_A[0]
LOAD_ADDED_OHM = REFERENCE_V / (STEP_A[1] - STEP_A[0])
V_IN = 40.0                     # the nominal conditions of requirements.yaml
STEP_AT_S = 60e-3
END_S = 100e-3

# Pass criteria, read from requirements.yaml
STEADY_TOLERANCE = float(REQUIREMENTS["REQ-VOUT"]["tolerance_relative"])
UNDERSHOOT_MAX = float(REQUIREMENTS["REQ-UNDERSHOOT"]["value"])
SETTLING_MAX_S = float(REQUIREMENTS["REQ-SETTLING"]["value"])
SETTLING_BAND = float(REQUIREMENTS["REQ-SETTLING"]["band_relative"])
RIPPLE_MAX_V = float(REQUIREMENTS["REQ-RIPPLE"]["value"])





def evaluate(q, t, v):
    t_raw, v_raw = t, v
    # Average over the last full switching period (only complete windows, so
    # no edge effect): removes the ripple, delays the signal by half a period.
    n = int(round(1.0 / q["switching_freq_Hz"] / (t[1] - t[0])))
    v_mean = np.convolve(v, np.ones(n) / n, mode="valid")
    t = t[n - 1:]
    before = (t > STEP_AT_S - 5e-3) & (t < STEP_AT_S)
    after = t >= STEP_AT_S
    steady = v_mean[before].mean()
    lowest = v_mean[after].min()
    band = np.abs(v_mean - REFERENCE_V) <= SETTLING_BAND * REFERENCE_V
    window = (t_raw > STEP_AT_S - 1e-3) & (t_raw < STEP_AT_S)
    ripple = v_raw[window].max() - v_raw[window].min()
    outside = np.where(after & ~band)[0]
    settling = (t[outside[-1]] - STEP_AT_S) if len(outside) else 0.0
    settled = len(outside) == 0 or outside[-1] < len(t) - 1 - int(1e-3 / (t[1] - t[0]))
    return [
        {"id": "BUCK-STEP-01a", "requirement": "REQ-VOUT", "title": "mean LOW1 voltage before the load step", "measured": float(steady),
         "expected": REFERENCE_V, "unit": "V", "criterion": f"within {STEADY_TOLERANCE:.0%} of {REFERENCE_V} V",
         "passed": bool(abs(steady - REFERENCE_V) <= STEADY_TOLERANCE * REFERENCE_V)},
        {"id": "BUCK-STEP-01b", "requirement": "REQ-UNDERSHOOT", "title": f"lowest LOW1 voltage after the {STEP_A[0]:g} A to {STEP_A[1]:g} A step", "measured": float(lowest),
         "expected": REFERENCE_V * (1 - UNDERSHOOT_MAX), "unit": "V", "criterion": f"at least {1 - UNDERSHOOT_MAX:.0%} of the reference",
         "passed": bool(lowest >= REFERENCE_V * (1 - UNDERSHOOT_MAX))},
        {"id": "BUCK-STEP-01c", "requirement": "REQ-SETTLING",
         "title": f"settling time back into ±{SETTLING_BAND:.0%} after the step", "measured": float(settling * 1e3),
         "expected": SETTLING_MAX_S * 1e3, "unit": "ms", "criterion": f"at most {SETTLING_MAX_S * 1e3:.0f} ms, and settled by the end",
         "passed": bool(settled and settling <= SETTLING_MAX_S)},
        {"id": "BUCK-STEP-01d", "requirement": "REQ-RIPPLE",
         "title": "LOW1 ripple, peak to peak, in the last 1 ms before the step", "measured": float(ripple * 1e3),
         "expected": RIPPLE_MAX_V * 1e3, "unit": "mV", "criterion": f"at most {RIPPLE_MAX_V * 1e3:.0f} mV",
         "passed": bool(ripple <= RIPPLE_MAX_V)},
    ]



def run(target, ctx):
    q = getattr(getattr(target, "harness", None), "q", None)
    if q is None:
        raise RuntimeError("BUCK-STEP-01 needs the simulator harness")
    harness = target.harness
    if abs(harness.load_ohm - LOAD_BEFORE_OHM) > 1e-9 or abs(harness.step_load_ohm - LOAD_ADDED_OHM) > 1e-9:
        raise RuntimeError(f"the simulator's loads ({harness.load_ohm:g} ohm, step {harness.step_load_ohm:g} ohm) are not "
                           f"REQ-UNDERSHOOT's ({LOAD_BEFORE_OHM:g} ohm, step {LOAD_ADDED_OHM:g} ohm)")
    ctx.log(f"Converter on at {REFERENCE_V} V, {V_IN} V in, load {LOAD_BEFORE_OHM} ohm")
    target.update("Converter", {"wVoltageRef_V": REFERENCE_V})
    target.update("Converter", {"Mode": 1})
    target.wait(STEP_AT_S)
    ctx.log(f"t = {STEP_AT_S * 1e3:g} ms: second load in ({LOAD_ADDED_OHM} ohm)")
    target.environment("load_step", True)
    target.wait(END_S - STEP_AT_S)
    target.update("Converter", {"Mode": 0})
    wave = target.capture()
    t, v, i_l, duty = wave["t"], wave["rV1Low_V"], wave["rI1Low_A"], wave["duty"]
    cases = evaluate(q, t, v)
    for c in cases:
        ctx.log(f"{'PASS' if c['passed'] else 'FAIL'} {c['id']}: {c['title']}: {c['measured']:.4g} {c['unit']} ({c['criterion']})")
    keep = slice(None, None, 10)                                   # 1 µs resolution for the overlay
    waveform = "t_s,rV1Low_V,rI1Low_A\n" + "\n".join(f"{a:.7g},{b:.7g},{c:.7g}" for a, b, c in zip(t[keep], v[keep], i_l[keep]))
    duty_csv = "t_s,duty\n" + "\n".join(f"{a:.7g},{b:.7g}" for a, b in duty)
    ctx.record("buck-step-01", "BUCK-STEP-01 load step", "L1", cases,
               inputs={"firmware/control.cpp": MISSION / "firmware" / "control.cpp",
                       "firmware/control.h": MISSION / "firmware" / "control.h",
                       "plant/params.yaml": MISSION / "plant" / "params.yaml",
                       "plant/twist_buck.py": MISSION / "plant" / "twist_buck.py",
                       "requirements.yaml": MISSION / "requirements.yaml"},
               files={"waveform.csv": waveform + "\n", "duty.csv": duty_csv + "\n"})
