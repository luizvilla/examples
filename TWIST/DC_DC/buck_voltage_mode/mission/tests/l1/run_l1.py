#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""L1 test of the buck mission: firmware/control.cpp, unchanged, closed loop on the plant model.

control.cpp and control_library are compiled into a shared library
(control_block.cpp only converts types) and called every control period,
as the critical task does on the Spin. The duty it returns is applied from
the next PWM period. Sensor scaling, calibration and the HRTIM are NOT
exercised here: on the simulator the controller sees the plant's exact
LOW1 voltage.

Test case BUCK-STEP-01: start-up to 12 V at 1 A, then a load step to 2 A.

Usage: tests/l1/run_l1.py [out-dir]     (default: records/l1-buck-step-01)
Needs a C++ compiler (CXX, default g++) and requirements.txt installed.
"""
import ctypes
import os
import pathlib
import subprocess
import sys

import numpy as np
import pulsim as p
import yaml

HERE = pathlib.Path(__file__).resolve().parent
TESTS = HERE.parent
MISSION = TESTS.parent
sys.path[:0] = [str(MISSION / "plant"), str(TESTS)]

import record  # noqa: E402
from twist_buck import build, load_params  # noqa: E402

REQUIREMENTS = {r["id"]: r for r in yaml.safe_load((MISSION / "requirements.yaml").read_text())["requirements"]}
CASE_IDS = ["BUCK-STEP-01a", "BUCK-STEP-01b", "BUCK-STEP-01c", "BUCK-STEP-01d"]

# Scenario of BUCK-STEP-01 (the nominal conditions of requirements.yaml)
REFERENCE_V = float(REQUIREMENTS["REQ-VOUT"]["value"])
CONTROL_PERIOD_S = 100e-6       # critical task period of main.cpp
LOAD_BEFORE_OHM = 12.0          # 1 A at 12 V
LOAD_ADDED_OHM = 12.0           # in parallel from the step: 2 A at 12 V
STEP_AT_S = 60e-3
END_S = 100e-3

# Pass criteria, read from requirements.yaml
STEADY_TOLERANCE = float(REQUIREMENTS["REQ-VOUT"]["tolerance_relative"])
UNDERSHOOT_MAX = float(REQUIREMENTS["REQ-UNDERSHOOT"]["value"])
SETTLING_MAX_S = float(REQUIREMENTS["REQ-SETTLING"]["value"])
SETTLING_BAND = float(REQUIREMENTS["REQ-SETTLING"]["band_relative"])
RIPPLE_MAX_V = float(REQUIREMENTS["REQ-RIPPLE"]["value"])


def control_library_dir():
    out = subprocess.run(["bash", "-c", f"source '{TESTS / 'deps.sh'}' && control_library_dir"],
                         check=True, capture_output=True, text=True)
    return pathlib.Path(out.stdout.strip())


def build_block(out: pathlib.Path) -> pathlib.Path:
    """Compile control.cpp + control_library into a shared library for this host."""
    lib = control_library_dir()
    target = out / ("control_block.dll" if os.name == "nt" else "libcontrol_block.so")
    cmd = [os.environ.get("CXX", "g++"), "-std=c++20", "-O2", "-shared", "-fPIC",
           f"-I{MISSION / 'firmware'}", f"-I{TESTS / 'shim'}", f"-I{lib / 'src'}",
           str(HERE / "control_block.cpp"), str(MISSION / "firmware" / "control.cpp"),
           str(lib / "src" / "pid.cpp"), "-o", str(target)]
    subprocess.run(cmd, check=True)
    return target


def run(block_path: pathlib.Path):
    block = ctypes.CDLL(str(block_path))
    block.block_init.argtypes = [ctypes.c_double]
    block.block_step.restype = ctypes.c_double
    block.block_step.argtypes = [ctypes.c_double] * 7
    block.block_init(CONTROL_PERIOD_S)

    q = load_params()
    plant = build(q, load_ohm=LOAD_BEFORE_OHM)
    b = plant.builder
    b.add_resistor("R_step", "low1", "step", LOAD_ADDED_OHM)
    b.add_switch("S_step", "step", "0", 1e3, 1e-9)
    names = list(b.state_var_names())
    i_v_low, i_v_high, i_i_l = names.index("V(low1)"), names.index("V(high)"), names.index("I(L1)")
    hs, ls, step = b.switch_index_of("S_hs"), b.switch_index_of("S_ls"), b.switch_index_of("S_step")
    period, dead = 1.0 / q["switching_freq_Hz"], q["dead_time_s"]

    state = {"applied": 0.0, "command": 0.0, "pwm_period": -1, "next_tick": 0.0}
    duty_log = []
    mask = p.SwitchStateMask(b.graph.num_switches)

    def switch_fn(t):
        k = int(t // period)
        if k != state["pwm_period"]:            # duty takes effect at the next PWM period
            state["pwm_period"], state["applied"] = k, state["command"]
        phase, d = t - k * period, state["applied"]
        mask.set(hs, dead <= phase < d * period)
        mask.set(ls, d * period + dead <= phase < period)
        mask.set(step, t >= STEP_AT_S)
        return mask

    def observer(t, x):
        if t >= state["next_tick"] - 1e-12:      # one control step per control period
            state["next_tick"] += CONTROL_PERIOD_S
            state["command"] = block.block_step(REFERENCE_V, x[i_v_low], x[i_i_l], 0.0, 0.0, x[i_v_high], 0.0)
            duty_log.append((t, state["command"]))

    res = p.simulate(b, t_end=END_S, dt=q["step_s"], engine="pwl", switch_fn=switch_fn,
                     step_observer=observer, store_every=4)
    return q, np.asarray(res.times), np.asarray(res.v("low1")), np.asarray(res.i("L1")), np.asarray(duty_log)


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
        {"id": "BUCK-STEP-01b", "requirement": "REQ-UNDERSHOOT", "title": "lowest LOW1 voltage after the 1 A to 2 A step", "measured": float(lowest),
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


def main(out: pathlib.Path):
    out.mkdir(parents=True, exist_ok=True)
    block_path = build_block(out)
    q, t, v, i_l, duty = run(block_path)
    cases = evaluate(q, t, v)
    for c in cases:
        print(f"{'PASS' if c['passed'] else 'FAIL'} {c['id']}: {c['title']}: {c['measured']:.4g} {c['unit']} ({c['criterion']})")
    keep = slice(None, None, 10)                                   # 1 µs resolution for the overlay
    np.savetxt(out / "waveform.csv", np.column_stack([t[keep], v[keep], i_l[keep]]),
               delimiter=",", header="t_s,v_low1_V,i_l_A", comments="", fmt="%.7g")
    np.savetxt(out / "duty.csv", duty, delimiter=",", header="t_s,duty", comments="", fmt="%.7g")
    lib = control_library_dir()
    passed = record.write(out, "BUCK-STEP-01 load step", "L1", "simulator", cases, MISSION,
                          {"firmware/control.cpp": MISSION / "firmware" / "control.cpp",
                           "firmware/control.h": MISSION / "firmware" / "control.h",
                           "plant/params.yaml": MISSION / "plant" / "params.yaml",
                           "plant/twist_buck.py": MISSION / "plant" / "twist_buck.py",
                           "requirements.yaml": MISSION / "requirements.yaml",
                           "control_block (built)": block_path},
                          {"pulsim": p.__version__, "numpy": np.__version__,
                           "control_library": subprocess.run(["git", "-C", str(lib), "rev-parse", "--short", "HEAD"],
                                                             capture_output=True, text=True).stdout.strip(),
                           "compiler": subprocess.run([os.environ.get("CXX", "g++"), "--version"],
                                                      capture_output=True, text=True).stdout.splitlines()[0]})
    print(f"record: {out / 'record.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main(pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else MISSION / "records" / "l1-buck-step-01"))
