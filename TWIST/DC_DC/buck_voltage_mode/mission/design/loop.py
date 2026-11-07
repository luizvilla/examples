# SPDX-License-Identifier: LGPL-2.1
"""The voltage loop of the buck on a Bode plot: plant G(s), the firmware's PI, margins.

Everything here is a formula you can read. The plant is the averaged buck
with the Twist's LOW-side capacitors (ceramics, and the electrolytic with its
ESR and switch resistance) and a resistive load:

    G(s) = V_in · Z(s) / (s·L + Z(s)),   Z = R_load ∥ 1/(s·C_cer) ∥ (R_esr + 1/(s·C_elec))

The controller is control_library's Pid as control.cpp uses it, sampled every
T (the critical task period):

    u[k] = Kp · (e[k] + (T/Ti) · Σ e[0..k])   →   C(z) = Kp · (1 + (T/Ti) · z/(z − 1))

The duty is held for one control period (zero-order hold) and applied at the
next PWM period. The loop gain is evaluated at z = e^(jωT):

    L(jω) = C(e^(jωT)) · (1 − e^(−jωT)) / (jωT) · e^(−jω·T_pwm) · G(jω)

This ignores aliasing, so it is trustworthy below about a tenth of the control
rate; check_against_pulsim() compares G with the switched model.

    python design/loop.py [--kp KP] [--ti TI] [--period-us T] [--json]
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sys

import numpy as np

MISSION = pathlib.Path(__file__).resolve().parents[1]
CONTROL_CPP = MISSION / "firmware" / "control.cpp"
_GAIN = re.compile(r"static const float (kp|Ti)\s*=\s*([-+0-9.eE]+)F?;")


def read_gains(path: pathlib.Path = CONTROL_CPP) -> dict:
    """Kp and Ti as written in control.cpp."""
    found = {name: float(value) for name, value in _GAIN.findall(path.read_text())}
    if set(found) != {"kp", "Ti"}:
        raise ValueError(f"{path.name}: expected 'static const float kp = ...F;' and '... Ti = ...F;'")
    return {"kp": found["kp"], "ti": found["Ti"]}


def write_gains(kp: float, ti: float, path: pathlib.Path = CONTROL_CPP) -> None:
    """Change only the two numbers in control.cpp; everything else stays as written."""
    if kp <= 0 or ti <= 0:
        raise ValueError("Kp and Ti must be positive")
    text = path.read_text()
    values = {"kp": kp, "Ti": ti}
    new = _GAIN.sub(lambda m: f"static const float {m.group(1)} = {values[m.group(1)]:.6g}F;", text)
    path.write_text(new)


def plant(s: np.ndarray, q: dict, v_in: float, load_ohm: float) -> np.ndarray:
    """Averaged control-to-output transfer function v_low / duty, in volts per unit duty."""
    z = 1.0 / (1.0 / load_ohm + s * q["ceramic_capacitance_F"])
    if q.get("electrolytic_connected", 1):
        z_elec = q["electrolytic_esr_ohm"] + q["electrolytic_switch_ohm"] + 1.0 / (s * q["electrolytic_capacitance_F"])
        z = 1.0 / (1.0 / z + 1.0 / z_elec)
    return v_in * z / (s * q["inductance_H"] + z)


def controller(w: np.ndarray, kp: float, ti: float, period_s: float) -> np.ndarray:
    z = np.exp(1j * w * period_s)
    return kp * (1.0 + (period_s / ti) * z / (z - 1.0))


def loop_gain(w: np.ndarray, q: dict, kp: float, ti: float, period_s: float, v_in: float, load_ohm: float) -> np.ndarray:
    hold = (1.0 - np.exp(-1j * w * period_s)) / (1j * w * period_s)
    pwm_delay = np.exp(-1j * w / q["switching_freq_Hz"])
    return controller(w, kp, ti, period_s) * hold * pwm_delay * plant(1j * w, q, v_in, load_ohm)


def margins(f: np.ndarray, L: np.ndarray) -> dict:
    """Crossover, phase margin and gain margin, read off the sampled curve (interpolated)."""
    mag = np.abs(L)
    phase = np.unwrap(np.angle(L))
    out = {"crossover_Hz": None, "phase_margin_deg": None, "phase_crossover_Hz": None, "gain_margin_dB": None}
    below = np.where((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
    if len(below):
        i = below[0]
        t = (math.log(mag[i]) - 0.0) / (math.log(mag[i]) - math.log(mag[i + 1]))
        fc = math.exp(math.log(f[i]) + t * (math.log(f[i + 1]) - math.log(f[i])))
        ph = phase[i] + t * (phase[i + 1] - phase[i])
        out["crossover_Hz"] = fc
        out["phase_margin_deg"] = math.degrees(ph) + 180.0
    cross = np.where((phase[:-1] > -math.pi) & (phase[1:] <= -math.pi))[0]
    if len(cross):
        i = cross[0]
        t = (phase[i] + math.pi) / (phase[i] - phase[i + 1])
        f180 = math.exp(math.log(f[i]) + t * (math.log(f[i + 1]) - math.log(f[i])))
        m = math.exp(math.log(mag[i]) + t * (math.log(mag[i + 1]) - math.log(mag[i])))
        out["phase_crossover_Hz"] = f180
        out["gain_margin_dB"] = -20.0 * math.log10(m)
    return out


def analyse(q: dict, kp: float, ti: float, period_s: float = 100e-6, v_in: float = 40.0, load_ohm: float = 12.0,
            f_min: float = 1.0, points: int = 400) -> dict:
    """Bode data (up to the control Nyquist frequency) and margins, ready for a plot."""
    f = np.logspace(math.log10(f_min), math.log10(0.5 / period_s), points)
    w = 2 * math.pi * f
    G = plant(1j * w, q, v_in, load_ohm)
    C = controller(w, kp, ti, period_s)
    L = loop_gain(w, q, kp, ti, period_s, v_in, load_ohm)

    def bode(H):
        return {"mag_dB": (20 * np.log10(np.abs(H))).tolist(), "phase_deg": np.degrees(np.unwrap(np.angle(H))).tolist()}
    c_total = q["ceramic_capacitance_F"] + (q["electrolytic_capacitance_F"] if q.get("electrolytic_connected", 1) else 0.0)
    return {"inputs": {"kp": kp, "ti_s": ti, "period_s": period_s, "v_in_V": v_in, "load_ohm": load_ohm},
            "resonance_Hz": 1.0 / (2 * math.pi * math.sqrt(q["inductance_H"] * c_total)),
            "f_Hz": f.tolist(), "plant": bode(G), "controller": bode(C), "loop": bode(L),
            "margins": margins(f, L)}


def check_against_pulsim(q: dict, frequencies, v_in: float = 40.0, load_ohm: float = 12.0, duty: float = 0.3):
    """G(jω) from the formula, and from Pulsim's sampled-data response of the switched model."""
    import pulsim as p
    sys.path.insert(0, str(MISSION / "plant"))
    from twist_buck import build
    params = dict(q)
    params["port_high_V"] = v_in
    plant_model = build(params, load_ohm=load_ohm)
    period = 1.0 / q["switching_freq_Hz"]
    n = plant_model.builder.graph.num_switches

    def switch_fn_of(u):
        return p.make_dead_time_pwm_pair_fn(q["switching_freq_Hz"], u, plant_model.hs, plant_model.ls, n, q["dead_time_s"])
    fr = p.frequency_response(plant_model.builder, period=period, dt=q["step_s"], switch_fn_of=switch_fn_of,
                              output="low1", frequencies=np.asarray(frequencies), u0=duty)
    analytic = plant(2j * math.pi * np.asarray(frequencies), q, v_in, load_ohm)
    return {"f_Hz": list(frequencies), "pulsim_dB": fr.magnitude_db.tolist(), "formula_dB": (20 * np.log10(np.abs(analytic))).tolist(),
            "pulsim_deg": fr.phase_deg.tolist(), "formula_deg": np.degrees(np.angle(analytic)).tolist()}


def main(argv=None) -> int:
    sys.path.insert(0, str(MISSION / "plant"))
    from twist_buck import load_params
    gains = read_gains()
    parser = argparse.ArgumentParser(description="Bode plot and margins of the buck voltage loop")
    parser.add_argument("--kp", type=float, default=gains["kp"], help="default: control.cpp")
    parser.add_argument("--ti", type=float, default=gains["ti"], help="default: control.cpp")
    parser.add_argument("--period-us", type=float, default=100.0, help="control period (critical task)")
    parser.add_argument("--v-in", type=float, default=40.0)
    parser.add_argument("--load-ohm", type=float, default=12.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--apply", action="store_true", help="write --kp and --ti into control.cpp")
    args = parser.parse_args(argv)
    if args.apply:
        write_gains(args.kp, args.ti)
    result = analyse(load_params(), args.kp, args.ti, args.period_us * 1e-6, args.v_in, args.load_ohm)
    result["gains_in_control_cpp"] = read_gains()
    if args.json:
        print(json.dumps(result))
        return 0
    m = result["margins"]

    def show(v, unit, fmt="{:.3g}"):
        return "none" if v is None else fmt.format(v) + f" {unit}"
    print(f"Kp {args.kp:g}, Ti {args.ti:g} s, control period {args.period_us:g} µs; filter resonance {result['resonance_Hz']:.0f} Hz")
    print(f"crossover {show(m['crossover_Hz'], 'Hz')}, phase margin {show(m['phase_margin_deg'], '°', '{:.1f}')}, "
          f"gain margin {show(m['gain_margin_dB'], 'dB', '{:.1f}')} at {show(m['phase_crossover_Hz'], 'Hz')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
