# SPDX-License-Identifier: LGPL-2.1
"""Analytic sizing of a buck converter in continuous conduction.

These are the formulas the student uses in the Design workbench.
`python design/sizing.py [--json]` sizes L and C from requirements.yaml and
compares them with the Twist (plant/params.yaml). The plant
model tests also use them as the reference the switched model must match.
Units: volts, amperes, henries, farads, hertz, seconds.
"""
import argparse
import json
import math
import pathlib
import sys

import yaml

MISSION = pathlib.Path(__file__).resolve().parents[1]


def effective_duty(duty: float, dead_time_s: float, f_sw: float) -> float:
    """Duty actually seen by the switch node when dead time is taken from the high-side on-time."""
    return duty - dead_time_s * f_sw


def output_voltage(v_in: float, duty: float) -> float:
    """Ideal buck in continuous conduction: V_out = D * V_in."""
    return duty * v_in


def inductor_ripple(v_in: float, v_out: float, inductance: float, f_sw: float) -> float:
    """Peak-to-peak inductor current ripple: (V_in - V_out) * D / (L * f), with D = V_out / V_in."""
    return (v_in - v_out) * (v_out / v_in) / (inductance * f_sw)


def output_ripple(delta_i: float, capacitance: float, f_sw: float) -> float:
    """Peak-to-peak output voltage ripple of an ideal capacitor: delta_I / (8 * C * f)."""
    return delta_i / (8.0 * capacitance * f_sw)


def resonance(inductance: float, capacitance: float) -> float:
    """Natural frequency of the output LC filter, in hertz."""
    return 1.0 / (2.0 * math.pi * math.sqrt(inductance * capacitance))


def inductance_for_ripple(v_in: float, v_out: float, delta_i: float, f_sw: float) -> float:
    """Smallest inductance that keeps the current ripple at or below delta_i."""
    return (v_in - v_out) * (v_out / v_in) / (delta_i * f_sw)


def capacitance_for_ripple(delta_i: float, delta_v: float, f_sw: float) -> float:
    """Smallest capacitance that keeps the output ripple at or below delta_v."""
    return delta_i / (8.0 * delta_v * f_sw)


def undershoot_lc(delta_i: float, inductance: float, capacitance: float) -> float:
    """Dip of an undamped LC filter when the load steps by delta_i and nothing reacts:
    delta_I * sqrt(L / C), the filter's characteristic impedance times the step."""
    return delta_i * math.sqrt(inductance / capacitance)


def capacitance_for_undershoot(delta_i: float, delta_v: float, inductance: float) -> float:
    """Smallest capacitance whose passive dip delta_I * sqrt(L / C) stays at or below delta_v."""
    return inductance * (delta_i / delta_v) ** 2


def undershoot_with_loop(delta_i: float, capacitance: float, crossover_Hz: float) -> float:
    """Dip when a voltage loop with crossover f_c catches the step: about delta_I / (2 pi f_c C).
    Only meaningful when f_c is below the LC resonance."""
    return delta_i / (2.0 * math.pi * crossover_Hz * capacitance)


# Sizing from the mission's requirements ----------------------------------------

def _requirement(reqs, rid):
    return next(r for r in reqs if r["id"] == rid)


def size(requirements: dict, params: dict, ripple_ratio: float = 0.4, control_period_s: float = 100e-6) -> dict:
    """Size L and C from requirements.yaml and compare with the Twist (plant/params.yaml values).

    ripple_ratio: the designer's choice of inductor current ripple, as a fraction of
    the largest load current (0.2 to 0.4 is usual).
    Returns {inputs, results: [{id, title, value, unit, formula, twist, meets}]}."""
    reqs = requirements["requirements"]
    vout = _requirement(reqs, "REQ-VOUT")
    cond = vout["conditions"]
    v_in, v_out = float(cond["port_high_V"]), float(vout["value"])
    step = _requirement(reqs, "REQ-UNDERSHOOT")["conditions"]["load_step_A"]
    i_before, i_after = float(step[0]), float(step[1])
    delta_step = i_after - i_before
    ripple_max = float(_requirement(reqs, "REQ-RIPPLE")["value"])
    undershoot_max = float(_requirement(reqs, "REQ-UNDERSHOOT")["value"]) * v_out
    f_sw = float(params["switching_freq_Hz"])
    L_twist = float(params["inductance_H"])
    C_cer = float(params["ceramic_capacitance_F"])
    C_elec = float(params["electrolytic_capacitance_F"]) if params.get("electrolytic_connected", 1) else 0.0
    C_twist = C_cer + C_elec

    delta_i = ripple_ratio * i_after
    L_min = inductance_for_ripple(v_in, v_out, delta_i, f_sw)
    ripple_i_twist = inductor_ripple(v_in, v_out, L_twist, f_sw)
    C_ripple = capacitance_for_ripple(ripple_i_twist, ripple_max, f_sw)
    C_dip = capacitance_for_undershoot(delta_step, undershoot_max, L_twist)
    f0 = resonance(L_twist, C_twist)
    f_c_needed = delta_step / (2.0 * math.pi * C_twist * undershoot_max)
    results = [
        {"id": "L", "title": f"Inductance for a current ripple of {ripple_ratio:.0%} of {i_after:g} A",
         "value": L_min, "unit": "H", "formula": "L ≥ (V_in − V_out)·D / (ΔI_L·f_sw)",
         "twist": L_twist, "meets": None,
         "note": f"A design choice, not a requirement: the Twist's {L_twist * 1e6:.3g} µH gives a ripple of "
                 f"{inductor_ripple(v_in, v_out, L_twist, f_sw) / i_after:.0%} of {i_after:g} A."},
        {"id": "ripple_i", "title": "Inductor current ripple with the Twist's L",
         "value": ripple_i_twist, "unit": "A", "formula": "ΔI_L = (V_in − V_out)·D / (L·f_sw)", "twist": None, "meets": None},
        {"id": "C_ripple", "title": f"Capacitance for REQ-RIPPLE ({ripple_max * 1e3:g} mV)",
         "value": C_ripple, "unit": "F", "formula": "C ≥ ΔI_L / (8·ΔV·f_sw)",
         "twist": C_cer, "meets": C_cer >= C_ripple,
         "note": "At 200 kHz the ceramics carry the ripple; the electrolytic's ESR keeps it out."},
        {"id": "C_dip", "title": f"Capacitance for REQ-UNDERSHOOT with no control ({undershoot_max:.2g} V)",
         "value": C_dip, "unit": "F", "formula": "C ≥ L·(ΔI_step / ΔV)²",
         "twist": C_twist, "meets": C_twist >= C_dip,
         "note": "The dip before any controller reacts. ESR damping is ignored, so this is pessimistic."},
        {"id": "dip_lc", "title": "Passive dip of the Twist's filter for the load step",
         "value": undershoot_lc(delta_step, L_twist, C_twist), "unit": "V", "formula": "ΔV = ΔI_step·√(L/C)",
         "twist": None, "meets": undershoot_lc(delta_step, L_twist, C_twist) <= undershoot_max},
        {"id": "f0", "title": "Output filter resonance (all LOW capacitors)",
         "value": f0, "unit": "Hz", "formula": "f₀ = 1 / (2π·√(L·C))", "twist": None, "meets": None},
        {"id": "f_c_needed", "title": "Loop crossover REQ-UNDERSHOOT needs, with the Twist's capacitors",
         "value": f_c_needed, "unit": "Hz", "formula": "f_c ≥ ΔI_step / (2π·C·ΔV)", "twist": None, "meets": None},
        {"id": "period_for_req", "title": "Control period for that crossover (f_control ≥ 10·f_c)",
         "value": 1.0 / (10.0 * f_c_needed), "unit": "s", "formula": "T_control ≤ 1 / (10·f_c)",
         "twist": control_period_s, "meets": control_period_s <= 1.0 / (10.0 * f_c_needed),
         "note": "A rule of thumb; tests/studies/undershoot_vs_period.py finds 4.9 % reachable at 20 µs."},
    ]
    return {"inputs": {"v_in_V": v_in, "v_out_V": v_out, "load_step_A": [i_before, i_after], "f_sw_Hz": f_sw,
                       "ripple_max_V": ripple_max, "undershoot_max_V": undershoot_max, "ripple_ratio": ripple_ratio,
                       "control_period_s": control_period_s},
            "twist": {"inductance_H": L_twist, "ceramic_capacitance_F": C_cer, "electrolytic_capacitance_F": C_elec},
            "results": results}


def _eng(value: float, unit: str) -> str:
    for factor, prefix in ((1e-9, "n"), (1e-6, "µ"), (1e-3, "m"), (1.0, ""), (1e3, "k")):
        if abs(value) < factor * 1000:
            return f"{value / factor:.3g} {prefix}{unit}"
    return f"{value:.3g} {unit}"


def main(argv=None) -> int:
    sys.path.insert(0, str(MISSION / "plant"))
    from twist_buck import load_params
    parser = argparse.ArgumentParser(description="Size L and C from the mission's requirements")
    parser.add_argument("--ripple-ratio", type=float, default=0.4)
    parser.add_argument("--control-period-us", type=float, default=100.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    result = size(yaml.safe_load((MISSION / "requirements.yaml").read_text()), load_params(),
                  args.ripple_ratio, args.control_period_us * 1e-6)
    if args.json:
        print(json.dumps(result, indent=1))
        return 0
    for r in result["results"]:
        twist = f"  Twist: {_eng(r['twist'], r['unit'])}" if r["twist"] is not None else ""
        verdict = "" if r["meets"] is None else ("  ok" if r["meets"] else "  NOT MET")
        print(f"{r['title']}: {_eng(r['value'], r['unit'])}   [{r['formula']}]{twist}{verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
