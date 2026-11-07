# SPDX-License-Identifier: LGPL-2.1
"""Analytic sizing of a buck converter in continuous conduction.

These are the formulas the student uses in the Design workbench. The plant
model tests also use them as the reference the switched model must match.
Units: volts, amperes, henries, farads, hertz, seconds.
"""
import math


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
