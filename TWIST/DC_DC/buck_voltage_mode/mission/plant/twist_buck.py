# SPDX-License-Identifier: LGPL-2.1
"""Switched plant model of one Twist leg in buck mode, built with Pulsim.

HIGH port source -> high-side and low-side switches (with body diodes) ->
inductor -> LOW1 ceramics, the switched electrolytic (with its ESR and the
on-resistance of its switch) and the load. Parameters come from params.yaml.

On the Twist 1.4.3 the two 33 uF electrolytics are on the HIGH side; LOW has
3 x 4.7 uF ceramics and a 47 uF electrolytic switched in by Q203
(shield.power.connectCapacitor).

    from twist_buck import load_params, build
    plant = build(load_params())
    result = plant.run_open_loop(duty=0.3, t_end=4e-3)
"""
from __future__ import annotations

import pathlib
from dataclasses import dataclass

import numpy as np
import pulsim as p
import yaml

HERE = pathlib.Path(__file__).resolve().parent
PULSIM_VERSION = "2.0.0"


def load_params(path: pathlib.Path = HERE / "params.yaml") -> dict:
    """Return {name: value} from params.yaml (sources stay in the file)."""
    raw = yaml.safe_load(path.read_text())
    # float(): PyYAML reads an unsigned exponent such as 200.0e3 as a string.
    values = {k: float(v["value"]) for k, v in raw.items() if isinstance(v, dict) and "value" in v}
    values["step_s"] = float(raw["simulation"]["step_s"])
    return values


@dataclass
class Trace:
    t: np.ndarray        # s
    v_low: np.ndarray    # V, LOW1 port voltage
    i_l: np.ndarray      # A, inductor current


class TwistBuck:
    def __init__(self, params: dict, load_ohm: float | None = None, electrolytic: bool | None = None):
        # A local build of the same release (2.0.0+exactedges, luizvilla/Theia#12) is accepted;
        # records stamp the full version.
        if p.__version__.split("+")[0] != PULSIM_VERSION:
            raise RuntimeError(f"plant model needs pulsim=={PULSIM_VERSION}, found {p.__version__}")
        self.params = dict(params)
        if load_ohm is not None:
            self.params["load_ohm"] = load_ohm
        if electrolytic is not None:
            self.params["electrolytic_connected"] = 1.0 if electrolytic else 0.0
        q = self.params
        b = p.CircuitBuilder()
        g_on, g_off = 1.0 / q["switch_on_ohm"], 1e-9
        b.add_voltage_source("V_high", "high", "0", q["port_high_V"])
        b.add_switch("S_hs", "high", "sw", g_on, g_off)
        b.add_switch("S_ls", "sw", "0", g_on, g_off)
        b.add_diode("D_hs", "sw", "high", g_on, g_off, q["diode_forward_V"])
        b.add_diode("D_ls", "0", "sw", g_on, g_off, q["diode_forward_V"])
        b.add_inductor("L1", "sw", "low1", q["inductance_H"])
        b.add_capacitor("C1", "low1", "0", q["ceramic_capacitance_F"])
        if q["electrolytic_connected"]:
            b.add_resistor("R_elec", "low1", "elec", q["electrolytic_esr_ohm"] + q["electrolytic_switch_ohm"])
            b.add_capacitor("C_elec", "elec", "0", q["electrolytic_capacitance_F"])
        b.add_resistor("R_load", "low1", "0", q["load_ohm"])
        self.builder = b
        self.hs = b.switch_index_of("S_hs")
        self.ls = b.switch_index_of("S_ls")

    def run_open_loop(self, duty, t_end: float) -> Trace:
        """Run with a fixed duty, or with duty(t) given as a function (held per period)."""
        q, b = self.params, self.builder
        n = b.graph.num_switches
        if callable(duty):
            period = 1.0 / q["switching_freq_Hz"]
            cache = {}

            def switch_fn(t):
                k = int(t // period)
                if k not in cache:
                    cache.clear()
                    cache[k] = p.make_dead_time_pwm_pair_fn(
                        q["switching_freq_Hz"], duty(k * period), self.hs, self.ls, n, q["dead_time_s"])
                return cache[k](t)
        else:
            switch_fn = p.make_dead_time_pwm_pair_fn(
                q["switching_freq_Hz"], duty, self.hs, self.ls, n, q["dead_time_s"])
        res = p.simulate(b, t_end=t_end, dt=q["step_s"], switch_fn=switch_fn)
        return Trace(np.asarray(res.times), np.asarray(res.v("low1")), np.asarray(res.i("L1")))


def build(params: dict, load_ohm: float | None = None, electrolytic: bool | None = None) -> TwistBuck:
    """electrolytic: None keeps params' electrolytic_connected; True or False overrides it."""
    return TwistBuck(params, load_ohm, electrolytic)
