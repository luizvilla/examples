# SPDX-License-Identifier: LGPL-2.1
"""The simulator target of the buck mission: main.cpp's part, around the unchanged control.cpp.

OwnStudio's procedures talk to this harness through ownstudio_lab.target, with
the same data object names as the board (data_objects.yaml):

  Converter/Mode, Converter/wVoltageRef_V        written by the procedure
  Measurements/r...                               read back
  load_step (environment)                         switches a second load in parallel,
                                                  what the bench would do by hand

As in main.cpp: every control period (100 µs) the measurements go to
control_step(), whose duty is applied at the next PWM period; any mode other
than 1 turns the PWM off; the PI is not reset when the converter starts. The
plant is plant/twist_buck.py (one leg; leg 2 reads the same as leg 1).
"""
from __future__ import annotations

import ctypes
import inspect
import math
import os
import pathlib
import subprocess
import sys

import numpy as np
import pulsim as p

MISSION = pathlib.Path(__file__).resolve().parents[1]
TESTS = MISSION / "tests"
sys.path[:0] = [str(MISSION / "plant")]
from twist_buck import build, load_params  # noqa: E402

CONTROL_PERIOD_S = 100e-6     # critical task period of main.cpp
# Pulsim with exact gate edges (simulate(edge_fn=...), luizvilla/Theia#12): PWM edges land where
# they fall, not on the 25 ns step grid. Without it the duty moves in 0.5 % steps.
EXACT_EDGES = "edge_fn" in inspect.signature(p.simulate).parameters
DEFAULT_REFERENCE_V = 12.0    # main.cpp's voltage_reference at boot


def control_library_dir() -> pathlib.Path:
    out = subprocess.run(["bash", "-c", f"source '{TESTS / 'deps.sh'}' && control_library_dir"],
                         check=True, capture_output=True, text=True)
    return pathlib.Path(out.stdout.strip())


def build_block(out: pathlib.Path) -> pathlib.Path:
    """Compile control.cpp + control_library into a shared library for this host."""
    lib = control_library_dir()
    out.mkdir(parents=True, exist_ok=True)
    target = out / ("control_block.dll" if os.name == "nt" else "libcontrol_block.so")
    cmd = [os.environ.get("CXX", "g++"), "-std=c++20", "-O2", "-shared", "-fPIC",
           f"-I{MISSION / 'firmware'}", f"-I{TESTS / 'shim'}", f"-I{lib / 'src'}",
           str(TESTS / "l1" / "control_block.cpp"), str(MISSION / "firmware" / "control.cpp"),
           str(lib / "src" / "pid.cpp"), "-o", str(target)]
    subprocess.run(cmd, check=True)
    return target


class Harness:
    def __init__(self, mission=MISSION, v_in: float | None = None, load_ohm: float = 12.0,
                 step_load_ohm: float = 24.0, build_dir: str | None = None, store_every: int = 4):
        import tempfile
        self.block_dir = pathlib.Path(build_dir) if build_dir else pathlib.Path(tempfile.mkdtemp(prefix="ownstudio-block-"))
        self.block_path = build_block(self.block_dir)
        self.block = ctypes.CDLL(str(self.block_path))
        self.block.block_init.argtypes = [ctypes.c_double]
        self.block.block_step.restype = ctypes.c_double
        self.block.block_step.argtypes = [ctypes.c_double] * 7
        self.block.block_init(CONTROL_PERIOD_S)

        q = load_params()
        if v_in is not None:
            q["port_high_V"] = float(v_in)
        self.q = q
        self.load_ohm = float(load_ohm)
        self.step_load_ohm = float(step_load_ohm)   # switched in parallel by load_step: 0.5 A more at 12 V
        plant = build(q, load_ohm=self.load_ohm)
        b = plant.builder
        b.add_resistor("R_step", "low1", "step", float(step_load_ohm))
        b.add_switch("S_step", "step", "0", 1e3, 1e-9)
        self.b = b
        names = list(b.state_var_names())
        self.i_v_low, self.i_v_high, self.i_i_l = names.index("V(low1)"), names.index("V(high)"), names.index("I(L1)")
        self.hs, self.ls, self.step_sw = b.switch_index_of("S_hs"), b.switch_index_of("S_ls"), b.switch_index_of("S_step")
        self.mask = p.SwitchStateMask(b.graph.num_switches)
        self.period, self.dead = 1.0 / q["switching_freq_Hz"], q["dead_time_s"]
        # With EXACT_EDGES, Pulsim splits a step at each PWM edge (edge_fn) and the duty is exact.
        # Without it, the fixed 25 ns step quantizes the edges: 200 steps per period, so the duty
        # moves in 0.5 % quanta (the Twist's HRTIM has 184 ps); the edges are then placed by step
        # index, never by comparing times, so the result does not depend on how the run is
        # segmented. (Carrying the rounding forward, sigma-delta, makes the mean exact but adds
        # about 30 mV of low-frequency ripple, so it is not done.)
        self.dt = q["step_s"]
        self.steps = int(round(self.period / self.dt))
        self.dead_steps = int(round(self.dead / self.dt))
        self.on_steps = 0
        self.store_every = store_every

        self.mode, self.reference, self.load_step = 0, DEFAULT_REFERENCE_V, False
        self.command, self.applied, self.pwm_k = 0.0, 0.0, -1
        self.t, self.next_tick, self.snapshot = 0.0, 0.0, None
        self.x = np.zeros(len(names))
        self.x[self.i_v_high] = q["port_high_V"]
        self.trace_t, self.trace_v, self.trace_i = [], [], []
        self.ticks = []

    # Data objects ------------------------------------------------------------

    def get(self, group):
        v_low, i_l, v_high = float(self.x[self.i_v_low]), float(self.x[self.i_i_l]), float(self.x[self.i_v_high])
        if group == "Measurements":
            return {"rV1Low_V": v_low, "rV2Low_V": v_low, "rVHigh_V": v_high, "rI1Low_A": i_l, "rI2Low_A": i_l,
                    "rIHigh_A": i_l * self.applied if self.mode == 1 else 0.0, "rTemp1_degC": 25.0,
                    "rTemp2_degC": 25.0, "rDuty": self.command}
        if group == "Converter":
            return {"Mode": self.mode, "wVoltageRef_V": self.reference}
        raise KeyError(f"the simulator has no data object group {group!r}")

    def update(self, group, values):
        if group != "Converter":
            raise KeyError(f"the simulator cannot write group {group!r}")
        for name, value in values.items():
            if name == "Mode":
                self.mode = int(value) if int(value) == 1 else 0
            elif name == "wVoltageRef_V":
                self.reference = float(value)
            else:
                raise KeyError(f"Converter/{name} is not writable")

    def environment(self, name, value):
        if name != "load_step":
            raise KeyError(f"the simulator has no environment setting {name!r}: load_step only")
        self.load_step = bool(value)

    def identity(self):
        lib = control_library_dir()
        commit = subprocess.run(["git", "-C", str(lib), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
        return {"model": "plant/twist_buck.py", "pulsim": p.__version__, "control_library": commit,
                "v_in_V": self.q["port_high_V"]}

    # Time --------------------------------------------------------------------

    def now(self):
        return self.t

    def _switch_fn(self, t):
        # The period index, robust to t landing a hair below a period start: a resumed segment's
        # times carry different rounding, and t // period would then shift the edges by one step.
        k = int(math.floor(t / self.period + 1e-6))
        if k != self.pwm_k:                     # the duty takes effect at the next PWM period
            self.on_steps = min(max(int(round(self.command * self.steps)), 0), self.steps)
            self.pwm_k, self.applied = k, self.command
        on = self.mode == 1
        if EXACT_EDGES:
            phase, d = max(0.0, t - k * self.period), min(max(self.applied, 0.0), 1.0)
            self.mask.set(self.hs, on and self.dead <= phase < d * self.period)
            self.mask.set(self.ls, on and d * self.period + self.dead <= phase < self.period)
            self.mask.set(self.step_sw, self.load_step)
            return self.mask
        step = int(round((t - k * self.period) / self.dt))
        self.mask.set(self.hs, on and self.dead_steps <= step < self.on_steps)
        self.mask.set(self.ls, on and self.on_steps + self.dead_steps <= step < self.steps)
        self.mask.set(self.step_sw, self.load_step)
        return self.mask

    def _edge_fn(self, t0, t1):
        """The first PWM edge strictly inside (t0, t1), for Pulsim's exact gate edges; inf if none.
        A period not yet started takes the duty the controller has just commanded."""
        if self.mode != 1:
            return math.inf
        k = int(math.floor(t0 / self.period + 1e-6))
        for kk in (k, k + 1, k + 2):
            d = self.applied if kk == self.pwm_k else self.command
            d = min(max(d, 0.0), 1.0)
            start = kk * self.period
            for e in (start, start + self.dead, start + d * self.period, start + d * self.period + self.dead):
                if t0 < e < t1:
                    return e
            if start > t1:
                break
        return math.inf

    def _observer(self, t, x):
        if t >= self.next_tick - 1e-12:         # one control step per control period, as the critical task
            self.next_tick += CONTROL_PERIOD_S
            if self.mode == 1:
                self.command = self.block.block_step(self.reference, x[self.i_v_low], x[self.i_i_l], x[self.i_v_low],
                                                     x[self.i_i_l], x[self.i_v_high], 0.0)
                self.ticks.append((t, self.command))

    def advance(self, seconds):
        if seconds <= 0:
            return
        end = self.t + seconds
        kwargs = {"resume_from": self.snapshot} if self.snapshot is not None else {}
        if EXACT_EDGES:
            kwargs["edge_fn"] = self._edge_fn
        res = p.simulate(self.b, t_start=self.t, t_end=end, dt=self.q["step_s"], engine="pwl", switch_fn=self._switch_fn,
                         step_observer=self._observer, store_every=self.store_every, **kwargs)
        self.snapshot = res.final_snapshot
        times = np.asarray(res.times)
        self.trace_t.append(times)
        self.trace_v.append(np.asarray(res.v("low1")))
        self.trace_i.append(np.asarray(res.i("L1")))
        self.x = np.asarray(res.states)[-1]
        self.t = end

    def capture(self):
        """Waveforms since the last capture, named like the board's scope channels: t, rV1Low_V,
        rI1Low_A (numpy arrays), and the control ticks as (t, duty) pairs (duty)."""
        out = {"t": np.concatenate(self.trace_t) if self.trace_t else np.array([]),
               "rV1Low_V": np.concatenate(self.trace_v) if self.trace_v else np.array([]),
               "rI1Low_A": np.concatenate(self.trace_i) if self.trace_i else np.array([]),
               "duty": np.asarray(self.ticks)}
        self.trace_t, self.trace_v, self.trace_i, self.ticks = [], [], [], []
        return out
