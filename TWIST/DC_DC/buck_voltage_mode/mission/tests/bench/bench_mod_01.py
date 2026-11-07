#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""BENCH-MOD-01: the plant model against the Twist, in steady state, with an operator.

Bench: HIGH from a supply at 30 V limited to 1 A, 33 ohm on LOW1 and on LOW2,
a multimeter. For each reference (5, 8, 12 V) the procedure switches the
converter on, lets it settle, averages the board's readings, and asks the
operator for the multimeter voltages and the supply current. It then runs the
plant model at the measured input voltage and loads, and compares.

Cases, per reference R (BENCH-MOD-01.<letter><R>):
  a  LOW1 by multimeter within 2 % of the reference       (the loop, judged by the meter)
  b  duty within 5 % of the model's duty for the metered LOW1 (the plant model)
  c  supply current within 15 % of the model's input current
  d  LOW1, LOW2 and HIGH sensor readings against the meter   (information only)

The converter is switched on only after the operator confirms, and always
switched off at the end, on any error, or if LOW1 exceeds 16 V or a current 2 A.

Usage: tests/bench/bench_mod_01.py [--board SERIAL] [out-dir]
"""
from __future__ import annotations

import argparse
import pathlib
import statistics
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
MISSION = HERE.parents[1]
sys.path[:0] = [str(HERE.parent), str(MISSION / "plant")]
import record  # noqa: E402

REFERENCES_V = (5.0, 8.0, 12.0)
LOAD_OHM = 33.0
SUPPLY = "30 V, current limited to 1 A"
V_LOW_MAX, I_MAX = 16.0, 2.0
SETTLE_S, AVERAGE_S, SAMPLE_S = 2.0, 2.0, 0.2
TOL_VOUT, TOL_DUTY, TOL_IIN = 0.02, 0.05, 0.15


class Abort(Exception):
    pass


def model_duty(v_in: float, v_out: float, load_ohm: float) -> tuple[float, float]:
    """(duty, input current) the plant model needs for v_out at v_in, two legs loaded alike.
    Two open-loop runs bracket the duty; the plant is close to linear in duty."""
    from twist_buck import build, load_params
    q = load_params()
    q["port_high_V"] = v_in

    def mean_out(d):
        tr = build(q, load_ohm=load_ohm).run_open_loop(d, 6e-3)
        return float(tr.v_low[tr.t > 4e-3].mean())
    d0 = v_out / v_in
    lo, hi = mean_out(d0 * 0.95), mean_out(d0 * 1.05)
    duty = d0 * 0.95 + (v_out - lo) / (hi - lo) * d0 * 0.10
    i_in = 2 * (v_out ** 2 / load_ohm) / v_in      # both legs; switching losses not modelled
    return duty, i_in


class Bench:
    """The procedure, apart from the board and the operator so it can be tested."""

    def __init__(self, link, ask, confirm, log=print, sleep=time.sleep, model=model_duty):
        self.link, self.ask, self.confirm, self.log, self.sleep, self.model = link, ask, confirm, log, sleep, model

    def measure(self, seconds):
        rows = []
        for i in range(int(round(seconds / SAMPLE_S)) + 1):
            if i:
                self.sleep(SAMPLE_S)
            m = self.link.get("Measurements")
            rows.append(m)
            if m["rV1Low_V"] > V_LOW_MAX or max(abs(m["rI1Low_A"]), abs(m["rI2Low_A"]), abs(m["rIHigh_A"])) > I_MAX:
                raise Abort(f"limit exceeded: {m}")
        return {k: statistics.fmean(r[k] for r in rows) for k in rows[0]}

    def run(self):
        cases, operator = [], {}
        if self.link.get("Converter")["Mode"] != 0:
            raise Abort("the converter is not idle at the start")
        if not self.confirm(f"Bench ready? HIGH from a supply at {SUPPLY}; {LOAD_OHM:g} ohm on LOW1 and on LOW2; "
                            "multimeter ready. The converter will be switched on."):
            raise Abort("the operator did not confirm the bench")
        try:
            for ref in REFERENCES_V:
                self.link.update("Converter", {"wVoltageRef_V": ref})
                self.link.update("Converter", {"Mode": 1})
                self.log(f"reference {ref:g} V: converter on, settling")
                self.measure(SETTLE_S)
                board = self.measure(AVERAGE_S)
                tag = f"{ref:g}"
                v1 = self.ask(f"Reference {ref:g} V. Multimeter on LOW1 (V)?")
                v2 = self.ask(f"Reference {ref:g} V. Multimeter on LOW2 (V)?")
                i_in = self.ask(f"Reference {ref:g} V. Supply current shown by the supply (A)?")
                v_in = self.ask(f"Reference {ref:g} V. Supply voltage shown by the supply (V)?")
                operator.update({f"V_LOW1_meter_{tag}V": v1, f"V_LOW2_meter_{tag}V": v2,
                                 f"I_supply_{tag}V": i_in, f"V_supply_{tag}V": v_in})
                duty_model, i_model = self.model(v_in, v1, LOAD_OHM)
                cases += [
                    {"id": f"BENCH-MOD-01.a{tag}", "title": f"LOW1 by multimeter, reference {tag} V",
                     "measured": v1, "expected": ref, "unit": "V", "criterion": f"within {TOL_VOUT:.0%}",
                     "passed": abs(v1 - ref) <= TOL_VOUT * ref},
                    {"id": f"BENCH-MOD-01.b{tag}", "title": f"Duty against the plant model's, reference {tag} V",
                     "measured": board["rDuty"], "expected": duty_model, "unit": "",
                     "criterion": f"within {TOL_DUTY:.0%} of the model's duty for {v1:.3g} V at {v_in:.3g} V in",
                     "passed": abs(board["rDuty"] - duty_model) <= TOL_DUTY * duty_model},
                    {"id": f"BENCH-MOD-01.c{tag}", "title": f"Supply current against the model's, reference {tag} V",
                     "measured": i_in, "expected": i_model, "unit": "A",
                     "criterion": f"within {TOL_IIN:.0%} (model: lossless, both legs at the metered voltage)",
                     "passed": abs(i_in - i_model) <= TOL_IIN * i_model},
                    {"id": f"BENCH-MOD-01.d{tag}", "title": f"Sensors against the meter, reference {tag} V",
                     "measured": f"V1 {board['rV1Low_V']:.3g} / V2 {board['rV2Low_V']:.3g} / VH {board['rVHigh_V']:.3g}",
                     "expected": f"V1 {v1:.3g} / V2 {v2:.3g} / VH {v_in:.3g}", "unit": "V",
                     "criterion": "information only", "verdict": "INFO", "board_mean": board},
                ]
        finally:
            self.link.update("Converter", {"Mode": 0})
            self.log("converter off")
        return cases, operator


def _ask_number(question):
    while True:
        try:
            return float(input(question + " ").replace(",", "."))
        except ValueError:
            print("a number, please")


def _confirm(question):
    return input(question + " [y/N] ").strip().lower() in ("y", "yes", "o", "oui")


def main(argv=None):
    from ownstudio_lab.boards import find_boards
    from ownstudio_lab.thingset import find_thingset_port
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", nargs="?", type=pathlib.Path)
    parser.add_argument("--board", help="USB serial number of the board")
    args = parser.parse_args(argv)
    boards = [b for b in find_boards() if not b.in_bootloader and (not args.board or b.serial_number == args.board)]
    if len(boards) != 1:
        print("error: connect exactly one board, or name it with --board", file=sys.stderr)
        return 2
    board = boards[0]
    device, link = find_thingset_port(board)
    if link is None:
        print(f"error: board {board.serial_number} has no port answering ThingSet", file=sys.stderr)
        return 2
    with link:
        build = link.get("Build")
        identity = {"serial_number": board.serial_number, "commit": build["rCommit"], "dirty": build["rDirty"],
                    "source_hash": build["rSourceHash"]}
        try:
            cases, operator = Bench(link, _ask_number, _confirm).run()
        except Abort as err:
            print(f"ABORTED: {err}", file=sys.stderr)
            return 1
    out = record.out_dir([str(args.out)] if args.out else [], "bench-mod-01")
    passed = record.write(out, "BENCH-MOD-01 plant model against the board", "bench", "board", cases, MISSION,
                          {"plant/params.yaml": MISSION / "plant" / "params.yaml",
                           "plant/twist_buck.py": MISSION / "plant" / "twist_buck.py"},
                          {"port": device, "supply": SUPPLY, "load_ohm": str(LOAD_OHM)}, identity, operator)
    for c in cases:
        print(f"{record.case_verdict(c)} {c['id']}: {c['title']}: "
              f"{record.show(c['measured'])} vs {record.show(c['expected'])} ({c['criterion']})")
    print(f"record: {out / 'record.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
