# SPDX-License-Identifier: LGPL-2.1
"""BUCK-REF-01 on a fake board (the simulator behind the firmware's ThingSet and scope), with its twin simulation.

Slow (about half a minute): the fake board runs the plant one control period at a time.
Run: python -m unittest tests.test_reference_step
"""
import json
import pathlib
import sys
import tempfile
import unittest

MISSION = pathlib.Path(__file__).resolve().parents[1]
PROCEDURE = MISSION / "tests" / "procedures" / "reference_step.py"

try:
    from ownstudio_lab import procedure as proc
    from ownstudio_lab.fakes import fake_board_target
    from ownstudio_lab.target import load_harness
except ImportError:          # ownstudio_lab older than step 4
    proc = None


class Answers:
    def __init__(self, answers):
        self.answers, self.logs, self.records = list(answers), [], []

    def log(self, text):
        self.logs.append(text)

    def ask(self, kind, question):
        return self.answers.pop(0)

    def record(self, path, verdict):
        self.records.append((path, verdict))

    def done(self, status, error=None):
        pass


@unittest.skipIf(proc is None, "needs ownstudio_lab from step 4 (fakes, arm, twin)")
class OnAFakeBoard(unittest.TestCase):
    def test_board_run_with_scope_and_twin(self):
        with tempfile.TemporaryDirectory() as tmp:
            build = pathlib.Path(tmp) / "block"
            harness = load_harness(MISSION, v_in=30.0, load_ohm=33.0, build_dir=str(build))
            target = fake_board_target(harness)
            io = Answers([True, 33.0])
            status = proc.run_procedure(MISSION, PROCEDURE, "board", io, allow_power=True,
                                        campaign=pathlib.Path(tmp) / "campaign", options={"hold_s": 0.1},
                                        target=target, twin=True)
            self.assertEqual(status, 0, io.logs)
            (board_path, _), (sim_path, _) = io.records
            board = json.loads((board_path / "record.json").read_text())
            sim = json.loads((sim_path / "record.json").read_text())

            self.assertEqual(board["conditions"], {"v_high_V": 30.0, "load_ohm": 33.0})
            self.assertEqual(sim["conditions"], board["conditions"])           # simulated at the board's conditions
            self.assertEqual(board["twin"], sim_path.name)
            settle = {r["target"]: next(c for c in r["cases"] if c["id"] == "BUCK-REF-01c") for r in (board, sim)}
            self.assertIn("measured", settle["board"])                       # no longer blocked on the board
            # Same plant, same control.cpp: the scope's 100 µs samples settle like the 1 µs simulation, within 2 periods.
            self.assertAlmostEqual(settle["board"]["measured"], settle["simulator"]["measured"], delta=0.2)
            wave = (board_path / "waveform.csv").read_text().splitlines()
            self.assertEqual(wave[0], "t_s,rV1Low_V,rI1Low_A")
            self.assertEqual(len(wave) - 1, 1024 - 205 + 200)                  # the scope's window from 20 ms before the step
            self.assertEqual(target.link.writes[-1], ("Converter", {"Mode": 0}))


if __name__ == "__main__":
    unittest.main()
