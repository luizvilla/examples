# SPDX-License-Identifier: LGPL-2.1
"""The mission can be solved: a reference PI meets every simulated requirement.

The example's PI (firmware/control.cpp as shipped) fails REQ-TRACKING on purpose:
that is the exercise. This test writes the reference solution's gains into a
copy of the mission and runs both procedures on the simulator, at the nominal
conditions and at the bench's (30 V in, 33 ohm). Slow: about two minutes.

The reference solution is for the instructor; students can read it here.
"""
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

MISSION = pathlib.Path(__file__).resolve().parents[1]
SOLUTION = {"kp": 0.0012, "ti": 7.5e-5}     # crossover about 100 Hz, far below the 3.5 kHz filter resonance

try:
    from ownstudio_lab import procedure as proc  # noqa: F401
except ImportError:
    proc = None


@unittest.skipIf(proc is None, "needs ownstudio_lab")
class Solvable(unittest.TestCase):
    def run_procedure(self, mission, name, *options):
        r = subprocess.run([sys.executable, "-m", "ownstudio_lab.procedure", ".", f"tests/procedures/{name}.py",
                            "--target", "simulator", "--campaign", str(mission.parent / "campaign"), *options],
                           cwd=mission, capture_output=True, text=True)
        return r.returncode, r.stdout + r.stderr

    def test_the_reference_solution_passes_every_simulated_requirement(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = pathlib.Path(tmp) / "mission"
            shutil.copytree(MISSION, m, ignore=shutil.ignore_patterns("records", ".git", "__pycache__"))
            subprocess.run(["git", "init", "-q", str(m)], check=True)
            sys.path.insert(0, str(m / "design"))
            import loop
            loop.write_gains(SOLUTION["kp"], SOLUTION["ti"], m / "firmware" / "control.cpp")
            for name, options in (("step_response", ()), ("reference_step", ()),
                                  ("reference_step", ("--option", "v_in=30", "--option", "load_ohm=33"))):
                status, out = self.run_procedure(m, name, *options)
                self.assertEqual(status, 0, f"{name} {options}:\n{out}")

    def test_the_example_pi_fails_tracking(self):
        """The starting point of the exercise: the shipped PI is too slow for REQ-TRACKING."""
        with tempfile.TemporaryDirectory() as tmp:
            m = pathlib.Path(tmp) / "mission"
            shutil.copytree(MISSION, m, ignore=shutil.ignore_patterns("records", ".git", "__pycache__"))
            subprocess.run(["git", "init", "-q", str(m)], check=True)
            status, out = self.run_procedure(m, "reference_step")
        self.assertEqual(status, 1, out)
        self.assertIn("REQ-TRACKING allows 10 ms", out)
        self.assertIn("(FAIL)", out)

if __name__ == "__main__":
    unittest.main()
