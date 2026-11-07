# SPDX-License-Identifier: LGPL-2.1
"""Consistency of the mission files. Run: python3 -m unittest discover -s tests -p 'test_*.py'"""
import ast
import pathlib
import unittest

import yaml

MISSION = pathlib.Path(__file__).resolve().parent.parent


def load(name):
    return yaml.safe_load((MISSION / name).read_text())


def l1_case_ids():
    """Test case ids the L1 runner reports, read from its source without importing Pulsim."""
    tree = ast.parse((MISSION / "tests" / "l1" / "run_l1.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "CASE_IDS" for t in node.targets):
            return set(ast.literal_eval(node.value))
    raise AssertionError("CASE_IDS not found in run_l1.py")


class Links(unittest.TestCase):
    def setUp(self):
        self.mission = load("mission.yaml")
        self.requirements = load(self.mission["requirements"])["requirements"]

    def test_every_requirement_is_verified_by_a_known_test_case(self):
        known = l1_case_ids()
        for r in self.requirements:
            with self.subTest(requirement=r["id"]):
                self.assertTrue(r.get("verified_by"), "no test case: shown as a gap on the Verification layer")
                self.assertLessEqual(set(r["verified_by"]), known)

    def test_every_test_case_verifies_a_requirement(self):
        linked = {c for r in self.requirements for c in r["verified_by"]}
        self.assertEqual(l1_case_ids() - linked, set())

    def test_files_opened_by_blocks_and_steps_exist(self):
        paths = [b["opens"] for b in self.mission["blocks"].values() if b.get("opens")]
        paths += [s["opens"] for s in self.mission["steps"] if s.get("opens")]
        for path in paths:
            with self.subTest(path=path):
                self.assertTrue((MISSION / path).exists())

    def test_starting_state_names_existing_writable_data_objects(self):
        objects = {o["name"] for g in load("data_objects.yaml")["groups"] for o in g["objects"]}
        for name in self.mission["starting_state"]:
            with self.subTest(name=name):
                self.assertIn(name, objects)
                self.assertFalse(name.startswith("r") and name[1:2].isupper(), "read-only objects cannot be set")

    def test_envelope_stays_in_the_teaching_profile(self):
        self.assertLessEqual(self.mission["envelope"]["HIGH"]["v_max_V"], 60.0)

    def test_firmware_default_reference_matches_the_requirement(self):
        vout = next(r for r in self.requirements if r["id"] == "REQ-VOUT")
        self.assertIn(f"voltage_reference = {vout['value']:g};", (MISSION / "firmware" / "main.cpp").read_text())
        self.assertEqual(self.mission["starting_state"]["wVoltageRef_V"], vout["value"])


if __name__ == "__main__":
    unittest.main()
