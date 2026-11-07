# SPDX-License-Identifier: LGPL-2.1
"""The pre-built firmware in bin/ matches its manifest."""
import hashlib
import json
import pathlib
import re
import unittest
import warnings

MISSION = pathlib.Path(__file__).resolve().parent.parent
BIN = MISSION / "bin"


class Prebuilt(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((BIN / "manifest.json").read_text())

    def test_image_matches_its_hash_and_size(self):
        image = (BIN / self.manifest["image"]).read_bytes()
        self.assertEqual(hashlib.sha256(image).hexdigest(), self.manifest["sha256"])
        self.assertEqual(len(image), self.manifest["size"])

    def test_built_from_committed_sources(self):
        build = self.manifest["build"]
        self.assertRegex(build["commit"], r"^[0-9a-f]{40}$")
        self.assertFalse(build["dirty"])
        self.assertRegex(build["source_hash"], r"^[0-9a-f]{16}$")

    def test_board_matches_the_mission(self):
        mission = (MISSION / "mission.yaml").read_text()
        version = re.search(r"version:\s*\"?([0-9.]+)", mission).group(1)
        self.assertEqual(self.manifest["shield"], "twist")
        self.assertEqual(self.manifest["shield_version"].replace("_", "."), version)

    def test_report_when_the_sources_moved_on(self):
        """Not a failure: editing the firmware is the student's job. The app shows it."""
        import importlib.util
        spec = importlib.util.spec_from_file_location("bi", MISSION / "tools" / "build_identity.py")
        bi = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bi)
        if bi.source_hash() != self.manifest["build"]["source_hash"]:
            warnings.warn("bin/ was built from other sources than the current firmware/")


if __name__ == "__main__":
    unittest.main()
