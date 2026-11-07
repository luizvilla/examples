# SPDX-License-Identifier: LGPL-2.1
"""The catalogue is well formed and agrees with requirements.yaml in both directions."""
import pathlib
import unittest

from ownstudio_lab import testing

MISSION = pathlib.Path(__file__).resolve().parents[1]


class CatalogueTest(unittest.TestCase):
    def test_catalogue_is_well_formed(self):
        testing.load_catalogue(MISSION)

    def test_requirements_and_catalogue_agree(self):
        self.assertEqual(testing.check_requirements(MISSION, testing.load_catalogue(MISSION)), [])

    def test_findings_name_catalogued_tests(self):
        ids = {t["id"] for t in testing.load_catalogue(MISSION)["tests"]}
        for f in testing.load_findings(MISSION):
            for c in f["criteria"]:
                self.assertIn(c.split(".")[0], ids, f"{f['issue']} names {c}")


if __name__ == "__main__":
    unittest.main()
