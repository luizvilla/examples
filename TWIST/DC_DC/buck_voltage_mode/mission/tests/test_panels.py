# SPDX-License-Identifier: LGPL-2.1
"""Panels in panels/ only use data objects the firmware declares."""
import json
import pathlib
import unittest

import yaml

MISSION = pathlib.Path(__file__).resolve().parent.parent


def declared():
    spec = yaml.safe_load((MISSION / "data_objects.yaml").read_text())
    paths, writable = set(), set()
    for group in spec["groups"]:
        for obj in group["objects"]:
            path = f"{group['name']}/{obj['name']}"
            paths.add(path)
            if not obj["name"].startswith("r"):
                writable.add(path)
    return paths, writable


def references(widget):
    s = widget.get("settings", {})
    reads = [d["var"] for d in [s.get("sourceDef")] + [x.get("a") for x in s.get("seriesDefs", [])] if d]
    return [r.strip("/") for r in reads], [s["writePath"].strip("/")] if "writePath" in s else []


class Panels(unittest.TestCase):
    def test_every_panel_uses_declared_data_objects(self):
        paths, writable = declared()
        panels = sorted((MISSION / "panels").glob("*.panel.json"))
        self.assertTrue(panels)
        for panel in panels:
            for pane in json.loads(panel.read_text())["panes"]:
                for widget in pane["widgets"]:
                    reads, writes = references(widget)
                    for path in reads:
                        self.assertIn(path, paths, f"{panel.name}: {widget['type']} reads unknown {path}")
                    for path in writes:
                        self.assertIn(path, writable, f"{panel.name}: {widget['type']} writes {path}, which is not writable")


if __name__ == "__main__":
    unittest.main()
