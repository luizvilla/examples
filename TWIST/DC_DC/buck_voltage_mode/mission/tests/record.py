# SPDX-License-Identifier: LGPL-2.1
"""Test records for this mission's runners.

Records, campaigns, the catalogue and the findings register are shared by every
mission and live in OwnStudio's Python package (ownstudio_lab.testing, installed
by requirements.txt). This module keeps the runners' old names.
"""
from __future__ import annotations

import pathlib
import sys

from ownstudio_lab.testing import _show as show  # noqa: F401
from ownstudio_lab.testing import case_verdict, new_campaign, write_record  # noqa: F401

MISSION = pathlib.Path(__file__).resolve().parents[1]


def write(out, procedure, level, target, cases, mission, inputs=None, versions=None, board=None, operator=None):
    return write_record(out, procedure, level, target, cases, mission, inputs, versions, board, operator)


def out_dir(argv: list[str], name: str) -> pathlib.Path:
    """The runner's output folder: the first argument, or <new campaign>/<name>."""
    return pathlib.Path(argv[0]) if argv else new_campaign(MISSION) / name


if __name__ == "__main__":     # tests/record.py new-campaign: print a new campaign folder
    if sys.argv[1:] == ["new-campaign"]:
        print(new_campaign(MISSION))
