#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""L2 link test: the board runs this mission's firmware and its ThingSet
link carries every data object of data_objects.yaml.

Usage: tests/l2/run_l2_link.py [out-dir] [--board SERIAL]

Needs a Twist on USB, flashed with this mission's firmware (bin/ or a
build of firmware/). The converter does not need power: the test never
sets the mode to power, and it reads measurements only to check their
types. Exits 0 and files no record when no board is connected.

Needs the ownstudio_lab package (OwnStudio's app/python).
"""
import argparse
import pathlib
import sys

import yaml

MISSION = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(MISSION / "tests"))
sys.path.insert(0, str(MISSION / "tools"))
import build_identity  # noqa: E402
import record  # noqa: E402

try:
    from ownstudio_lab.boards import find_boards
    from ownstudio_lab.thingset import ThingSetError, find_thingset_port
except ImportError:
    print("SKIPPED: the ownstudio_lab package is not installed (pip install -e <OwnStudio>/app/python)")
    sys.exit(0)

IDLE = 0
TEST_REFERENCE = 11.0          # within the envelope, and different from the 12 V default
TYPES = {"float": (int, float), "uint8": int, "uint16": int, "uint32": int, "int32": int,
         "bool": bool, "string": str}


def matches(value, kind):
    if kind != "bool" and isinstance(value, bool):
        return False
    return isinstance(value, TYPES[kind])


def check(link, spec):
    cases = []

    def case(id_, title, measured, expected, passed, criterion):
        cases.append({"id": id_, "title": title, "measured": measured, "expected": expected,
                      "unit": "", "passed": bool(passed), "criterion": criterion})

    build = link.get("Build")
    sources = build_identity.source_hash()
    case("LINK-01", "Board runs this mission's sources", build["rSourceHash"], sources,
         build["rSourceHash"] == sources, "source hash equal")

    wrong = []
    for group in spec["groups"]:
        values = link.get(group["name"])
        for obj in group["objects"]:
            if obj["name"] not in values or not matches(values[obj["name"]], obj["type"]):
                wrong.append(f"{group['name']}/{obj['name']}")
    case("LINK-02", "Every data object is on the board with its type", len(wrong), 0, not wrong,
         "missing or mistyped = 0" + (f" ({', '.join(wrong)})" if wrong else ""))

    root = [name for name, _ in link.tree("")]
    dupes = sorted({n for n in root if root.count(n) > 1})
    case("LINK-03", "No two root objects share a name", len(dupes), 0, not dupes,
         "duplicates = 0" + (f" ({', '.join(dupes)})" if dupes else ""))

    before = link.get("Converter/wVoltageRef_V")
    try:
        link.update("Converter", {"wVoltageRef_V": TEST_REFERENCE})
        readback = link.get("Converter/wVoltageRef_V")
    finally:
        link.update("Converter", {"wVoltageRef_V": before})
    case("LINK-04", "A written reference reads back", readback, TEST_REFERENCE,
         abs(readback - TEST_REFERENCE) < 0.01, "abs(readback − written) < 0.01 V")

    link.update("Converter", {"Mode": IDLE})
    mode = link.get("Converter/Mode")
    case("LINK-05", "The converter is idle", mode, IDLE, mode == IDLE, "Mode = 0 (PWM off)")

    try:
        link.update("Measurements", {"rDuty": 0.5})
        code = "accepted"
    except ThingSetError as err:
        code = f":{err.code:02X}" if err.code is not None else "no answer"
    case("LINK-06", "A read-only object refuses writes", code, ":A3", code == ":A3", "answer is :A3 Forbidden")
    return cases, build


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", nargs="?", type=pathlib.Path, default=MISSION / "out" / "l2_link")
    parser.add_argument("--board", help="USB serial number of the board")
    args = parser.parse_args(argv)

    boards = [b for b in find_boards() if not b.in_bootloader and (not args.board or b.serial_number == args.board)]
    if not boards:
        print("SKIPPED: no board connected")
        return 0
    if len(boards) > 1:
        print("error: several boards are connected; choose one with --board", file=sys.stderr)
        return 2
    board = boards[0]
    device, link = find_thingset_port(board)
    if link is None:
        print(f"FAIL: board {board.serial_number} has no port answering ThingSet "
              "(is it running a mission firmware?)", file=sys.stderr)
        return 1
    spec = yaml.safe_load((MISSION / "data_objects.yaml").read_text())
    with link:
        cases, build = check(link, spec)
    for c in cases:
        print(f"{'PASS' if c['passed'] else 'FAIL'} {c['id']}: {c['title']}: "
              f"{record.show(c['measured'])} ({c['criterion']})")
    passed = record.write(args.out, "LINK ThingSet link", "L2", "board", cases, MISSION,
                          {"data_objects.yaml": MISSION / "data_objects.yaml"},
                          {"port": device},
                          board={"serial_number": board.serial_number, "product": board.product,
                                 "commit": build["rCommit"], "dirty": build["rDirty"],
                                 "source_hash": build["rSourceHash"]})
    print(f"record: {args.out / 'record.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
