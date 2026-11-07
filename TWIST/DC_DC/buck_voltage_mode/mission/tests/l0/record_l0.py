#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""File the L0 results as a test record.

Usage: tests/run_l0.sh BUILD | tests/l0/record_l0.py OUT
Reads the 'PASS|FAIL <id>: <what> (got <x>, expected <y>)' lines of test_control.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
MISSION = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
import record  # noqa: E402

LINE = re.compile(r"^(PASS|FAIL) (\S+): (.*) \(got (\S+), expected (\S+)\)$")


def main(argv):
    out = record.out_dir(argv, "l0-control")
    cases, versions = [], {}
    for line in sys.stdin:
        print(line, end="")
        if line.startswith("control_library "):
            versions["control_library"] = line.split()[1]
        m = LINE.match(line.strip())
        if m:
            verdict, cid, what, got, expected = m.groups()
            cases.append({"id": cid, "title": what, "criterion": what, "measured": float(got),
                          "expected": float(expected), "unit": "", "passed": verdict == "PASS"})
    if not cases:
        print("error: no L0 result lines", file=sys.stderr)
        return 1
    passed = record.write(out, "L0 control law", "L0", "pc", cases, MISSION,
                          {"firmware/control.cpp": MISSION / "firmware" / "control.cpp"}, versions)
    print(f"record: {out / 'record.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
