# SPDX-License-Identifier: LGPL-2.1
"""Write test records: one JSON file for tools, one Markdown page for people.

A record stamps what was tested (mission commit and dirty flag, file hashes,
tool versions) next to every criterion's measured value and verdict.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import pathlib
import platform
import subprocess


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mission_identity(mission: pathlib.Path) -> dict:
    def git(*args):
        return subprocess.run(["git", "-C", str(mission), *args], capture_output=True, text=True).stdout.strip()
    return {"commit": git("rev-parse", "HEAD") or "unknown",
            "dirty": bool(git("status", "--porcelain", "--", ".")) }


def write(out: pathlib.Path, procedure: str, level: str, target: str, cases: list[dict],
          mission: pathlib.Path, inputs: dict[str, pathlib.Path], versions: dict[str, str]) -> bool:
    """cases: [{id, title, criterion, measured, expected, unit, passed}]. Returns True if all pass."""
    out.mkdir(parents=True, exist_ok=True)
    passed = all(c["passed"] for c in cases)
    record = {
        "procedure": procedure, "level": level, "target": target,
        "date": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "verdict": "PASS" if passed else "FAIL",
        "mission": mission_identity(mission),
        "inputs": {name: sha256(path) for name, path in inputs.items()},
        "versions": {"python": platform.python_version(), **versions},
        "cases": cases,
    }
    (out / "record.json").write_text(json.dumps(record, indent=2) + "\n")
    lines = [f"# Test record: {procedure}", "",
             f"- Verdict: **{record['verdict']}**",
             f"- Level {level}, target {target}, {record['date']}",
             f"- Mission commit `{record['mission']['commit'][:12]}`" + (" (dirty: not reproducible)" if record["mission"]["dirty"] else ""),
             "- Versions: " + ", ".join(f"{k} {v}" for k, v in record["versions"].items()), "",
             "| Case | Requirement | Check | Measured | Expected | Criterion | Verdict |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for c in cases:
        unit = c.get("unit", "")
        lines.append(f"| {c['id']} | {c.get('requirement', '—')} | {c['title']} | {c['measured']:.4g} {unit} | "
                     f"{c['expected']:.4g} {unit} | {c['criterion']} | {'PASS' if c['passed'] else 'FAIL'} |")
    lines += ["", "Input hashes (SHA-256):", ""] + [f"- `{k}`: `{v[:16]}…`" for k, v in record["inputs"].items()]
    (out / "record.md").write_text("\n".join(lines) + "\n")
    return passed
