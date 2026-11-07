#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""Copy a firmware build into bin/ with its manifest.

Usage: tools/release.py <core-dir> [pio-env]

Run by tools/build_firmware.sh --release after a successful build. Refuses a
build from dirty sources, so the pre-built binary always names a commit.
bin/manifest.json links the binary's SHA-256 to the build identity the board
reports over ThingSet (group Build).
"""
import argparse
import hashlib
import json
import pathlib
import shutil
import subprocess
import sys

MISSION = pathlib.Path(__file__).resolve().parent.parent
BIN = MISSION / "bin"
IMAGE = "firmware.mcuboot.bin"
sys.path.insert(0, str(MISSION / "tools"))
import build_identity  # noqa: E402


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def core_commit(core):
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=core, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def stamped_identity(core):
    """The identity actually compiled in, read back from the build copy's header."""
    values = {}
    for line in (core / "src" / "build_identity.h").read_text().splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[0] == "#define":
            values[parts[1]] = parts[2].strip('"')
    return {"commit": values["BUILD_COMMIT"], "dirty": values["BUILD_DIRTY"] == "true",
            "source_hash": values["BUILD_SOURCE_HASH"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("core", type=pathlib.Path)
    parser.add_argument("env", nargs="?", default="USB")
    args = parser.parse_args(argv)

    ident = stamped_identity(args.core)
    if ident["dirty"] or ident["commit"] == "unknown":
        print("release: refusing a build from uncommitted sources; commit first", file=sys.stderr)
        return 1
    if ident["source_hash"] != build_identity.source_hash():
        print("release: the sources changed since the build; build again", file=sys.stderr)
        return 1
    image = args.core / ".pio" / "build" / args.env / IMAGE
    BIN.mkdir(exist_ok=True)
    shutil.copyfile(image, BIN / IMAGE)
    ini = (MISSION / "firmware" / "app.ini").read_text()
    shield_version = next(l.split("=")[1].strip() for l in ini.splitlines() if l.startswith("board_shield_version"))
    manifest = {
        "image": IMAGE,
        "sha256": sha256(BIN / IMAGE),
        "size": (BIN / IMAGE).stat().st_size,
        "build": ident,
        "core_commit": core_commit(args.core),
        "board": "spin",
        "shield": "twist",
        "shield_version": shield_version,
        "upload": "mcuboot-serial",
    }
    (BIN / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"release: bin/{IMAGE} {manifest['sha256'][:16]} sources {ident['source_hash']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
