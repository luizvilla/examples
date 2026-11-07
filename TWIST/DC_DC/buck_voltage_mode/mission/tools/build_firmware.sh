#!/usr/bin/env bash
# Build the mission firmware against a throwaway OwnTech Core checkout.
#
# Usage: tools/build_firmware.sh [--release] <core-dir> [pio-env]
#
# --release also copies the image to bin/ with bin/manifest.json
# (tools/release.py); it refuses sources that are not committed.
#
# <core-dir> must be a Core checkout used only for this build: the script
# overwrites its src/ with the mission's firmware/ files. The mission folder
# itself is never modified. pio-env defaults to USB.
set -euo pipefail

release=0
if [ "${1:-}" = "--release" ]; then release=1; shift; fi
core="${1:?usage: tools/build_firmware.sh [--release] <core-dir> [pio-env]}"
env="${2:-USB}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pio="${PIO:-$HOME/.platformio/penv/bin/pio}"

[ -f "$core/platformio.ini" ] || { echo "error: $core is not an OwnTech Core checkout" >&2; exit 2; }

# Replace the Core's application with the mission firmware.
find "$core/src" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
cp -r "$here/firmware/." "$core/src/"
# Stamp the build identity (commit, dirty flag, source hash) into the copy.
python3 "$here/tools/build_identity.py" "$core/src/build_identity.h"

(cd "$core" && "$pio" run -e "$env")
if [ "$release" = 1 ]; then
    python3 "$here/tools/release.py" "$core" "$env"
fi
