#!/usr/bin/env bash
# Build the mission firmware against a throwaway OwnTech Core checkout.
#
# Usage: tools/build_firmware.sh <core-dir> [pio-env]
#
# <core-dir> must be a Core checkout used only for this build: the script
# overwrites its src/ with the mission's firmware/ files. The mission folder
# itself is never modified. pio-env defaults to USB.
set -euo pipefail

core="${1:?usage: tools/build_firmware.sh <core-dir> [pio-env]}"
env="${2:-USB}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pio="${PIO:-$HOME/.platformio/penv/bin/pio}"

[ -f "$core/platformio.ini" ] || { echo "error: $core is not an OwnTech Core checkout" >&2; exit 2; }

# Replace the Core's application with the mission firmware.
find "$core/src" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
cp -r "$here/firmware/." "$core/src/"

cd "$core"
"$pio" run -e "$env"
