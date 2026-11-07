#!/usr/bin/env bash
# Build and run the L0 test cases of the control law on the PC.
#
# Usage: tests/run_l0.sh [build-dir]   (default: a temporary directory)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mission="$(cd "$here/.." && pwd)"
source "$here/deps.sh"
lib="$(control_library_dir)"
out="${1:-$(mktemp -d)}"
mkdir -p "$out"

"${CXX:-g++}" -std=c++20 -O2 -Wall -Wextra \
    -I"$mission/firmware" -I"$here/shim" -I"$lib/src" \
    "$here/l0/test_control.cpp" "$mission/firmware/control.cpp" "$lib/src/pid.cpp" \
    -o "$out/test_control"
echo "control_library $(git -C "$lib" rev-parse --short HEAD 2>/dev/null || echo unknown)"
"$out/test_control"
