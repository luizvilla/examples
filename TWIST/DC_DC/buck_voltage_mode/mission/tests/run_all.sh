#!/usr/bin/env bash
# Run every check of the mission that needs no board. Exit code 1 if any
# check fails. A failing L1 verdict is reported, not hidden.
#
# Usage: tests/run_all.sh [out-dir]   (default: out/)
# PYTHON must have requirements.txt installed (default: python3).
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mission="$(cd "$here/.." && pwd)"
out="${1:-$mission/out}"
py="${PYTHON:-python3}"
status=0
step() { echo; echo "== $1"; shift; "$@" || status=1; }

cd "$mission"
step "generated data objects are up to date" "$py" tools/gen_data_objects.py --check
step "mission files and generator tests" "$py" -m unittest discover -s tests -p 'test_*.py'
step "L0: control law on the PC" tests/run_l0.sh "$out/l0"
step "plant model tests" "$py" plant/model_tests/run_model_tests.py "$out/model_tests"
step "L1: control law closed loop on the plant model" "$py" tests/l1/run_l1.py "$out/l1"
echo
[ $status -eq 0 ] && echo "ALL PASSED" || echo "SOME CHECKS FAILED (see above and the records in $out)"
exit $status
