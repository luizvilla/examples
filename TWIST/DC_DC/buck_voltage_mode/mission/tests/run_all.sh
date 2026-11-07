#!/usr/bin/env bash
# Run every check of the mission that needs no board. Exit code 1 if any
# check fails. A failing L1 verdict is reported, not hidden.
#
# Every run is a new campaign: records/<date>_<commit>_<time>/, one folder per
# procedure. At the end, records/control-center.html shows where the validation
# stands (all campaigns, the catalogue and the findings register).
#
# Usage: tests/run_all.sh [campaign-dir]   (default: a new campaign in records/)
# PYTHON must have requirements.txt installed (default: python3).
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mission="$(cd "$here/.." && pwd)"
py="${PYTHON:-python3}"
out="${1:-$("$py" "$here/record.py" new-campaign)}"
build="$(mktemp -d)"
status=0
step() { echo; echo "== $1"; shift; "$@" || status=1; }

cd "$mission"
step "generated data objects are up to date" "$py" tools/gen_data_objects.py --check
step "mission files and generator tests" "$py" -m unittest discover -s tests -p 'test_*.py'
l0() { set -o pipefail; tests/run_l0.sh "$build/l0" | "$py" tests/l0/record_l0.py "$out/l0-control"; }
step "L0: control law on the PC" l0
step "plant model tests" "$py" plant/model_tests/run_model_tests.py "$out/model-tests"
step "L1: control law closed loop on the plant model" "$py" tests/l1/run_l1.py "$out/l1-buck-step-01"
step "control center" "$py" -m ownstudio_lab.testing "$mission" --html "$mission/records/control-center.html"
echo
[ $status -eq 0 ] && echo "ALL PASSED" || echo "SOME CHECKS FAILED (see above and the campaign $out)"
exit $status
