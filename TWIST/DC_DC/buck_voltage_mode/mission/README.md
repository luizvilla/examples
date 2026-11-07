# Mission: make the Twist hold 12 V under a load step

The first OwnStudio mission, built on the buck voltage-mode example. The student sizes the buck's output filter and compares it with the real Twist. Then they make the PI controller hold 12 V when the load doubles, first in the Pulsim simulator (L1), then on the board (L2).

OwnStudio opens this folder through `mission.yaml`. Every file is plain text, and everything also works without OwnStudio.

## Contents

| Path | What it is |
| --- | --- |
| `mission.yaml` | Board, five-block description, ports, safety envelope (HIGH ≤ 60 V), starting state, steps |
| `requirements.yaml` | Four requirements on LOW1, each with its number and the test case that verifies it. The test runners read their criteria from here |
| `data_objects.yaml` | Every value exchanged with the computer through ThingSet. `tools/gen_data_objects.py` generates `firmware/data_objects.{h,cpp}` from it |
| `firmware/main.cpp` | Setup, tasks, sensors and PWM: the example, with ThingSet instead of the serial menu |
| `firmware/control.cpp` | The control law. Plain C++ with no hardware calls, so the same file runs on the Spin and in Pulsim |
| `firmware/app.ini`, `app.conf`, `app.overlay` | PlatformIO libraries, Zephyr options (ThingSet shell) and the second USB serial port |
| `design/sizing.py`, `design/loop.py` | Sizing of L and C from the requirements, and the PI on a Bode plot with its margins (`--json` for OwnStudio's Design workbench) |
| `plant/` | Pulsim model of one Twist leg, its parameters with sources, and its model tests |
| `tests/` | The test catalogue and findings register; L0 tests of the control law; procedures in `tests/procedures/` (load step, reference step) that run on the simulator (`tests/sim_harness.py`) or the board; the L2 link test; the bench check `tests/bench/`; studies; consistency tests |
| `panels/` | Lab panels in the subset of modular's dashboard format that OwnStudio reads (gauges, numbers, setpoint, power button, time plot) |
| `bin/` | The pre-built firmware and its manifest (SHA-256 and build identity), so the Lab works before anyone compiles |
| `tools/` | Data object generator, firmware build (`--release` refreshes `bin/`), build identity |

## Running the checks

The checks need a C++20 compiler and a Python with `requirements.txt` installed (Pulsim 2.0.0, NumPy, PyYAML).

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
PYTHON=.venv/bin/python tests/run_all.sh
```

To build the firmware, use a throwaway OwnTech Core checkout; the script overwrites its `src/`:

```sh
tools/build_firmware.sh /path/to/throwaway/Core
```

The firmware reports its build identity over ThingSet (group `Build`): the git commit, a dirty flag and a hash of `firmware/` plus `data_objects.yaml`.

With a Twist on USB (no power needed), flash `bin/` and run the L2 link test. Both use OwnStudio's `ownstudio_lab` package (`pip install -e <OwnStudio>/app/python`) and OwnTech's `mcumgr`:

```sh
python -m ownstudio_lab.flash bin/manifest.json --mcumgr /path/to/mcumgr
tests/l2/run_l2_link.py
```

Tests are registered as in Core2. `tests/catalogue.yaml` declares every test once; each run files test records (a JSON file for tools, a Markdown page for people) in a new campaign folder, `records/<date>_<commit>_<time>/`; `tests/findings.yaml` links failing criteria to issues. `tests/run_all.sh` ends by writing `records/control-center.html`, which shows where the validation stands; OwnStudio's Lab and Home show the same states. The template ignores `records/` in git. The shared tooling is OwnStudio's `ownstudio_lab.testing`, installed by `requirements.txt`.

The link test checks that the board runs this folder's sources, that every data object is there with its type, and that writes work. It never sets the mode to power.

A procedure runs on either target with OwnStudio's runner (or from the Lab):

```sh
python -m ownstudio_lab.procedure . tests/procedures/step_response.py --target simulator
python -m ownstudio_lab.procedure . tests/procedures/reference_step.py --target board --allow-power   # converter on: at the bench only
```

## Status (2026-11-07)

| Check | Result |
| --- | --- |
| Firmware build against OwnTech Core 8111a0d | Builds: flash 79 %, RAM 41 % |
| L0, control law (6 cases) | Pass |
| Plant model tests (5 checks against the analytic buck) | Pass |
| L1, BUCK-STEP-01 | 3 of 4 pass. **REQ-UNDERSHOOT fails**: LOW1 dips to 11.32 V on the 1 A to 2 A step (5 % allows 11.4 V). The dip is the output filter's own response. A study finds that no PI at the firmware's 100 µs period gets below 5.67 % ([issue #11](https://github.com/luizvilla/Theia/issues/11)); the requirement is Luiz's decision |
| L1, BUCK-REF-01 (10 V → 12 V) | Pass on the simulator |
| L2, link test on the Twist (6 cases) | Pass |
| Bench, BENCH-MOD-01 and BUCK-REF-01 on the board | Not run yet: need an operator at the bench |
| L2, the load step on the board (BUCK-STEP-02) | Blocked: needs a switched load and the firmware scope |

## Known limits

- On the simulator, the controller sees the plant's exact LOW1 voltage. Sensor scaling, calibration and the HRTIM are tested only on the board (L2).
- Several plant parameters are placeholders (see `plant/README.md`). The model agrees with first principles, but it hasn't been compared with a real Twist yet.
- The firmware has been flashed and its ThingSet link tested on an unpowered Twist only. Measurements read sensor offsets, not real values.
