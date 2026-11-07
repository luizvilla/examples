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
| `design/sizing.py` | Analytic buck formulas used for sizing |
| `plant/` | Pulsim model of one Twist leg, its parameters with sources, and its model tests |
| `tests/` | L0 tests of the control law, the L1 load-step test, consistency tests, the test-record writer |

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

## Status (2026-11-07)

| Check | Result |
| --- | --- |
| Firmware build against OwnTech Core 8111a0d | Builds: flash 79 %, RAM 41 % |
| L0, control law (6 cases) | Pass |
| Plant model tests (5 checks against the analytic buck) | Pass |
| L1, BUCK-STEP-01 | 3 of 4 pass. **REQ-UNDERSHOOT fails**: LOW1 dips to 11.11 V on the 1 A to 2 A step (5 % allows 11.4 V). The dip is the output filter's own response; the example's PI is too slow to damp it. Fixing that is the student's task |
| L2, on the board | Not run yet |

## Known limits

- On the simulator, the controller sees the plant's exact LOW1 voltage. Sensor scaling, calibration and the HRTIM are tested only on the board (L2).
- Several plant parameters are placeholders (see `plant/README.md`). The model agrees with first principles, but it hasn't been compared with a real Twist yet.
- The firmware has not been flashed on a board since ThingSet replaced the serial menu.
