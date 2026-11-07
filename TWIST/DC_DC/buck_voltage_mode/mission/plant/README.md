# Twist buck plant model

A switched model of one Twist leg in buck mode, written with Pulsim 2.0.0's Python builder (`twist_buck.py`). The HIGH port source feeds the high-side and low-side switches, each with a body diode. Then come the 33 µH inductor, the LOW1 capacitor and the load. The leg switches at 200 kHz with dead time. Every parameter is in `params.yaml`, with its source and a confidence level.

## Model tests

`model_tests/run_model_tests.py` checks the switched model against the analytic buck of `design/sizing.py`, then writes a test record (`out/model_tests/record.md` and `record.json`).

| Case | Check | Tolerance |
| --- | --- | --- |
| PM-01a, PM-01b | Mean LOW1 voltage at duty 0.3 and 0.5, against (D − t_dead · f) · V_high | 2 % |
| PM-02 | Inductor current ripple | 10 % |
| PM-03 | LOW1 voltage ripple, against ΔI / (8 C f) | 15 % |
| PM-04 | Output filter resonance, from the start-up ringing at 60 Ω | 5 % |

Run it with a Python that has `requirements.txt` installed:

```sh
python plant/model_tests/run_model_tests.py
```

## Not yet done

- **Comparison with the board.** Run the buck example on a Twist and file the same quantities as a bench record. Until then, the model is only shown to agree with first principles, not with the hardware.
- **LOW1 capacitance (found in step 3).** The two 33 µF electrolytics are on the HIGH side. LOW has 3 × 4.7 µF ceramics (C206, C209, C212) and a 47 µF electrolytic (C203, EEE-FN2A470UL, ESR 0.70 Ω) switched in by Q203; the firmware connects it with `shield.power.connectCapacitor(ALL)`. The ceramics' DC-bias derating is not modelled.
- **Low-confidence parameters.** The dead time (take it from Core's HRTIM configuration and the driver), the on-resistance of Q203, and the switch and diode parameters. No inductor DCR is modelled yet.
- **A PulsimGUI view.** Core2 keeps its plant as a `.pulsim` schematic. Here the builder script is the source, and a `.pulsim` view for PulsimGUI is still to be generated from it.
