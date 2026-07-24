# OwnVerter Measurements Enable

This example initializes the OwnVerter power shield in Buck mode and enables the default measurements, without ever starting the PWM or driving a duty cycle. There is no power flow; it only reads back and logs:

- `I1_LOW`, `I2_LOW`, `I3_LOW`
- `I_HIGH`
- `V1_LOW`, `V2_LOW`, `V3_LOW`
- `V_HIGH`
- The three leg temperatures (`T1`, `T2`, `T3`)

## Notes

- The serial interface continuously prints `Vhigh`, `Ihigh`, `V1`, `V2`, `V3`, `I1`, `I2`, `I3`, `T1`, `T2` and `T3`.
- OwnVerter multiplexes its three leg temperatures on a single ADC channel, so only one of them can be selected at a time. The example cycles through `TEMP_1` → `TEMP_2` → `TEMP_3`, giving the analog mux a few background task iterations to settle between switches before each value is read back.
- This example is a starting point for checking that sensor readings look right before moving on to examples that actually drive the PWM (`power_open_loop_manual_duty`, `sine_modulation_ot_sin`, etc.).
