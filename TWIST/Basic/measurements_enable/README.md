# Twist Measurements Enable

This example initializes the Twist power shield in Buck mode and enables the default measurements, without ever starting the PWM or driving a duty cycle. There is no power flow; it only reads back and logs:

- `I1_LOW`
- `I2_LOW`
- `I_HIGH`
- `V1_LOW`
- `V2_LOW`
- `V_HIGH`

## Notes

- The serial interface continuously prints a colon-separated line: `I1_LOW:V1_LOW:I2_LOW:V2_LOW:I_HIGH:V_HIGH`.
- This example is a starting point for checking that sensor readings look right before moving on to examples that actually drive the PWM (Open-Loop PWM, buck voltage mode, buck current mode, etc.).
