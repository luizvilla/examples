# OwnVerter Measurements Enable

This second beginner OwnVerter example keeps the same manual open-loop PWM interface and adds default OwnVerter measurements:

- `I1_LOW`
- `I2_LOW`
- `I_HIGH`
- `V1_LOW`
- `V2_LOW`
- `V_HIGH`

## Serial commands

- `h`: print the menu
- `p`: enable PWM output
- `i`: disable PWM output
- `u`: increase duty cycle
- `d`: decrease duty cycle
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously

## Notes

- The printed serial line shows the duty cycle followed by the measured values.
- This example is a bridge between pure PWM control and the later closed-loop control examples.
