# OwnVerter Open-Loop Manual Duty

This is the first beginner OwnVerter example. It shows how to:

- initialize the OwnVerter power stage
- enter and leave power mode from the serial console
- change a PWM duty cycle manually

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

- The duty cycle is limited to the `[0.05 ; 0.95]` range.
- All three legs receive the same duty cycle in this first example.
- This example is intentionally simple and does not use the sensor stack yet.
