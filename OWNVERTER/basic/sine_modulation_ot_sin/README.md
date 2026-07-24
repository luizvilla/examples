# OwnVerter Sine Modulation With `ot_sin`

This third beginner OwnVerter example replaces the fixed duty cycle with a sinusoidal modulation law built with `ot_sin`.

It shows how to:

- generate an electrical angle ramp
- convert a modulation index into three phase duty cycles
- observe the relation between angle, sine values, and PWM duties

## Serial commands

- `h`: print the menu
- `p`: enable modulation
- `i`: disable modulation
- `u`: increase the modulation index
- `d`: decrease the modulation index
- `f`: increase the electrical frequency
- `g`: decrease the electrical frequency
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously

## Notes

- The modulation index is limited to keep all duties inside a safe range.
- This example is still open loop: no current or position control is active.
