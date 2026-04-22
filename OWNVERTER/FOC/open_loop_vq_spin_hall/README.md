# OwnVerter Open-Loop `Vq` Spin With Hall Sensors

This example is the first motor-spinning example of the beginner FOC path.

It uses:

- the Hall-based Position API from `app.overlay`
- a commanded rotating electrical angle
- a fixed `Vq` reference in the Park frame

## Serial commands

- `h`: print the menu
- `p`: request power mode
- `i`: request idle mode
- `o`: restart current offset calibration
- `u`: increase `Vq`
- `d`: decrease `Vq`
- `f`: increase commanded electrical speed
- `g`: decrease commanded electrical speed
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously

## Notes

- The commanded angle is open loop.
- The measured Hall angle is still logged so the user can compare commanded and measured rotor motion.
