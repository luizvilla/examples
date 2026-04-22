# OwnVerter `Iq` Current Control With Hall Sensors

This example closes the inner FOC current loop while keeping the Hall-based Position API.

It shows how to:

- transform measured phase currents into the `dq` frame
- regulate `Iq`
- keep `Id = 0`
- observe the difference between current reference and measured current

## Serial commands

- `h`: print the menu
- `p`: request power mode
- `i`: request idle mode
- `o`: restart current offset calibration
- `u`: increase `Iq_ref`
- `d`: decrease `Iq_ref`
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously
