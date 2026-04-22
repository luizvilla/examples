# OwnVerter Speed Control With Hall Sensors

This example adds an outer speed loop on top of the existing `Iq` current loop.

The user commands speed, the speed PI generates `Iq_ref`, and the current regulators generate the phase voltage references.

## Serial commands

- `h`: print the menu
- `p`: request power mode
- `i`: request idle mode
- `o`: restart current offset calibration
- `u`: increase speed reference
- `d`: decrease speed reference
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously
