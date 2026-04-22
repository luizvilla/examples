# OwnVerter Rotor Angle Visibility With Hall Sensors

This example introduces the Hall-based position feedback path without enabling the power stage.

It shows how to:

- initialize the default Hall sensor from `app.overlay`
- update the Position API in the critical task
- read electrical angle, mechanical angle, and electrical speed

## Serial commands

- `h`: print the menu
- `p`: start live sensor acquisition
- `i`: stop live sensor acquisition
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously

## Notes

- `app.overlay` selects Hall sensing and provides the motor parameters needed by the Position API.
- This example does not energize the motor phases.
