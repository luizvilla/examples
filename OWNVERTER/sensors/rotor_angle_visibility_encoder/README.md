# OwnVerter Rotor Angle Visibility With Incremental Encoder

This example introduces the ABZ encoder-based position feedback path without enabling the power stage.

It shows how to:

- initialize the default incremental encoder from `app.overlay`
- update the Position API in the critical task
- read raw encoder counts together with mechanical and electrical angle and speed

## Serial commands

- `h`: print the menu
- `p`: start live sensor acquisition
- `i`: stop live sensor acquisition
- `r`: dump the scope buffer
- `q`: restart the scope buffer
- `m`: replay the scope buffer continuously

## Notes

- `app.overlay` selects the ABZ sensor and provides the motor and encoder parameters needed by the Position API.
- This example does not energize the motor phases.
