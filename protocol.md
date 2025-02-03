# Protocol
This document describes the protocol for communication with the rotator being
developed by the UNL Aerospace Rocketry Club.

All commands are 4 characters long. All arguments are separated by spaces.
Commands are terminated with a newline. The final argument of all commands must
be a checksum (TBD).

Optional arguments are specified with `{}`.

Horizontal axis is azimuth, vertical axis is altitude.

Positive and negative must be specified.

Commands respond with `OK\n` if successful, `ERR <REASON>\n` if not. Some commands
respond with return arguments after the `OK` and before the `\n`.

# Version 1.0.0

## Types
command:    `ABCD` # 4 ASCII characters

string arg: `ABC`  # 3 ASCII characters

float:      `+000.000` or `-000.000` # Signed floating point

integer:    `+0000000` or `-0000000` # 7 digit raw integer


## Commands
### `DVER`
Args: `float`

Description: Degrees to move to in the vertical axis.

### `DHOR`
Args: `float`

Description: Degrees to move to in the horizontal axis.

### `CALV`
Args: `{SET}`

Description: Automagically calibrates vertical axis. Specifying SET sets 
vertical calibration position to current position.

### `CALH`
Args:

Description: Sets horizontal calibration position to current position.

### `MOVV`
Args: `integer`

Description: Moves by the specified number of steps in the vertical axis.

### `MOVH`
Args: `integer`

Description: Moves by the specified number of steps in the horizontal axis.

### `GETP`
Args:

Returns: `float float`

Description: Gets the current position for both Vertical then Horizontal.

### `INFO`
Args:

Returns: `TBD`

Description: Gets info!

### `SSPD`
Args: `integer`

Description: Sets speed for both axes.

### `GSPD`
Args:

Returns: `integer`

Description: Gets speed for both axes.
