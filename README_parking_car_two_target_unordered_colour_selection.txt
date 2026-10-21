Two-target parking code - unordered colour selection

This package is based on the latest Python and ESP32 source files supplied in the conversation.

Changed in the Python controller only:
- Removes alphabetical sorting from initial target-colour learning.
- Uses set membership for available/remaining target colours.
- Removes alphabetical sorting from simultaneous visible-target selection.
- Either unparked learned colour can latch the target and start the existing approach/parking logic.
- Once a colour has been parked, it remains excluded; after UNPARK the search is for the remaining learned colour.

No ESP32 control logic or motion parameters were changed; the included .ino is copied from the latest supplied source.

Validation: Python bytecode syntax check passed. Not tested on the physical car. Before running, flash the included matching firmware only if it is not already the exact latest firmware you are using. Test with wheels clear / adequate space and keep Ctrl+C available.
