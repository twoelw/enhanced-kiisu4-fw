# Kiisu V4a/V4b companion firmware

To provide compatibility with Flipper Zero Firmwares on Kiisu development boards.

## Status

Display init & data transfer - working
Power control - working
LP5562 led driver emulation - partial
BQ27220 emulation - working
BQ25896 emulation - working

## Building

Use CMAKE or VisualGDB project (Companion\KiisuCompanion.sln)

## Flashing via Kiisu Companion Bridge

This method will work only for Kiisus produced from 1 September 2025 or later.
Use Kiisu Companion Bridge app (https://github.com/twoelw/kiisu-companion-bridge). Follow the link and see instructions.
Kiisu Companion Bridge also included into Kiisu firmware (https://github.com/kiisu-io/kiisu4).

## Flashing via SWD or USB interface

This method will work always, buy you need to make or buy SWD programmer or USB adapter.
Get manual for your board here: https://github.com/kiisu-io/kiisu4 and follow the instructions.
