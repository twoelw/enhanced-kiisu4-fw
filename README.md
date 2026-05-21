# Kiisu V4a/V4b companion firmware

[![Build firmware](https://github.com/kiisu-io/kiisu4-companion-fw/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/kiisu-io/kiisu4-companion-fw/actions/workflows/build.yml)

To provide compatibility with Flipper Zero Firmwares on Kiisu development boards.

## Status

* Display init & data transfer - working
* Power control - working
* LP5562 led driver emulation - partial
* BQ27220 emulation - working
* BQ25896 emulation - working

## Building

Two options, both produce identical firmware (`KiisuCompanion.elf` + `KiisuCompanion.bin`).

### From the command line (CMake + arm-none-eabi-gcc)

Requires `arm-none-eabi-gcc` (tested with 14.2.Rel1 and 15.2.Rel1), `cmake` (>= 3.22) and `ninja`.

```sh
cmake --preset Debug              # or Release
cmake --build build/Debug         # or build/Release
```

The post-build step produces a `.bin` next to the `.elf`. To flash via ST-Link:

```sh
cmake --build build/Debug --target flash
```

### From VisualGDB

Open `Companion/KiisuCompanion.sln`; the project drives the same CMake configuration.

### Prebuilt binaries

Every push to `main` and every pull request triggers a CI build of both `Debug` and `Release` presets. Artifacts (`.elf` + `.bin`) are attached to each run — see the [Actions tab](https://github.com/kiisu-io/kiisu4-companion-fw/actions/workflows/build.yml).

## Flashing via Kiisu Companion Bridge

* This method will work only for Kiisus produced from 1 September 2025 or later.
* Use Kiisu Companion Bridge app (https://github.com/twoelw/kiisu-companion-bridge). Follow the link and see instructions.
* Kiisu Companion Bridge also included into Kiisu firmware (https://github.com/kiisu-io/kiisu4).

## Flashing via SWD or USB interface

* This method will work always, but you need to make or buy SWD programmer or USB adapter.
* Get manual for your board here: https://github.com/kiisu-io/kiisu4 and follow the instructions.

## Other resources

- [Kiisu.io website](https://kiisu.io)
- [Buy Kiisu here](https://store.rainwalker.ee/products/kiisu-v4)
- [Documentation, schematics and binaries for Kiisu V4](https://github.com/kiisu-io/kiisu4)
- [Our Discord Community](https://discord.gg/kiisu) can help with your questions
  
- [Fork of Flipper Zero official firmware for Kiisu](https://github.com/kiisu-io/kiisu-firmware)
- [Twoelw's GitHub](https://github.com/twoelw) with useful apps and firmware for Kiisu.

- [Cases and stuff for 3D printing on Printables](https://www.printables.com/@planmarks/collections/2364779)
- [Cases and stuff for 3D printing on Makerworld](https://makerworld.com/ru/collections/6517412-kiisu-devboard)
