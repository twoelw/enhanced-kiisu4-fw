# Kiisu V4a/V4b companion firmware

[![Build firmware](https://github.com/kiisu-io/kiisu4-companion-fw/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/kiisu-io/kiisu4-companion-fw/actions/workflows/build.yml)

To provide compatibility with Flipper Zero Firmwares on Kiisu development boards.

## Flashing via Kiisu Companion Bridge

The easiest method for end users — no SWD programmer or USB adapter required. The Bridge app is shipped inside the Kiisu fork of the Flipper Zero firmware ([kiisu-io/kiisu-firmware](https://github.com/kiisu-io/kiisu-firmware)). Standalone upstream sources live at [twoelw/kiisu-companion-bridge](https://github.com/twoelw/kiisu-companion-bridge).

**Requirements:** Kiisu V4a/V4b produced from 1 September 2025 or later.

**Steps:**

1. Get a `KiisuCompanion.bin` and copy it to the device's SD card. The easiest source is the rolling [Latest release](https://github.com/kiisu-io/kiisu4-companion-fw/releases/latest) — automatically rebuilt from `main`, no GitHub login needed. Alternatively, build it locally (`build/Release/KiisuCompanion.bin`).
2. On the Kiisu, navigate to: **[OK] → Apps → Kiisu → Kiisu Companion Bridge**.
3. Choose **Select BIN file** and pick the `.bin` you copied.
4. The update takes a while. Wait for **both** the audible beep **and** the on-screen success message.

> ⚠️ **Do not remove the battery or unplug USB before the beep.** The companion MCU's flash is being rewritten and interrupting power mid-write will leave the device in a non-bootable state recoverable only via SWD.
>
> If you heard the beep but the success message never appeared, it is safe to power-cycle the device (remove battery and USB, then reinsert) — the flash write has already completed.

## Flashing via SWD or USB interface

* This method will work always, but you need to make or buy SWD programmer or USB adapter.
* Get manual for your board here: https://github.com/kiisu-io/kiisu4 and follow the instructions.

## Status

* Display init & data transfer - working
* Power control - working
* LP5562 led driver emulation - partial
* BQ27220 emulation - working
* BQ25896 emulation - working

## Building

Use the **Release** preset for any firmware that will actually run on hardware — the Debug preset is too slow to keep up with the host MCU's SPI display stream.

### From the command line (CMake + arm-none-eabi-gcc)

Requires `arm-none-eabi-gcc` 15.2.Rel1 (the version pinned in CI), `cmake` (>= 3.22) and `ninja`.

```sh
cmake --preset Release
cmake --build build/Release
```

The post-build step produces a `.bin` next to the `.elf`. To flash via ST-Link:

```sh
cmake --build build/Release --target flash
```

### From VisualGDB

Open `Companion/KiisuCompanion.sln`; the project drives the same CMake configuration.

### Prebuilt binaries

Every push to `main` publishes the resulting `.elf` and `.bin` to the rolling [Latest release](https://github.com/kiisu-io/kiisu4-companion-fw/releases/latest) — public, no login required. Per-run artifacts (including PR builds) are also available under the [Actions tab](https://github.com/kiisu-io/kiisu4-companion-fw/actions/workflows/build.yml) for 90 days.

## Other resources

- [Kiisu.io website](https://kiisu.io)
- [Buy Kiisu here](https://store.rainwalker.ee/products/kiisu-v4)
- [Documentation, schematics and binaries for Kiisu V4](https://github.com/kiisu-io/kiisu4)
- [Our Discord Community](https://discord.gg/kiisu) can help with your questions
  
- [Fork of Flipper Zero official firmware for Kiisu](https://github.com/kiisu-io/kiisu-firmware)
- [Twoelw's GitHub](https://github.com/twoelw) with useful apps and firmware for Kiisu.

- [Cases and stuff for 3D printing on Printables](https://www.printables.com/@planmarks/collections/2364779)
- [Cases and stuff for 3D printing on Makerworld](https://makerworld.com/ru/collections/6517412-kiisu-devboard)
