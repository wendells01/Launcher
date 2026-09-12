# Launcher for ATS Mini

This is a fork of [bmorcelli/Launcher](https://github.com/bmorcelli/Launcher),
maintained for one board only: the ATS Mini handheld radio (ESP32-S3, 16 MB
flash, 8 MB PSRAM). Everything here outside `boards/ats-mini` is upstream
code; the port itself lives in `boards/ats-mini` plus a small set of static
OTA hooks in `src/onlineLauncher.cpp`.

## What is the ATS Mini port

The ATS Mini has no SD card slot, a 170x320 GC9307 display on an 8-bit
parallel bus, and a rotary encoder with push button instead of the usual
M5 buttons. The port adapts Launcher to that hardware: encoder navigation,
runtime display-variant detection, NVS-only settings, and a static OTA
catalog so firmware can be installed over WiFi with no USB cable and no SD.

The Bruce pentest firmware for the same radio lives in a separate repo:
[wendells01/bruce-ats-mini](https://github.com/wendells01/bruce-ats-mini).
It is installable from this Launcher via the OTA catalog below.

## Hardware pinout (ATS Mini)

| GPIO | Role |
|---|---|
| 39, 40, 41, 42, 45, 46, 47, 48 | Display data bus D0-D7 (8-bit parallel) |
| 6 / 7 / 5 / 8 / 9 / 38 | Display CS / DC / RST / WR / RD / backlight |
| 2 / 1 / 21 | Encoder A / B / push (key, internal pull-up) |
| 18 / 17 / 16 / 15 | I2C SDA / SCL, SI4732 reset / power |
| 4 | Battery voltage (ADC only, no fuel gauge) |

## Display and encoder

The display is driven as ST7789 over an 8-bit parallel bus. ATS Mini units
shipped with three panel variants, told apart at boot by the RDDID reply;
mirrored/inverted panels get MADCTL 0xE8 and high-gamma panels get an
alternate gamma curve. Rotation defaults to landscape.

The encoder uses 1 detent = 1 step, push = Enter (short press), Esc on
long press. A short edge-armed latch keeps quick taps from being lost
between input polls; power-off detection is non-blocking.

## Static OTA catalog

The catalog is hosted in
[wendells01/ats-mini-hub](https://github.com/wendells01/ats-mini-hub)
(`ats-mini-ota.json`, 4 entries: ATS Mini Original v2.38, Bruce ATS Mini
v1.0.0, Chinese Firmware, ATS-Mini Marauder). The firmware fetches it at
runtime. Installs flash the app slice of the merged image into the OTA
slot, creating the target data partition (spiffs or littlefs) first, so
installed firmware boots with a formattable filesystem.

## Flashing

Flash the merged binary at address `0x0` with ESPFlash (or esptool):

`esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 460800 write_flash 0x0 Launcher-ats-mini.bin`

Use DIO flash mode at 40 MHz. QIO at 80 MHz hangs the loader on this
board. Download the binary from
[Releases](https://github.com/wendells01/Launcher/releases) (semver tags,
`v0.1.0` through `v1.0.0`).

## Dev workflow

There is no CI for this port. Build locally with PlatformIO and publish
by hand:

```
pio run -e ats-mini
gh release create vX.Y.Z Launcher-ats-mini.bin
```

Future releases continue the semver line (`v1.0.1`, `v1.0.2`, ...).
Do not rebuild old tags; each release binary is immutable.

## Credits

Upstream: [bmorcelli/Launcher](https://github.com/bmorcelli/Launcher).
User-contributed fixes in this port: encoder step/pull-up correction;
knob click handling ported from Bruce; tap latch and TLS root fix for
OTA downloads; app-slice OTA install; data-partition creation on
install; static catalog hosted in ats-mini-hub.
