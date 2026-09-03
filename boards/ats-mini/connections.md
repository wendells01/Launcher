# ATS Mini - Connections

## Display (GC9307/ST7789 170×320, 8-bit Parallel)

| Signal | GPIO | Notes |
|--------|------|-------|
| TFT_CS | 6 | Chip select, active LOW |
| TFT_DC | 7 | Data/Command select |
| TFT_RST | 5 | Reset, active LOW (GC9307 reset pulse needed) |
| TFT_WR | 8 | Write strobe |
| TFT_RD | 9 | Read strobe, keep HIGH when not reading |
| TFT_D0 | 39 | Data bit 0 |
| TFT_D1 | 40 | Data bit 1 |
| TFT_D2 | 41 | Data bit 2 |
| TFT_D3 | 42 | Data bit 3 |
| TFT_D4 | 45 | Data bit 4 (NOT GPIO43!) |
| TFT_D5 | 46 | Data bit 5 (NOT GPIO44!) |
| TFT_D6 | 47 | Data bit 6 |
| TFT_D7 | 48 | Data bit 7 |
| TFT_BL | 38 | Backlight (PWM, active HIGH) |

**Note:** GPIO43/44 are NOT used for display data. D4-D7 are on GPIO45-48.
Display controller is GC9307 (not pure ST7789). Requires panel ID detection
at runtime (RDDID 0x04 byte 3: 0x93=mirrored, 0x85=high gamma, 0xB3=default).

## Encoder

| Signal | GPIO | Notes |
|--------|------|-------|
| Encoder A | 2 | Quadrature A |
| Encoder B | 1 | Quadrature B |
| Encoder KEY | 21 | Push button (active LOW) |

**Note:** No dedicated Esc/Back button. Long-press KEY (~600ms) maps to Esc.

## I2C Bus (shared)

| Signal | GPIO | Notes |
|--------|------|-------|
| SDA | 18 | Shared with SI4732 radio |
| SCL | 17 | Shared with SI4732 radio |

**Note:** I2C bus is shared with the SI4732 FM radio chip. External I2C
modules may conflict. Use SPI header (GPIO11-14) for external modules.

## Battery

| Signal | GPIO | Notes |
|--------|------|-------|
| BAT_ADC | 4 | Battery voltage (3.5-4.2V, no divider) |

## Power

| Signal | GPIO | Notes |
|--------|------|-------|
| SI4732_POWER | 15 | Radio power (not used in Launcher) |
| SI4732_RESET | 16 | Radio reset (not used in Launcher) |

## Audio (not used in Launcher)

| Signal | GPIO | Notes |
|--------|------|-------|
| AUDIO_MUTE | 3 | Audio mute (not used) |
| AUDIO_AMP_EN | 10 | Amplifier enable (not used) |

## Free GPIOs (available for external modules)

| Signal | GPIO | Notes |
|--------|------|-------|
| SPI header | 11-14 | Available for SPI modules (CC1101, etc.) |

## Boot Considerations

- TFT_RD (GPIO9) should be HIGH at boot
- TFT_CS (GPIO6) should be HIGH at boot (not selected)
- Encoder KEY (GPIO21) is active LOW with internal pullup
- Display data bus (GPIO39-42, 45-48) is shared across multiple IO banks

## Hardware Info

- MCU: ESP32-S3-WROOM-1 (16MB flash, 8MB PSRAM octal)
- Display: GC9307 170×320 8-bit parallel ST7789 compatible
- Radio: SI4732 (not used in Launcher port)
- No SD card slot
- No touch screen
- No PMIC (battery directly connected to ADC)
