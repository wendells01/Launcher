/***************************************************************************************
** File:       interface.cpp
** Board:      ATS Mini (ESP32-S3 + GC9307 170x320 parallel 8-bit + rotary encoder)
**
** HAL wiring for M5Launcher.  Implements the 8 functions that
** Launcher/include/interface.h declares as weak in main.cpp.
**
** Hardware summary:
**   Display : GC9307 controller (masquerades as ST7789), 170x320, 8-bit parallel
**             D0-D7 on GPIO 39-42 / 45-48, WR=8, RD=9, CS=6, DC=7, RST=5
**   Backlight: GPIO 38, PWM via HAL
**   Encoder  : A=2, B=1, push-button=21 (no dedicated Esc button)
**   Battery  : ADC on GPIO 4, no voltage divider
**   Power    : deep-sleep, wake on encoder push (GPIO 21 LOW)
***************************************************************************************/

#include "idf/launcher_platform.h"
#include "powerSave.h"
#include <interface.h>

#include "hal/bright/bright.h"
#include "hal/device.h"
#include "hal/inputs/encoder.h"

// ---------------------------------------------------------------------------
// Display panel: always GC9307 on the ATS Mini.
//
// The PAR8 (Arduino_ESP32PAR8) data bus is write-only — there is no read
// path, so runtime RDDID detection is impossible.  All known ATS Mini
// units ship with the GC9307 variant identified as did3 == 0x93 (mirrored
// and inverted).  We apply that correction unconditionally in
// _post_setup_gpio().
//
// The ardgfx backend's tft->writecommand() is a deliberate no-op.
// Low-level panel commands must go through the concrete data bus:
//   tft->dataBus()->writeCommand(cmd)
//   tft->dataBus()->write(data)
// Or the convenience wrappers:
//   tft->dataBus()->sendCommand(cmd)   // beginWrite + writeCommand + endWrite
//   tft->dataBus()->sendData(data)     // beginWrite + write + endWrite
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Encoder configuration
// ---------------------------------------------------------------------------
static DeviceEncoder encoderCfg() {
    DeviceEncoder cfg;
    cfg.pin_a = 2;    // Encoder A
    cfg.pin_b = 1;    // Encoder B
    cfg.pin_sel = 21; // Encoder push-button (Select)
    cfg.pin_esc = -1; // No dedicated Esc button on ATS Mini
    return cfg;
}

// ---------------------------------------------------------------------------
// Long-press Esc detection state
//
// Since pin_esc == -1, the HAL encoder driver will never set EscPress.
// We track GPIO 21 hold time ourselves: if the push-button is held LOW
// for > 600 ms, we fire EscPress and consume the press so it does not
// also register as SelPress.
// ---------------------------------------------------------------------------
static unsigned long _escPressStart = 0;
static bool _escConsumed = false;

/***************************************************************************************
** Function name: _setup_gpio()
** Location:      main.cpp (called first, before tft->begin())
** Description:   Basic GPIO init — TFT control pins, reset sequence, encoder
***************************************************************************************/
void _setup_gpio() {
    // TFT control pins as outputs
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST, OUTPUT);
    pinMode(TFT_WR, OUTPUT);
    pinMode(TFT_RD, OUTPUT);
    pinMode(TFT_BL, OUTPUT);

    // Idle states
    digitalWrite(TFT_RD, HIGH);
    digitalWrite(TFT_CS, HIGH);

    // GC9307 reset sequence (proven in Bruce port):
    // LOW for 100 ms, then HIGH for 200 ms before the bus takes over.
    digitalWrite(TFT_RST, LOW);
    delay(100);
    digitalWrite(TFT_RST, HIGH);
    delay(200);

    // Encoder pins — internal pull-ups, init via HAL
    hal_encoder_init(encoderCfg(), EncoderLatchMode::TWO03);
}

/***************************************************************************************
** Function name: _post_setup_gpio()
** Location:      main.cpp (called after tft->begin() + setRotation)
** Description:   Backlight PWM + GC9307 display correction (full RDDID detection)
***************************************************************************************/
void _post_setup_gpio() {
    // Backlight: attach PWM on GPIO 38, set full brightness (percent 0-100)
    hal_bright_attach(TFT_BL);
    hal_bright_set(TFT_BL, 100);

    // --- GC9307 panel correction via RDDID detection ---
    //
    // Read panel ID (RDDID 0x04, byte 3) to detect panel variant:
    //   0x93 = mirrored & inverted (needs invertDisplay(false) + MADCTL 0xE8)
    //   0x85 = high gamma (needs GAMSET curve 8 + WRCACE 0xB1)
    //   0xB3 = normal (no correction needed)
    //
    // main.cpp:236 calls invertDisplay(true) globally before this runs,
    // so 0x93 correction must call invertDisplay(false) to override.

    tft->begin();
    tft->setRotation(3);

    uint8_t did3 = tft->readcommand8(0x04, 3);

    if (did3 == 0x93) {
        // Panel 0x93: mirrored & inverted variant
        tft->invertDisplay(false);  // Override global invertDisplay(true)
        tft->writecommand(0x36);    // MADCTL
        tft->writedata(0xE8);       // MV | MX | MY | BGR — corrects mirroring
    } else if (did3 == 0x85) {
        // Panel 0x85: high gamma variant
        tft->writecommand(0x26);    // GAMSET
        tft->writedata(8);          // Gamma curve 8
        tft->writecommand(0x55);    // WRCACE / brightness control
        tft->writedata(0xB1);       // Enable content adaptive brightness
    }
    // 0xB3 = normal panel, no correction needed
}

/***************************************************************************************
** Function name: getBattery()
** Location:      display.cpp
** Description:   Battery percentage 0-100 from ADC on GPIO 4
***************************************************************************************/
int getBattery() {
    static bool adcInitialized = false;
    if (!adcInitialized) {
        analogSetAttenuation(ADC_11db);
        adcInitialized = true;
    }

    uint32_t mv = analogReadMilliVolts(ANALOG_BAT_PIN);

    // Approximate Li-ion voltage curve (no fuel-gauge IC on this board).
    // No voltage divider — ADC reads the battery directly.
    const float MIN_VOLTAGE = 3500.0f; // 0 %
    const float MAX_VOLTAGE = 4200.0f; // 100 %

    int percent = (int)(((float)mv - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0f);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}

/***************************************************************************************
** Function name: _setBrightness()
** Location:      settings.cpp
** Description:   Set backlight brightness 0-100 via HAL PWM
***************************************************************************************/
void _setBrightness(uint8_t brightval) { hal_bright_set(TFT_BL, brightval); }

/***************************************************************************************
** Function name: InputHandler()
** Location:      main.cpp (called every loop cycle from taskInputHandler)
** Description:   Read encoder rotation/press; detect long-press for Esc
***************************************************************************************/
void InputHandler(void) {
    // Reset per-cycle press flags
    checkPowerSaveTime();
    PrevPress = false;
    NextPress = false;
    SelPress = false;
    AnyKeyPress = false;
    EscPress = false;

    // Let the HAL read encoder rotation and short press.
    // With pin_esc == -1 the HAL never sets EscPress.
    hal_encoder_poll(encoderCfg());

    // --- Board-specific long-press Esc detection ---
    // GPIO 21 is the encoder push-button.  Track how long it has been
    // held LOW.  If > 600 ms, fire EscPress and consume the press.
    bool btnHeld = (digitalRead(encoderCfg().pin_sel) == LOW);
    unsigned long now = millis();

    if (btnHeld) {
        if (_escPressStart == 0) {
            _escPressStart = now;
            _escConsumed = false;
        }
        if (!_escConsumed && (now - _escPressStart) > 600) {
            EscPress = true;
            SelPress = false; // consume: don't also fire Select
            _escConsumed = true;
        }
    } else {
        // Button released — reset timer
        _escPressStart = 0;
        _escConsumed = false;
    }

    // Mark AnyKeyPress if any logical input was registered
    if (PrevPress || NextPress || SelPress || EscPress) { AnyKeyPress = true; }

    // Debounce: if any key is pressed, wait briefly so the same press
    // is not re-read on the next cycle.
    if (AnyKeyPress) {
        unsigned long t = launcherMillis();
        while ((launcherMillis() - t) < 200 && (PrevPress || NextPress || SelPress || EscPress)) {
            // encoder may still be moving; HAL poll not needed here
        }
    }
}

/***************************************************************************************
** Function name: powerOff()
** Location:      mykeyboard.cpp
** Description:   Enter deep-sleep; wake on encoder push (GPIO 21 LOW)
***************************************************************************************/
void powerOff() {
    hal_bright_set(TFT_BL, 0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_deep_sleep_start();
}

/***************************************************************************************
** Function name: reboot()
** Location:      mykeyboard.cpp
** Description:   Restart the device
***************************************************************************************/
void reboot() { ESP.restart(); }

/***************************************************************************************
** Function name: checkReboot()
** Location:      mykeyboard.cpp
** Description:   Long-press encoder button (>2 s) triggers powerOff
***************************************************************************************/
void checkReboot() {
    if (digitalRead(encoderCfg().pin_sel) != LOW) return;

    vTaskSuspend(xHandle);
    unsigned long start = launcherMillis();
    while (digitalRead(encoderCfg().pin_sel) == LOW) {
        if (launcherMillis() - start > 2000) { powerOff(); }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskResume(xHandle);
}
