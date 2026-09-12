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

#include <Arduino.h>
#include <ArduinoJson.h>

#include "hal/bright/bright.h"
#include "hal/device.h"
#include "hal/inputs/encoder.h"
#include "partition_install_layout.h"

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
    cfg.pin_a = 2;     // Encoder A
    cfg.pin_b = 1;     // Encoder B
    cfg.pin_sel = 21;  // Encoder push-button (Select)
    cfg.pin_esc = -1;  // No dedicated Esc button on ATS Mini
    cfg.pullup = true; // Active-low push button idles HIGH via internal pull-up
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

// ---------------------------------------------------------------------------
// Short-tap bridge: edge-armed 250 ms latch that re-asserts SelPress across
// taskInputHandler wipes (resetGlobals every ~75 ms, main.cpp:88-99) while
// the HAL 200 ms re-fire gate (encoder.cpp:51) suppresses re-assertion.
// Without it a tap released before the menu's next check() (:1270, once per
// loopOptions iteration, stalled by PAR8 flushes and 200 ms scroll steps at
// display.cpp:34) is lost; rotation is unaffected (quadrature via GPIO
// interrupts, encoder.cpp:35-37).
// Window counts from the PRESS edge and never extends on re-press, so the
// remainder left after consume is shorter than any menu-transition path
// (full drawOptions render + PAR8 flush before the next level's first
// check; OTA/SD/WUI/CFG actions slower still) — one Enter per tap, no
// double-fire. Suppressed while _escConsumed (long-press Esc) and while the
// screen is dim/off (HAL wake-tap swallow, encoder.cpp:56-59).
// ---------------------------------------------------------------------------
static bool _selPrevHeld = false;
static bool _selLatched = false;
static unsigned long _selLatchUntil = 0;
// 250 ms covers the worst menu-check gap with margin: the input task samples
// every ~75 ms once AnyKeyPress is set (main.cpp:88) and a redraw iteration
// adds a full drawOptions render + PAR8 flush (display.cpp:718-869, worst when
// displayScrollingText fires a scroll step: 200 ms deadTime gate at
// display.cpp:34, each step a full tft->display flush). No double-Enter: the
// flag is consumed destructively by check() (globals.h:151-156), and after
// consume the menu unwinds through drawOptionsErase + operation() + break
// (display.cpp:1270-1274) plus a full render before any next menu's first
// check — strictly longer than a steady-state iteration, so the leftover
// window always expires mid-transition.
static const unsigned long kSelLatchMs = 250;

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
    hal_encoder_init(encoderCfg(), EncoderLatchMode::FOUR3);
}

/***************************************************************************************
** Function name: _post_setup_gpio()
** Location:      main.cpp (called after tft->begin() + setRotation)
** Description:   Backlight PWM + GC9307 display init (stock Arduino_GFX)
***************************************************************************************/
void _post_setup_gpio() {
    // Backlight: attach PWM on GPIO 38, set full brightness (percent 0-100)
    hal_bright_attach(TFT_BL);
    hal_bright_set(TFT_BL, 100);

    // --- GC9307 panel notes (no auto-detection possible here) ---
    //
    // Bruce (TFT_eSPI) reads the panel ID via RDDID (0x04, byte 3) and applies
    // per-panel fixes (0x93 = MADCTL 0xE8, 0x85 = GAMSET/WRCACE). That path does
    // NOT exist in this stack: Arduino_GFX's parallel bus (Arduino_ESP32PAR8)
    // implements no read methods, and tft_display::writecommand() is a
    // deliberate no-op — raw register writes must go through tft->dataBus().
    //
    // Default = stock Arduino_GFX ST7789 init (same as the CYD-2432S028 PAR8
    // precedent, which works fine). If a panel shows mirrored/inverted colors,
    // enable ONE bus-level correction below (verified API: dataBus() ->
    // Arduino_ESP32PAR8 -> writeCommand()/write()):
    //
    //   auto *bus = tft->dataBus();
    //   bus->beginWrite();
    //   // 0x93 variant: bus->writeCommand(0x36); bus->write(0xE8);
    //   // 0x85 variant: bus->writeCommand(0x26); bus->write(8);
    //   bus->endWrite();

    tft->begin();
    tft->setRotation(3);
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
    // Rotation + short-press Select come straight from the HAL, matching the
    // reference encoder board (lilygo-t-embed-cc1101). The task loop already
    // calls resetGlobals() before every cycle, so no flag clearing here, and
    // no busy-wait: spinning this task while flags are set holds the input
    // lock and lets the next resetGlobals() wipe an unconsumed SelPress.
    // With pin_esc == -1 the HAL never sets EscPress (see detector below).
    hal_encoder_poll(encoderCfg());

    // --- Board-specific long-press Esc detection (non-blocking) ---
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

    // Esc-consume invariant (companion to the Bruce-port checkReboot below):
    // once a >600 ms hold has fired EscPress, keep Select suppressed while
    // the button stays down. hal_encoder_poll() re-asserts SelPress on every
    // level poll past its 200 ms re-fire gate, so without this a long-press
    // would deliver BOTH Esc and Select on release.
    if (_escConsumed) SelPress = false;

    // Short-tap bridge: re-assert SelPress from the edge-armed latch so one
    // menu check() always observes the tap despite intervening wipes.
    if (!_escConsumed && !isScreenOff && !dimmer) {
        bool latchLive = _selLatched && (long)(now - _selLatchUntil) < 0;
        if (btnHeld && !_selPrevHeld && !latchLive) {
            _selLatched = true;
            _selLatchUntil = now + kSelLatchMs;
            latchLive = true;
        }
        if (latchLive) SelPress = true;
        else _selLatched = false;
    } else {
        _selLatched = false;
    }
    _selPrevHeld = btnHeld;
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
    // Non-blocking power-off detector: the old version parked the loop task in
    // `while (LOW) vTaskDelay(100)` for the whole hold, so any press held past
    // the 250 ms tap latch expired before the menu's next check(SelPress) and
    // the tap was lost (main-menu loss window was the hold itself, up to the
    // ~2 s power-off threshold). Same 100 ms tick accounting, same >2 s
    // powerOff, but the loop task keeps reaching the menu checks every pass,
    // so the latch only has to cover input phase + one redraw iteration.
    // Short taps return after one read, exactly like before.
    static unsigned long lastTick = 0;
    static unsigned int heldTicks = 0;
    unsigned long now = millis();
    if (digitalRead(encoderCfg().pin_sel) != LOW) {
        heldTicks = 0;
        lastTick = now;
        return;
    }
    if (now - lastTick >= 100) {
        lastTick = now;
        if (++heldTicks > 20) { powerOff(); } // ~2 s long press
    }
}

// ---------------------------------------------------------------------------
// Static OTA catalog (ats-mini only).
//
// LauncherHub has no ats-mini entries, so these strong overrides replace the
// weak default-false hooks in src/onlineLauncher.cpp and serve a static
// catalog fetched at runtime from a public JSON file. Fetch failures surface
// through displayError and fail closed (no partial flash, no Hub fallback).
// Installs reuse installFirmwareDynamic with merged factory images (@0x0);
// the direct URLs are used as-is because the Hub download proxy only serves
// Hub-registered fids.
// ---------------------------------------------------------------------------

extern bool getInfo(const String &serverUrl, JsonDocument &doc, JsonDocument *filter);
extern bool installFirmwareDynamic(
    const String &fileAddr, const String &file, uint32_t appSize, uint32_t appPartitionSize,
    uint32_t appOffset, bool nb, std::vector<LauncherInstallDataPartition> &dataPartitions,
    const String &installedName
);
extern void displayError(String txt, bool waitKeyPress);

// ---------------------------------------------------------------------------
// Static merged-image app slices (ats-mini only).
//
// Both catalog assets are merged factory images (bootloader + partition table
// + app). installFirmwareDynamic with nb=true flashes the stream from byte 0
// into the OTA app slot, so the bootloader bytes land where a valid app image
// must be and verification/boot never switch away from Launcher. The Hub
// manifest path handles this via install "source_offset"/"image_size"
// (onlineLauncher.cpp installFirmwareFromManifest:786-789, nb = appOffset==0)
// and the SD path via the embedded table + measured image length
// (sd_functions.cpp updateFromSD:822-842 + effectiveSdAppSize:616-631).
// We do the same SD-less: flashRawRangeFromHttp already skips sourceOffset
// bytes of the HTTP stream (onlineLauncher.cpp:530-532), so no SD staging
// file is needed (SDCARD_CS=-1 on this board).
//
// Slices below were measured offline from the release assets (partition table
// at 0x8000, ESP-image segment walk like measureSdEspImage):
//   - Bruce v1.0.0 (3786496 B): app@0x10000, 3720960 B = image tail
//     (image is 16 B short of canonical pad; identical bytes to the SD path's
//     file.size()-offset fallback, updateFromSD:828-830).
//   - Original v2.38 (8388608 B): app@0x10000, 1698544 B measured image
//     length (declared factory partition is 0x300000).
//   - English v2025.09.22, hub release ats-mini-english-v2025.09.22
//     (3378864 B): app@0x10000, 3313328 B. 6-segment walk, magic e9
//     (`e906 024f 4c6f 3740 ee00 0000 0900 0000`); canonical end lands 16 B
//     past EOF (same Bruce pattern), so the SD-path tail fallback
//     file.size()-offset applies: 3378864-0x10000 = 3313328.
//   - Marauder v2026.09.09, hub release ats-mini-marauder-v2026.09.09
//     (8388608 B): app@0x10000, 2022928 B measured image length (7-segment
//     walk, magic e9 `e907 023f ac5f 3740 ee00 0000 0900 0000`); equals the
//     ats-mini.ino.bin split length exactly.
// image_size must match exactly; any catalog change fails closed here so a
// silently swapped image can never be flashed with a stale slice.
// ---------------------------------------------------------------------------
struct AtsMiniStaticAppSlice {
    const char *fid;
    uint32_t imageSize;
    uint32_t appOffset;
    uint32_t appSize;
};

static const AtsMiniStaticAppSlice kAtsMiniStaticAppSlices[] = {
    {"bruce-ats-mini",    3786496, 0x10000, 3720960},
    {"ats-mini-original", 8388608, 0x10000, 1698544},
    {"ats-mini-english",  3378864, 0x10000, 3313328},
    {"ats-mini-marauder", 8388608, 0x10000, 2022928},
};

static bool
atsMiniStaticAppSlice(const String &fid, uint32_t imageSize, uint32_t &appOffset, uint32_t &appSize) {
    for (const auto &s : kAtsMiniStaticAppSlices) {
        if (fid == s.fid) {
            if (imageSize != s.imageSize) {
                displayError("Static image changed");
                return false;
            }
            appOffset = s.appOffset;
            appSize = s.appSize;
            return true;
        }
    }
    displayError("Firmware not in static catalog");
    return false;
}

// ---------------------------------------------------------------------------
// Static-install data partitions (ats-mini only).
//
// Root cause of the Bruce "LittleFS is Full" boot failure: this path used to
// pass an empty vector to installFirmwareDynamic, so launcherSelectInstallLayout
// -> launcherPartitionCreateOtaApp created ONLY the app entry. The installed
// layout had no data partition; Bruce begin_storage() (LittleFS.begin fails,
// LittleFS.format() fails with no partition) leaves totalBytes()==0, so
// checkLittleFsSize() reports full on every FS access. Creating the entry in
// the generated table is sufficient SD-less: launcherPrepareInstallDataPartitions
// (partition_install_layout.cpp) is pure table math (no SD), and the
// installFirmwareDynamic data loop skips flashing when copySize==0
// (onlineLauncher.cpp: "if (!dp.hasEntry || dp.copySize == 0) continue") —
// Bruce formats the empty partition itself at boot (begin_storage mount+format).
//
// Measured merged-asset tables (partition table at 0x8000, verified offline):
//   - Bruce v1.0.0 (3786496 B): spiffs, type 0x01, subtype 0x82, label
//     "spiffs", @0x810000 size 0x7F0000; region lies past EOF (no FS content).
//   - Original v2.38 (8388608 B): littlefs, type 0x01, subtype 0x83, label
//     "littlefs", @0x610000 size 0x1D0000; region reads all-0xFF (empty).
//   - English v2025.09.22 (3378864 B): spiffs, type 0x01, subtype 0x82,
//     label "spiffs", @0xC90000 size 0x360000 (app0/app1 0x640000 each).
//   - Marauder v2026.09.09 (8388608 B): littlefs, type 0x01, subtype 0x83,
//     label "littlefs", @0x610000 size 0x1D0000 (same layout as Original).
//
// Sizing mirrors the Hub manifest branch (onlineLauncher.cpp
// installFirmwareFromManifest):
//   struct fields mirrored: { subtype, label, sourceOffset, partitionSize,
//     copySize, sourceUrl } — payload-less, so sourceOffset/copySize stay 0.
//   Hub sizing lines mirrored:
//     "if (dp.label != "label" && dp.copySize > 0 &&
//         declaredSize > LAUNCHER_DEFAULT_SPIFFS_SIZE)
//          dp.partitionSize = declaredSize;
//      else if (declaredSize > LAUNCHER_DEFAULT_SPIFFS_THRESHOLD)
//          dp.partitionSize = LAUNCHER_INSTALL_USE_REMAINING_SPIFFS_SIZE;
//      else
//          dp.partitionSize = LAUNCHER_DEFAULT_SPIFFS_SIZE;"
//   - Bruce: declared 0x7F0000 > LAUNCHER_DEFAULT_SPIFFS_THRESHOLD (0x500000 on
//     this board) -> LAUNCHER_INSTALL_USE_REMAINING_SPIFFS_SIZE. The 0x7F0000
//     is NOT copied wholesale: the OTA slot remainder on this 16 MB layout is
//     smaller, and USE_REMAINING maximizes Bruce's FS (JS, configs) in whatever
//     free space is left, exactly as the Hub path would.
//   - Original: declared 0x1D0000 <= 0x500000 threshold and no payload, so both
//     the Hub branch and the SD branch (sd_functions.cpp: partitionEmpty &&
//     declaredSize <= THRESHOLD -> DEFAULT) yield LAUNCHER_DEFAULT_SPIFFS_SIZE
//     (0x70000 at runtime on >4 MB flash). Added because the merged table
//     declares a littlefs the radio firmware mounts for settings — same
//     missing-partition failure mode if absent; harmless (empty, validated) if
//     a future image stops using it.
//   - English: declared spiffs 0x360000 <= 0x500000 threshold, copySize=0, so
//     the Hub branch (copySize>0 required for declared-size adoption) yields
//     LAUNCHER_DEFAULT_SPIFFS_SIZE — same bucket as Original.
//   - Marauder: declared littlefs 0x1D0000 <= threshold, copySize=0 ->
//     LAUNCHER_DEFAULT_SPIFFS_SIZE — identical to Original (same table).
//
// Fit arithmetic (16 MB flash = 0x1000000; support_files/custom_16Mb.csv ends
// at coredump 0x190000+0x10000 = 0x1A0000; free = 0xE60000):
//   - Bruce app slot alignUp(3720960, 0x10000) = 0x390000 at 0x1A0000 ->
//     0x1A0000-0x530000; data takes largest remainder 0x530000-0x1000000.
//   - Original app slot alignUp(1698544, 0x10000) = 0x1A0000 at 0x1A0000 ->
//     0x1A0000-0x340000; data fixed 0x70000; required 0x210000 << 0xE60000.
//   - English app slot alignUp(3313328, 0x10000) = 0x330000 at 0x1A0000 ->
//     0x1A0000-0x4D0000; data fixed 0x70000; required 0x3A0000 << 0xE60000.
//   - Marauder app slot alignUp(2022928, 0x10000) = 0x1F0000 at 0x1A0000 ->
//     0x1A0000-0x390000; data fixed 0x70000; required 0x260000 << 0xE60000.
// Unknown fid fails closed via displayError, no flash.
// ---------------------------------------------------------------------------
static bool
atsMiniStaticDataPartitions(const String &fid, std::vector<LauncherInstallDataPartition> &dataPartitions) {
    dataPartitions.clear();
    if (fid == "bruce-ats-mini") {
        LauncherInstallDataPartition dp;
        dp.subtype = 0x82; // SPIFFS, mirrors Bruce merged-table entry
        dp.label = "spiffs";
        dp.sourceOffset = 0;
        dp.partitionSize = LAUNCHER_INSTALL_USE_REMAINING_SPIFFS_SIZE;
        dp.copySize = 0;
        dataPartitions.push_back(dp);
        return true;
    }
    if (fid == "ats-mini-original") {
        LauncherInstallDataPartition dp;
        dp.subtype = 0x83; // LittleFS, mirrors Original merged-table entry
        dp.label = "littlefs";
        dp.sourceOffset = 0;
        dp.partitionSize = LAUNCHER_DEFAULT_SPIFFS_SIZE;
        dp.copySize = 0;
        dataPartitions.push_back(dp);
        return true;
    }
    if (fid == "ats-mini-english") {
        LauncherInstallDataPartition dp;
        dp.subtype = 0x82; // SPIFFS, mirrors English merged-table entry
        dp.label = "spiffs";
        dp.sourceOffset = 0;
        dp.partitionSize = LAUNCHER_DEFAULT_SPIFFS_SIZE;
        dp.copySize = 0;
        dataPartitions.push_back(dp);
        return true;
    }
    if (fid == "ats-mini-marauder") {
        LauncherInstallDataPartition dp;
        dp.subtype = 0x83; // LittleFS, mirrors Marauder merged-table entry
        dp.label = "littlefs";
        dp.sourceOffset = 0;
        dp.partitionSize = LAUNCHER_DEFAULT_SPIFFS_SIZE;
        dp.copySize = 0;
        dataPartitions.push_back(dp);
        return true;
    }
    displayError("Firmware not in static catalog");
    return false;
}

static const char *kAtsMiniOtaCatalogUrl =
    "https://raw.githubusercontent.com/wendells01/ats-mini-hub/main/ats-mini-ota.json";

static bool fetchAtsMiniOtaCatalog(JsonDocument &catalog) {
    if (!getInfo(String(kAtsMiniOtaCatalogUrl), catalog, nullptr)) {
        displayError("Static OTA catalog fetch failed");
        return false;
    }
    return true;
}

bool launcherStaticOtaList(JsonDocument &listDoc) {
    if (!fetchAtsMiniOtaCatalog(listDoc)) return false;
    if (listDoc["items"].isNull()) {
        displayError("Static OTA catalog has no items");
        return false;
    }
    return true;
}

bool launcherStaticOtaVersions(const String &fid, JsonDocument &versionsDoc) {
    JsonDocument catalog;
    if (!fetchAtsMiniOtaCatalog(catalog)) return false;
    JsonObject detail = catalog["details"][fid].as<JsonObject>();
    if (detail.isNull() || detail["versions"].isNull()) {
        displayError("Firmware not in static catalog");
        return false;
    }
    versionsDoc["name"] = detail["name"].as<String>();
    versionsDoc["author"] = detail["author"].as<String>();
    versionsDoc["fid"] = fid;
    versionsDoc["star"] = detail["star"].as<bool>();
    versionsDoc["versions"].to<JsonArray>();
    for (JsonObject v : detail["versions"].as<JsonArray>()) {
        JsonObject out = versionsDoc["versions"].add<JsonObject>();
        out["version"] = v["version"].as<String>();
        out["published_at"] = v["published_at"].as<String>();
        out["file"] = v["file"].as<String>();
    }
    return true;
}

bool launcherStaticOtaInstall(const String &fid, const String &version, const String &installedName) {
    JsonDocument catalog;
    if (!fetchAtsMiniOtaCatalog(catalog)) return false;
    JsonObject detail = catalog["details"][fid].as<JsonObject>();
    if (detail.isNull()) {
        displayError("Firmware not in static catalog");
        return false;
    }
    for (JsonObject v : detail["versions"].as<JsonArray>()) {
        if (v["version"].as<String>() != version) continue;
        String file = v["file"].as<String>();
        uint32_t imageSize = v["image_size"].as<uint32_t>();
        if (file.isEmpty() || imageSize == 0) {
            displayError("Bad static install info");
            return false;
        }
        // Merged factory image: flash only the app slice into the OTA slot
        // (nb=false skips appOffset source bytes), mirroring the Hub manifest
        // path. Unknown/modified images fail closed inside the lookup.
        uint32_t appOffset = 0;
        uint32_t appSize = 0;
        if (!atsMiniStaticAppSlice(fid, imageSize, appOffset, appSize)) return false;
        std::vector<LauncherInstallDataPartition> dataPartitions;
        if (!atsMiniStaticDataPartitions(fid, dataPartitions)) return false;
        String name = detail["name"].as<String>() + " - " + version;
        if (installedName.length() && detail["name"].as<String>().isEmpty()) name = installedName;
        if (!installFirmwareDynamic(file, file, appSize, appSize, appOffset, false, dataPartitions, name)) {
            launcherDelayMs(2500);
        }
        return true;
    }
    displayError("Version not in static catalog");
    return false;
}
