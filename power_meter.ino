// =============================================================================
// Power Meter / Datalogger with 3-Point Calibration Wizard
// Hardware: Arduino Pro Mini, INA219 (raw I2C), U8X8 SH1106 OLED, SdFat v2
// Features: EEPROM cal storage, V-min hold, peak-hold bar, marker button,
//           SD auto-recovery, diagnostic markers, delta logging,
//           state-machine calibration wizard with abort & transactional save.
// =============================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <U8x8lib.h>
#include <SdFat.h>

// -----------------------------------------------------------------------------
// EEPROM layout
// Addr 0      : magic byte (0xAC = calibration valid)
// Addr 1–4    : V_SLOPE   (float)
// Addr 5–8    : V_INTERCEPT
// Addr 9–12   : I_SLOPE
// Addr 13–16  : I_INTERCEPT
// -----------------------------------------------------------------------------
#define EEPROM_MAGIC_ADDR  0
#define EEPROM_MAGIC_VAL   0xAC
#define EEPROM_VSLOPE_ADDR 1
#define EEPROM_VIOFF_ADDR  5
#define EEPROM_ISLOPE_ADDR 9
#define EEPROM_IIOFF_ADDR  13

// -----------------------------------------------------------------------------
// Calibration globals — loaded from EEPROM in setup(); NOT const
// -----------------------------------------------------------------------------
float V_SLOPE     = 1.0f;
float V_INTERCEPT = 0.0f;
float I_SLOPE     = 1.0f;
float I_INTERCEPT = 0.0f;

// -----------------------------------------------------------------------------
// Hardware config
// -----------------------------------------------------------------------------
#define SD_CS_PIN           10
#define MARKER_PIN          2
#define SERIAL_BAUD         9600
#define INA219_ADDR         0x40
#define EMA_ALPHA           0.1f
#define LOG_V_DELTA         0.05f
#define LOG_I_DELTA         5.0f
#define LOG_TIME_MAX        5000UL
#define SYNC_INTERVAL       2000UL
#define SD_RETRY_INTERVAL   10000UL
#define PEAK_DECAY_MS       100UL
#define PEAK_DECAY_STEP_MA  100.0f
#define BAR_SCALE_MA        3000.0f
#define SHUNT_OHMS          0.1f
#define DEBOUNCE_MS         50UL
#define LONG_PRESS_MS       2000UL  // hold duration that triggers min/max reset

// INA219 registers & config word
#define INA219_REG_CFG   0x00
#define INA219_REG_SHUNT 0x01
#define INA219_REG_BUS   0x02
#define INA219_CFG_VAL   0x3FFF   // 32 V, PGA/8, 12-bit, continuous

// -----------------------------------------------------------------------------
// Calibration wizard — state machine states
// -----------------------------------------------------------------------------
enum CalState : uint8_t {
    CAL_IDLE = 0,
    CAL_SET_SOURCE,     // OLED prompts user to set supply/load, press button
    CAL_WAIT_BUTTON,    // waiting for physical button press (snapshot trigger)
    CAL_WAIT_SERIAL,    // raw frozen; waiting for exact meter value via Serial
    CAL_DONE,           // all 3 points collected; compute & confirm save
    CAL_ABORT
};

// -----------------------------------------------------------------------------
// Objects
// -----------------------------------------------------------------------------
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
SdFat  sd;
SdFile logFile;

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
bool          sdActive        = false;
unsigned long lastSDRetry     = 0;
uint32_t      uptimeSec       = 0;
unsigned long lastUptimeTick  = 0;
unsigned long lastLogTime     = 0;
unsigned long lastSyncTime    = 0;
unsigned long lastPeakDecay   = 0;
float prevLogV        = -999.0f;
float prevLogI        = -999.0f;
float emaCurrentMA    = 0.0f;
float peakCurrentMA   = 0.0f;
float minVoltage      = 9999.0f;
float maxVoltage      = -9999.0f;
float maxCurrentMA    = 0.0f;
float maxPower        = 0.0f;     // running max power (mW)
bool  holdResetReq    = false;    // long-press sets this; serviced after
                                  // sensor read so reset uses clean values

// Marker LED flash (non-blocking)
#define MARKER_LED_MS   180UL     // flash duration in ms
bool          ledOn          = false;
unsigned long ledOffAt       = 0;  // millis() at which to turn the LED off
bool  emaInit         = false;

// Button state for short / long press (non-blocking)
bool          lastBtnState     = HIGH;
bool          btnShortPress    = false;
bool          btnLongPress     = false;
unsigned long lastDebounceTime = 0;
unsigned long btnPressStart    = 0;
bool          debounceState    = HIGH;
bool          longFired        = false;

// Calibration wizard runtime
CalState      calState         = CAL_IDLE;
bool          isCalibrating    = false;  // true while cal wizard owns the OLED
uint8_t       calMode          = 0;     // 0 = voltage, 1 = current
uint8_t       calStep          = 0;     // 0–2
float         calRaw[3];               // frozen raw INA219 readings
float         calRef[3];               // user-entered reference values
// Target prompts stored in flash; indexed [mode][step]
// Voltage: ~3 V, ~7 V, ~12 V   Current: ~20 mA, ~500 mA, ~1000 mA
const float CAL_TARGETS[2][3] PROGMEM = {
    { 3.0f,  7.0f, 12.0f  },
    { 20.0f, 500.0f, 1000.0f }
};

// =============================================================================
// EEPROM helpers
// =============================================================================
static void eepromLoad() {
    if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
        Serial.println(F("EEPROM: no cal, using defaults"));
        return;
    }
    EEPROM.get(EEPROM_VSLOPE_ADDR, V_SLOPE);
    EEPROM.get(EEPROM_VIOFF_ADDR,  V_INTERCEPT);
    EEPROM.get(EEPROM_ISLOPE_ADDR, I_SLOPE);
    EEPROM.get(EEPROM_IIOFF_ADDR,  I_INTERCEPT);
    Serial.println(F("EEPROM: cal loaded"));
}

static void eepromSaveAll() {
    EEPROM.put(EEPROM_VSLOPE_ADDR, V_SLOPE);
    EEPROM.put(EEPROM_VIOFF_ADDR,  V_INTERCEPT);
    EEPROM.put(EEPROM_ISLOPE_ADDR, I_SLOPE);
    EEPROM.put(EEPROM_IIOFF_ADDR,  I_INTERCEPT);
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
}

// Write neutral defaults to EEPROM and apply immediately (factory reset).
static void eepromFactoryReset() {
    V_SLOPE = 1.0f; V_INTERCEPT = 0.0f;
    I_SLOPE = 1.0f; I_INTERCEPT = 0.0f;
    eepromSaveAll();   // reuses the same write path; EEPROM.put() skips
                       // unchanged bytes so this is still wear-efficient.
    Serial.println(F("Factory reset: cal defaults written to EEPROM."));
}

// =============================================================================
// Minimal INA219 driver — raw I2C
// =============================================================================
static void ina219_write(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    Wire.endTransmission();
}

static int16_t ina219_read(uint8_t reg) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2);
    uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
    return (int16_t)raw;
}

static void  ina219_init()      { ina219_write(INA219_REG_CFG, INA219_CFG_VAL); }
static float ina219_busV()      { return (float)((ina219_read(INA219_REG_BUS) >> 3) & 0x1FFF) * 0.004f; }
static float ina219_shuntMV()   { return (float)ina219_read(INA219_REG_SHUNT) * 0.01f; }
static float ina219_currentMA() { return ina219_shuntMV() / SHUNT_OHMS; }

// =============================================================================
// Calibrated readings
// =============================================================================
static float getRawVoltage()   { return ina219_busV() + ina219_shuntMV() * 0.001f; }
static float getRawCurrentMA() { return ina219_currentMA(); }
static float getCalV()         { return getRawVoltage()   * V_SLOPE + V_INTERCEPT; }
static float getCalIma()       { return getRawCurrentMA() * I_SLOPE + I_INTERCEPT; }

// =============================================================================
// Linear regression — 3 points
// =============================================================================
static void linReg3(float x0, float y0, float x1, float y1, float x2, float y2,
                    float &m, float &b)
{
    float sx  = x0+x1+x2,          sy  = y0+y1+y2;
    float sxy = x0*y0+x1*y1+x2*y2, sx2 = x0*x0+x1*x1+x2*x2;
    float den = 3.0f*sx2 - sx*sx;
    if (fabsf(den) < 1e-9f) { m = 1.0f; b = 0.0f; return; }
    m = (3.0f*sxy - sx*sy) / den;
    b = (sy - m*sx) / 3.0f;
}

// =============================================================================
// Uptime helpers
// =============================================================================
static inline void writeU8Pair(char* buf, uint8_t &pos, uint8_t val) {
    buf[pos++] = '0' + val / 10;
    buf[pos++] = '0' + val % 10;
}

static uint8_t buildUptimeStr(char* dst) {
    uint32_t s = uptimeSec;
    uint32_t d = s / 86400UL; s %= 86400UL;
    uint8_t  h   = (uint8_t)(s / 3600UL); s %= 3600UL;
    uint8_t  m   = (uint8_t)(s / 60UL);   s %= 60UL;
    uint8_t  sec = (uint8_t)s;
    uint8_t pos = 0;
    if (d >= 100) { dst[pos++] = '0' + (char)(d / 100); d %= 100; }
    if (d >=  10) { dst[pos++] = '0' + (char)(d /  10); d %=  10; }
    dst[pos++] = '0' + (char)d; dst[pos++] = ':';
    writeU8Pair(dst, pos, h); dst[pos++] = ':';
    writeU8Pair(dst, pos, m); dst[pos++] = ':';
    writeU8Pair(dst, pos, sec);
    dst[pos] = '\0';
    return pos;
}

static void buildUptimeRow(char* ut) {
    uint8_t len = buildUptimeStr(ut);
    while (len < 13) ut[len++] = ' ';
    ut[13] = sdActive ? ' ' : '!';
    ut[14] = 'S'; ut[15] = 'D'; ut[16] = '\0';
}

// =============================================================================
// OLED helpers
// =============================================================================

// =============================================================================
// Calibration wizard — OLED prompt strings (all in flash)
// =============================================================================
// Voltage target prompts
const char CAL_V0_L1[] PROGMEM = "Set supply ~3V";
const char CAL_V1_L1[] PROGMEM = "Set supply ~7V";
const char CAL_V2_L1[] PROGMEM = "Set supply ~12V";
const char CAL_I0_L1[] PROGMEM = "Set load ~20mA";
const char CAL_I1_L1[] PROGMEM = "Set load ~500mA";
const char CAL_I2_L1[] PROGMEM = "Set load ~1000mA";

// Pointer tables in flash — indexed [step]
const char* const CAL_V_L1[3] PROGMEM = { CAL_V0_L1, CAL_V1_L1, CAL_V2_L1 };
const char* const CAL_I_L1[3] PROGMEM = { CAL_I0_L1, CAL_I1_L1, CAL_I2_L1 };

// Helper: read a PROGMEM pointer-to-PROGMEM-string and cast for U8x8/Serial.
// Returns a transient PGM pointer — use immediately, do not store.
static const __FlashStringHelper* calPromptLine(uint8_t mode, uint8_t step) {
    const char* const* tbl = (mode == 0)
        ? (const char* const*)CAL_V_L1
        : (const char* const*)CAL_I_L1;
    return (const __FlashStringHelper*)pgm_read_ptr(&tbl[step]);
}

// =============================================================================
// drawCalScreen — one-time styled "CALIBRATION" banner. Called once at wizard
// entry; the display is then left completely static until the wizard exits.
//
//   Row 1: ================  (16 '=' — full-width top rule)
//   Row 3:    CALIBRATION    (centered title)
//   Row 4:   VOLTAGE MODE    (or CURRENT MODE — centered subtitle)
//   Row 6: ================  (16 '=' — full-width bottom rule)
// =============================================================================
static void drawCalScreen(uint8_t mode) {
    u8x8.clear();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    // Top and bottom rules — exactly 16 '=' fill the panel width
    u8x8.drawString(0, 1, "================");
    u8x8.drawString(0, 6, "================");
    // Centered title: "CALIBRATION" is 11 chars → start at col 3 (≈ centered)
    u8x8.drawString(3, 3, "CALIBRATION");
    // Centered subtitle by mode: "VOLTAGE MODE"/"CURRENT MODE" are 12 chars → col 2
    u8x8.drawString(2, 4, mode == 0 ? "VOLTAGE MODE" : "CURRENT MODE");
}

// =============================================================================
// Calibration wizard state machine
// Call from loop() every iteration while calState != CAL_IDLE.
// Returns true while still running, false when finished (idle or abort).
// =============================================================================
static bool runCalSM() {

    // ── Global abort check: 'Q' typed at any point ───────────────────────────
    if (Serial.available()) {
        char peek = (char)Serial.peek();
        if (peek == 'Q' || peek == 'q') {
            while (Serial.available()) Serial.read();
            calState = CAL_ABORT;
        }
    }

    switch (calState) {

        // ── CAL_SET_SOURCE ────────────────────────────────────────────────────
        // Take ownership of the OLED, wipe every row cleanly, then paint the
        case CAL_SET_SOURCE: {
            // Display is left untouched (static "CALIBRATION" message stays).
            // All step guidance goes to Serial only.
            Serial.print(F("[Cal step ")); Serial.print(calStep + 1);
            Serial.print(F("/3] ")); Serial.println(calPromptLine(calMode, calStep));
            Serial.println(F("Press the MARKER button to capture, or Q to abort."));

            calState = CAL_WAIT_BUTTON;
            break;
        }

        // ── CAL_WAIT_BUTTON ───────────────────────────────────────────────────
        // Spin here until physical button fires (btnPressed flag) or abort.
        case CAL_WAIT_BUTTON: {
            if (btnShortPress) {
                btnShortPress = false;

                // Freeze raw INA219 reading for this step
                calRaw[calStep] = (calMode == 0) ? getRawVoltage() : getRawCurrentMA();

                // Display untouched — report capture via Serial only
                Serial.print(F("Raw captured: "));
                {
                    char b[10];
                    dtostrf(calRaw[calStep], 10, 5, b);
                    Serial.println(b);
                }
                Serial.print(calMode == 0 ? F("Enter exact V: ") : F("Enter exact mA: "));

                // Flush any leftover Serial bytes before entering serial-wait
                while (Serial.available()) Serial.read();
                calState = CAL_WAIT_SERIAL;
            }
            break;
        }

        // ── CAL_WAIT_SERIAL ───────────────────────────────────────────────────
        // Block until a float + newline arrives. 'Q' already handled above.
        case CAL_WAIT_SERIAL: {
            if (Serial.available()) {
                float val = Serial.parseFloat();
                while (Serial.available()) Serial.read();  // drain rest of line

                calRef[calStep] = val;

                // Echo confirmation
                {
                    char b[10];
                    Serial.print(F("  ref="));
                    dtostrf(val, 8, 4, b); Serial.print(b);
                    Serial.print(F("  raw="));
                    dtostrf(calRaw[calStep], 8, 4, b); Serial.println(b);
                }

                calStep++;
                if (calStep >= 3) {
                    calState = CAL_DONE;
                } else {
                    calState = CAL_SET_SOURCE;  // next point
                }
            }
            break;
        }

        // ── CAL_DONE ─────────────────────────────────────────────────────────
        // All 3 points collected. Compute regression, apply to live globals,
        // then perform single atomic EEPROM save.
        case CAL_DONE: {
            float m, b;
            linReg3(calRaw[0], calRef[0],
                    calRaw[1], calRef[1],
                    calRaw[2], calRef[2],
                    m, b);

            // Apply to live globals immediately
            if (calMode == 0) { V_SLOPE = m; V_INTERCEPT = b; }
            else               { I_SLOPE = m; I_INTERCEPT = b; }

            // Single atomic EEPROM write of all four floats + magic byte
            eepromSaveAll();

            // Print result
            Serial.println(F("-- Cal complete, saved to EEPROM --"));
            {
                char buf[12];
                if (calMode == 0) {
                    Serial.print(F("V_SLOPE="));
                    dtostrf(m, 10, 7, buf); Serial.print(buf);
                    Serial.print(F("  V_INTERCEPT="));
                    dtostrf(b, 10, 7, buf); Serial.println(buf);
                } else {
                    Serial.print(F("I_SLOPE="));
                    dtostrf(m, 10, 7, buf); Serial.print(buf);
                    Serial.print(F("  I_INTERCEPT="));
                    dtostrf(b, 10, 7, buf); Serial.println(buf);
                }
            }

            // Wizard finished — single clear, then hand display back to loop().
            u8x8.clear();
            isCalibrating = false;      // updateOLED resumes next loop pass
            calState = CAL_IDLE;
            return false;
        }

        // ── CAL_ABORT ─────────────────────────────────────────────────────────
        case CAL_ABORT: {
            Serial.println(F("Cal aborted. EEPROM unchanged."));

            // Single clear, then hand display back to loop().
            u8x8.clear();
            isCalibrating = false;      // updateOLED resumes next loop pass
            calState = CAL_IDLE;
            return false;
        }

        default: break;
    }

    return (calState != CAL_IDLE);
}

// =============================================================================
// OLED — normal operation display
//
// Layout (128x64 px = 16x8 tile grid). NO double-height glyph starts on row 0
// (avoids the SSD1306 page-0 corruption). Row 0 is small-font status.
//
//   Row 0   : [small]   "0:00:12:34    SD"   uptime + SD health tag
//   Row 1   : [small]   "min          peak"  faint labels for the holds below
//   Row 2-3 : [1x2]     "V:  12.34 V"        voltage  (double-height)
//   Row 4-5 : [1x2]     "I: 123.45mA"        current  (double-height)
//   Row 6-7 : [1x2]     "P: 1.234 W"         power    (double-height)
//
// Decimal alignment: every value uses dtostrf width=7, so the decimal point
// lands on the same tile column on all three large rows. The minVoltage and
// peakCurrentMA holds are shown small on row 1 so they never disturb the big
// value rows or their alignment.
// =============================================================================
static void updateOLED(float volts, float iMA, float pmW) {
    // Safety net; the primary block is if(!isCalibrating) in loop().
    if (isCalibrating) return;

    char b[9];   // dtostrf(width=7) → 7 + sign + null = 9 worst case

    // ── Row 0: uptime + SD status (small font) ───────────────────────────────
    {
        char ut[17];
        buildUptimeRow(ut);            // "D:HH:MM:SS   SD" / " !SD"
        u8x8.setFont(u8x8_font_chroma48medium8_r);
        u8x8.setCursor(0, 0);
        u8x8.print(ut);
    }

    // ── Row 1: small-font MIN / MAX hold readouts ────────────────────────────
    // Left  cols 0-7 : "VMIN" + value (w=4 p=1)
    // Right cols 8-15: "IMAX" + value (w=4 p=0, in mA)
    // Both labels are 4 chars so the two halves are symmetric (8 + 8).
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 1);
    u8x8.print(F("VMIN"));
    if (minVoltage > 999.0f) {
        u8x8.print(F("----"));
    } else {
        dtostrf(minVoltage, 4, 1, b); u8x8.print(b);
    }
    u8x8.setCursor(8, 1);
    u8x8.print(F("IMAX"));
    dtostrf(maxCurrentMA, 4, 0, b); u8x8.print(b);

    // ── Large value rows — UNIFORM FORMAT FOR PERFECT DECIMAL ALIGNMENT ───────
    //
    // Every value uses dtostrf(width=7, precision=2). Because width and
    // precision are identical on all three rows, the decimal point lands on
    // the exact same tile column (col 5) on every line, regardless of range.
    //
    //   Label (2) | number (7, w=7 p=2) | unit (7)  = 16 tiles
    //   "V:"      | "  12.34"            | " V     "
    //   "I:"      | " 123.45"            | " mA    "
    //   "P:"      | "   1.23"            | " W     "
    //               ^col2   ^col5 decimal
    //
    // Auto-ranging converts the magnitude (mA→A, mW→W) but KEEPS p=2 so the
    // decimal column never moves. Labels are all 2 chars ("V:", "I:", "P:")
    // so the number field always starts at the same column too.

    u8x8.setFont(u8x8_font_8x13B_1x2_r);

    // Rows 2-3: Voltage
    u8x8.setCursor(0, 2);
    u8x8.print(F("V:"));
    dtostrf(volts, 7, 2, b);
    u8x8.print(b);
    u8x8.print(F(" V     "));

    // Rows 4-5: Current (auto-range mA / A, precision locked at 2)
    u8x8.setCursor(0, 4);
    u8x8.print(F("I:"));
    if (iMA >= 1000.0f) {
        dtostrf(iMA * 0.001f, 7, 2, b);
        u8x8.print(b);
        u8x8.print(F(" A     "));
    } else {
        dtostrf(iMA, 7, 2, b);
        u8x8.print(b);
        u8x8.print(F(" mA    "));
    }

    // Rows 6-7: Power (auto-range mW / W, precision locked at 2)
    u8x8.setCursor(0, 6);
    u8x8.print(F("P:"));
    if (pmW >= 1000.0f) {
        dtostrf(pmW * 0.001f, 7, 2, b);
        u8x8.print(b);
        u8x8.print(F(" W     "));
    } else {
        dtostrf(pmW, 7, 2, b);
        u8x8.print(b);
        u8x8.print(F(" mW    "));
    }
}

// =============================================================================
// SD helpers
// =============================================================================
static void logMarker(const __FlashStringHelper* event) {
    if (!logFile.isOpen()) return;
    char ut[13];
    buildUptimeStr(ut);
    logFile.print(F("--- "));
    logFile.print(event);
    logFile.print(F(" at "));
    logFile.print(ut);
    logFile.println(F(" ---"));
    logFile.sync();
}

static void markSDRemoved() {
    if (sdActive) {
        logMarker(F("SD REMOVED"));
        if (logFile.isOpen()) logFile.close();
    }
    sdActive = false;
}

static void initSD() {
    if (logFile.isOpen()) logFile.close();
    if (!sd.begin(SD_CS_PIN)) { sdActive = false; Serial.println(F("SD fail")); return; }
    if (!logFile.open("POWER.CSV", O_RDWR | O_CREAT | O_AT_END)) {
        sdActive = false; Serial.println(F("File fail")); return;
    }
    if (logFile.fileSize() == 0) {
        logFile.println(F("Uptime,V,mA,mW"));
        logFile.sync();
    }
    sdActive = true;
    Serial.println(F("SD OK"));
    logMarker(F("SD CONNECTED"));
}

static void logToSD(float volts, float iMA, float pmW) {
    if (!logFile.isOpen()) { markSDRemoved(); return; }
    char  b[10], ut[13];
    bool  ok = true;
    buildUptimeStr(ut);
    ok &= (bool)logFile.print(ut);    ok &= (bool)logFile.print(',');
    dtostrf(volts, 7, 4, b); ok &= (bool)logFile.print(b); ok &= (bool)logFile.print(',');
    dtostrf(iMA,   8, 3, b); ok &= (bool)logFile.print(b); ok &= (bool)logFile.print(',');
    dtostrf(pmW,   8, 3, b); ok &= (bool)logFile.println(b);
    if (ok) { if (!logFile.sync()) markSDRemoved(); }
    else    { markSDRemoved(); }
}

// =============================================================================
// Button poll — non-blocking, debounced. Marker fires INSTANTLY on the press
// (falling edge) so the Serial "MARKER" appears at the exact moment of press,
// not on release. Long-press (>= LONG_PRESS_MS) additionally triggers the
// min/max reset while the button is still held.
//
//   Quick tap   → btnShortPress (marker only)
//   Long hold   → btnShortPress on press + btnLongPress after 2 s (reset)
//
// Debounce: a state change must be stable for DEBOUNCE_MS before it counts.
// pollButton() is called every loop() pass, so it keeps working even while
// the SD card is mid-write (the write happens AFTER the flags are consumed).
// =============================================================================
static void pollButton() {
    bool raw = digitalRead(MARKER_PIN);
    unsigned long now = millis();

    // Debounce: any raw change restarts the stability timer
    if (raw != lastBtnState) {
        lastDebounceTime = now;
        lastBtnState = raw;
    }
    if ((now - lastDebounceTime) < DEBOUNCE_MS) return;  // not stable yet

    // ── Falling edge (HIGH→LOW): button just pressed ────────────────────────
    if (debounceState == HIGH && raw == LOW) {
        debounceState = LOW;
        btnPressStart = now;
        longFired     = false;
        btnShortPress = true;   // INSTANT marker — consumed same loop pass
    }

    // ── Still held: fire long-press reset once at the 2 s threshold ─────────
    if (debounceState == LOW && raw == LOW && !longFired) {
        if ((now - btnPressStart) >= LONG_PRESS_MS) {
            btnLongPress = true;
            longFired    = true;  // one-shot while held
        }
    }

    // ── Rising edge (LOW→HIGH): button released ─────────────────────────────
    if (debounceState == LOW && raw == HIGH) {
        debounceState = HIGH;    // ready for next press; marker already fired
    }
}

// =============================================================================
// setup
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("PowerMeter v=VolCal i=CurCal"));

    eepromLoad();

    pinMode(MARKER_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Wire.begin();
    ina219_init();

    u8x8.begin();
    u8x8.setContrast(255);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clear();
    u8x8.setCursor(0, 0); u8x8.print(F("Power Meter"));
    u8x8.setCursor(0, 1); u8x8.print(F("Starting..."));

    initSD();

    unsigned long t = millis();
    lastUptimeTick = lastLogTime = lastSyncTime =
        lastPeakDecay = lastSDRetry = lastDebounceTime = t;

    delay(1000);
    u8x8.clear();
}

// =============================================================================
// loop
// =============================================================================
void loop() {
    unsigned long now = millis();

    // ── Uptime tick ──────────────────────────────────────────────────────────
    if (now - lastUptimeTick >= 1000UL) { uptimeSec++; lastUptimeTick += 1000UL; }

    // ── Non-blocking marker LED: turn off once the flash window elapses ──────
    if (ledOn && (long)(now - ledOffAt) >= 0) {
        digitalWrite(LED_BUILTIN, LOW);
        ledOn = false;
    }

    // ── Button poll (used by both wizard and normal marker logic) ─────────────
    pollButton();

    // ── Calibration wizard — takes over the loop while active ────────────────
    if (calState != CAL_IDLE) {
        runCalSM();
        return;     // skip normal measurement/display/logging while in wizard
    }

    // ── Start calibration wizard on 'v' or 'i' ───────────────────────────────
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        while (Serial.available()) Serial.read();
        if (cmd == 'v' || cmd == 'V') {
            calMode = 0; calStep = 0;
            isCalibrating = true;            // loop() will not call updateOLED
            drawCalScreen(0);                // ONE-TIME styled banner; static after
            calState = CAL_SET_SOURCE;
            Serial.println(F("Starting voltage calibration. Q to abort."));
            return;
        } else if (cmd == 'i' || cmd == 'I') {
            calMode = 1; calStep = 0;
            isCalibrating = true;            // loop() will not call updateOLED
            drawCalScreen(1);                // ONE-TIME styled banner; static after
            calState = CAL_SET_SOURCE;
            Serial.println(F("Starting current calibration. Q to abort."));
            return;
        } else if (cmd == 'z' || cmd == 'Z') {
            eepromFactoryReset();
        }
    }

    // ── Button actions: marker fires the instant the button is pressed ─────
    if (btnShortPress) {
        btnShortPress = false;
        // Instant visual feedback: LED on now, scheduled off via millis().
        digitalWrite(LED_BUILTIN, HIGH);
        ledOn    = true;
        ledOffAt = now + MARKER_LED_MS;
        // Serial FIRST, flushed immediately so it appears in the terminal at
        // the exact moment of the press — before the (slower) SD write.
        Serial.println(F("MARKER"));
        Serial.flush();                 // block until TX buffer is drained
        logMarker(F("MANUAL MARKER"));  // then write+sync to the SD card
    }
    if (btnLongPress) {
        btnLongPress = false;
        // Defer the actual reset until AFTER the sensor readings are taken
        // and clamped/filtered below — otherwise the baseline could be a raw
        // negative/floating value and the holds would show negative numbers.
        holdResetReq = true;
        logMarker(F("MIN/MAX RESET"));
        Serial.println(F("Long press: min/max/peak reset to live values."));
    }

    // ── SD auto-recovery ─────────────────────────────────────────────────────
    if (!sdActive && (now - lastSDRetry >= SD_RETRY_INTERVAL)) {
        lastSDRetry = now;
        Serial.println(F("SD retry..."));
        initSD();
    }

    // ── Sensor readings ──────────────────────────────────────────────────────
    float volts = getCalV();
    float iMA   = getCalIma();
    // Clamp negative readings caused by floating/unconnected INA219 inputs.
    // A real load can never produce negative bus voltage; negative shunt
    // current (reverse current) is valid in some setups — keep it for current
    // but clamp the bus voltage which is always >= 0 V physically.
    if (volts < 0.0f) volts = 0.0f;
    if (iMA   < 0.0f) iMA   = 0.0f;

    emaCurrentMA = emaInit
        ? EMA_ALPHA * iMA + (1.0f - EMA_ALPHA) * emaCurrentMA
        : iMA;
    emaInit = true;
    iMA = emaCurrentMA;

    float pmW = volts * iMA;

    // ── Deferred hold reset (long-press) ─────────────────────────────────────
    // Now that volts/iMA/pmW are the clamped, filtered values the display uses,
    // seed every hold to the live baseline. No zeros, no raw/negative leakage,
    // display and internal trackers synchronized in one place.
    if (holdResetReq) {
        holdResetReq  = false;
        minVoltage    = volts;   maxVoltage   = volts;
        peakCurrentMA = iMA;     maxCurrentMA = iMA;
        maxPower      = pmW;
    }

    // ── V-min / V-max hold ───────────────────────────────────────────────────
    if (volts < minVoltage) minVoltage = volts;
    if (volts > maxVoltage) maxVoltage = volts;

    // ── Peak / max hold ──────────────────────────────────────────────────────
    if (iMA >= peakCurrentMA) peakCurrentMA = iMA;
    if (iMA >  maxCurrentMA)  maxCurrentMA  = iMA;
    if (pmW >  maxPower)      maxPower      = pmW;
    if (now - lastPeakDecay >= PEAK_DECAY_MS) {
        lastPeakDecay = now;
        if (peakCurrentMA > iMA) {
            peakCurrentMA -= PEAK_DECAY_STEP_MA;
            if (peakCurrentMA < iMA) peakCurrentMA = iMA;
        }
    }

    // ── Display ──────────────────────────────────────────────────────────────
    if (!isCalibrating) {
        updateOLED(volts, iMA, pmW);
    }

    // ── Delta logging ────────────────────────────────────────────────────────
    bool doLog = (fabsf(volts - prevLogV) > LOG_V_DELTA)
              || (fabsf(iMA   - prevLogI) > LOG_I_DELTA)
              || (now - lastLogTime >= LOG_TIME_MAX);
    if (doLog) {
        logToSD(volts, iMA, pmW);
        prevLogV = volts; prevLogI = iMA;
        lastLogTime = lastSyncTime = now;
    }

    // ── Periodic sync heartbeat ───────────────────────────────────────────────
    if (sdActive && logFile.isOpen() && (now - lastSyncTime >= SYNC_INTERVAL)) {
        if (!logFile.sync()) markSDRemoved();
        lastSyncTime = now;
    }

    delay(250);
}
