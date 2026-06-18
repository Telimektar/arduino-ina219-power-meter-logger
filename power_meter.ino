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
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
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

// Print a single 8x8 row padded to exactly 16 chars to clear stale content.
// Cursor must be set by caller.
static void oledPrintRow(const __FlashStringHelper* msg) {
    // Print the flash string, then pad with spaces to col 16
    // We track length by printing char-by-char from PROGMEM.
    const char* p = (const char*)msg;
    uint8_t len = 0;
    char c;
    while ((c = pgm_read_byte(p++)) != '\0') { u8x8.print(c); len++; }
    while (len++ < 16) u8x8.print(' ');
}

// Two-line (double-height) OLED prompt used during calibration.
// row: starting tile row (0,2,4 …)
// line1/line2: flash strings, each ≤ 16 chars.
static void oledCalPrompt(uint8_t row,
                          const __FlashStringHelper* line1,
                          const __FlashStringHelper* line2)
{
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, row);     oledPrintRow(line1);
    u8x8.setCursor(0, row + 1); oledPrintRow(line2);
}

// Fill the entire screen with blank tiles (used on cal entry/exit).
static void oledClearAll() { u8x8.clear(); }

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
const char CAL_BTN[]   PROGMEM = "Press BTN to snap";
const char CAL_SER[]   PROGMEM = "Enter meter val:";
const char CAL_DONE_L1[] PROGMEM = "Cal done!";
const char CAL_DONE_L2[] PROGMEM = "Saved to EEPROM";
const char CAL_ABORT_L1[] PROGMEM = "Cal ABORTED";
const char CAL_ABORT_L2[] PROGMEM = "No changes saved";

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
        // Show OLED prompt for this step; move straight to button-wait.
        case CAL_SET_SOURCE: {
            oledClearAll();

            // Line 0-1: target prompt
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.setCursor(0, 0);
            oledPrintRow(calPromptLine(calMode, calStep));
            u8x8.setCursor(0, 1);
            oledPrintRow((const __FlashStringHelper*)CAL_BTN);

            // Line 2: step counter "Step X/3"
            u8x8.setCursor(0, 3);
            u8x8.print(F("Step "));
            u8x8.print(calStep + 1);
            u8x8.print(F("/3  Q=abort  "));

            // Serial mirror
            Serial.print(F("[Cal step ")); Serial.print(calStep + 1);
            Serial.print(F("/3] ")); Serial.println(calPromptLine(calMode, calStep));
            Serial.println(F("Press the MARKER button to capture, or Q to abort."));

            calState = CAL_WAIT_BUTTON;
            break;
        }

        // ── CAL_WAIT_BUTTON ───────────────────────────────────────────────────
        // Spin here until physical button fires (btnPressed flag) or abort.
        case CAL_WAIT_BUTTON: {
            if (btnPressed) {
                btnPressed = false;

                // Freeze raw INA219 reading for this step
                calRaw[calStep] = (calMode == 0) ? getRawVoltage() : getRawCurrentMA();

                // Show "frozen" feedback on OLED row 5
                {
                    char b[10];
                    u8x8.setCursor(0, 5);
                    u8x8.print(F("Raw:"));
                    dtostrf(calRaw[calStep], 8, 4, b);
                    u8x8.print(b);
                }

                // Update OLED row 6-7 to ask for Serial input
                u8x8.setCursor(0, 6);
                oledPrintRow((const __FlashStringHelper*)CAL_SER);
                u8x8.setCursor(0, 7);
                oledPrintRow(F("(type + Enter)  "));

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

            // OLED confirmation
            oledClearAll();
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.setCursor(0, 3);
            oledPrintRow((const __FlashStringHelper*)CAL_DONE_L1);
            u8x8.setCursor(0, 4);
            oledPrintRow((const __FlashStringHelper*)CAL_DONE_L2);
            delay(2000);

            oledClearAll();
            calState = CAL_IDLE;
            return false;
        }

        // ── CAL_ABORT ─────────────────────────────────────────────────────────
        case CAL_ABORT: {
            Serial.println(F("Cal aborted. EEPROM unchanged."));

            oledClearAll();
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.setCursor(0, 3);
            oledPrintRow((const __FlashStringHelper*)CAL_ABORT_L1);
            u8x8.setCursor(0, 4);
            oledPrintRow((const __FlashStringHelper*)CAL_ABORT_L2);
            delay(2000);

            oledClearAll();
            calState = CAL_IDLE;
            return false;
        }

        default: break;
    }

    return (calState != CAL_IDLE);
}

// =============================================================================
// OLED — normal operation display
// Row 0-1: Voltage + V-min  Row 2-3: Current  Row 4-5: Power
// Row 6  : Peak-hold bar    Row 7  : Uptime + SD tag
// =============================================================================
static void updateOLED(float volts, float iMA, float pmW) {
    char b[8];

    u8x8.setFont(u8x8_font_8x13B_1x2_r);

    // Row 0-1 – Voltage + V-min ("V:  5.234 Vm5.2")
    u8x8.setCursor(0, 0);
    u8x8.print(F("V:"));
    dtostrf(volts, 7, 3, b); u8x8.print(b);
    u8x8.print(F(" V"));
    u8x8.print('m');
    if (minVoltage > 999.0f) { u8x8.print(F("---")); }
    else { dtostrf(minVoltage, 3, 1, b); u8x8.print(b); }

    // Row 2-3 – Current
    u8x8.setCursor(0, 2);
    u8x8.print(F("I:"));
    if (iMA >= 1000.0f) {
        dtostrf(iMA * 0.001f, 6, 3, b);
        u8x8.print(b); u8x8.print(F(" A  "));
    } else {
        dtostrf(iMA, 6, 1, b);
        u8x8.print(b); u8x8.print(F("mA  "));
    }

    // Row 4-5 – Power
    u8x8.setCursor(0, 4);
    u8x8.print(F("P:"));
    if (pmW >= 1000.0f) {
        dtostrf(pmW * 0.001f, 6, 3, b);
        u8x8.print(b); u8x8.print(F(" W  "));
    } else {
        dtostrf(pmW, 6, 1, b);
        u8x8.print(b); u8x8.print(F("mW  "));
    }

    u8x8.setFont(u8x8_font_chroma48medium8_r);

    // Row 6 – Peak-hold VU bar
    {
        char line[17];
        uint8_t live = (uint8_t)((iMA           / BAR_SCALE_MA) * 16.0f);
        uint8_t pk   = (uint8_t)((peakCurrentMA / BAR_SCALE_MA) * 16.0f);
        if (live > 16) live = 16;
        if (pk   > 16) pk   = 16;
        if (pk   >  0) pk--;
        memset(line, ' ', 16); line[16] = '\0';
        for (uint8_t c = 0; c < live; c++) line[c] = '=';
        if (peakCurrentMA > 0.0f) line[pk] = '|';
        u8x8.setCursor(0, 6);
        u8x8.print(line);
    }

    // Row 7 – Uptime + SD tag
    {
        char ut[17];
        buildUptimeRow(ut);
        u8x8.setCursor(0, 7);
        u8x8.print(ut);
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
// Button poll — non-blocking short / long press, no delay().
// Short press : button released before LONG_PRESS_MS  → btnShortPress flag.
// Long press  : button held >= LONG_PRESS_MS          → btnLongPress flag
//               (fires once while held; does not repeat).
// =============================================================================
static void pollButton() {
    bool raw = digitalRead(MARKER_PIN);
    unsigned long now = millis();

    // Debounce: reset timer on any change
    if (raw != lastBtnState) {
        lastDebounceTime = now;
        lastBtnState = raw;
    }

    if ((now - lastDebounceTime) < DEBOUNCE_MS) return; // still bouncing

    // Stable LOW: button is being held
    if (debounceState == HIGH && raw == LOW) {
        // Falling edge — record press start
        btnPressStart = now;
        longFired     = false;
        debounceState = LOW;
    }

    // Still held: check for long-press threshold
    if (debounceState == LOW && raw == LOW && !longFired) {
        if ((now - btnPressStart) >= LONG_PRESS_MS) {
            btnLongPress = true;
            longFired    = true;  // one-shot: won't re-fire while held
        }
    }

    // Rising edge: button released
    if (debounceState == LOW && raw == HIGH) {
        debounceState = HIGH;
        if (!longFired) {
            btnShortPress = true;  // released before long-press threshold
        }
        longFired = false;
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
            calMode = 0; calStep = 0; calState = CAL_SET_SOURCE;
            Serial.println(F("Starting voltage calibration. Q to abort."));
            return;
        } else if (cmd == 'i' || cmd == 'I') {
            calMode = 1; calStep = 0; calState = CAL_SET_SOURCE;
            Serial.println(F("Starting current calibration. Q to abort."));
            return;
        } else if (cmd == 'z' || cmd == 'Z') {
            eepromFactoryReset();
        }
    }

    // ── Button actions: short press = marker, long press = reset holds ─────
    if (btnShortPress) {
        btnShortPress = false;
        logMarker(F("MANUAL MARKER"));
        Serial.println(F("--- MARKER ---"));
    }
    if (btnLongPress) {
        btnLongPress = false;
        float snapV = getCalV();
        float snapI = getCalIma();
        if (snapI < 0.0f) snapI = 0.0f;
        minVoltage    = snapV;   maxVoltage   = snapV;
        peakCurrentMA = snapI;   maxCurrentMA = snapI;
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
    if (iMA < 0.0f) iMA = 0.0f;

    emaCurrentMA = emaInit
        ? EMA_ALPHA * iMA + (1.0f - EMA_ALPHA) * emaCurrentMA
        : iMA;
    emaInit = true;
    iMA = emaCurrentMA;

    float pmW = volts * iMA;

    // ── V-min hold ───────────────────────────────────────────────────────────
    if (volts < minVoltage) minVoltage = volts;
    if (volts > maxVoltage) maxVoltage = volts;

    // ── Peak-hold ────────────────────────────────────────────────────────────
    if (iMA >= peakCurrentMA) peakCurrentMA = iMA;
    if (iMA >  maxCurrentMA)  maxCurrentMA  = iMA;
    if (now - lastPeakDecay >= PEAK_DECAY_MS) {
        lastPeakDecay = now;
        if (peakCurrentMA > iMA) {
            peakCurrentMA -= PEAK_DECAY_STEP_MA;
            if (peakCurrentMA < iMA) peakCurrentMA = iMA;
        }
    }

    // ── Display ──────────────────────────────────────────────────────────────
    updateOLED(volts, iMA, pmW);

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
