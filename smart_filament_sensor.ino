#include <Wire.h>
#include <AS5600.h>
#define FASTLED_INTERNAL       // Suppress FastLED pragma messages
#include <FastLED.h>
#include <Preferences.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define NEO_PIN 2
#define NUM_LEDS 1
#define FW_VERSION "2.5.0"

Preferences prefs;
AS5600 as5600;
CRGB leds[NUM_LEDS];

// ── Settings (NVS persisted) ──
float degrees_per_mm;
int sensitivity;        // Runout: minimum steps to count as "filament moving"
int noise_threshold;    // Deadband: ignore encoder jitter below this
int direction_multiplier;
int led_brightness;     // 1-255

// ── Encoder State ──
uint16_t lastAngle = 0;
long long global_steps = 0; 
long long session_start_steps = 0;
long long odometer_reset_steps = 0;
float lastLoggedMm = -999.0; 

// ── Mode Flags ──
bool measure_active = false;
bool test_active = false;
float test_target = 10.0;

// ── Timing ──
unsigned long lastMoveTime = 0;
unsigned long lastMoveLedTime = 0;
const unsigned long TIMEOUT_MS = 5000;
const unsigned long DEEP_IDLE_TIMEOUT_MS = 300000; // 5 minutes (5 * 60 * 1000)

// ── LED Animation ──
CRGB currentColor = CRGB::Black;
CRGB targetColor = CRGB::Black;

// ── Distance Calculation ──
double calculateDistance(long long steps) {
    if (degrees_per_mm < 0.01f) return 0.0;
    double distance = ((double)steps * 360.0) / (4096.0 * (double)degrees_per_mm);
    return distance * (double)direction_multiplier;
}

// ── LED Animation Engine (FastLED, ultra-smooth) ──
//
// Rate-limited to LED_FPS. FastLED.show() disables interrupts while it
// bit-bangs the WS2812 line and then waits out the reset latch; calling it
// on every loop iteration spent most of the loop budget on an LED nobody can
// see change faster than ~60Hz, and stole time from encoder polling. The
// encoder is read every iteration regardless -- only the LED is throttled.
static unsigned long lastLedUpdate = 0;
const unsigned long LED_INTERVAL_MS = 1000 / 60;   // 60 FPS

void updateLed() {
    unsigned long now = millis();
    if (now - lastLedUpdate < LED_INTERVAL_MS) return;
    lastLedUpdate = now;
    
    if (test_active) {
        // CALIBRATION: Smooth fade to warm yellow
        targetColor = CRGB(255, 180, 0);
        nblend(currentColor, targetColor, 30);
        leds[0] = currentColor;
    } 
    else if (measure_active) {
        // MEASURE: Smooth fade to green
        targetColor = CRGB(0, 255, 0);
        nblend(currentColor, targetColor, 30);
        leds[0] = currentColor;
    }
    else if (now - lastMoveLedTime < 1500) {
        // MOVEMENT: Direct 16-bit blue pulse (no nblend lag)
        uint16_t pulse16 = beatsin16(100, 0, 65535);
        uint8_t pulse = pulse16 >> 8;
        currentColor = CRGB(0, 0, pulse);
        leds[0] = currentColor;
    }
    else if (now - lastMoveTime >= DEEP_IDLE_TIMEOUT_MS) {
        // DEEP IDLE: Completely off after 5 minutes of no movement
        currentColor = CRGB::Black;
        leds[0] = currentColor;
    }
    else if (now - lastMoveTime >= 5000) {
        // IDLE: Direct 16-bit white breathing (no nblend, maximum smoothness)
        uint16_t breath16 = beatsin16(15, 0, 65535); // 0 ensures it fades to completely black
        uint8_t breath = breath16 >> 8;
        currentColor = CRGB(breath, breath, breath);
        leds[0] = currentColor;
    }
    else {
        // TRANSITION: Gentle fade to dim white
        targetColor = CRGB(30, 30, 40);
        nblend(currentColor, targetColor, 8);
        leds[0] = currentColor;
    }
    
    // Apply global brightness (preserves temporal dithering at low levels)
    FastLED.setBrightness(led_brightness);
    FastLED.show();
}

void printHelp() {
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║       SMART FILAMENT SENSOR - HELP MENU v" FW_VERSION "     ║");
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║  COMMANDS:                                          ║");
    Serial.println("║  ─────────────────────────────────────────────────  ║");
    Serial.println("║  HELP / ?        Show this help menu                ║");
    Serial.println("║  STATUS          Show current settings & state      ║");
    Serial.println("║  START [mm]      Start calibration (default 10mm)   ║");
    Serial.println("║  MEASURE         Start distance measurement         ║");
    Serial.println("║  STOP / END      Stop active mode, go to standby   ║");
    Serial.println("║  VERSION         Show firmware version              ║");
    Serial.println("║  GET_MM          Get odometer mm since last reset     ║");
    Serial.println("║  GET_MM_RESET    Get mm and atomically reset counter  ║");
    Serial.println("║  RESET_MM        Reset odometer counter               ║");
    Serial.println("║  HEALTH          Report magnet health (ok/weak/strong)║");
    Serial.println("║  GET_CAL         Report calibration factor (deg/mm)   ║");
    Serial.println("║  GET_STEPS       Report raw encoder steps             ║");
    Serial.println("║  DIAG            Dump AS5600 registers (diagnostics)  ║");
    Serial.println("║                                                     ║");
    Serial.println("║  SETTINGS:                                          ║");
    Serial.println("║  ─────────────────────────────────────────────────  ║");
    Serial.println("║  SET SENS <1-50>     Runout detection sensitivity   ║");
    Serial.println("║  SET NOISE <1-20>    Noise filter threshold (steps) ║");
    Serial.println("║  SET DIR <1/-1>      Encoder direction multiplier   ║");
    Serial.println("║  SET BRIGHT <1-255>  LED brightness                 ║");
    Serial.println("║  SET CAL <float>     Calibration factor (deg/mm)    ║");
    Serial.println("║  RESET               Reset all settings to default  ║");
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║  CURRENT SETTINGS:                                  ║");
    Serial.println("║  ─────────────────────────────────────────────────  ║");
    Serial.printf( "║  Calibration : %.4f deg/mm                   ║\n", degrees_per_mm);
    Serial.printf( "║  Sensitivity : %-3d (min steps to register movement) ║\n", sensitivity);
    Serial.printf( "║  Noise Filter: %-3d (deadband, encoder steps)       ║\n", noise_threshold);
    Serial.printf( "║  Direction   : %-2d                                  ║\n", direction_multiplier);
    Serial.printf( "║  Brightness  : %-3d / 255                           ║\n", led_brightness);
    Serial.printf( "║  Timeout     : %lu ms                            ║\n", TIMEOUT_MS);
    Serial.println("║                                                     ║");
    Serial.println("║  LED STATES:                                        ║");
    Serial.println("║  ─────────────────────────────────────────────────  ║");
    Serial.println("║  White Breathing = Idle (5s+ no movement)           ║");
    Serial.println("║  LED Off         = Deep Idle (5m+ no movement)      ║");
    Serial.println("║  Blue Pulsing    = Filament moving                  ║");
    Serial.println("║  Solid Green     = Measure mode                     ║");
    Serial.println("║  Solid Yellow    = Calibration mode                 ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.println();
}

void printStatus() {
    Serial.println("\n┌─── STATUS ────────────────────────────┐");
    Serial.printf( "│ Mode : %-12s  Magnet: %-9s │\n",
                   test_active ? "CALIBRATION" : (measure_active ? "MEASURE" : "STANDBY"),
                   as5600.detectMagnet() ? "OK" : "NO MAGNET");
    Serial.printf( "│ CAL:%.4f  SENS:%d  NOISE:%d  DIR:%d  │\n", 
                   degrees_per_mm, sensitivity, noise_threshold, direction_multiplier);
    Serial.printf( "│ BRIGHT:%-3d                                    │\n",
                   led_brightness);
    Serial.printf( "│ AGC:%d  Steps:%lld                         │\n", 
                   as5600.readAGC(), global_steps);
    Serial.println("└───────────────────────────────────────┘");
}

// ═══════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); 
    as5600.begin();
    
    // AS5600 Configuration for maximum responsiveness.
    //
    // setSlowFilter takes the raw SF bit pattern, NOT a "level". Per the
    // AS5600 datasheet CONF[1:0]: 0 = 16x, 1 = 8x, 2 = 4x, 3 = 2x. Passing 0
    // therefore selected the SLOWEST, most heavily filtered mode -- the exact
    // opposite of the old comment. 3 = 2x is the minimum filtering.
    as5600.setWatchDog(0);     // Always-on mode (no low-power sleep)
    as5600.setSlowFilter(3);   // 2x = minimum slow filtering
    as5600.setFastFilter(0);   // 0 = slow filter only (fast filter disabled)
    as5600.setHysteresis(0);   // No hardware hysteresis (handled in software)
    
    // FastLED setup
    FastLED.addLeds<WS2812B, NEO_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setCorrection(TypicalSMD5050);
    FastLED.setDither(BINARY_DITHER);      // Temporal dithering for extra smoothness
    FastLED.setMaxPowerInMilliWatts(500);   // Power safety limit
    
    prefs.begin("sensor", false);
    degrees_per_mm    = prefs.getFloat("cal", 12.5);
    sensitivity       = prefs.getInt("sens", 5);
    noise_threshold   = prefs.getInt("noise", 3);
    direction_multiplier = prefs.getInt("dir", 1);
    led_brightness    = prefs.getInt("bright", 50);

    leds[0] = CRGB(50, 50, 50);
    FastLED.show();
    
    // Warmup: read a few times to stabilize
    for (int i = 0; i < 5; i++) {
        as5600.rawAngle();
        delay(2);
    }
    lastAngle = as5600.rawAngle();
    lastMoveTime = millis();
    lastMoveLedTime = 0; // Start in idle mode
    
    // Magnet check on boot
    Serial.printf("\n>>> SMART FILAMENT SENSOR v%s\n", FW_VERSION);
    if (!as5600.detectMagnet()) {
        Serial.println(">>> WARNING: No magnet detected! Check sensor positioning.");
    } else {
        uint8_t agc = as5600.readAGC();
        // At 3.3V the AGC range is 0-128 and the ideal value is the middle
        // of it (~64). Both extremes hurt accuracy, so report both.
        if (as5600.magnetTooWeak()) {
            Serial.printf(">>> WARNING: Magnet too weak (AGC %d)! Move it closer.\n", agc);
        } else if (as5600.magnetTooStrong()) {
            Serial.printf(">>> WARNING: Magnet too strong (AGC %d)! Increase the air gap.\n", agc);
        } else {
            Serial.printf(">>> Magnet OK (AGC: %d, ideal ~64 at 3.3V)\n", agc);
        }
    }
    Serial.println(">>> Type 'HELP' or '?' for commands.\n");
}

// ═══════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════
void loop() {
    // ── 1. Command Parser ──
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
        String upperInput = input;
        upperInput.toUpperCase();

        if (upperInput == "HELP" || upperInput == "?") {
            printHelp();
        }
        else if (upperInput == "VERSION") {
            Serial.printf(">>> Smart Filament Sensor v%s\n", FW_VERSION);
        }
        else if (upperInput == "PING") {
            Serial.println("PONG");
        }
        else if (upperInput == "GET_MM") {
            double measured = calculateDistance(global_steps - odometer_reset_steps);
            Serial.printf("MM:%.4f\n", measured);
        }
        else if (upperInput == "RESET_MM") {
            odometer_reset_steps = global_steps;
            Serial.println("MM:RESET");
        }
        else if (upperInput == "GET_MM_RESET") {
            double measured = calculateDistance(global_steps - odometer_reset_steps);
            odometer_reset_steps = global_steps;
            Serial.printf("MM:%.4f\n", measured);
        }
        else if (upperInput == "GET_CAL") {
            // Machine-readable calibration factor. Klipper needs this to tell
            // "the encoder is slipping" apart from "the stored deg/mm is
            // wrong" -- previously the value was only visible in the human
            // STATUS block, which the Klipper module does not parse.
            Serial.printf("CAL:%.4f\n", degrees_per_mm);
        }
        else if (upperInput == "DIAG") {
            // Full AS5600 register dump on one machine-readable line.
            // ZPOS/MPOS/MANG non-default => readAngle() would have been
            // scaled; CONF holds the filter/hysteresis/watchdog bits; ZMCO
            // counts how many times the chip has been permanently burned.
            Serial.printf(
                "DIAG:fw=%s,cal=%.4f,dir=%d,noise=%d,sens=%d,"
                "zmco=%u,zpos=%u,mpos=%u,mang=%u,conf=0x%04X,"
                "agc=%u,mag=%u,magnet=%u,steps=%lld\n",
                FW_VERSION, degrees_per_mm, direction_multiplier,
                noise_threshold, sensitivity,
                (unsigned)as5600.getZMCO(), (unsigned)as5600.getZPosition(),
                (unsigned)as5600.getMPosition(), (unsigned)as5600.getMaxAngle(),
                (unsigned)as5600.getConfiguration(),
                (unsigned)as5600.readAGC(), (unsigned)as5600.readMagnitude(),
                (unsigned)(as5600.detectMagnet() ? 1 : 0), global_steps);
        }
        else if (upperInput == "GET_STEPS") {
            // Raw encoder steps since the last odometer reset. Lets the host
            // recompute mm with any deg/mm without reflashing.
            Serial.printf("STEPS:%lld\n", global_steps - odometer_reset_steps);
        }
        else if (upperInput == "HEALTH") {
            // Report magnet health for Klipper monitoring.
            //
            // The old code tested `agc > 240` for "too weak". That test can
            // never fire on this board: the AS5600 AGC range is 0-255 at 5V
            // but only 0-128 at 3.3V, and the ESP32-C3 runs the sensor at
            // 3.3V. So the weak-magnet warning was dead code, and there was
            // no strong-magnet warning at all -- an AGC pinned at 0 (magnet
            // far too close) reported "ok".
            //
            // Use the chip's own MH / ML status bits instead of guessing
            // from AGC. Per the datasheet the ideal AGC sits in the MIDDLE
            // of its range, so ~64 at 3.3V; adjust the air gap to get there.
            bool detected = as5600.detectMagnet();
            uint8_t agc = as5600.readAGC();
            if (!detected) {
                Serial.println("HEALTH:no_magnet");
            } else if (as5600.magnetTooWeak()) {
                Serial.printf("HEALTH:too_weak:%d\n", agc);
            } else if (as5600.magnetTooStrong()) {
                Serial.printf("HEALTH:too_strong:%d\n", agc);
            } else {
                Serial.printf("HEALTH:ok:%d\n", agc);
            }
        }
        else if (upperInput == "APPLY") {
            if (test_active) {
                test_active = false;
                long long test_steps = global_steps - session_start_steps;
                float measured_mm = fabs(calculateDistance(test_steps));
                if (llabs(test_steps) > 10) { 
                    float correction = test_target / measured_mm;
                    if (correction > 0.33 && correction < 3.0) {
                        degrees_per_mm = degrees_per_mm / correction;
                        prefs.putFloat("cal", degrees_per_mm); 
                        Serial.printf("\n>>> CALIBRATION SUCCESS!\n");
                        Serial.printf(">>> Measured: %.2f mm | Target: %.1f mm\n", measured_mm, test_target);
                        Serial.printf(">>> New Cal Factor: %.4f deg/mm (saved)\n", degrees_per_mm);
                Serial.printf("CAL:%.4f\n", degrees_per_mm);
                    } else {
                        Serial.println("\n>>> ERROR: Calibration result too far off. Try again.");
                        Serial.printf(">>> Measured: %.2f mm | Target: %.1f mm | Correction: %.2fx\n", measured_mm, test_target, correction);
                    }
                } else {
                    Serial.println("\n>>> ERROR: No significant movement detected, calibration cancelled.");
                }
            } else {
                Serial.println(">>> ERROR: Calibration not active.");
            }
        }
        else if (upperInput.startsWith("SET SENS ")) {
            int val = upperInput.substring(9).toInt();
            if (val >= 1 && val <= 50) {
                sensitivity = val;
                prefs.putInt("sens", sensitivity);
                Serial.printf(">>> Sensitivity set to %d (saved)\n", sensitivity);
            } else {
                Serial.println(">>> ERROR: Sensitivity must be 1-50");
            }
        }
        else if (upperInput.startsWith("SET DIR ")) {
            int val = upperInput.substring(8).toInt();
            if (val == 1 || val == -1) {
                direction_multiplier = val;
                prefs.putInt("dir", direction_multiplier);
                Serial.printf(">>> Direction set to %d (saved)\n", direction_multiplier);
            } else {
                Serial.println(">>> ERROR: Direction must be 1 or -1");
            }
        }
        else if (upperInput.startsWith("SET BRIGHT ")) {
            int val = upperInput.substring(11).toInt();
            if (val >= 1 && val <= 255) {
                led_brightness = val;
                prefs.putInt("bright", led_brightness);
                Serial.printf(">>> Brightness set to %d (saved)\n", led_brightness);
            } else {
                Serial.println(">>> ERROR: Brightness must be 1-255");
            }
        }
        else if (upperInput.startsWith("SET NOISE ")) {
            int val = upperInput.substring(10).toInt();
            if (val >= 1 && val <= 20) {
                noise_threshold = val;
                prefs.putInt("noise", noise_threshold);
                Serial.printf(">>> Noise threshold set to %d steps (saved)\n", noise_threshold);
            } else {
                Serial.println(">>> ERROR: Noise threshold must be 1-20");
            }
        }
        else if (upperInput.startsWith("SET CAL ")) {
            float val = input.substring(8).toFloat();
            if (val > 0.01 && val < 1000.0) {
                degrees_per_mm = val;
                prefs.putFloat("cal", degrees_per_mm);
                Serial.printf(">>> Calibration factor set to %.4f deg/mm (saved)\n", degrees_per_mm);
                Serial.printf("CAL:%.4f\n", degrees_per_mm);
            } else {
                Serial.println(">>> ERROR: Cal factor must be between 0.01 and 1000.0");
            }
        }
        else if (upperInput == "RESET") {
            degrees_per_mm = 12.5;
            sensitivity = 5;
            noise_threshold = 3;
            direction_multiplier = 1;
            led_brightness = 50;
            prefs.putFloat("cal", degrees_per_mm);
            prefs.putInt("sens", sensitivity);
            prefs.putInt("noise", noise_threshold);
            prefs.putInt("dir", direction_multiplier);
            prefs.putInt("bright", led_brightness);
            Serial.println(">>> All settings reset to defaults (saved)");
            printHelp();
        }
        else if (upperInput.startsWith("START")) {
            int spaceIndex = input.indexOf(' ');
            test_target = (spaceIndex != -1) ? input.substring(spaceIndex + 1).toFloat() : 10.0;
            if (test_target < 1.0 || test_target > 500.0) {
                Serial.println(">>> ERROR: Target must be 1-500 mm");
            } else {
                session_start_steps = global_steps;
                test_active = true; measure_active = false;
                lastLoggedMm = -999.0;
                lastMoveTime = millis();
                Serial.printf(">>> CALIBRATION ACTIVE: Push exactly %.1f mm of filament, then wait.\n", test_target);
            }
        } 
        else if (upperInput == "MEASURE") {
            session_start_steps = global_steps;
            measure_active = true; test_active = false;
            lastLoggedMm = -999.0;
            lastMoveTime = millis();
            Serial.println(">>> MEASURE ACTIVE");
        }
        else if (upperInput == "STATUS") {
            printStatus();
        }
        else if (upperInput == "STOP" || upperInput == "END") {
            if (measure_active) {
                long long rel_steps = global_steps - session_start_steps;
                float final_mm = calculateDistance(rel_steps);
                Serial.printf(">>> MEASURE ENDED: Total = %.2f mm (%lld steps)\n", final_mm, rel_steps);
            }
            measure_active = false; test_active = false;
            Serial.println(">>> STANDBY");
        }
        else {
            Serial.println(">>> Unknown command. Type 'HELP' or '?' for available commands.");
        }
        } // end if (input.length() > 0)
    }

    // ── 2. Encoder Reading ──
    //
    // rawAngle(), NOT readAngle(). readAngle() returns the ANGLE register,
    // which the chip scales and clamps according to ZPOS / MPOS / MANG. If
    // those registers hold anything other than defaults, part of every
    // revolution saturates at 0 or 4095 and those encoder steps are lost for
    // good -- a constant multiplicative under-read that looks exactly like a
    // partial clog and that calibration cannot reliably compensate (the loss
    // depends on where in the revolution you start). rawAngle() is the
    // unprocessed 12-bit reading and is immune to all of it. Run DIAG to see
    // what those registers actually contain.
    uint16_t currentAngle = as5600.rawAngle();
    int rawDelta = (int)currentAngle - (int)lastAngle;

    // Handle 12-bit rollover
    if (rawDelta > 2048) rawDelta -= 4096;
    else if (rawDelta < -2048) rawDelta += 4096;

    // Noise deadband.
    //
    // The accumulator must receive the TRUE delta. lastAngle advances to
    // currentAngle below, so whatever is not added here is discarded for
    // good. A 3-sample median filter used to sit at this point and fed its
    // filtered value to global_steps while still advancing lastAngle — so
    // every difference between rawDelta and the median was silently lost.
    // Real acceleration looks exactly like a spike to a median, which is why
    // a slow steady calibration extrude measured ~99% accurate while a real
    // print (constant accel/decel) under-read by up to 80%.
    //
    // Glitch protection: the rollover handling above already bounds rawDelta
    // to +/-2048 steps, about 14mm of filament in a single loop iteration —
    // physically unreachable at any real print speed.
    //
    // Below the deadband lastAngle is intentionally NOT advanced, so slow
    // movement accumulates across iterations instead of being filtered away.
    if (abs(rawDelta) >= noise_threshold) {
        global_steps += rawDelta;
        lastAngle = currentAngle;

        if (measure_active || test_active) {
            long long rel_steps = global_steps - session_start_steps;
            float current_mm = calculateDistance(rel_steps);
            
            // Log every 1mm change (avoids flooding Klipper console)
            if (fabs(current_mm - lastLoggedMm) >= 1.0f) {
                Serial.printf("[%s] %.2f mm (Steps: %lld)\n", 
                             (test_active ? "CAL" : "MEA"), current_mm, rel_steps);
                lastLoggedMm = current_mm;
            }
            lastMoveTime = millis();
            lastMoveLedTime = millis();
        } 
        else if (abs(rawDelta) >= sensitivity) {
            lastMoveTime = millis();
            lastMoveLedTime = millis();
        }
    }

    // ── 3. Calibration Auto-Save ──
    if (test_active && (millis() - lastMoveTime >= TIMEOUT_MS)) {
        test_active = false;
        long long test_steps = global_steps - session_start_steps;
        float measured_mm = fabs(calculateDistance(test_steps));
        
        if (llabs(test_steps) > 10) { 
            // Correct calibration formula
            float correction = test_target / measured_mm;
            // Sanity bounds (max 3x adjustment per calibration)
            if (correction > 0.33 && correction < 3.0) {
                degrees_per_mm = degrees_per_mm / correction;
                prefs.putFloat("cal", degrees_per_mm); 
                Serial.printf("\n>>> CALIBRATION SUCCESS!\n");
                Serial.printf(">>> Measured: %.2f mm | Target: %.1f mm\n", measured_mm, test_target);
                Serial.printf(">>> New Cal Factor: %.4f deg/mm (saved)\n", degrees_per_mm);
                Serial.printf("CAL:%.4f\n", degrees_per_mm);
            } else {
                Serial.println("\n>>> ERROR: Calibration result too far off. Try again.");
                Serial.printf(">>> Measured: %.2f mm | Target: %.1f mm | Correction: %.2fx\n", measured_mm, test_target, correction);
            }
        } else {
            Serial.println("\n>>> ERROR: No significant movement detected, calibration cancelled.");
        }
    } 
    
    // ── 5. LED Animation (always runs, non-blocking) ──
    updateLed();
}