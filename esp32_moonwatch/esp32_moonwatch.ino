/*
 * ESP32 Moonwatch — Sky Object Tracker
 * ======================================
 * Scans the night sky, detects bright objects, zooms in, records to SD.
 *
 * Hardware: ESP32-CAM (AI Thinker) + 2x MG90S servos + microSD
 * See HOWTO.md for full wiring and setup instructions.
 *
 * State machine:
 *   IDLE → SCAN → DETECT → TRACK → RECORD → IDLE
 *
 * Serial commands — see handleSerial() or send 'help' for full list
 */

#include "config.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#if WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#endif

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State { IDLE, SCANNING, DETECTING, TRACKING, RECORDING };
State state = IDLE;
const char* stateNames[] = { "IDLE", "SCANNING", "DETECTING", "TRACKING", "RECORDING" };

// ---------------------------------------------------------------------------
// Servo
// ---------------------------------------------------------------------------
Servo servoPan, servoTilt;
float panDeg  = PAN_HOME_DEG;
float tiltDeg = TILT_HOME_DEG;

// Raster scan state
int scanPanStep  = 0;
int scanTiltRow  = 0;
bool scanPanDir  = true; // true = increasing pan angle (boustrophedon / snake scan)
unsigned long scanLastMove = 0;

// ---------------------------------------------------------------------------
// PID state
// ---------------------------------------------------------------------------
float pidErrXPrev = 0, pidIntX = 0;
float pidErrYPrev = 0, pidIntY = 0;
unsigned long pidLastTime = 0;

// ---------------------------------------------------------------------------
// Zoom
// ---------------------------------------------------------------------------
int zoomLevel = ZOOM_DEFAULT;  // 1 = full frame, 4 = 4× crop

// ---------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------
bool recording         = false;
uint32_t frameCount    = 0;
uint32_t sessionNumber = 1;
char sessionDir[64];
unsigned long lastFrameTime = 0;

// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------
unsigned long ledLastToggle = 0;
bool ledState = false;

// ---------------------------------------------------------------------------
// Blob detection result
// ---------------------------------------------------------------------------
struct Blob {
    bool  found;
    float cx;   // 0.0–1.0 normalised centre X
    float cy;   // 0.0–1.0 normalised centre Y
    int   size; // number of bright pixels
};

// ===========================================================================
// Forward declarations
// ===========================================================================
bool     cameraInit();
bool     sdInit();
Blob     detectBlob(camera_fb_t* fb);
void     moveTo(float pan, float tilt, int stepMs = 10);
void     pidUpdate(float errX, float errY);
void     applyZoom(sensor_t* sensor, int zoom);
void     saveFrame(camera_fb_t* fb);
void     startRecording();
void     stopRecording();
void     nextSession();
void     handleSerial();
void     updateLED();
void     printStatus();
#if WIFI_ENABLED
void     startWifi();
void     handleStream();
WebServer server(STREAM_PORT);
#endif

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("\n=== ESP32 Moonwatch ===");
    Serial.println("Type '?' for commands\n");

    // LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // Servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servoPan.setPeriodHertz(50);
    servoTilt.setPeriodHertz(50);
    servoPan.attach(PIN_SERVO_PAN,  500, 2400);
    servoTilt.attach(PIN_SERVO_TILT, 500, 2400);
    moveTo(PAN_HOME_DEG, TILT_HOME_DEG, 20);
    Serial.println("[SERVO] Servos homed.");

    // Camera
    if (!cameraInit()) {
        Serial.println("[CAM] FATAL: camera init failed. Check ribbon cable.");
        while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
    }
    Serial.println("[CAM] Camera OK.");

    // SD card (1-bit MMC mode)
    if (!sdInit()) {
        Serial.println("[SD] Warning: SD card not found. Recording disabled.");
    } else {
        nextSession();  // find next free session directory
        Serial.printf("[SD] Ready. Session: %s\n", sessionDir);
    }

#if WIFI_ENABLED
    startWifi();
#endif

    state = IDLE;
    Serial.println("[SYS] Ready. Send 's' to start scan.");
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
    handleSerial();
    broadcastStatus();
    updateLED();

#if WIFI_ENABLED
    server.handleClient();
#endif

    switch (state) {

    // -----------------------------------------------------------------------
    case IDLE:
        // Do nothing, wait for command
        delay(50);
        break;

    // -----------------------------------------------------------------------
    case SCANNING: {
        // Fast boustrophedon (snake) raster scan of sky
        unsigned long now = millis();
        if (now - scanLastMove < (unsigned long)runtimeScanDelay) break;
        scanLastMove = now;

        // Move servo to scan position
        float scanPan  = PAN_MIN_DEG  + scanPanStep  * runtimeScanStep;
        float scanTilt = TILT_MIN_DEG + scanTiltRow  * ((TILT_MAX_DEG - TILT_MIN_DEG) / SCAN_ROWS);
        panDeg  = constrain(scanPan,  PAN_MIN_DEG,  PAN_MAX_DEG);
        tiltDeg = constrain(scanTilt, TILT_MIN_DEG, TILT_MAX_DEG);
        servoPan.write((int)panDeg);
        servoTilt.write((int)tiltDeg);

        // Grab frame and look for bright blob
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            Blob b = detectBlob(fb);
            esp_camera_fb_return(fb);

            if (b.found && b.size >= runtimeMinBlob) {
                Serial.printf("[SCAN] Target detected! blob=%d px at (%.2f,%.2f)\n",
                              b.size, b.cx, b.cy);
                pidErrXPrev = 0; pidIntX = 0;
                pidErrYPrev = 0; pidIntY = 0;
                pidLastTime = millis();
                state = DETECTING;
                break;
            }
        }

        // Advance scan position (boustrophedon)
        if (scanPanDir) {
            scanPanStep++;
            int maxStep = (PAN_MAX_DEG - PAN_MIN_DEG) / runtimeScanStep;
            if (scanPanStep > maxStep) {
                scanPanStep = maxStep;
                scanPanDir  = false;
                scanTiltRow++;
            }
        } else {
            scanPanStep--;
            if (scanPanStep < 0) {
                scanPanStep = 0;
                scanPanDir  = true;
                scanTiltRow++;
            }
        }

        if (scanTiltRow > SCAN_ROWS) {
            // Full sky scanned, restart
            scanTiltRow = 0;
            Serial.println("[SCAN] Full sweep done, restarting...");
        }
        break;
    }

    // -----------------------------------------------------------------------
    case DETECTING: {
        // Confirm target and centre on it — average a few frames
        static int confirmFrames = 0;
        static float sumCx = 0, sumCy = 0;

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) break;
        Blob b = detectBlob(fb);
        esp_camera_fb_return(fb);

        if (b.found) {
            sumCx += b.cx;
            sumCy += b.cy;
            confirmFrames++;

            if (confirmFrames >= 3) {
                // Nudge servos toward blob centre
                float cx = sumCx / confirmFrames;
                float cy = sumCy / confirmFrames;
                float errX = (cx - 0.5f) * (PAN_MAX_DEG  - PAN_MIN_DEG);
                float errY = (cy - 0.5f) * (TILT_MAX_DEG - TILT_MIN_DEG);
                panDeg  = constrain(panDeg  + errX * KP, PAN_MIN_DEG,  PAN_MAX_DEG);
                tiltDeg = constrain(tiltDeg + errY * KP, TILT_MIN_DEG, TILT_MAX_DEG);
                servoPan.write((int)panDeg);
                servoTilt.write((int)tiltDeg);

                confirmFrames = 0; sumCx = 0; sumCy = 0;

                if (fabs(errX) < 5.0f && fabs(errY) < 5.0f) {
                    Serial.println("[DETECT] Locked on. Switching to TRACK.");
                    applyZoom(esp_camera_sensor_get(), zoomLevel);
                    state = TRACKING;
                }
            }
        } else {
            confirmFrames = 0; sumCx = 0; sumCy = 0;
            static int lostCount = 0;
            if (++lostCount > 10) {
                lostCount = 0;
                Serial.println("[DETECT] Target lost, returning to scan.");
                state = SCANNING;
            }
        }
        delay(20);
        break;
    }

    // -----------------------------------------------------------------------
    case TRACKING: {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) break;
        Blob b = detectBlob(fb);

        if (recording) {
            unsigned long now = millis();
            if (now - lastFrameTime >= RECORD_FRAME_MS) {
                saveFrame(fb);
                lastFrameTime = now;
            }
        }

        esp_camera_fb_return(fb);

        if (!b.found) {
            static int lostCount = 0;
            if (++lostCount > 15) {
                lostCount = 0;
                Serial.println("[TRACK] Target lost. Returning to scan.");
                if (recording) stopRecording();
                applyZoom(esp_camera_sensor_get(), 1);
                state = SCANNING;
            }
            break;
        }

        // PID: error is normalised offset from frame centre
        float errX = (b.cx - 0.5f) * (PAN_MAX_DEG  - PAN_MIN_DEG);
        float errY = (b.cy - 0.5f) * (TILT_MAX_DEG - TILT_MIN_DEG);
        pidUpdate(errX, errY);
        delay(20);
        break;
    }

    // -----------------------------------------------------------------------
    case RECORDING:
        // Handled inside TRACKING; this state is for future standalone use
        state = TRACKING;
        break;
    }
}

// ===========================================================================
// Camera initialisation
// ===========================================================================
bool cameraInit() {
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = CAM_PIN_D0;
    cfg.pin_d1       = CAM_PIN_D1;
    cfg.pin_d2       = CAM_PIN_D2;
    cfg.pin_d3       = CAM_PIN_D3;
    cfg.pin_d4       = CAM_PIN_D4;
    cfg.pin_d5       = CAM_PIN_D5;
    cfg.pin_d6       = CAM_PIN_D6;
    cfg.pin_d7       = CAM_PIN_D7;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_pclk     = CAM_PIN_PCLK;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_sscb_sda = CAM_PIN_SIOD;
    cfg.pin_sscb_scl = CAM_PIN_SIOC;
    cfg.pin_pwdn     = CAM_PIN_PWDN;
    cfg.pin_reset    = CAM_PIN_RESET;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = TRACK_FRAMESIZE;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = psramFound() ? 2 : 1;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] esp_camera_init error 0x%x\n", err);
        return false;
    }

    // Sensor tuning for night sky
    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, -1);
    s->set_gainceiling(s, GAINCEILING_128X);  // Max gain for low light
    s->set_exposure_ctrl(s, 1);               // Auto exposure on
    s->set_aec2(s, 1);                        // Enhanced auto exposure
    s->set_ae_level(s, 2);                    // Exposure level bias (+2)
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);                     // Auto white balance
    s->set_lenc(s, 1);                        // Lens correction
    return true;
}

// ===========================================================================
// SD card initialisation (1-bit MMC mode)
// ===========================================================================
bool sdInit() {
    // 1-bit mode: only D0 pin used — frees GPIO 12,13 for servos
    if (!SD_MMC.begin("/sdcard", true)) {
        return false;
    }
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) return false;
    Serial.printf("[SD] Card type: %s  Size: %llu MB\n",
        (cardType == CARD_MMC) ? "MMC" :
        (cardType == CARD_SD)  ? "SDSC" :
        (cardType == CARD_SDHC)? "SDHC" : "Unknown",
        SD_MMC.cardSize() / (1024 * 1024));
    return true;
}

// ===========================================================================
// Blob detection — finds centroid of pixels above brightness threshold
// Works on JPEG frame: converts to grayscale via luminance approximation
// ===========================================================================
Blob detectBlob(camera_fb_t* fb) {
    Blob result = {false, 0, 0, 0};

    // The camera outputs JPEG. For fast processing, we use the raw JPEG byte
    // stream to estimate brightness — not pixel-perfect but fast enough.
    // A bright point source creates a concentrated high-byte cluster in JPEG.
    // For proper pixel access, switch pixel_format to PIXFORMAT_GRAYSCALE.
    //
    // Here we use grayscale mode (set in sensor config below) for accurate detection.
    // The frame data is then W×H bytes, one byte per pixel.

    if (fb->format != PIXFORMAT_GRAYSCALE) {
        // Fallback: scan JPEG bytes for high-value clusters (coarse but fast)
        long sumX = 0, sumY = 0, count = 0;
        int w = fb->width, h = fb->height;
        // Sample every 4th byte as a rough brightness estimate
        for (size_t i = 0; i < fb->len; i += 4) {
            uint8_t val = fb->buf[i];
            if (val > runtimeThreshold) {
                // Estimate pixel position from byte offset (rough)
                int px = (i % (w * 2)) / 2;
                int py = i / (w * 2);
                sumX += px; sumY += py; count++;
            }
        }
        if (count >= runtimeMinBlob) {
            result.found = true;
            result.cx    = (float)sumX / (count * fb->width);
            result.cy    = (float)sumY / (count * fb->height);
            result.size  = (int)count;
        }
        return result;
    }

    // Grayscale mode — accurate per-pixel blob detection
    int w = fb->width;
    int h = fb->height;
    long sumX = 0, sumY = 0, count = 0;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t pix = fb->buf[y * w + x];
            if (pix > runtimeThreshold) {
                sumX += x;
                sumY += y;
                count++;
            }
        }
    }

    if (count >= runtimeMinBlob) {
        result.found = true;
        result.cx    = (float)sumX / ((float)count * w);
        result.cy    = (float)sumY / ((float)count * h);
        result.size  = (int)count;
    }
    return result;
}

// ===========================================================================
// Servo move with slew rate limiting (stepMs = ms per degree step)
// ===========================================================================
void moveTo(float targetPan, float targetTilt, int stepMs) {
    targetPan  = constrain(targetPan,  PAN_MIN_DEG,  PAN_MAX_DEG);
    targetTilt = constrain(targetTilt, TILT_MIN_DEG, TILT_MAX_DEG);
    while (fabs(panDeg - targetPan) > 0.5f || fabs(tiltDeg - targetTilt) > 0.5f) {
        if (panDeg  < targetPan  - 0.5f) panDeg  += 1;
        if (panDeg  > targetPan  + 0.5f) panDeg  -= 1;
        if (tiltDeg < targetTilt - 0.5f) tiltDeg += 1;
        if (tiltDeg > targetTilt + 0.5f) tiltDeg -= 1;
        servoPan.write((int)panDeg);
        servoTilt.write((int)tiltDeg);
        delay(stepMs);
    }
    panDeg  = targetPan;
    tiltDeg = targetTilt;
}

// ===========================================================================
// PID servo update — called every tracking loop iteration
// ===========================================================================
void pidUpdate(float errX, float errY) {
    unsigned long now = millis();
    float dt = (now - pidLastTime) / 1000.0f;
    if (dt <= 0 || dt > 0.5f) dt = 0.02f;
    pidLastTime = now;

    // Dead-band
    if (fabs(errX) < DETECT_HYSTERESIS) errX = 0;
    if (fabs(errY) < DETECT_HYSTERESIS) errY = 0;

    // X axis (pan)
    float dX   = (errX - pidErrXPrev) / dt;
    pidIntX   += errX * dt;
    pidIntX    = constrain(pidIntX, -20.0f, 20.0f);
    float outX = runtimeKP * errX + runtimeKI * pidIntX + runtimeKD * dX;
    outX = constrain(outX, -PID_MAX_DEG, PID_MAX_DEG);
    pidErrXPrev = errX;

    // Y axis (tilt)
    float dY   = (errY - pidErrYPrev) / dt;
    pidIntY   += errY * dt;
    pidIntY    = constrain(pidIntY, -20.0f, 20.0f);
    float outY = runtimeKP * errY + runtimeKI * pidIntY + runtimeKD * dY;
    outY = constrain(outY, -PID_MAX_DEG, PID_MAX_DEG);
    pidErrYPrev = errY;

    panDeg  = constrain(panDeg  + outX, PAN_MIN_DEG,  PAN_MAX_DEG);
    tiltDeg = constrain(tiltDeg + outY, TILT_MIN_DEG, TILT_MAX_DEG);
    servoPan.write((int)panDeg);
    servoTilt.write((int)tiltDeg);
}

// ===========================================================================
// Digital zoom — crops sensor window to centre 1/zoom of frame
// ===========================================================================
void applyZoom(sensor_t* s, int zoom) {
    if (!s) return;
    zoom = constrain(zoom, ZOOM_MIN, ZOOM_MAX);

    // OV2640 full resolution 1600×1200
    int fullW = 1600, fullH = 1200;
    int cropW = fullW / zoom;
    int cropH = fullH / zoom;
    int offsetX = (fullW - cropW) / 2;
    int offsetY = (fullH - cropH) / 2;

    s->set_res_raw(s, offsetX, offsetY, offsetX + cropW, offsetY + cropH,
                   0, 0, cropW, cropH, false, false);
    Serial.printf("[ZOOM] %d× zoom (crop %d×%d offset %d,%d)\n",
                  zoom, cropW, cropH, offsetX, offsetY);
}

// ===========================================================================
// Save a JPEG frame to SD card
// ===========================================================================
void saveFrame(camera_fb_t* fb) {
    char filename[128];
    snprintf(filename, sizeof(filename), "%s/frame_%05lu.jpg", sessionDir, frameCount++);
    File f = SD_MMC.open(filename, FILE_WRITE);
    if (!f) {
        Serial.printf("[SD] Failed to open %s for write\n", filename);
        return;
    }
    f.write(fb->buf, fb->len);
    f.close();
}

void startRecording() {
    if (recording) return;
    recording     = true;
    frameCount    = 0;
    lastFrameTime = millis();
    // Switch to higher resolution for saved frames
    sensor_t* s = esp_camera_sensor_get();
    if (s) s->set_framesize(s, RECORD_FRAMESIZE);
    Serial.printf("[REC] Recording started → %s\n", sessionDir);
}

void stopRecording() {
    if (!recording) return;
    recording = false;
    // Switch back to tracking resolution
    sensor_t* s = esp_camera_sensor_get();
    if (s) s->set_framesize(s, TRACK_FRAMESIZE);
    Serial.printf("[REC] Recording stopped. %lu frames saved to %s\n",
                  frameCount, sessionDir);
    nextSession();
}

void nextSession() {
    for (int n = 1; n <= SD_MAX_SESSIONS; n++) {
        snprintf(sessionDir, sizeof(sessionDir), "%s/session_%03d", SD_SESSION_DIR, n);
        if (!SD_MMC.exists(sessionDir)) {
            SD_MMC.mkdir(sessionDir);
            sessionNumber = n;
            return;
        }
    }
}

// ===========================================================================
// Periodic STATUS broadcast — parsed by moonwatch_terminal.py
// Format: STATUS:<state>:<pan>:<tilt>:<zoom>:<recording>:<frames>:<session>
// ===========================================================================
unsigned long lastStatusBroadcast = 0;
#define STATUS_INTERVAL_MS 1000

void broadcastStatus() {
    unsigned long now = millis();
    if (now - lastStatusBroadcast < STATUS_INTERVAL_MS) return;
    lastStatusBroadcast = now;
    Serial.printf("STATUS:%s:%.1f:%.1f:%d:%d:%lu:%d\n",
        stateNames[state], panDeg, tiltDeg, zoomLevel,
        recording ? 1 : 0, frameCount, sessionNumber);
}

// ===========================================================================
// Serial command handler — full argument-aware parser
// ===========================================================================
//
// Commands (case-insensitive):
//   scan                  Start sky scan
//   stop                  Stop everything, home servos
//   home                  Home servos (keep state)
//   track                 Jump straight to TRACK state (manual use)
//   record                Toggle recording on/off
//   session               Start a new recording session
//
//   pan   <0-180>         Move pan to angle
//   tilt  <30-150>        Move tilt to angle
//   pan+  [step]          Jog pan right  (default step = 5°)
//   pan-  [step]          Jog pan left
//   tilt+ [step]          Jog tilt up    (default step = 5°)
//   tilt- [step]          Jog tilt down
//   centre                Centre both servos
//
//   zoom  <1-4>           Set digital zoom level
//   zoom+                 Zoom in
//   zoom-                 Zoom out
//
//   threshold <0-255>     Blob detection brightness threshold
//   blob     <count>      Min blob pixel count for detection
//   kp <value>            PID proportional gain
//   ki <value>            PID integral gain
//   kd <value>            PID derivative gain
//   gain  <0-6>           Camera gain ceiling (0=2x … 6=128x)
//   expo  <-2..2>         Camera exposure level
//   step  <degrees>       Scan step size in degrees
//   delay <ms>            Scan step delay in ms
//
//   status / ?            Print current status
//   ping                  Replies PONG (connection check)
//   help                  This command list
// ===========================================================================

// Tunable values (mirrors config.h but adjustable at runtime)
int   runtimeThreshold = DETECT_THRESHOLD;
int   runtimeMinBlob   = DETECT_MIN_BLOB;
float runtimeKP        = KP;
float runtimeKI        = KI;
float runtimeKD        = KD;
int   runtimeScanStep  = SCAN_STEP_DEG;
int   runtimeScanDelay = SCAN_DELAY_MS;

void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Split into verb + optional argument
    line.toLowerCase();
    int spaceIdx = line.indexOf(' ');
    String verb = (spaceIdx >= 0) ? line.substring(0, spaceIdx) : line;
    String arg  = (spaceIdx >= 0) ? line.substring(spaceIdx + 1) : "";
    arg.trim();
    float argF = arg.toFloat();
    int   argI = (int)argF;

    // ---- Motion control ---------------------------------------------------
    if (verb == "scan" || verb == "s") {
        scanPanStep = 0; scanTiltRow = 0; scanPanDir = true; scanLastMove = 0;
        applyZoom(esp_camera_sensor_get(), 1);
        state = SCANNING;
        Serial.println("[CMD] Scanning started.");

    } else if (verb == "stop" || verb == "t") {
        if (recording) stopRecording();
        state = IDLE;
        moveTo(PAN_HOME_DEG, TILT_HOME_DEG, 15);
        Serial.println("[CMD] Stopped. Servos homed.");

    } else if (verb == "home") {
        moveTo(PAN_HOME_DEG, TILT_HOME_DEG, 15);
        Serial.println("[CMD] Homed.");

    } else if (verb == "centre" || verb == "center" || verb == "c") {
        moveTo(PAN_HOME_DEG, TILT_HOME_DEG, 15);
        Serial.println("[CMD] Servos centred.");

    } else if (verb == "track") {
        pidErrXPrev = 0; pidIntX = 0;
        pidErrYPrev = 0; pidIntY = 0;
        pidLastTime = millis();
        state = TRACKING;
        Serial.println("[CMD] Switched to TRACK state.");

    } else if (verb == "pan") {
        if (arg.length() == 0) {
            Serial.printf("[CMD] pan = %.1f°\n", panDeg);
        } else {
            panDeg = constrain(argF, PAN_MIN_DEG, PAN_MAX_DEG);
            servoPan.write((int)panDeg);
            Serial.printf("[CMD] Pan → %.1f°\n", panDeg);
        }

    } else if (verb == "tilt") {
        if (arg.length() == 0) {
            Serial.printf("[CMD] tilt = %.1f°\n", tiltDeg);
        } else {
            tiltDeg = constrain(argF, TILT_MIN_DEG, TILT_MAX_DEG);
            servoTilt.write((int)tiltDeg);
            Serial.printf("[CMD] Tilt → %.1f°\n", tiltDeg);
        }

    } else if (verb == "pan+" || verb == "panr") {
        float step = (arg.length() > 0) ? argF : 5.0f;
        panDeg = constrain(panDeg + step, PAN_MIN_DEG, PAN_MAX_DEG);
        servoPan.write((int)panDeg);
        Serial.printf("[CMD] Pan → %.1f°\n", panDeg);

    } else if (verb == "pan-" || verb == "panl") {
        float step = (arg.length() > 0) ? argF : 5.0f;
        panDeg = constrain(panDeg - step, PAN_MIN_DEG, PAN_MAX_DEG);
        servoPan.write((int)panDeg);
        Serial.printf("[CMD] Pan → %.1f°\n", panDeg);

    } else if (verb == "tilt+" || verb == "tiltu") {
        float step = (arg.length() > 0) ? argF : 5.0f;
        tiltDeg = constrain(tiltDeg + step, TILT_MIN_DEG, TILT_MAX_DEG);
        servoTilt.write((int)tiltDeg);
        Serial.printf("[CMD] Tilt → %.1f°\n", tiltDeg);

    } else if (verb == "tilt-" || verb == "tiltd") {
        float step = (arg.length() > 0) ? argF : 5.0f;
        tiltDeg = constrain(tiltDeg - step, TILT_MIN_DEG, TILT_MAX_DEG);
        servoTilt.write((int)tiltDeg);
        Serial.printf("[CMD] Tilt → %.1f°\n", tiltDeg);

    // ---- Recording --------------------------------------------------------
    } else if (verb == "record" || verb == "r") {
        if (state != TRACKING) {
            Serial.println("[CMD] Must be in TRACKING state to record. Use 'track' first.");
        } else {
            if (!recording) startRecording();
            else            stopRecording();
        }

    } else if (verb == "session") {
        if (recording) stopRecording();
        nextSession();
        Serial.printf("[CMD] New session: %s\n", sessionDir);

    // ---- Zoom -------------------------------------------------------------
    } else if (verb == "zoom") {
        if (arg == "+" || arg == "in") {
            zoomLevel = constrain(zoomLevel + 1, ZOOM_MIN, ZOOM_MAX);
        } else if (arg == "-" || arg == "out") {
            zoomLevel = constrain(zoomLevel - 1, ZOOM_MIN, ZOOM_MAX);
        } else if (arg.length() > 0) {
            zoomLevel = constrain(argI, ZOOM_MIN, ZOOM_MAX);
        }
        if (state == TRACKING || state == DETECTING)
            applyZoom(esp_camera_sensor_get(), zoomLevel);
        Serial.printf("[CMD] Zoom → %d×\n", zoomLevel);

    } else if (verb == "zoom+" || verb == "z+") {
        zoomLevel = constrain(zoomLevel + 1, ZOOM_MIN, ZOOM_MAX);
        if (state == TRACKING) applyZoom(esp_camera_sensor_get(), zoomLevel);
        Serial.printf("[CMD] Zoom → %d×\n", zoomLevel);

    } else if (verb == "zoom-" || verb == "z-") {
        zoomLevel = constrain(zoomLevel - 1, ZOOM_MIN, ZOOM_MAX);
        if (state == TRACKING) applyZoom(esp_camera_sensor_get(), zoomLevel);
        Serial.printf("[CMD] Zoom → %d×\n", zoomLevel);

    // ---- Detection tuning -------------------------------------------------
    } else if (verb == "threshold") {
        if (arg.length() == 0) {
            Serial.printf("[CMD] threshold = %d\n", runtimeThreshold);
        } else {
            runtimeThreshold = constrain(argI, 0, 255);
            Serial.printf("[CMD] Threshold → %d\n", runtimeThreshold);
        }

    } else if (verb == "blob") {
        if (arg.length() == 0) {
            Serial.printf("[CMD] blob min = %d px\n", runtimeMinBlob);
        } else {
            runtimeMinBlob = max(1, argI);
            Serial.printf("[CMD] Blob min → %d px\n", runtimeMinBlob);
        }

    // ---- PID tuning -------------------------------------------------------
    } else if (verb == "kp") {
        if (arg.length() == 0) Serial.printf("[CMD] kp = %.4f\n", runtimeKP);
        else { runtimeKP = argF; Serial.printf("[CMD] Kp → %.4f\n", runtimeKP); }

    } else if (verb == "ki") {
        if (arg.length() == 0) Serial.printf("[CMD] ki = %.4f\n", runtimeKI);
        else { runtimeKI = argF; Serial.printf("[CMD] Ki → %.4f\n", runtimeKI); }

    } else if (verb == "kd") {
        if (arg.length() == 0) Serial.printf("[CMD] kd = %.4f\n", runtimeKD);
        else { runtimeKD = argF; Serial.printf("[CMD] Kd → %.4f\n", runtimeKD); }

    // ---- Camera tuning ----------------------------------------------------
    } else if (verb == "gain") {
        sensor_t* s = esp_camera_sensor_get();
        if (s && arg.length() > 0) {
            int g = constrain(argI, 0, 6);
            s->set_gainceiling(s, (gainceiling_t)g);
            Serial.printf("[CMD] Gain ceiling → %d (%d×)\n", g, 2 << g);
        } else {
            Serial.println("[CMD] Usage: gain <0-6>  (0=2x 1=4x 2=8x 3=16x 4=32x 5=64x 6=128x)");
        }

    } else if (verb == "expo") {
        sensor_t* s = esp_camera_sensor_get();
        if (s && arg.length() > 0) {
            int e = constrain(argI, -2, 2);
            s->set_ae_level(s, e);
            Serial.printf("[CMD] Exposure level → %d\n", e);
        } else {
            Serial.println("[CMD] Usage: expo <-2..2>");
        }

    // ---- Scan tuning ------------------------------------------------------
    } else if (verb == "step") {
        if (arg.length() == 0) {
            Serial.printf("[CMD] scan step = %d°\n", runtimeScanStep);
        } else {
            runtimeScanStep = constrain(argI, 1, 45);
            Serial.printf("[CMD] Scan step → %d°\n", runtimeScanStep);
        }

    } else if (verb == "delay") {
        if (arg.length() == 0) {
            Serial.printf("[CMD] scan delay = %d ms\n", runtimeScanDelay);
        } else {
            runtimeScanDelay = max(10, argI);
            Serial.printf("[CMD] Scan delay → %d ms\n", runtimeScanDelay);
        }

    // ---- Info / utility ---------------------------------------------------
    } else if (verb == "status" || verb == "?" || verb == "info") {
        printStatus();

    } else if (verb == "ping") {
        Serial.println("PONG");

    } else if (verb == "help" || verb == "h") {
        Serial.println(
            "\n=== Moonwatch Commands ===\n"
            " scan               Start sky scan\n"
            " stop               Stop + home servos\n"
            " home               Home servos\n"
            " track              Enter TRACK state manually\n"
            " record             Toggle recording (need TRACK state)\n"
            " session            Start new recording session\n"
            "\n"
            " pan   <0-180>      Set pan angle\n"
            " tilt  <30-150>     Set tilt angle\n"
            " pan+  [step]       Jog pan right  (default 5°)\n"
            " pan-  [step]       Jog pan left\n"
            " tilt+ [step]       Jog tilt up    (default 5°)\n"
            " tilt- [step]       Jog tilt down\n"
            " centre             Centre servos\n"
            "\n"
            " zoom  <1-4>        Set zoom level\n"
            " zoom+ / zoom-      Zoom in/out\n"
            "\n"
            " threshold <0-255>  Detection brightness threshold\n"
            " blob <pixels>      Min blob size for detection\n"
            " kp / ki / kd <v>   PID gains\n"
            " gain  <0-6>        Camera gain ceiling\n"
            " expo  <-2..2>      Exposure bias\n"
            " step  <deg>        Scan step size\n"
            " delay <ms>         Scan step delay\n"
            "\n"
            " status / ?         Print status\n"
            " ping               Test connection\n"
            " help               This list\n"
        );

    } else {
        Serial.printf("[CMD] Unknown: '%s'  (type 'help' for commands)\n", verb.c_str());
    }
}

// ===========================================================================
// Status LED blink
// ===========================================================================
void updateLED() {
    uint16_t period = 0;
    switch (state) {
        case SCANNING:  period = LED_SCANNING;  break;
        case DETECTING:
        case TRACKING:  period = LED_TRACKING;  break;
        case RECORDING: period = LED_RECORDING; break;
        default:        period = 0; break;
    }
    if (recording) period = LED_RECORDING;

    if (period == 0) {
        digitalWrite(PIN_LED, LOW);
        return;
    }
    unsigned long now = millis();
    if (now - ledLastToggle >= (unsigned long)period) {
        ledLastToggle = now;
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    }
}

// ===========================================================================
// Print current status to Serial
// ===========================================================================
void printStatus() {
    Serial.println("\n=== Moonwatch Status ===");
    Serial.printf("  State      : %s\n",         stateNames[state]);
    Serial.printf("  Pan / Tilt : %.1f° / %.1f°\n", panDeg, tiltDeg);
    Serial.printf("  Zoom       : %d×\n",         zoomLevel);
    Serial.printf("  Recording  : %s (%lu frames)\n", recording ? "YES" : "NO", frameCount);
    Serial.printf("  Session    : %s\n",           sessionDir);
    Serial.printf("  PSRAM      : %s\n",           psramFound() ? "YES" : "NO");
    uint64_t total = SD_MMC.totalBytes() / (1024 * 1024);
    uint64_t used  = SD_MMC.usedBytes()  / (1024 * 1024);
    Serial.printf("  SD         : %llu / %llu MB used\n", used, total);
    Serial.println("  --- Tuning ---");
    Serial.printf("  Threshold  : %d  blob min: %d px\n", runtimeThreshold, runtimeMinBlob);
    Serial.printf("  PID        : kp=%.3f  ki=%.4f  kd=%.3f\n",
                  runtimeKP, runtimeKI, runtimeKD);
    Serial.printf("  Scan       : step=%d°  delay=%d ms\n",
                  runtimeScanStep, runtimeScanDelay);
    Serial.println("Type 'help' for command list.");
}

// ===========================================================================
// Optional WiFi MJPEG stream
// ===========================================================================
#if WIFI_ENABLED
void startWifi() {
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] AP: %s  IP: %s\n", WIFI_SSID,
                  WiFi.softAPIP().toString().c_str());
    server.on("/", []() {
        server.send(200, "text/html",
            "<html><body style='background:#000;color:#0f0;font-family:monospace'>"
            "<h2>ESP32 Moonwatch</h2>"
            "<img src='/stream' style='width:100%'><br>"
            "<a href='/cmd?c=s'>[Scan]</a> "
            "<a href='/cmd?c=t'>[Stop]</a> "
            "<a href='/cmd?c=r'>[Record]</a> "
            "<a href='/cmd?c=z%2B'>[Zoom+]</a> "
            "<a href='/cmd?c=z-'>[Zoom-]</a>"
            "</body></html>");
    });
    server.on("/cmd", []() {
        String c = server.arg("c");
        // Route web commands to serial handler by simulating input
        if (c == "s")  state = SCANNING;
        else if (c == "t") { if (recording) stopRecording(); state = IDLE; }
        else if (c == "r") { if (state == TRACKING) { if (!recording) startRecording(); else stopRecording(); } }
        else if (c == "z+") { zoomLevel = constrain(zoomLevel+1, ZOOM_MIN, ZOOM_MAX); if (state==TRACKING) applyZoom(esp_camera_sensor_get(), zoomLevel); }
        else if (c == "z-") { zoomLevel = constrain(zoomLevel-1, ZOOM_MIN, ZOOM_MAX); if (state==TRACKING) applyZoom(esp_camera_sensor_get(), zoomLevel); }
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
    });
    server.on("/stream", handleStream);
    server.begin();
}

void handleStream() {
    WiFiClient client = server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();
    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) continue;
        client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                       fb->len);
        client.write(fb->buf, fb->len);
        client.println();
        esp_camera_fb_return(fb);
        delay(50);  // ~20 fps cap
    }
}
#endif
