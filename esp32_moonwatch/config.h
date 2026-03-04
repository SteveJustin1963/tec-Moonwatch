#pragma once

// =============================================================================
// ESP32 Moonwatch — Configuration
// Edit this file to match your hardware wiring and preferences.
// =============================================================================

// -----------------------------------------------------------------------------
// Servo GPIO pins (ESP32-CAM AI Thinker, 1-bit SD mode)
// -----------------------------------------------------------------------------
#define PIN_SERVO_PAN   12
#define PIN_SERVO_TILT  13

// Servo angle limits (degrees)
#define PAN_MIN_DEG     0
#define PAN_MAX_DEG     180
#define TILT_MIN_DEG    30    // Don't tilt below horizon
#define TILT_MAX_DEG    150   // Don't tilt past vertical

// Home position (point at sky centre)
#define PAN_HOME_DEG    90
#define TILT_HOME_DEG   80

// -----------------------------------------------------------------------------
// Scan pattern
// -----------------------------------------------------------------------------
#define SCAN_STEP_DEG   10    // Degrees per step (smaller = finer, slower)
#define SCAN_DELAY_MS   80    // ms to wait at each step before capturing frame
#define SCAN_ROWS       10    // Number of tilt rows in the raster scan
#define SCAN_SPEED_MS   15    // Servo move interval during scan (lower = faster)

// -----------------------------------------------------------------------------
// Detection settings
// -----------------------------------------------------------------------------
#define DETECT_THRESHOLD   180   // Pixel brightness (0–255) to count as "bright"
#define DETECT_MIN_BLOB    20    // Min bright pixels to declare target found
#define DETECT_HYSTERESIS  5     // Pixel dead-band — don't move if error < this

// -----------------------------------------------------------------------------
// PID tracking gains
// -----------------------------------------------------------------------------
#define KP  0.15f
#define KI  0.001f
#define KD  0.05f

// Maximum PID output clamped to this many degrees per loop
#define PID_MAX_DEG  8.0f

// -----------------------------------------------------------------------------
// Digital zoom
// -----------------------------------------------------------------------------
#define ZOOM_MIN        1       // 1 = no zoom (full frame)
#define ZOOM_MAX        4       // 4 = 4× zoom (crops to 25% of frame area)
#define ZOOM_DEFAULT    1

// -----------------------------------------------------------------------------
// Recording
// -----------------------------------------------------------------------------
#define RECORD_FPS          10        // Target frames per second to SD card
#define RECORD_FRAME_MS     (1000 / RECORD_FPS)
#define SD_SESSION_DIR      "/moonwatch"
#define SD_MAX_SESSIONS     999

// -----------------------------------------------------------------------------
// Camera model — uncomment ONE
// -----------------------------------------------------------------------------
#define CAMERA_MODEL_AI_THINKER    // ESP32-CAM (AI Thinker) — most common
// #define CAMERA_MODEL_WROVER_KIT
// #define CAMERA_MODEL_ESP_EYE

// Camera resolution for tracking (lower = faster processing)
// FRAMESIZE_QVGA = 320×240, FRAMESIZE_VGA = 640×480, FRAMESIZE_SVGA = 800×600
#define TRACK_FRAMESIZE     FRAMESIZE_QVGA
#define RECORD_FRAMESIZE    FRAMESIZE_VGA   // Higher res for saved frames

// JPEG quality 0–63 (lower number = higher quality, larger file)
#define RECORD_JPEG_QUALITY  10

// -----------------------------------------------------------------------------
// WiFi live stream (optional — set to 1 to enable)
// -----------------------------------------------------------------------------
#define WIFI_ENABLED    0
#define WIFI_SSID       "Moonwatch"
#define WIFI_PASSWORD   "ufo12345"
// Stream port
#define STREAM_PORT     80

// -----------------------------------------------------------------------------
// Status LED
// -----------------------------------------------------------------------------
#define PIN_LED         4     // Onboard flash LED on AI Thinker
// LED blink patterns: 0=off, 1=on, >1=blink period in ms
#define LED_SCANNING    500
#define LED_TRACKING    100
#define LED_RECORDING   50

// -----------------------------------------------------------------------------
// Serial baud rate
// -----------------------------------------------------------------------------
#define SERIAL_BAUD     115200

// -----------------------------------------------------------------------------
// Camera pin map — do not edit unless changing board
// -----------------------------------------------------------------------------
#if defined(CAMERA_MODEL_AI_THINKER)
  #define CAM_PIN_PWDN    32
  #define CAM_PIN_RESET   -1
  #define CAM_PIN_XCLK     0
  #define CAM_PIN_SIOD    26
  #define CAM_PIN_SIOC    27
  #define CAM_PIN_D7      35
  #define CAM_PIN_D6      34
  #define CAM_PIN_D5      39
  #define CAM_PIN_D4      36
  #define CAM_PIN_D3      21
  #define CAM_PIN_D2      19
  #define CAM_PIN_D1      18
  #define CAM_PIN_D0       5
  #define CAM_PIN_VSYNC   25
  #define CAM_PIN_HREF    23
  #define CAM_PIN_PCLK    22
  // SD card (1-bit mode — frees GPIO 12,13 for servos)
  #define SD_PIN_CLK      14
  #define SD_PIN_CMD      15
  #define SD_PIN_D0        2

#elif defined(CAMERA_MODEL_WROVER_KIT)
  #define CAM_PIN_PWDN    -1
  #define CAM_PIN_RESET   -1
  #define CAM_PIN_XCLK    21
  #define CAM_PIN_SIOD    26
  #define CAM_PIN_SIOC    27
  #define CAM_PIN_D7      35
  #define CAM_PIN_D6      34
  #define CAM_PIN_D5      39
  #define CAM_PIN_D4      36
  #define CAM_PIN_D3      19
  #define CAM_PIN_D2      18
  #define CAM_PIN_D1       5
  #define CAM_PIN_D0       4
  #define CAM_PIN_VSYNC   25
  #define CAM_PIN_HREF    23
  #define CAM_PIN_PCLK    22
#endif
