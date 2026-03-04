# ESP32 Moonwatch — Sky Tracker Build Guide

Track, zoom, and record night sky objects (planes, satellites, drones, UAPs) using
an ESP32-CAM and two servo motors.

---

## Overview

```
[ Night Sky ]
      │
  [OV2640 Camera] ─── Detects bright blob
      │
  [ESP32-CAM]  ─── State machine: SCAN → DETECT → TRACK → RECORD
      │                  │
  [Pan Servo]        [SD Card]  ← saves JPEG frames
  [Tilt Servo]           │
                     [PC]  ← stitch into video with ffmpeg
```

**States:**
1. **SCAN** — fast raster sweep of sky, analyse each frame for a bright blob
2. **DETECT** — blob found, centre servos on it
3. **TRACK** — PID loop keeps target centred; digital zoom applied
4. **RECORD** — JPEG frames saved to SD card at up to 10 fps

---

## Hardware

### Core (required)

| Part | Notes | Cost (AUD est.) |
|------|-------|-----------------|
| ESP32-CAM (AI Thinker) | OV2640 camera + SD slot built in | $8–12 |
| FTDI USB-UART adapter (3.3 V) | For flashing | $5 |
| 2× MG90S metal-gear micro servo | Pan + tilt; ~2 kg·cm each | $6 |
| Pan-tilt servo bracket kit | 3D-printed or aluminium | $5–10 |
| MicroSD card (8 GB+) FAT32 | For recordings | $5 |
| 5 V / 2 A USB power supply | Servos + ESP32 | $6 |
| Breadboard + jumper wires | Prototyping | $3 |

### Optional upgrades

| Part | Why | Cost |
|------|-----|------|
| OV5640 camera module (5 MP, autofocus) | 5× more resolution → more digital zoom | $8 |
| PCA9685 I2C servo driver | Frees up GPIO; smoother PWM | $5 |
| BPW34 photodiode × 4 | Faster initial sky scan than camera | $4 |
| 50 mm f/1.8 C-mount lens + CS adapter | Optical zoom instead of digital | $15–40 |
| ESP32-S3 DevKit + external camera | More RAM, faster, more GPIO | $15 |

---

## Wiring

### ESP32-CAM pin constraints

The AI Thinker ESP32-CAM shares most GPIO with the camera and SD card.
Use **1-bit SD mode** (set in `config.h`) to free GPIO 12 and 13 for servos.

```
ESP32-CAM          Device
─────────────────────────────────
GPIO 12    ──────► Pan servo  (signal)
GPIO 13    ──────► Tilt servo (signal)
GND        ──────► Servo GND
5 V (ext)  ──────► Servo VCC   ← do NOT power servos from ESP32 3.3V pin

GPIO 14    SD CLK   (built-in)
GPIO 15    SD CMD   (built-in)
GPIO 2     SD D0    (built-in, 1-bit mode only)
GPIO 4     Onboard flash LED (used as status indicator)

GPIO 0     BOOT — pull LOW to flash, float to run
GPIO 1/3   UART TX/RX (for serial debug)
```

> **Power tip**: Servos draw 200–600 mA each under load. Power them from the USB
> 5 V rail directly (not the 3.3 V regulator). Share only GND with the ESP32.

### Schematic (ASCII)

```
USB 5 V ───┬──────────────────── Servo VCC (both servos)
           │
           └── ESP32-CAM 5V pin

GND ───────┬── ESP32-CAM GND
           └── Servo GND (both servos)

GPIO 12 ───── Pan servo signal
GPIO 13 ───── Tilt servo signal

[MicroSD card]  ── inserted in ESP32-CAM slot
[OV2640 camera] ── built-in ribbon cable
```

---

## Software Setup

### 1. Install Arduino IDE + ESP32 board

```bash
# In Arduino IDE → Preferences → Additional Boards Manager URLs:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Then: Tools → Board Manager → search "esp32" → install Espressif Systems esp32
```

### 2. Install libraries

In Arduino IDE → Library Manager, install:

| Library | Version | Purpose |
|---------|---------|---------|
| ESP32Servo | latest | Servo PWM control |
| esp32-camera | (bundled with ESP32 board package) | Camera driver |

The SD and EEPROM libraries are built into the ESP32 board package.

### 3. Open the sketch

```
File → Open → esp32_moonwatch/esp32_moonwatch.ino
```

Edit `config.h` to match your wiring and preferences.

### 4. Flash the ESP32-CAM

1. Connect FTDI adapter:
   ```
   FTDI GND  → ESP32-CAM GND
   FTDI VCC  → ESP32-CAM 5V
   FTDI TX   → ESP32-CAM GPIO 3 (RX)
   FTDI RX   → ESP32-CAM GPIO 1 (TX)
   ```
2. Pull GPIO 0 LOW (hold BOOT button or wire GPIO 0 → GND)
3. Press Reset, then upload from Arduino IDE
4. After upload: release GPIO 0, press Reset

### 5. Board settings

```
Board:         AI Thinker ESP32-CAM
Upload Speed:  115200
Partition:     Huge APP (3MB No OTA/1MB SPIFFS)
PSRAM:         Enabled
```

---

## Laptop Terminal Control

The recommended way to control Moonwatch is with the included Python terminal,
which gives you colour-coded output, command history, tab completion, and a live
status bar.

### Install and run

```bash
pip install pyserial          # only external dependency

python3 moonwatch_terminal.py            # auto-detects ESP32 port
python3 moonwatch_terminal.py /dev/ttyUSB0   # Linux explicit port
python3 moonwatch_terminal.py COM3           # Windows explicit port
```

> The terminal auto-detects CP210x, CH340, and FTDI USB-serial chips.
> If it can't find the port, run with `ports` as argument to list all available ports.

### Terminal features

| Feature | Detail |
|---------|--------|
| Live status bar | State, pan/tilt, zoom, recording, SD usage — refreshes every 2 s |
| Colour-coded output | Cyan=camera, green=tracking, red=recording, yellow=SD, dim=system |
| Command history | Up/Down arrows; saved to `~/.moonwatch_history` between sessions |
| Tab completion | Press Tab to complete any command |
| Connection indicator | Shows CONNECTED / DISCONNECTED in status bar |

### STATUS protocol

The ESP32 broadcasts a status line every second, parsed silently by the terminal:

```
STATUS:<state>:<pan>:<tilt>:<zoom>:<recording>:<frames>:<session>
```

You can also read this directly if connecting via another tool (e.g. `screen`, `minicom`).

---

## Operation

### Full command reference (115200 baud)

Type `help` on the ESP32 or in the terminal at any time to see this list.

#### Motion

| Command | Action |
|---------|--------|
| `scan` | Start boustrophedon sky scan |
| `stop` | Stop everything and home servos |
| `home` | Home servos, keep current state |
| `track` | Enter TRACK state manually (for manual pointing) |
| `centre` | Move both servos to home position |

#### Servo positioning

| Command | Action |
|---------|--------|
| `pan <0–180>` | Move pan to absolute angle |
| `tilt <30–150>` | Move tilt to absolute angle |
| `pan+` / `pan+ <step>` | Jog pan right (default 5°) |
| `pan-` / `pan- <step>` | Jog pan left |
| `tilt+` / `tilt+ <step>` | Jog tilt up (default 5°) |
| `tilt-` / `tilt- <step>` | Jog tilt down |
| `pan` | Print current pan angle |
| `tilt` | Print current tilt angle |

#### Recording

| Command | Action |
|---------|--------|
| `record` | Toggle recording on/off (must be in TRACK state) |
| `session` | End current session, start new one |

#### Zoom

| Command | Action |
|---------|--------|
| `zoom <1–4>` | Set digital zoom level directly |
| `zoom+` | Zoom in one step |
| `zoom-` | Zoom out one step |

#### Detection tuning (live, no reflash needed)

| Command | Default | Action |
|---------|---------|--------|
| `threshold <0–255>` | 180 | Blob brightness threshold |
| `blob <pixels>` | 20 | Minimum blob size to declare target found |
| `step <deg>` | 10 | Scan step size in degrees |
| `delay <ms>` | 80 | Pause per scan step |

#### PID tuning (live)

| Command | Default | Action |
|---------|---------|--------|
| `kp <value>` | 0.15 | Proportional gain |
| `ki <value>` | 0.001 | Integral gain |
| `kd <value>` | 0.05 | Derivative gain |

#### Camera tuning (live)

| Command | Action |
|---------|--------|
| `gain <0–6>` | Gain ceiling: 0=2× 1=4× 2=8× 3=16× 4=32× 5=64× 6=128× |
| `expo <-2..2>` | Exposure bias (positive = brighter) |

#### Utility

| Command | Action |
|---------|--------|
| `status` / `?` | Print full status and tuning values |
| `ping` | Replies `PONG` — use to test connection |
| `help` | Print command list on ESP32 Serial |

### Using plain serial terminals (no Python)

Any serial terminal at 115200 baud works — `screen`, `minicom`, Arduino Serial Monitor,
PuTTY, etc. You will see the raw `STATUS:` lines scroll past; just ignore them or
filter with `grep -v STATUS`.

```bash
# Linux / Mac
screen /dev/ttyUSB0 115200
minicom -D /dev/ttyUSB0 -b 115200

# Exit screen: Ctrl+A then K
```

### WiFi live preview (optional)

If `WIFI_ENABLED` is set to `1` in `config.h`, the ESP32 creates a hotspot:

- SSID: `Moonwatch`
- Password: `ufo12345`
- Open browser: `http://192.168.4.1` — live MJPEG stream + control buttons

### Recording output

Frames are saved to SD card as:

```
/moonwatch/
  session_001/
    frame_00001.jpg
    frame_00002.jpg
    ...
```

Convert to video on your PC:

```bash
# Linux / Mac
ffmpeg -framerate 10 -i frame_%05d.jpg -c:v libx264 -crf 18 ufo_hunt.mp4

# Windows (same command, install ffmpeg from ffmpeg.org)
```

---

## Tuning Guide

All tuning values can be changed **live via serial** without reflashing.
`config.h` sets the power-on defaults; serial commands override them at runtime.

### Scan speed

```
step <deg>     Smaller = finer grid, slower sweep.   Default: 10°
delay <ms>     Wait per step before capturing frame. Default: 80 ms
```

For a quick first-pass scan use `step 15 delay 50`. Once you know where
targets appear in your sky, tighten to `step 5 delay 100` for that zone.

### Detection threshold

```
threshold <0–255>   Brightness a pixel must exceed to count as "bright". Default: 180
blob <pixels>       Min bright-pixel count to call it a real target.      Default: 20
```

- **Moonless dark sky**: try `threshold 150 blob 10`
- **Near city lights / moon up**: try `threshold 210 blob 30`
- **Only want aircraft strobes**: `threshold 230 blob 5`

### PID tuning

```
kp <value>   Proportional — increase if tracking is sluggish.           Default: 0.15
ki <value>   Integral     — increase if target drifts off-centre.       Default: 0.001
kd <value>   Derivative   — increase if servos oscillate/hunt.          Default: 0.05
```

Tuning workflow:
1. Set `ki 0` and `kd 0` first. Raise `kp` until servos track but just start to oscillate.
2. Back `kp` off 20%, then raise `kd` to damp the oscillation.
3. Finally add a small `ki` to eliminate steady-state drift.

### Zoom

Digital zoom crops the sensor window to the centre fraction of the frame.

```
zoom 1   Full frame  (no zoom)
zoom 2   2× zoom     (50% crop)
zoom 3   3× zoom     (33% crop)
zoom 4   4× zoom     (25% crop) — maximum recommended for OV2640
```

Maximum useful zoom before pixelation:
- OV2640 (2 MP): **3×**
- OV5640 (5 MP): **6×** (set `ZOOM_MAX 6` in config.h)

For optical zoom, attach a motorised C-mount lens and add a zoom servo on GPIO 14
(requires disabling SD 4-bit mode — see config comments).

### Camera sensitivity

```
gain <0–6>    Gain ceiling: 0=2× … 6=128×.  Default: 6 (128× for night sky)
expo <-2..2>  Exposure bias.                 Default: 2 (biased bright for faint objects)
```

If the camera is washing out bright stars, reduce `expo 0` or `gain 4`.

---

## Upgrade Path

```
Weekend build   → ESP32-CAM + LDRs + basic servos (~$25)
Improved        → OV5640 + PCA9685 servo driver + photodiodes (~$45)
Serious tracker → ESP32-S3 + 5 MP + optical zoom lens + NEMA17 steppers (~$120)
Observatory     → Add Raspberry Pi co-processor for real OpenCV tracking
```

---

## Safety Notes

- **Lasers**: If you add a laser pointer to the rig, keep it under 1 mW (Class 1/2).
  Never aim at aircraft or people. Check local regulations.
- **Servos**: Secure the pan-tilt rig to a tripod — a falling servo rig can damage equipment.
- **Waterproofing**: Electronics are not weatherproof. Use a project box outdoors.
