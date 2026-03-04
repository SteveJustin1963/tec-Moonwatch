#!/usr/bin/env python3
"""
moonwatch_terminal.py — Laptop serial terminal for ESP32 Moonwatch

Usage:
    python3 moonwatch_terminal.py                 # auto-detect port
    python3 moonwatch_terminal.py /dev/ttyUSB0    # explicit port
    python3 moonwatch_terminal.py COM3            # Windows

Requires:  pip install pyserial
Optional:  pip install colorama   (colour output on Windows)

Keys:
    Up/Down arrows  Command history
    Tab             Tab completion
    Ctrl+C / q      Quit
"""

import sys
import os
import threading
import time
import re
import glob as _glob
import readline  # command history + tab completion (Linux/Mac built-in)

# Serial
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)

# Colour — works without colorama too (ANSI escape codes)
try:
    import colorama
    colorama.init()
    HAS_COLOR = True
except ImportError:
    HAS_COLOR = True  # ANSI codes work natively on Linux/Mac terminals

BAUD = 115200

# ---------------------------------------------------------------------------
# ANSI colours
# ---------------------------------------------------------------------------
class C:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN    = "\033[96m"
    WHITE   = "\033[97m"
    DIM     = "\033[2m"

def col(text, colour):
    return f"{colour}{text}{C.RESET}"

# ---------------------------------------------------------------------------
# Global state (updated by reader thread from STATUS: lines)
# ---------------------------------------------------------------------------
class TrackerState:
    def __init__(self):
        self.state     = "?"
        self.pan       = 0.0
        self.tilt      = 0.0
        self.zoom      = 1
        self.recording = False
        self.frames    = 0
        self.session   = 0
        self.lock      = threading.Lock()
        self.connected = False

status = TrackerState()

# ---------------------------------------------------------------------------
# Status bar (drawn above prompt)
# ---------------------------------------------------------------------------
STATUS_LINE_ROWS = 2   # number of lines the status header occupies

def state_colour(s):
    return {
        "IDLE":      C.DIM,
        "SCANNING":  C.CYAN,
        "DETECTING": C.YELLOW,
        "TRACKING":  C.GREEN,
        "RECORDING": C.RED,
    }.get(s, C.WHITE)

def draw_status():
    with status.lock:
        s         = status.state
        pan       = status.pan
        tilt      = status.tilt
        zoom      = status.zoom
        recording = status.recording
        frames    = status.frames
        session   = status.session
        connected = status.connected

    sc = state_colour(s)
    rec_str = col(f" ● REC {frames}fr", C.RED + C.BOLD) if recording else col("  ○ idle", C.DIM)
    conn_str = col("CONNECTED", C.GREEN) if connected else col("DISCONNECTED", C.RED)

    line1 = (
        f"{C.BOLD}[ Moonwatch ]{C.RESET}  "
        f"{conn_str}  │  "
        f"State: {sc}{C.BOLD}{s:<10}{C.RESET}  │  "
        f"Pan:{col(f'{pan:>6.1f}°', C.CYAN)}  "
        f"Tilt:{col(f'{tilt:>6.1f}°', C.CYAN)}"
    )
    line2 = (
        f"Zoom:{col(f'{zoom}×', C.MAGENTA)}  │  "
        f"Session:{col(str(session), C.YELLOW)}  "
        f"{rec_str}  │  "
        f"{col('help', C.DIM)} for commands"
    )

    # Move to top of terminal, print status, move back
    # Using a simple approach: print above prompt area
    sys.stdout.write(f"\r{line1}\n{line2}\n")
    sys.stdout.flush()

# ---------------------------------------------------------------------------
# Colour-code incoming lines from ESP32
# ---------------------------------------------------------------------------
def colour_line(line: str) -> str:
    line = line.rstrip()
    if line.startswith("STATUS:"):
        return None  # handled silently (parsed below)
    if line.startswith("PONG"):
        return col("  ↩ PONG", C.GREEN)
    if "[CAM]" in line:
        return col(line, C.CYAN)
    if "[TRACK]" in line or "[DETECT]" in line:
        return col(line, C.GREEN)
    if "[SCAN]" in line:
        return col(line, C.BLUE)
    if "[REC]" in line:
        return col(line, C.RED)
    if "[SD]" in line:
        return col(line, C.YELLOW)
    if "[CMD]" in line:
        return col(line, C.MAGENTA)
    if "[SYS]" in line or "[SERVO]" in line:
        return col(line, C.DIM)
    if "FATAL" in line or "error" in line.lower():
        return col(line, C.RED + C.BOLD)
    if "Warning" in line or "warn" in line.lower():
        return col(line, C.YELLOW)
    if line.startswith("==="):
        return col(line, C.BOLD + C.WHITE)
    if line.startswith("  ---"):
        return col(line, C.DIM)
    return line

def parse_status(line: str):
    """Parse STATUS:<state>:<pan>:<tilt>:<zoom>:<recording>:<frames>:<session>"""
    parts = line.split(":")
    if len(parts) < 8:
        return
    with status.lock:
        status.state     = parts[1]
        status.pan       = float(parts[2])
        status.tilt      = float(parts[3])
        status.zoom      = int(parts[4])
        status.recording = parts[5] == "1"
        status.frames    = int(parts[6])
        status.session   = int(parts[7])

# ---------------------------------------------------------------------------
# Reader thread — reads ESP32 output continuously
# ---------------------------------------------------------------------------
def reader_thread(ser: serial.Serial):
    with status.lock:
        status.connected = True
    try:
        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip()
                if line.startswith("STATUS:"):
                    parse_status(line)
                    continue
                coloured = colour_line(line)
                if coloured is not None:
                    # Print below the current prompt line
                    sys.stdout.write(f"\r{coloured}\n")
                    # Reprint prompt
                    sys.stdout.write(f"{C.BOLD}{C.GREEN}moonwatch>{C.RESET} ")
                    sys.stdout.flush()
            except (UnicodeDecodeError, serial.SerialException):
                break
    except Exception:
        pass
    with status.lock:
        status.connected = False
    sys.stdout.write(f"\n{col('[SERIAL] Connection lost.', C.RED)}\n")
    sys.stdout.flush()

# ---------------------------------------------------------------------------
# Tab completion
# ---------------------------------------------------------------------------
COMMANDS = [
    "scan", "stop", "home", "track", "record", "session", "centre", "center",
    "pan", "tilt", "pan+", "pan-", "tilt+", "tilt-",
    "zoom", "zoom+", "zoom-",
    "threshold", "blob", "kp", "ki", "kd",
    "gain", "expo", "step", "delay",
    "status", "ping", "help", "quit", "exit",
]

def completer(text, state):
    options = [c for c in COMMANDS if c.startswith(text)]
    return options[state] if state < len(options) else None

readline.set_completer(completer)
readline.parse_and_bind("tab: complete")
readline.set_completer_delims(" \t")

# ---------------------------------------------------------------------------
# History
# ---------------------------------------------------------------------------
HISTORY_FILE = os.path.expanduser("~/.moonwatch_history")
try:
    readline.read_history_file(HISTORY_FILE)
except FileNotFoundError:
    pass
readline.set_history_length(200)

def save_history():
    try:
        readline.write_history_file(HISTORY_FILE)
    except Exception:
        pass

# ---------------------------------------------------------------------------
# Port auto-detection
# ---------------------------------------------------------------------------
def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    # Prefer known ESP32 USB chips
    preferred_ids = [
        "10c4:ea60",   # CP210x (very common on ESP32-CAM)
        "1a86:7523",   # CH340 (cheap clones)
        "0403:6001",   # FTDI FT232
        "0403:6015",   # FTDI FT231
    ]
    for port in ports:
        vid_pid = f"{port.vid:04x}:{port.pid:04x}" if port.vid else ""
        if vid_pid in preferred_ids:
            return port.device
    # Fallback: first USB serial
    for port in ports:
        if "USB" in port.device.upper() or "ACM" in port.device.upper():
            return port.device
    # Windows COM ports
    for port in ports:
        if port.device.startswith("COM"):
            return port.device
    return None

def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print(col("No serial ports found.", C.RED))
        return
    print(col("Available serial ports:", C.CYAN))
    for p in ports:
        print(f"  {col(p.device, C.WHITE)}  {col(p.description, C.DIM)}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    # Determine port
    if len(sys.argv) > 1 and sys.argv[1] not in ("-h", "--help"):
        port = sys.argv[1]
    else:
        port = find_port()

    if port is None:
        print(col("ERROR: No ESP32 serial port found.", C.RED))
        list_ports()
        print(f"\nUsage: {col('python3 moonwatch_terminal.py /dev/ttyUSB0', C.CYAN)}")
        sys.exit(1)

    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD

    print(f"\n{col('ESP32 Moonwatch Terminal', C.BOLD + C.WHITE)}")
    print(f"Connecting to {col(port, C.CYAN)} @ {col(str(baud), C.YELLOW)} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(1.5)   # let ESP32 boot / reset
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(col(f"ERROR: {e}", C.RED))
        list_ports()
        sys.exit(1)

    print(col(f"Connected. Type 'help' for commands. Ctrl+C or 'quit' to exit.\n", C.GREEN))

    # Start reader thread
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    # Send ping to get initial status
    time.sleep(0.3)
    ser.write(b"ping\n")
    time.sleep(0.3)
    ser.write(b"status\n")

    PROMPT = f"{C.BOLD}{C.GREEN}moonwatch>{C.RESET} "

    # Draw initial status header
    draw_status()

    # Status refresh thread
    def status_refresh():
        while True:
            time.sleep(2)
            draw_status()
    st = threading.Thread(target=status_refresh, daemon=True)
    st.start()

    # Main input loop
    while True:
        try:
            cmd = input(PROMPT).strip()
        except (EOFError, KeyboardInterrupt):
            print(f"\n{col('Bye!', C.YELLOW)}")
            break

        if not cmd:
            continue

        if cmd.lower() in ("quit", "exit", "q"):
            print(col("Bye!", C.YELLOW))
            break

        # Local-only commands
        if cmd.lower() == "ports":
            list_ports()
            continue

        if cmd.lower() in ("cls", "clear"):
            os.system("clear" if os.name != "nt" else "cls")
            draw_status()
            continue

        # Send to ESP32
        try:
            ser.write((cmd + "\n").encode("utf-8"))
        except serial.SerialException as e:
            print(col(f"[SERIAL] Write error: {e}", C.RED))
            break

    save_history()
    try:
        ser.close()
    except Exception:
        pass

if __name__ == "__main__":
    main()
