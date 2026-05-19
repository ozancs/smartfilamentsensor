# Smart Filament Sensor

> **⚠️ WARNING: KLIPPER INTEGRATION IS NOT FULLY TESTED YET. YOU MAY ENCOUNTER BUGS OR UNEXPECTED BEHAVIOR.**

High-precision filament motion sensor for 3D printers. Uses an **ESP32-C3** and **AS5600** magnetic encoder to track filament movement at sub-millimeter accuracy. Detects clogs, runouts, and slippage in real-time with native **Klipper** integration.

> **Build video coming soon** — in the meantime, follow the **[Installation Guide](INSTALLATION_GUIDE.md)** and use the [interactive CAD model](https://a360.co/4fwGLLL) for reference.

---

## Features

- **Sub-mm accuracy** — AS5600 contactless encoder, 4096 steps/revolution, median-filtered
- **Native Klipper module** — klippy extra that compares commanded extrusion vs. actual encoder reading
- **Automatic clog detection** — configurable detection window and tolerance, pauses print on mismatch
- **One-command calibration** — `SFS_CALIBRATE LENGTH=50`, extrude, done. Saved to flash permanently
- **Status LED** — WS2812B with smooth breathing/pulse animations (idle, moving, calibrating)
- **Windows console app** — standalone .exe for calibration, live measurement, settings, and firmware flashing
- **No drift** — absolute encoder, no step counting errors over time

---

## Bill of Materials

| Component | Qty | Notes |
|---|---|---|
| ESP32-C3 Super Mini | 1 | Main MCU |
| AS5600 Encoder Module | 1 | With diametrical magnet |
| WS2812B NeoPixel 5050 | 1 | Status LED (single round PCB) |
| Grooved Bearing (U604ZZ) | 2 | OD 13mm, ID 4mm |
| Dowel Pin 3mm x 15mm | 1 | Bearing axle |
| O-Ring (~11mm OD) | 1 | Filament grip on bearing |
| PC4-M6 Fitting | 2 | Bowden tube entry/exit |
| Compression Spring | 1 | Pen-style, cut to length |
| M3 Screw (M3x8–M3x10) | 1 | |
| M2 Self-Tapping Screw | 2 | AS5600 mount + ESP32 mount |

Full BOM with images: [`3d_print_files_and_bom/bom.html`](3d_print_files_and_bom/bom.html)

---

## 3D Printed Parts

All files are in [`3d_print_files_and_bom/`](3d_print_files_and_bom/) as print-ready 3MF:

| Part | File |
|---|---|
| Sensor Body (with text) | `SmartFilament_Body_w_Text.3mf` |
| Sensor Body (no text) | `SmartFilament_Body_w-out_Text.3mf` |
| Back Cover | `BackCover.3mf` |
| Magnet Holder | `MagnetHolder.3mf` |
| Spring Arm | `Spring_Arm.3mf` |

**[Interactive CAD Model (Fusion 360)](https://a360.co/4fwGLLL)** — rotate, inspect, and measure all parts online.

---

## Firmware

### Quick Flash (Windows)

1. Download the console app `.exe` from [Releases](../../releases)
2. Connect the ESP32 via USB
3. Open the app → **Firmware** tab → **Start Firmware Flash**

### Build from Source (Arduino IDE)

1. Install board: **ESP32-C3** (Arduino ESP32 core)
2. Install libraries: `AS5600`, `FastLED`
3. Open `smart_filament_sensor.ino`, select board **ESP32C3 Dev Module**
4. In **Tools** menu, set **USB CDC On Boot: Enabled**
5. Upload

---

## Klipper Integration

The sensor works as a native Klipper module. ESP32 only measures — all clog detection logic runs inside Klipper.

### Setup

```bash
# 1. Copy the klippy module
cp klipper/smart_filament_sensor.py ~/klipper/klippy/extras/

# 2. Restart Klipper
sudo systemctl restart klipper
```

### printer.cfg

```ini
[smart_filament_sensor my_sensor]
serial: /dev/ttyUSB0
baud: 115200
detection_length: 7.0   # mm of extrusion between each check
tolerance: 2.0           # max allowed deviation (mm) before clog
pause_on_clog: True
clog_gcode: PAUSE
```

### GCode Commands

| Command | Description |
|---|---|
| `SFS_STATUS` | Show sensor state |
| `SFS_ENABLE` / `SFS_DISABLE` | Toggle clog detection |
| `SFS_RESET` | Re-sync extruder position and odometer |
| `SFS_CALIBRATE [LENGTH=50]` | Start calibration |
| `SFS_CALIBRATE_APPLY` | Save calibration immediately |
| `SFS_CALIBRATE_STOP` | Cancel calibration |
| `SFS_SET BRIGHT=80` | Change LED brightness |
| `SFS_SET SENS=5 NOISE=3` | Change sensitivity and noise filter |
| `SFS_SET DIR=-1` | Reverse encoder direction |

### Calibration via Klipper

```gcode
SFS_CALIBRATE LENGTH=50    ; start calibration
G1 E50 F100                ; extrude 50mm slowly
SFS_CALIBRATE_APPLY        ; save immediately (or wait 5s for auto-save)
```

Full guide: [`klipper/KLIPPER_GUIDE.txt`](klipper/KLIPPER_GUIDE.txt)

---

## Console App (Windows)

Standalone desktop application for calibration, live measurement, settings management, and firmware flashing.

Download the portable `.exe` from [Releases](../../releases) — no installation required.

### Features
- **Dashboard** — calibration wizard, live mm measurement, all sensor settings
- **Serial Monitor** — raw terminal with quick-action buttons
- **Firmware** — one-click ESP32 flash with bundled esptool
- **Klipper Guide** — printer.cfg config, full Python module code, PAUSE macros

---

## How It Works

```
Klipper: "I extruded 7.0mm"  →  GET_MM_RESET  →  ESP32
ESP32:   "Encoder saw 6.8mm" ←  MM:6.8000     ←  ESP32

Deviation = |7.0 - 6.8| = 0.2mm < 2.0mm tolerance → OK

Klipper: "I extruded 7.0mm"  →  GET_MM_RESET  →  ESP32
ESP32:   "Encoder saw 1.2mm" ←  MM:1.2000     ←  ESP32

Deviation = |7.0 - 1.2| = 5.8mm > 2.0mm tolerance → PAUSE
```

---

## LED Status

| LED State | Meaning |
|---|---|
| White breathing | Idle (5s+ no movement) |
| LED off | Deep idle (5min+ no movement) |
| Blue pulsing | Filament moving |
| Solid green | Measure mode |
| Solid yellow | Calibration mode |

---

## Project Structure

```
smart_filament_sensor/
├── smart_filament_sensor.ino     # ESP32 firmware (Arduino)
├── klipper/
│   ├── smart_filament_sensor.py  # Klipper klippy module
│   └── KLIPPER_GUIDE.txt         # Full integration guide
├── firmware/                     # Pre-built binaries for flashing
├── console_app/                  # Electron desktop app source
├── 3d_print_files_and_bom/       # 3MF files + BOM
│   ├── SmartFilament_Body_*.3mf
│   ├── BackCover.3mf
│   ├── MagnetHolder.3mf
│   ├── Spring_Arm.3mf
│   └── bom.html
└── README.md
```

---

## License

Open-source. Free to use, modify, and distribute.

Created by **Ozan Sahin**
