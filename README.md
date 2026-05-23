# Smart Filament Sensor

High-precision filament motion sensor for 3D printers. Uses an **ESP32-C3** and **AS5600** magnetic encoder to detect clogs, runouts, and slippage in real-time with native **Klipper** integration.

> **Build video coming soon** — in the meantime, follow the **[Installation Guide](INSTALLATION_GUIDE.md)** and use the [interactive CAD model](https://a360.co/4fwGLLL) for reference.

---

## Features

- **Sub-mm accuracy** — AS5600 contactless encoder (4096 steps/rev), median-filtered, no drift
- **Native Klipper module** — time-based underextrusion detection, single bad readings never pause your print
- **Auto-detect serial port** — no need to manually set `/dev/ttyACM0`, finds ESP32 automatically
- **Underextrusion tracking** — rolling average exposed to Moonraker/Mainsail/Fluidd dashboards
- **Magnet health monitoring** — continuous AGC tracking, warns on weak/missing magnet
- **Sensor watchdog** — detects USB disconnection, warns user, blocks commands when offline
- **Homing awareness** — pauses detection during homing to prevent false triggers
- **One-command calibration** — `SFS_CALIBRATE LENGTH=50`, extrude, done
- **Status LED** — WS2812B breathing/pulse animations (idle, moving, calibrating)
- **Windows console app** — portable .exe for calibration, live measurement, settings, firmware flashing

---

## Bill of Materials

| Component | Qty | Notes |
|---|---|---|
| ESP32-C3 Super Mini | 1 | Main MCU |
| AS5600 Encoder Module | 1 | With diametrical magnet |
| WS2812B NeoPixel 5050 | 1 | Status LED |
| Grooved Bearing (U604ZZ) | 2 | OD 13mm, ID 4mm |
| Dowel Pin 3mm x 15mm | 1 | Bearing axle |
| O-Ring (~11mm OD) | 1 | Filament grip |
| PC4-M6 Fitting | 2 | Bowden tube entry/exit |
| Compression Spring | 1 | Pen-style, cut to length |
| M3 Screw (M3x8-M3x10) | 1 | |
| M2 Self-Tapping Screw | 2 | AS5600 + ESP32 mount |

Full BOM: [`3d_print_files_and_bom/bom.html`](3d_print_files_and_bom/bom.html)

---

## 3D Printed Parts

Files in [`3d_print_files_and_bom/`](3d_print_files_and_bom/) (print-ready 3MF):

- `SmartFilament_Body_w_Text.3mf` / `SmartFilament_Body_w-out_Text.3mf`
- `BackCover.3mf` / `MagnetHolder.3mf` / `Spring_Arm.3mf`

**[Interactive CAD Model (Fusion 360)](https://a360.co/4fwGLLL)**

---

## Firmware

### Quick Flash (Windows)
1. Download `Smart Filament Sensor Console v2.1.exe`
2. Connect ESP32 via USB → open app → **Firmware** tab → **Flash**

### Build from Source (Arduino IDE)
1. Board: **ESP32C3 Dev Module** (Arduino ESP32 core)
2. Libraries: `AS5600`, `FastLED`
3. Tools → **USB CDC On Boot: Enabled**
4. Open `smart_filament_sensor.ino` → Upload

---

## Klipper Integration

ESP32 only measures filament movement. All clog detection logic runs inside Klipper.

### Setup (with auto-update)

```bash
cd ~
git clone https://github.com/ozancs/smartfilamentsensor.git
~/smartfilamentsensor/install.sh
```

Add to `moonraker.conf` for automatic updates:
```ini
[update_manager smart_filament_sensor]
type: git_repo
path: ~/smartfilamentsensor
origin: https://github.com/ozancs/smartfilamentsensor.git
install_script: install.sh
primary_branch: main
managed_services: klipper
```

**Manual install:** Upload `smart_filament_sensor.py` via Mainsail/Fluidd, SSH and `cp ~/printer_data/config/smart_filament_sensor.py ~/klipper/klippy/extras/`

### printer.cfg

```ini
[smart_filament_sensor sfs]
serial: auto                      # auto-detects ESP32 (or use /dev/serial/by-id/...)
baud: 115200
detection_length: 7.0             # mm between each check
pause_on_runout: True
runout_gcode: PAUSE
underextrusion_max_rate: 0.5      # 0.0-1.0, pause if actual < 50% of expected
underextrusion_period: 5.0        # seconds underextrusion must persist before pause
health_check_interval: 30.0       # seconds between health checks
```

### GCode Commands

**Detection:**
| Command | Description |
|---|---|
| `SFS_STATUS` | Sensor state, health, underextrusion rate |
| `SFS_ENABLE` / `SFS_DISABLE` | Toggle clog detection |
| `SFS_RESET` | Re-sync position, reset stats |

**Calibration:**
| Command | Description |
|---|---|
| `SFS_AUTO_CALIBRATE TEMP=200 LENGTH=50` | One-command: heat, extrude, save |
| `SFS_CALIBRATE LENGTH=50` | Manual calibration start |
| `SFS_CALIBRATE_APPLY` | Save calibration immediately |
| `SFS_CALIBRATE_STOP` | Cancel calibration |

**ESP32 Settings:**
| Command | Description |
|---|---|
| `SFS_SET SENS=5 NOISE=3` | Sensitivity + noise filter |
| `SFS_SET BRIGHT=80` | LED brightness (1-255) |
| `SFS_SET DIR=-1` | Reverse encoder direction |
| `SFS_SET CAL=12.5` | Manual calibration factor (deg/mm) |

### Calibration

```gcode
; Automatic (recommended) — heats, extrudes, saves:
SFS_AUTO_CALIBRATE TEMP=200 LENGTH=50 SPEED=100

; Manual — you control the extrusion:
SFS_CALIBRATE LENGTH=50
G1 E50 F100
SFS_CALIBRATE_APPLY
```

Full guide: [`klipper_guide/KLIPPER_GUIDE.txt`](klipper_guide/KLIPPER_GUIDE.txt)

---

## Console App (Windows)

Portable `.exe` — no installation required.

- **Dashboard** — calibration wizard, live mm readout, sensor settings
- **Serial Monitor** — raw terminal with quick-action buttons for all commands
- **Firmware** — one-click ESP32 flash with bundled esptool
- **Klipper Guide** — config snippets and full Python module

---

## How It Works

```
Every 7mm of extrusion, Klipper asks ESP32 how much filament actually moved.

Klipper: 7.0mm commanded  →  ESP32: 6.8mm measured  →  97% flow ✓
Klipper: 7.0mm commanded  →  ESP32: 6.2mm measured  →  89% flow ✓
Klipper: 7.0mm commanded  →  ESP32: 1.2mm measured  →  17% flow ✗ (timer starts)
  ... underextrusion persists for 5 seconds ...             → PAUSE
  ... or flow recovers before 5 seconds ...                 → timer resets, continue
```

---

## LED Status

| LED | Meaning |
|---|---|
| White breathing | Idle |
| Blue pulsing | Filament moving |
| Solid green | Measure mode |
| Solid yellow | Calibration mode |
| LED off | Deep idle (5min+) |

---

## Project Structure

```
smartfilamentsensor/
├── klipper_guide/
│   ├── smart_filament_sensor.py   # Klipper module (v2.5)
│   └── KLIPPER_GUIDE.txt
├── 3d_print_files_and_bom/        # 3MF + BOM
├── photos/
├── firmware/                       # Pre-built .bin files
├── smart_filament_sensor.ino       # ESP32 firmware source
├── INSTALLATION_GUIDE.md
└── README.md
```

---

## License

Open-source. Free to use, modify, and distribute.

Created by **Ozan Sahin**
