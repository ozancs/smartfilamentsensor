# Smart Filament Sensor

High-precision filament motion sensor for 3D printers. Uses an **ESP32-C3** and **AS5600** magnetic encoder to detect clogs, runouts, and slippage in real-time with native **Klipper** integration.

> **Build video coming soon** — in the meantime, follow the **[Installation Guide](INSTALLATION_GUIDE.md)** and use the [interactive CAD model](https://a360.co/4fwGLLL) for reference.

---

## Features

- **Sub-mm accuracy** — AS5600 contactless encoder (4096 steps/rev), no drift
- **Drag-immune clog detection** — averages over 200mm of *extrusion distance*, not seconds, so bowden drag cannot fake a clog (see [How It Works](#how-it-works))
- **Self-checking calibration** — measures encoder-vs-extruder gain live; if it drifts, the module warns instead of pausing your print for a calibration fault
- **Auto-detect serial port** — no need to manually set `/dev/ttyACM0`, finds ESP32 automatically
- **Underextrusion tracking** — rolling average exposed to Moonraker/Mainsail/Fluidd dashboards
- **Magnet health monitoring** — uses the AS5600's own MH/ML status bits, warns on weak, **too strong**, or missing magnet
- **Sensor watchdog** — detects USB disconnection, warns user, blocks commands when offline
- **Auto-reconnect** — recovers from USB dropouts on its own, no `FIRMWARE_RESTART` needed
- **No false pauses on dropout** — a disconnect means "no data", never "no filament"; detection state resyncs instead of tripping a clog
- **Homing awareness** — pauses detection during homing to prevent false triggers
- **One-command calibration** — `SFS_AUTO_CALIBRATE LENGTH=100`, heats, extrudes, saves
- **Built-in diagnostics** — `SFS_DIAG` dumps every AS5600 register for troubleshooting
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

**[Interactive CAD Model (Fusion 360)](https://a360.co/3QY3KFT)**

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

**Manual install:** Upload `smart_filament_sensor.py` via Mainsail/Fluidd, then SSH and
`mv ~/printer_data/config/smart_filament_sensor.py ~/klipper/klippy/extras/`

Use `mv`, not `cp` — a leftover copy in the config folder shows up in the
Mainsail/Fluidd file list as if it were a config file, and editing that copy
silently does nothing because Klipper loads the one in `klippy/extras/`.

### printer.cfg

```ini
[smart_filament_sensor sfs]
serial: auto                      # auto-detects ESP32 (or use /dev/serial/by-id/...)
baud: 115200
detection_length: 7.0             # mm of extrusion between each encoder read
detection_window: 200.0           # mm of extrusion averaged per clog decision
underextrusion_max_rate: 0.5      # 0.0-1.0, trip if actual < 50% of expected
min_window_samples: 8             # reads required before any decision is made
encoder_scale: 1.0                # multiplies encoder mm (steady bias correction)
gain_tolerance: 0.25              # |gain-1| above this disarms detection
pause_on_runout: True
runout_gcode: PAUSE
health_check_interval: 30.0       # seconds between health checks
```

**A fixed `serial:` path is strongly recommended over `auto`.** With `auto`,
port probing writes `PING` to candidate ports; if one of them is a Klipper MCU
this can crash the printer. Find yours with `ls /dev/serial/by-id/`.

<details>
<summary><b>Upgrading from v2.x — what changed</b></summary>

`underextrusion_period` is gone. It used to set the averaging window in
*seconds*; the window is now set in *millimetres of extrusion* via
`detection_window`. Old configs still load (the value is reused as a
sample-staleness timeout), but you should replace it.

If you previously loosened `underextrusion_max_rate` to stop false pauses,
put it back to `0.5` — the distance window removes the reason you raised it.
</details>

### GCode Commands

**Detection:**
| Command | Description |
|---|---|
| `SFS_STATUS` | Sensor state, magnet health, encoder gain, window fill, armed/not |
| `SFS_ENABLE` / `SFS_DISABLE` | Toggle clog detection |
| `SFS_RESET` | Re-sync position, restart gain measurement |
| `SFS_DIAG` | Dump firmware version, calibration factor + all AS5600 registers (needs firmware >= 2.4.0) |
| `SFS_SUSPEND` / `SFS_RESUME` | Silence the module during probing (nestable) |

**Calibration:**
| Command | Description |
|---|---|
| `SFS_AUTO_CALIBRATE TEMP=240 LENGTH=100` | One-command: heat, extrude, save |
| `SFS_CALIBRATE LENGTH=100` | Manual calibration start |
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
SFS_AUTO_CALIBRATE TEMP=240 LENGTH=100 SPEED=100

; Manual — you control the extrusion:
SFS_CALIBRATE LENGTH=100
G1 E100 F100
SFS_CALIBRATE_APPLY
```

Use **100mm, not 50mm**. The wheel travels ~30mm of filament per revolution,
so a short extrude samples only part of one turn and any per-revolution error
folds straight into the result.

### Verifying calibration

Calibrating is not the same as being calibrated. Run a normal print, then:

```gcode
SFS_STATUS
```

Look at **`encoder_gain`** — the long-run ratio of measured to commanded
extrusion. It should settle near `1.000`.

| `encoder_gain` | Meaning |
|---|---|
| ~1.00 | Correct. Clog detection is armed. |
| < 0.90 | The wheel is turning less than the filament moves. Detection **disarms itself** and warns. |
| > 1.10 | Reading too much movement — check magnet placement. |
| negative | The encoder counts backwards. Flip it with `SFS_SET DIR=-1` (or `DIR=1`), then `SFS_RESET`. Usually means the magnet was reinstalled the other way up. |

A low gain that *survives recalibration* is mechanical, not a config problem:
the O-ring is slipping on the filament. Check spring pressure, O-ring
condition, and that the filament path through the sensor is straight.

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
Every 7mm of extrusion, Klipper asks the ESP32 how much filament actually moved.
Those readings are accumulated until they cover 200mm of commanded extrusion,
and only then is a clog decision made.

  read  1:  7.0mm commanded  →  -4.8mm measured     (toolhead pulled filament back)
  read  2:  7.0mm commanded  →  15.2mm measured     (and let it go again)
  read  3:  7.0mm commanded  →   6.9mm measured
  ...
  window full: 200mm commanded → 197mm measured     → 1.5% deficit ✓ keep printing
  window full: 200mm commanded →  22mm measured     → 89% deficit  ✗ PAUSE
```

### Why the window is measured in millimetres, not seconds

The sensor is frame-mounted and reaches the toolhead through a bowden tube, so
the filament between them is not rigidly coupled. As the toolhead moves, the
path length changes and filament is dragged back and forth through the encoder;
buckling inside the tube stores and releases even more. On real prints,
individual 7mm reads came back anywhere from **-13.8mm to +17.2mm** — with
retraction set to only 0.8mm.

This drag is bounded in millimetres and averages to zero over distance. A real
clog, by contrast, reads ~100% deficit no matter how long you look. So the
longer the window *in millimetres*, the better drag separates from clogs:

| Averaging window | Worst false reading from drag alone |
|---|---|
| 25 mm | 260 % |
| 50 mm | 106 % |
| 100 mm | 88 % |
| 150 mm | 58 % |
| **200 mm** | **41 %** |
| 300 mm | 26 % |

At 200mm the 50% threshold has real margin, and a total clog is still caught
after ~115mm of extrusion (roughly 25 seconds at typical flow).

A time-based window cannot do this. Detection reads arrive about every 2
seconds on a real print, so a 5-second average held a median of **three**
samples — not enough to average anything. That is what produced false clogs on
first layers and solid infill, where long sweeping moves drag the filament
hardest.

### Detection is disarmed when it cannot be trusted

A steady calibration error is indistinguishable from a partial clog inside any
single window. So instead of guessing, the module tracks the long-run
encoder-to-extruder gain and refuses to decide when that gain is off:

```
clog detection NOT ARMED.
  Encoder reads 63% of commanded extrusion (should be ~100%).
  This is a calibration problem, not a clog — the module will not pause
  the print for it.
```

---

## Magnet Health

`SFS_STATUS` reports the encoder's AGC (Automatic Gain Control) value. Per the
AS5600 datasheet the ideal AGC is the **middle** of its range — and the range
depends on supply voltage:

| Supply | AGC range | Ideal |
|---|---|---|
| 3.3V (ESP32-C3) | 0-128 | ~64 |
| 5V | 0-255 | ~128 |

AGC pinned at **0** means the magnet is too close (too strong); pinned at the
**top** means it is too far. Both degrade angle accuracy. Adjust the air gap
between the AS5600 board and the magnet — 1-2mm is the target — until AGC
lands near the middle.

Run `SFS_DIAG` to see AGC, magnitude, and every configuration register at once.

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
│   ├── smart_filament_sensor.py   # Klipper module
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
