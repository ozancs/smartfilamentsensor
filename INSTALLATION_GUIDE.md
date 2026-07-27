# Smart Filament Sensor — Installation Guide

Step-by-step assembly, wiring, and first-run instructions. No special tools required beyond a soldering iron and basic screwdrivers.

> **Tip:** Open the [interactive CAD model](https://a360.co/4fwGLLL) side-by-side while building — you can rotate and measure every part.

---

## 1. Prepare the Bearing & Magnet Assembly

### 1.1 O-Ring on Bearing

Take one of the two **U604ZZ grooved bearings** (OD 13mm, ID 4mm) and stretch the **O-Ring (~11mm OD)** over its groove. The O-ring sits in the bearing's channel and provides grip so the filament drives the bearing as it passes through.

### 1.2 Magnet Holder

Take the **MagnetHolder** (3D printed part: `MagnetHolder.3mf`) and attach it to the top of this bearing using small strips of **double-sided tape on the edges only**.

> **Important:** Only tape the outer edges of the magnet holder to the bearing. The center must spin freely — this is how the encoder tracks rotation.

### 1.3 Magnet

Press the **diametrical magnet** (included with the AS5600 module) into the pocket on top of the magnet holder. A drop of glue or friction fit will keep it in place. Make sure it is centered and sits flush.

---

## 2. Main Body Assembly

### 2.1 Bearing into Body

Place the bearing + magnet holder assembly into the **SmartFilament Body** (3D printed part: `SmartFilament_Body_w_Text.3mf` or `SmartFilament_Body_w-out_Text.3mf`). It should drop into its seat with minimal play — slight movement is fine, but it must not fall out when tilted.

### 2.2 Spring Arm & Second Bearing

Take the **Spring Arm** (3D printed part: `Spring_Arm.3mf`) and press the second **U604ZZ grooved bearing** into the center pocket. This should be a snug fit. If the bearing is loose due to print tolerances, wrap a thin strip of tape around the bearing's outer edge and press it in until fully seated.

### 2.3 Dowel Pin & Spring Arm

Insert the **3mm x 15mm dowel pin** (smooth hardened steel) into the pivot hole on the main body. Then slide the spring arm (with bearing installed) onto the dowel pin.

### 2.4 Spring

Place the **compression spring** (pen-style) into its slot between the body and the spring arm. The spring arm should press the second bearing against the first, sandwiching the filament path.

> **Feel check:** Push the spring arm by hand. It should offer firm but smooth resistance — not so stiff that filament can't push through, not so loose that it won't grip. If the spring is too strong, trim it shorter with wire cutters. This is a feel-based adjustment, there's no exact spec.

### 2.5 PTFE Fittings

Thread the two **PC4-M6 pneumatic fittings** into the entry and exit holes on the main body. They should screw in smoothly by hand. If the threads are tight from print tolerances, wrap a tiny bit of tape around the threads for a tighter seal.

---

## 3. Wiring

### 3.1 Pin Reference

**AS5600 Encoder → ESP32-C3:**

| AS5600 Pin | ESP32-C3 Pin | Notes |
|---|---|---|
| VCC | 3.3V | **3.3V only** — do not use 5V |
| GND | GND | |
| SDA | GPIO 4 | I2C data |
| SCL | GPIO 5 | I2C clock |

**WS2812B NeoPixel LED → ESP32-C3** *(optional)*:

| LED Pin | ESP32-C3 Pin | Notes |
|---|---|---|
| VCC | 5V (VBUS) | Needs 5V for full brightness |
| GND | GND | |
| DIN | GPIO 2 | Data in |

> **Note:** The status LED is optional. If you don't want it, skip all LED wiring. The sensor works perfectly without it.

### 3.2 Soldering

Cut wires to appropriate lengths — leave enough slack so nothing is under tension when the enclosure is closed, but not so long that wires bunch up and interfere with the moving spring arm.

Solder all connections according to the pin table above. Double-check polarity on the LED (VCC/GND) and the AS5600 (3.3V, not 5V).

---

## 4. Electronics Installation

### 4.1 Status LED

Press the **WS2812B NeoPixel** into its square pocket on the main body. It's a tight fit by design — push it in firmly but carefully to avoid damaging the LED or its solder joints. **Pay attention to orientation** — the square LED should align with the square pocket. Check that the DIN wire faces the correct direction before pressing in.

### 4.2 ESP32-C3

Stick a small piece of **double-sided foam tape** to the back of the ESP32-C3 Super Mini. Place it into its slot in the main body. The foam tape keeps it in place and provides insulation.

Secure the ESP32 with **2x M2 self-tapping screws** through the screw holes at the rear of the ESP32 slot. These prevent the board from sliding backward — the foam tape alone isn't enough.

### 4.3 AS5600 Encoder

The AS5600 module mounts directly above the magnet. The main body has **2 wide holes for M3 heat-set inserts** — press the inserts in with a soldering iron, then secure the AS5600 from above with **2x M3 screws**.

On the opposite side, use **2x M2 self-tapping screws** through the remaining mounting holes. The AS5600 is held by **4 screws total** (2x M3 + 2x M2) for a solid, vibration-free mount.

> **Critical:** The AS5600 chip must sit centered directly above the magnet, with a small air gap (1-2mm). If it's too far, the sensor won't detect the magnet. Too close and it might touch the spinning magnet holder.

---

## 5. Back Cover & Mounting

### 5.1 M3 Nuts

The **BackCover** (3D printed part: `BackCover.3mf`) has **2 recessed pockets** on its inner face for M3 nuts. Press a nut into each pocket — these are for mounting the sensor to your printer frame later.

### 5.2 Close the Enclosure

Route all wires so they don't get pinched or interfere with the spring arm mechanism. Then attach the back cover to the main body using **6x M2 self-tapping screws**.

> **Cable check:** Before fully tightening, gently push the spring arm to make sure no wires are caught in the mechanism.

### 5.3 Printer Mounting

Use the 2x M3 bolt holes (through the back cover nuts) to mount the sensor to your printer's frame, bowden tube path, or a custom bracket. Mounting options and brackets may be added in the future.

---

## 6. Firmware & First Run

### Option A: Windows Console App (Recommended)

1. Download **Smart Filament Sensor Console.exe** from [Releases](../../releases)
2. Connect the ESP32 to your PC via USB
3. The app auto-detects the sensor — click **Connect**
4. Go to **Firmware** tab → **Start Firmware Flash**
5. After flashing, go to **Dashboard** → run calibration

### Option B: Arduino IDE

1. Install **Arduino ESP32 core** (board: ESP32C3 Dev Module)
2. Install libraries: `AS5600`, `FastLED`
3. In **Tools** menu: set **USB CDC On Boot → Enabled**
4. Open `smart_filament_sensor.ino` and upload
5. Open Serial Monitor at 115200 baud, type `HELP`

### First Calibration

The sensor ships with a default calibration factor (12.5 deg/mm) that will be inaccurate for your specific build. **You must calibrate before use.**

**Using the Console App:**
1. Go to Dashboard → Calibration card
2. Set target to 50mm, click **Start Calibration**
3. Push exactly 50mm of filament through the sensor by hand (use a ruler)
4. Wait 5 seconds — calibration auto-saves

**Using Klipper:**
```gcode
SFS_CALIBRATE LENGTH=50
G1 E50 F100
SFS_CALIBRATE_APPLY
```

**Using Serial Terminal:**
```
START 50
(push 50mm of filament, wait 5 seconds)
```

After calibration, verify with the **Measure** mode — push a known length and check that the reading matches.

---

## 7. Klipper Setup

See the full [Klipper Integration Guide](klipper_guide/KLIPPER_GUIDE.txt) for detailed instructions.

Quick start:
```bash
cp klipper_guide/smart_filament_sensor.py ~/klipper/klippy/extras/
sudo systemctl restart klipper
```

`cp` is correct here — this copies out of the cloned repo, which must keep its
own copy for `git pull` / update_manager to work. If you instead uploaded the
file through Mainsail/Fluidd, use `mv` so no stray copy is left in your config
folder.

Add to `printer.cfg`:
```ini
[smart_filament_sensor my_sensor]
serial: auto                    # or a fixed /dev/serial/by-id/... path
baud: 115200
detection_length: 7.0           # mm of extrusion per encoder read
detection_window: 200.0         # mm of extrusion averaged per clog decision
underextrusion_max_rate: 0.5    # deficit that counts as a clog
min_window_samples: 8           # reads required before deciding
encoder_scale: 1.0              # encoder mm multiplier (bias correction)
gain_tolerance: 0.25            # |gain-1| above this disarms detection
pause_on_runout: True
runout_gcode: PAUSE
health_check_interval: 30.0     # seconds between magnet health checks
```

---

## Troubleshooting

| Problem | Solution |
|---|---|
| Sensor not detected on USB | Make sure **USB CDC On Boot** is enabled in Arduino IDE. Reflash if needed. |
| No magnet detected (warning on boot) | Check magnet is centered in the holder, AS5600 air gap is 1-2mm. |
| Magnet too strong / too weak | Adjust the AS5600-to-magnet air gap. Run `SFS_DIAG`: ideal AGC is the MIDDLE of its range (~64 at 3.3V, ~128 at 5V). AGC pinned at 0 means the magnet is too close. |
| Inconsistent readings | Re-calibrate with a longer distance (100mm). Check O-ring grip and bearing spin. |
| Spring too stiff | Trim the compression spring shorter. |
| PTFE fittings loose | Wrap thread tape around the fitting threads. |
| Wires caught in mechanism | Re-route wires away from the spring arm, re-close back cover. |
| "Sensor disconnected" warning in Klipper | Check USB cable, try a different port. Sensor auto-reconnects when restored. |
| False clog during homing | Update to v2.2 — homing awareness automatically pauses detection during homing moves. |
