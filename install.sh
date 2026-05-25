#!/bin/bash
# Smart Filament Sensor — Klipper Module Installer
#
# Usage:
#   cd ~
#   git clone https://github.com/ozancs/smartfilamentsensor.git
#   ~/smartfilamentsensor/install.sh
#
# This creates a symlink so Klipper always uses the latest version.
# Updates via Moonraker update_manager will pull new code automatically.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="${SCRIPT_DIR}/klipper_guide/smart_filament_sensor.py"
DST="${HOME}/klipper/klippy/extras/smart_filament_sensor.py"

echo "============================================"
echo "  Smart Filament Sensor — Installer"
echo "============================================"
echo ""

# Check source exists
if [ ! -f "$SRC" ]; then
    echo "ERROR: Cannot find ${SRC}"
    echo "Make sure you cloned the full repository."
    exit 1
fi

# Check klipper directory exists
if [ ! -d "${HOME}/klipper/klippy/extras" ]; then
    echo "ERROR: Klipper extras directory not found at ${HOME}/klipper/klippy/extras"
    echo "Is Klipper installed?"
    exit 1
fi

# Remove old file or symlink if exists
if [ -e "$DST" ] || [ -L "$DST" ]; then
    echo "Removing existing: ${DST}"
    rm -f "$DST"
fi

# Create symlink
ln -s "$SRC" "$DST"
echo "Symlink created:"
echo "  ${DST} -> ${SRC}"
echo ""

# Add to Klipper's git exclude so Mainsail/Fluidd won't show
# "Repo has untracked source files" warning
EXCLUDE_FILE="${HOME}/klipper/.git/info/exclude"
EXCLUDE_ENTRY="klippy/extras/smart_filament_sensor.py"
if [ -f "$EXCLUDE_FILE" ]; then
    if ! grep -qF "$EXCLUDE_ENTRY" "$EXCLUDE_FILE" 2>/dev/null; then
        echo "$EXCLUDE_ENTRY" >> "$EXCLUDE_FILE"
        echo "Added to Klipper git exclude (no more untracked file warning)"
    fi
fi
echo ""

# Restart Klipper
echo "Restarting Klipper..."
sudo systemctl restart klipper
echo ""
echo "Done! Add the sensor config to your printer.cfg:"
echo ""
echo "  [smart_filament_sensor sfs]"
echo "  serial: auto"
echo "  baud: 115200"
echo "  detection_length: 7.0"
echo "  pause_on_runout: True"
echo "  runout_gcode: PAUSE"
echo "  underextrusion_max_rate: 0.5"
echo "  underextrusion_period: 5.0"
echo "  health_check_interval: 30.0"
echo ""
echo "Then restart Klipper: sudo systemctl restart klipper"
echo "============================================"
