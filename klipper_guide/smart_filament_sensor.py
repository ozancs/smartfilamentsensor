# smart_filament_sensor.py — Klipper klippy extra module
#
# ═══════════════════════════════════════════════════════════════════════════
#  SMART FILAMENT SENSOR — Klipper Native Module
# ═══════════════════════════════════════════════════════════════════════════
#
#  ESP32 is a pure measurement device. It only reports how many mm of
#  filament the encoder measured. All clog detection logic (window size,
#  tolerance, pause decision) lives here in Klipper.
#
# ─── PROTOCOL ─────────────────────────────────────────────────────────────
#
#   Klipper → ESP32:  "GET_MM_RESET\n"   read encoder mm + atomically reset
#   ESP32   → Klipper: "MM:<float>\n"    mm measured since last reset
#
#   Klipper → ESP32:  "GET_MM\n"         read encoder mm (no reset)
#   Klipper → ESP32:  "RESET_MM\n"       reset counter only
#
#   Klipper → ESP32:  "START <mm>\n"     start calibration with target mm
#   Klipper → ESP32:  "APPLY\n"          force-save calibration immediately
#   Klipper → ESP32:  "STOP\n"           cancel calibration / measure
#   Klipper → ESP32:  "STATUS\n"         query current sensor state
#
#   Klipper → ESP32:  "SET SENS <n>\n"   sensitivity (1-50)
#   Klipper → ESP32:  "SET NOISE <n>\n"  noise threshold (1-20)
#   Klipper → ESP32:  "SET BRIGHT <n>\n" LED brightness (1-255)
#   Klipper → ESP32:  "SET DIR <n>\n"    encoder direction (1 or -1)
#   Klipper → ESP32:  "SET CAL <f>\n"    calibration factor (deg/mm)
#
# ─── INSTALLATION ─────────────────────────────────────────────────────────
#
#   1. Copy this file to ~/klipper/klippy/extras/smart_filament_sensor.py
#   2. Add the config section below to your printer.cfg
#   3. Restart Klipper:  sudo systemctl restart klipper
#
# ─── printer.cfg ──────────────────────────────────────────────────────────
#
#   [smart_filament_sensor my_sensor]
#   serial: /dev/ttyUSB0          # ESP32 serial port
#   baud: 115200                  # must match ESP32 firmware
#   detection_length: 7.0         # mm of extrusion between each clog check
#   tolerance: 2.0                # max allowed deviation (mm) before clog
#   pause_on_clog: True           # automatically pause on clog detection
#   clog_gcode: PAUSE             # gcode to run when clog detected
#
# ─── GCODE COMMANDS ───────────────────────────────────────────────────────
#
#   Clog Detection:
#     SFS_STATUS          Show sensor state, detection window, etc.
#     SFS_ENABLE          Enable clog detection
#     SFS_DISABLE         Disable clog detection (sensor stays connected)
#     SFS_RESET           Re-sync extruder position and reset ESP32 odometer
#
#   Calibration:
#     SFS_CALIBRATE [LENGTH=10] Start calibration. Extrude LENGTH mm of
#                                  filament, then wait 5s for auto-save
#                                  or run SFS_CALIBRATE_APPLY.
#     SFS_CALIBRATE_APPLY       Force-save calibration result immediately
#     SFS_CALIBRATE_STOP        Cancel active calibration
#
#   ESP32 Settings:
#     SFS_SET SENS=<1-50>       Movement detection sensitivity (encoder steps)
#     SFS_SET NOISE=<1-20>      Noise filter deadband (encoder steps)
#     SFS_SET BRIGHT=<1-255>    Status LED brightness
#     SFS_SET DIR=<1 or -1>     Encoder direction multiplier
#     SFS_SET CAL=<float>       Calibration factor (deg/mm) — manual override
#
#     Multiple parameters can be combined in one command:
#       SFS_SET SENS=5 NOISE=3 BRIGHT=80
#
# ─── CALIBRATION EXAMPLE ──────────────────────────────────────────────────
#
#   ; From Klipper console or macro:
#   SFS_CALIBRATE LENGTH=50     ; start calibration, target 50mm
#   G1 E50 F100                    ; extrude 50mm slowly
#   ; (wait 5 seconds for auto-save, or:)
#   SFS_CALIBRATE_APPLY         ; save immediately
#
#   ; The new calibration factor is saved to ESP32 NVS flash.
#   ; It persists across power cycles — no need to recalibrate.
#
# ═══════════════════════════════════════════════════════════════════════════

import serial
import threading
import logging

class SmartFilamentSensor:
    def __init__(self, config):
        self.printer  = config.get_printer()
        self.reactor  = self.printer.get_reactor()
        self.gcode    = self.printer.lookup_object('gcode')
        self.name     = config.get_name().split()[-1]

        # Config — all decision logic stays here, not on ESP32
        self.serial_port      = config.get('serial')
        self.baud_rate        = config.getint('baud', 115200)
        self.detection_length = config.getfloat('detection_length', 7.0, above=0.)
        self.tolerance        = config.getfloat('tolerance', 2.0, above=0.)
        self.pause_on_clog    = config.getboolean('pause_on_clog', True)
        self.clog_gcode       = config.get('clog_gcode', 'PAUSE')

        # Runtime state
        self._serial           = None
        self._serial_lock      = threading.Lock()
        self._enabled          = True
        self._last_e_pos       = None   # Klipper E position at last check
        self._pending_expected = None   # mm we expected when we sent GET_MM_RESET
        self._calibrating      = False  # True while calibration is active

        self.printer.register_event_handler('klippy:connect',    self._handle_connect)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)

        self.gcode.register_command('SFS_STATUS',  self.cmd_STATUS,
            desc="Show smart filament sensor state")
        self.gcode.register_command('SFS_ENABLE',  self.cmd_ENABLE,
            desc="Enable clog detection")
        self.gcode.register_command('SFS_DISABLE', self.cmd_DISABLE,
            desc="Disable detection without disconnecting")
        self.gcode.register_command('SFS_RESET',   self.cmd_RESET,
            desc="Re-sync Klipper position and reset ESP32 odometer")
        self.gcode.register_command('SFS_CALIBRATE', self.cmd_CALIBRATE,
            desc="Start ESP32 encoder calibration. Usage: SFS_CALIBRATE [LENGTH=10]")
        self.gcode.register_command('SFS_CALIBRATE_APPLY', self.cmd_CALIBRATE_APPLY,
            desc="Force-save calibration result immediately (skip the 5s idle timeout)")
        self.gcode.register_command('SFS_CALIBRATE_STOP', self.cmd_CALIBRATE_STOP,
            desc="Cancel active calibration")
        self.gcode.register_command('SFS_SET', self.cmd_SET,
            desc="Change ESP32 settings. Usage: SFS_SET [SENS=] [NOISE=] [BRIGHT=] [DIR=] [CAL=]")

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def _handle_connect(self):
        try:
            self._serial = serial.Serial(
                self.serial_port, self.baud_rate, timeout=0.1)
        except Exception as e:
            raise self.printer.config_error(
                "SmartFilamentSensor '%s': cannot open %s: %s"
                % (self.name, self.serial_port, e))

        logging.info("SmartFilamentSensor '%s': connected to %s"
                     % (self.name, self.serial_port))

        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self.reactor.register_timer(
            self._extrusion_check, self.reactor.monotonic() + 2.0)

    def _handle_disconnect(self):
        if self._serial:
            self._serial.close()

    # ── Serial I/O ───────────────────────────────────────────────────────────

    def _send(self, cmd):
        if self._serial and self._serial.is_open:
            with self._serial_lock:
                try:
                    self._serial.write((cmd + '\n').encode())
                except Exception as e:
                    logging.error("SmartFilamentSensor '%s': write error: %s"
                                  % (self.name, e))

    def _serial_reader(self):
        while True:
            try:
                if self._serial and self._serial.is_open:
                    raw = self._serial.readline()
                    if raw:
                        line = raw.decode('utf-8', errors='replace').strip()
                        if line:
                            self._handle_line(line)
            except Exception as e:
                logging.error("SmartFilamentSensor '%s': read error: %s"
                              % (self.name, e))

    def _handle_line(self, line):
        """Called from reader thread."""
        logging.debug("SmartFilamentSensor '%s' RX: %s" % (self.name, line))

        # Forward calibration output to Klipper console
        if self._calibrating and (
                line.startswith("[CAL]") or
                line.startswith(">>> CALIBRATION") or
                line.startswith(">>> Measured:") or
                line.startswith(">>> New Cal Factor:") or
                line.startswith(">>> ERROR:")):
            msg = line
            self.reactor.register_async_callback(
                lambda et, m=msg: self.gcode.respond_info(m))
            if "CALIBRATION SUCCESS" in line or ">>> ERROR:" in line:
                self._calibrating = False
            return

        if line.startswith("MM:") and line != "MM:RESET":
            try:
                actual_mm = float(line[3:])
            except ValueError:
                return
            expected = self._pending_expected
            self._pending_expected = None
            if expected is None:
                return
            diff = abs(actual_mm - expected)
            logging.info(
                "SmartFilamentSensor '%s': expected=%.2fmm actual=%.2fmm diff=%.2fmm"
                % (self.name, expected, actual_mm, diff))
            if self._enabled and diff > self.tolerance:
                logging.warning(
                    "SmartFilamentSensor '%s': CLOG detected (diff %.2fmm > tolerance %.2fmm)"
                    % (self.name, diff, self.tolerance))
                if self.pause_on_clog:
                    self.reactor.register_async_callback(self._action_clog)


    # ── Extrusion Tracker ────────────────────────────────────────────────────

    def _get_e_pos(self):
        try:
            return self.printer.lookup_object('toolhead').get_position()[3]
        except Exception:
            return None

    def _is_printing(self):
        try:
            return self.printer.lookup_object('print_stats').state == 'printing'
        except Exception:
            return False

    def _extrusion_check(self, eventtime):
        if not self._is_printing():
            self._last_e_pos = self._get_e_pos()
            return eventtime + 0.5

        current_e = self._get_e_pos()
        if current_e is None:
            return eventtime + 0.5

        if self._last_e_pos is None:
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.5

        delta = current_e - self._last_e_pos

        if delta < 0:
            # G92 E0 or similar — re-sync without triggering a check
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.5

        if delta >= self.detection_length:
            # Ask ESP32 how much filament actually moved; it resets its own
            # counter atomically so we don't race with continued movement.
            self._pending_expected = delta
            self._send("GET_MM_RESET")
            self._last_e_pos = current_e

        return eventtime + 0.5

    # ── Gcode Actions ────────────────────────────────────────────────────────

    def _action_clog(self, eventtime):
        logging.info("SmartFilamentSensor '%s': executing clog gcode" % self.name)
        try:
            self.gcode.run_script(self.clog_gcode)
        except Exception as e:
            logging.error("SmartFilamentSensor '%s': clog gcode failed: %s"
                          % (self.name, e))

    # ── GCode Commands ───────────────────────────────────────────────────────

    def cmd_STATUS(self, gcmd):
        e     = self._get_e_pos()
        last  = self._last_e_pos or 0.0
        since = (e - last) if e is not None else 0.0
        gcmd.respond_info(
            "SmartFilamentSensor '%s':\n"
            "  enabled=%s  printing=%s\n"
            "  detection_length=%.1fmm  tolerance=%.1fmm\n"
            "  since_last_check=%.2fmm"
            % (self.name, self._enabled, self._is_printing(),
               self.detection_length, self.tolerance, since))

    def cmd_ENABLE(self, gcmd):
        self._enabled = True
        self._last_e_pos = self._get_e_pos()
        self._send("RESET_MM")
        gcmd.respond_info("SmartFilamentSensor '%s': enabled" % self.name)

    def cmd_DISABLE(self, gcmd):
        self._enabled = False
        gcmd.respond_info("SmartFilamentSensor '%s': disabled" % self.name)

    def cmd_RESET(self, gcmd):
        self._last_e_pos = self._get_e_pos()
        self._pending_expected = None
        self._send("RESET_MM")
        gcmd.respond_info("SmartFilamentSensor '%s': re-synced" % self.name)

    def cmd_CALIBRATE(self, gcmd):
        length = gcmd.get_float('LENGTH', 10.0, above=0.)
        self._calibrating = True
        self._send("START %.1f" % length)
        gcmd.respond_info(
            "SmartFilamentSensor '%s': calibration started (target=%.1fmm).\n"
            "Now extrude exactly %.1fmm of filament, then wait 5s for auto-save\n"
            "or run SFS_CALIBRATE_APPLY to save immediately."
            % (self.name, length, length))

    def cmd_CALIBRATE_APPLY(self, gcmd):
        if not self._calibrating:
            gcmd.respond_info("SmartFilamentSensor '%s': no calibration active" % self.name)
            return
        self._send("APPLY")
        gcmd.respond_info("SmartFilamentSensor '%s': apply sent, waiting for result..." % self.name)

    def cmd_CALIBRATE_STOP(self, gcmd):
        self._calibrating = False
        self._send("STOP")
        gcmd.respond_info("SmartFilamentSensor '%s': calibration cancelled" % self.name)

    def cmd_SET(self, gcmd):
        sent = []
        sens = gcmd.get_int('SENS', None)
        if sens is not None:
            self._send("SET SENS %d" % sens)
            sent.append("SENS=%d" % sens)
        noise = gcmd.get_int('NOISE', None)
        if noise is not None:
            self._send("SET NOISE %d" % noise)
            sent.append("NOISE=%d" % noise)
        bright = gcmd.get_int('BRIGHT', None)
        if bright is not None:
            self._send("SET BRIGHT %d" % bright)
            sent.append("BRIGHT=%d" % bright)
        direction = gcmd.get_int('DIR', None)
        if direction is not None:
            self._send("SET DIR %d" % direction)
            sent.append("DIR=%d" % direction)
        cal = gcmd.get_float('CAL', None)
        if cal is not None:
            self._send("SET CAL %.4f" % cal)
            sent.append("CAL=%.4f" % cal)
        if sent:
            gcmd.respond_info("SmartFilamentSensor '%s': set %s" % (self.name, ", ".join(sent)))
        else:
            gcmd.respond_info(
                "SmartFilamentSensor '%s': no parameters given.\n"
                "Usage: SFS_SET [SENS=] [NOISE=] [BRIGHT=] [DIR=] [CAL=]"
                % self.name)


def load_config(config):
    return SmartFilamentSensor(config)
