# smart_filament_sensor.py — Klipper klippy extra module
#
# ═══════════════════════════════════════════════════════════════════════════
#  SMART FILAMENT SENSOR — Klipper Native Module v2.2
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
#   Klipper → ESP32:  "HEALTH\n"         query magnet health
#   ESP32   → Klipper: "HEALTH:<state>\n"  ok:<agc>, too_weak, too_strong, no_magnet
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
#   underextrusion_period: 10.0   # seconds to average underextrusion rate
#   health_check_interval: 30.0   # seconds between magnet health checks
#
# ─── GCODE COMMANDS ───────────────────────────────────────────────────────
#
#   Clog Detection:
#     SFS_STATUS          Show sensor state, health, underextrusion rate
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
# ═══════════════════════════════════════════════════════════════════════════

import serial
import threading
import logging
import time

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
        self.underextrusion_period = config.getfloat(
            'underextrusion_period', 10.0, above=0.)
        self.health_check_interval = config.getfloat(
            'health_check_interval', 30.0, above=0.)

        # Runtime state
        self._serial           = None
        self._serial_lock      = threading.Lock()
        self._enabled          = True
        self._last_e_pos       = None   # Klipper E position at last check
        self._pending_expected = None   # mm we expected when we sent GET_MM_RESET
        self._calibrating      = False  # True while calibration is active

        # Sensor connection tracking
        self._connected        = False
        self._last_response_time = 0.0
        # Timeout must be > health_check_interval so we don't false-trigger
        # between periodic HEALTH queries
        self._connection_timeout = self.health_check_interval + 10.0

        # Magnet health tracking
        self._magnet_state     = 'unknown'  # ok, too_weak, too_strong, no_magnet
        self._magnet_agc       = 0

        # Underextrusion rate tracking
        self._extrusion_samples = []  # list of (timestamp, expected, actual)
        self._underextrusion_rate = 0.0  # 0.0 = perfect, 1.0 = full clog

        # Homing awareness
        self._homing           = False

        self.printer.register_event_handler('klippy:connect',    self._handle_connect)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)
        self.printer.register_event_handler(
            'homing:homing_move_begin', self._handle_homing_begin)
        self.printer.register_event_handler(
            'homing:homing_move_end', self._handle_homing_end)

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
            self._connected = True
            self._last_response_time = time.monotonic()
            logging.info("SmartFilamentSensor '%s': connected to %s"
                         % (self.name, self.serial_port))

            self._reader_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self._reader_thread.start()

            # Ping sensor immediately so we get a response before first health check
            self._send("HEALTH")
        except Exception as e:
            logging.warning(
                "SmartFilamentSensor '%s': cannot open %s: %s "
                "(sensor will be detected when plugged in)"
                % (self.name, self.serial_port, e))
            self._connected = False
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': sensor not found on %s. "
                "Plug in the sensor and run FIRMWARE_RESTART."
                % (self.name, self.serial_port))

        # Extrusion check timer (250ms for faster response)
        self.reactor.register_timer(
            self._extrusion_check, self.reactor.monotonic() + 2.0)

        # Health check timer (magnet + connection monitoring)
        self.reactor.register_timer(
            self._health_check, self.reactor.monotonic() + self.health_check_interval)

    def _handle_disconnect(self):
        if self._serial:
            self._serial.close()
        self._connected = False

    # ── Homing Awareness ─────────────────────────────────────────────────────

    def _handle_homing_begin(self, hmove):
        self._homing = True

    def _handle_homing_end(self, hmove):
        self._homing = False
        # Re-sync after homing to avoid false triggers
        self._last_e_pos = self._get_e_pos()
        self._send("RESET_MM")

    # ── Serial I/O ───────────────────────────────────────────────────────────

    def _send(self, cmd):
        if self._serial and self._serial.is_open:
            with self._serial_lock:
                try:
                    self._serial.write((cmd + '\n').encode())
                except Exception as e:
                    logging.error("SmartFilamentSensor '%s': write error: %s"
                                  % (self.name, e))
                    if self._connected:
                        self._connected = False
                        self.reactor.register_async_callback(
                            lambda et: self.gcode.respond_info(
                                "SmartFilamentSensor '%s': WARNING - sensor "
                                "disconnected! (write error)" % self.name))

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

        # Any valid response = sensor is alive
        self._last_response_time = time.monotonic()
        if not self._connected:
            self._connected = True
            logging.info("SmartFilamentSensor '%s': sensor reconnected"
                         % self.name)
            self.reactor.register_async_callback(
                lambda et: self.gcode.respond_info(
                    "SmartFilamentSensor '%s': sensor reconnected" % self.name))

        # Health response
        if line.startswith("HEALTH:"):
            health = line[7:]
            if health.startswith("ok:"):
                self._magnet_state = 'ok'
                try:
                    self._magnet_agc = int(health[3:])
                except ValueError:
                    self._magnet_agc = 0
            elif health == "no_magnet":
                self._magnet_state = 'no_magnet'
                self._magnet_agc = 0
                logging.warning(
                    "SmartFilamentSensor '%s': NO MAGNET DETECTED!"
                    % self.name)
                if self._enabled and self._is_printing():
                    self.reactor.register_async_callback(self._action_magnet_error)
            elif health == "too_weak":
                self._magnet_state = 'too_weak'
                logging.warning(
                    "SmartFilamentSensor '%s': magnet too weak" % self.name)
            return

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

            # Calculate extrusion rate
            if expected > 0:
                rate = actual_mm / expected  # 1.0 = perfect, 0.0 = full clog
            else:
                rate = 1.0

            # Record sample for underextrusion averaging
            now = time.monotonic()
            self._extrusion_samples.append((now, expected, actual_mm))
            # Prune old samples outside the averaging period
            cutoff = now - self.underextrusion_period
            self._extrusion_samples = [
                s for s in self._extrusion_samples if s[0] >= cutoff]
            # Calculate rolling underextrusion rate
            self._update_underextrusion_rate()

            diff = abs(actual_mm - expected)
            logging.info(
                "SmartFilamentSensor '%s': expected=%.2fmm actual=%.2fmm "
                "diff=%.2fmm rate=%.1f%%"
                % (self.name, expected, actual_mm, diff, rate * 100))
            if self._enabled and diff > self.tolerance:
                logging.warning(
                    "SmartFilamentSensor '%s': CLOG detected "
                    "(diff %.2fmm > tolerance %.2fmm)"
                    % (self.name, diff, self.tolerance))
                if self.pause_on_clog:
                    self.reactor.register_async_callback(self._action_clog)

    def _update_underextrusion_rate(self):
        if not self._extrusion_samples:
            self._underextrusion_rate = 0.0
            return
        total_expected = sum(s[1] for s in self._extrusion_samples)
        total_actual = sum(s[2] for s in self._extrusion_samples)
        if total_expected > 0:
            # underextrusion_rate: 0.0 = perfect, 1.0 = total clog
            self._underextrusion_rate = max(
                0.0, 1.0 - (total_actual / total_expected))
        else:
            self._underextrusion_rate = 0.0

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
        # Skip during homing to avoid false triggers
        if self._homing:
            return eventtime + 0.25

        if not self._is_printing():
            self._last_e_pos = self._get_e_pos()
            return eventtime + 0.25

        current_e = self._get_e_pos()
        if current_e is None:
            return eventtime + 0.25

        if self._last_e_pos is None:
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.25

        delta = current_e - self._last_e_pos

        if delta < 0:
            # G92 E0 or similar — re-sync without triggering a check
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.25

        if delta >= self.detection_length:
            # Ask ESP32 how much filament actually moved; it resets its own
            # counter atomically so we don't race with continued movement.
            self._pending_expected = delta
            self._send("GET_MM_RESET")
            self._last_e_pos = current_e

        return eventtime + 0.25

    # ── Health Check Timer ───────────────────────────────────────────────────

    def _health_check(self, eventtime):
        # Check sensor connection
        now = time.monotonic()
        if now - self._last_response_time > self._connection_timeout:
            if self._connected:
                self._connected = False
                logging.warning(
                    "SmartFilamentSensor '%s': sensor disconnected "
                    "(no response for %.0fs)"
                    % (self.name, self._connection_timeout))
                self.reactor.register_async_callback(
                    lambda et: self.gcode.respond_info(
                        "SmartFilamentSensor '%s': WARNING - sensor "
                        "disconnected! Check USB connection." % self.name))

        # Request magnet health from ESP32
        self._send("HEALTH")

        return eventtime + self.health_check_interval

    # ── Gcode Actions ────────────────────────────────────────────────────────

    def _action_clog(self, eventtime):
        logging.info("SmartFilamentSensor '%s': executing clog gcode" % self.name)
        try:
            self.gcode.run_script(self.clog_gcode)
        except Exception as e:
            logging.error("SmartFilamentSensor '%s': clog gcode failed: %s"
                          % (self.name, e))

    def _action_magnet_error(self, eventtime):
        logging.warning(
            "SmartFilamentSensor '%s': magnet error during print!" % self.name)
        try:
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': WARNING - Magnet not detected! "
                "Sensor readings may be unreliable. Check magnet positioning."
                % self.name)
        except Exception:
            pass

    # ── Moonraker / Dashboard Status ─────────────────────────────────────────

    def get_status(self, eventtime):
        return {
            'enabled': self._enabled,
            'sensor_connected': self._connected,
            'magnet_state': self._magnet_state,
            'magnet_agc': self._magnet_agc,
            'underextrusion_rate': round(self._underextrusion_rate, 4),
            'detection_length': self.detection_length,
            'tolerance': self.tolerance,
            'is_printing': self._is_printing(),
            'is_homing': self._homing,
        }

    # ── GCode Commands ───────────────────────────────────────────────────────

    def cmd_STATUS(self, gcmd):
        # Check if serial port is still valid
        if self._serial and not self._serial.is_open:
            self._connected = False
        e     = self._get_e_pos()
        last  = self._last_e_pos or 0.0
        since = (e - last) if e is not None else 0.0
        gcmd.respond_info(
            "SmartFilamentSensor '%s':\n"
            "  enabled=%s  printing=%s  homing=%s\n"
            "  sensor_connected=%s  magnet=%s (AGC:%d)\n"
            "  detection_length=%.1fmm  tolerance=%.1fmm\n"
            "  underextrusion_rate=%.1f%%\n"
            "  since_last_check=%.2fmm"
            % (self.name, self._enabled, self._is_printing(),
               self._homing, self._connected, self._magnet_state,
               self._magnet_agc, self.detection_length, self.tolerance,
               self._underextrusion_rate * 100, since))

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
        self._extrusion_samples = []
        self._underextrusion_rate = 0.0
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

def load_config_prefix(config):
    return SmartFilamentSensor(config)
