# smart_filament_sensor.py — Klipper klippy extra module
#
# ═══════════════════════════════════════════════════════════════════════════
#  SMART FILAMENT SENSOR — Klipper Native Module v2.5
# ═══════════════════════════════════════════════════════════════════════════
#
#  ESP32 is a pure measurement device. It only reports how many mm of
#  filament the encoder measured. All clog detection logic (window size,
#  underextrusion detection, pause decision) lives here in Klipper.
#
# ─── PROTOCOL ─────────────────────────────────────────────────────────────
#
#   Klipper → ESP32:  "GET_MM_RESET\n"   read encoder mm + atomically reset
#   ESP32   → Klipper: "MM:<float>\n"    mm measured since last reset
#
#   Klipper → ESP32:  "GET_MM\n"         read encoder mm (no reset)
#   Klipper → ESP32:  "RESET_MM\n"       reset counter only
#   Klipper → ESP32:  "HEALTH\n"         query magnet health
#   ESP32   → Klipper: "HEALTH:<state>\n"  ok:<agc>, too_weak, no_magnet
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
#   1. Upload this file via Mainsail/Fluidd (Machine tab)
#   2. SSH: cp ~/printer_data/config/smart_filament_sensor.py ~/klipper/klippy/extras/
#   3. Add config section to printer.cfg
#   4. Restart Klipper:  sudo systemctl restart klipper
#
# ─── printer.cfg ──────────────────────────────────────────────────────────
#
#   [smart_filament_sensor my_sensor]
#   serial: auto                  # auto-detect ESP32, or use /dev/serial/by-id/...
#   baud: 115200                  # must match ESP32 firmware
#   detection_length: 7.0         # mm of extrusion between each check
#   pause_on_runout: True         # automatically pause on clog/runout
#   runout_gcode: PAUSE           # gcode to run when clog detected
#   underextrusion_max_rate: 0.5  # 0.0-1.0, underextrusion ratio to trigger alarm
#   underextrusion_period: 5.0    # seconds underextrusion must persist before pause
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
import glob
import os

# Expected PING→PONG identifier for auto-detect (avoids grabbing CAN/EBB ports)
_SFS_IDENTITY = "PONG"

class SmartFilamentSensor:
    def __init__(self, config):
        self.printer  = config.get_printer()
        self.reactor  = self.printer.get_reactor()
        self.gcode    = self.printer.lookup_object('gcode')
        self.name     = config.get_name().split()[-1]

        # Config — all decision logic stays here, not on ESP32
        self.serial_port      = config.get('serial', 'auto')
        self.baud_rate        = config.getint('baud', 115200)
        self.detection_length = config.getfloat('detection_length', 7.0, above=0.)
        self.pause_on_runout  = config.getboolean('pause_on_runout', True)
        self.runout_gcode     = config.get('runout_gcode', 'PAUSE')
        self.underextrusion_max_rate = config.getfloat(
            'underextrusion_max_rate', 0.5, minval=0., maxval=1.)
        self.underextrusion_period = config.getfloat(
            'underextrusion_period', 5.0, above=0.)
        self.health_check_interval = config.getfloat(
            'health_check_interval', 30.0, above=0.)

        # Runtime state
        self._serial           = None
        self._serial_lock      = threading.Lock()
        self._enabled          = True
        self._last_e_pos       = None   # Klipper E position at last check
        self._pending_expected = None   # mm we expected when we sent GET_MM_RESET
        self._calibrating      = False  # True while calibration is active
        self._active_port      = None   # Actual port we connected to

        # Sensor connection tracking
        self._connected        = False
        self._last_response_time = 0.0
        # Timeout must be > health_check_interval so we don't false-trigger
        # between periodic HEALTH queries
        self._connection_timeout = self.health_check_interval + 10.0

        # Magnet health tracking
        self._magnet_state     = 'unknown'  # ok, too_weak, no_magnet
        self._magnet_agc       = 0

        # Underextrusion rate tracking (Roadrunner-style time-based)
        self._extrusion_samples = []  # list of (timestamp, expected, actual)
        self._underextrusion_rate = 0.0  # 0.0 = perfect, 1.0 = full clog
        self._underextrusion_start_time = None  # when rate first exceeded threshold
        self._runout_triggered = False  # prevent repeated triggers

        # Homing awareness
        self._homing           = False

        # Calibration validation — expected range for deg/mm
        self._cal_min = 3.0   # below this = something very wrong
        self._cal_max = 50.0  # above this = something very wrong

        # Reader thread alive tracking (#3 fix)
        self._reader_alive     = False
        self._reader_died      = False

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

    def _auto_detect_port(self):
        """Try to find ESP32 serial port automatically.
        Uses PING/PONG handshake to avoid grabbing CAN adapters or EBB boards.
        """
        # 1. Check /dev/serial/by-id/ for stable symlinks
        by_id = glob.glob('/dev/serial/by-id/*')
        for path in by_id:
            lower = path.lower()
            if 'esp' in lower or 'cp210' in lower or 'ch340' in lower:
                # Verify with PING/PONG before committing
                if self._verify_sensor(path):
                    return path

        # 2. Fallback: try /dev/ttyACM* and /dev/ttyUSB* with PING verification
        candidates = sorted(
            glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
        for port in candidates:
            if self._verify_sensor(port):
                return port

        return None

    def _verify_sensor(self, port):
        """Send PING and expect PONG — confirms this is our ESP32 sensor."""
        try:
            test = serial.Serial(port, self.baud_rate, timeout=1.0)
            # Flush any boot garbage
            test.reset_input_buffer()
            time.sleep(0.1)
            test.reset_input_buffer()
            test.write(b'PING\n')
            # Read up to 5 lines (skip boot messages)
            for _ in range(5):
                response = test.readline().decode('utf-8', errors='replace').strip()
                if response == _SFS_IDENTITY:
                    test.close()
                    logging.info(
                        "SmartFilamentSensor '%s': verified sensor on %s"
                        % (self.name, port))
                    return True
            test.close()
        except Exception:
            pass
        return False

    def _handle_connect(self):
        # Resolve port: auto-detect or use configured path
        port = self.serial_port
        if port == 'auto':
            detected = self._auto_detect_port()
            if detected:
                port = detected
            else:
                logging.warning(
                    "SmartFilamentSensor '%s': auto-detect failed, "
                    "no ESP32 sensor found on any serial port"
                    % self.name)
                self._connected = False
                self.gcode.respond_info(
                    "SmartFilamentSensor '%s': sensor not found. "
                    "Plug in the sensor and run FIRMWARE_RESTART."
                    % self.name)
                # Still register timers so health check can retry
                self.reactor.register_timer(
                    self._extrusion_check, self.reactor.monotonic() + 2.0)
                self.reactor.register_timer(
                    self._health_check,
                    self.reactor.monotonic() + self.health_check_interval)
                return

        try:
            self._serial = serial.Serial(port, self.baud_rate, timeout=0.1)
            self._connected = True
            self._last_response_time = time.monotonic()
            self._active_port = port
            logging.info("SmartFilamentSensor '%s': connected to %s"
                         % (self.name, port))

            self._reader_alive = True
            self._reader_died = False
            self._reader_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self._reader_thread.start()

            # Ping sensor immediately so we get a response before first health check
            self._send("HEALTH")
        except Exception as e:
            logging.warning(
                "SmartFilamentSensor '%s': cannot open %s: %s "
                "(sensor will be detected when plugged in)"
                % (self.name, port, e))
            self._connected = False
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': sensor not found on %s. "
                "Plug in the sensor and run FIRMWARE_RESTART."
                % (self.name, port))

        # Extrusion check timer (250ms for faster response)
        self.reactor.register_timer(
            self._extrusion_check, self.reactor.monotonic() + 2.0)

        # Health check timer (magnet + connection monitoring)
        self.reactor.register_timer(
            self._health_check, self.reactor.monotonic() + self.health_check_interval)

    def _handle_disconnect(self):
        self._reader_alive = False
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
        """Background thread — reads lines from ESP32.
        If this thread dies, health_check will detect it and warn the user.
        """
        try:
            while self._reader_alive:
                try:
                    if self._serial and self._serial.is_open:
                        raw = self._serial.readline()
                        if raw:
                            line = raw.decode('utf-8', errors='replace').strip()
                            if line:
                                self._handle_line(line)
                    else:
                        break
                except serial.SerialException as e:
                    logging.error(
                        "SmartFilamentSensor '%s': serial error: %s"
                        % (self.name, e))
                    break
                except Exception as e:
                    logging.error(
                        "SmartFilamentSensor '%s': read error: %s"
                        % (self.name, e))
        finally:
            # Signal that reader thread has exited
            self._reader_alive = False
            if self._connected:
                self._connected = False
                self._reader_died = True
                logging.error(
                    "SmartFilamentSensor '%s': reader thread died!" % self.name)

    def _handle_line(self, line):
        """Called from reader thread."""
        logging.debug("SmartFilamentSensor '%s' RX: %s" % (self.name, line))

        # Any valid response = sensor is alive
        self._last_response_time = time.monotonic()
        if not self._connected:
            self._connected = True
            self._reader_died = False
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
        if self._calibrating:
            if (line.startswith("[CAL]") or
                    line.startswith(">>> CALIBRATION") or
                    line.startswith(">>> Measured:") or
                    line.startswith(">>> New Cal Factor:") or
                    line.startswith(">>> ERROR:")):
                msg = line
                self.reactor.register_async_callback(
                    lambda et, m=msg: self.gcode.respond_info(m))

                # Calibration validation (#4)
                if ">>> New Cal Factor:" in line:
                    self._validate_calibration(line)

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

            # Record sample for underextrusion averaging
            now = time.monotonic()
            self._extrusion_samples.append((now, expected, actual_mm))
            # Prune old samples outside the averaging period
            cutoff = now - self.underextrusion_period
            self._extrusion_samples = [
                s for s in self._extrusion_samples if s[0] >= cutoff]
            # Calculate rolling underextrusion rate
            self._update_underextrusion_rate()

            # Log each reading
            if expected > 0:
                rate = actual_mm / expected
            else:
                rate = 1.0
            logging.info(
                "SmartFilamentSensor '%s': expected=%.2fmm actual=%.2fmm "
                "rate=%.0f%% underextrusion=%.1f%%"
                % (self.name, expected, actual_mm, rate * 100,
                   self._underextrusion_rate * 100))

            # Time-based underextrusion detection (Roadrunner-style)
            if self._enabled:
                self._check_underextrusion(now)

    def _check_underextrusion(self, now):
        """Time-based clog detection: only pause if underextrusion
        exceeds threshold continuously for underextrusion_period seconds.
        Single bad readings are ignored — only sustained problems trigger.
        """
        if self._underextrusion_rate > self.underextrusion_max_rate:
            # Rate is bad — start or continue timer
            if self._underextrusion_start_time is None:
                self._underextrusion_start_time = now
                elapsed = 0.0
            else:
                elapsed = now - self._underextrusion_start_time

            logging.warning(
                "SmartFilamentSensor '%s': underextrusion %.1f%% > %.1f%% "
                "(%.1fs / %.1fs)"
                % (self.name, self._underextrusion_rate * 100,
                   self.underextrusion_max_rate * 100,
                   elapsed, self.underextrusion_period))

            if elapsed >= self.underextrusion_period:
                if not self._runout_triggered:
                    self._runout_triggered = True
                    logging.warning(
                        "SmartFilamentSensor '%s': CLOG CONFIRMED - "
                        "underextrusion %.1f%% sustained for %.1fs"
                        % (self.name, self._underextrusion_rate * 100,
                           elapsed))
                    if self.pause_on_runout:
                        self.reactor.register_async_callback(self._action_clog)
        else:
            # Rate is OK — reset timer
            if self._underextrusion_start_time is not None:
                logging.info(
                    "SmartFilamentSensor '%s': underextrusion recovered "
                    "(%.1f%%)" % (self.name,
                                  self._underextrusion_rate * 100))
            self._underextrusion_start_time = None
            self._runout_triggered = False

    def _validate_calibration(self, line):
        """Check if calibration result is reasonable (#4)."""
        try:
            # Parse ">>> New Cal Factor: 12.6178 deg/mm (saved)"
            parts = line.split(':')
            if len(parts) >= 2:
                val_str = parts[-1].strip().split()[0]
                cal_value = float(val_str)
                if cal_value < self._cal_min or cal_value > self._cal_max:
                    self.reactor.register_async_callback(
                        lambda et, v=cal_value: self.gcode.respond_info(
                            "SmartFilamentSensor '%s': WARNING - calibration "
                            "result %.2f deg/mm is unusual (expected %.0f-%.0f).\n"
                            "Check: magnet position, encoder direction (SFS_SET DIR=-1), "
                            "or re-calibrate with longer extrusion."
                            % (self.name, v, self._cal_min, self._cal_max)))
        except (ValueError, IndexError):
            pass

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

        # Retraction filter: ignore negative E moves (retract/unretract)
        # Typical retraction: 0.5-6mm. G92 E0 can jump hundreds of mm.
        if delta < -20.0:
            # Huge negative = G92 E0 or slicer E reset, re-sync encoder
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.25
        elif delta < 0:
            # Normal retraction (up to 20mm), just track position
            # Encoder sees retract+unretract, net movement cancels out
            self._last_e_pos = current_e
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
        # Check if reader thread died (#3)
        if self._reader_died and self._connected:
            self._connected = False
            self._reader_died = False
            logging.error(
                "SmartFilamentSensor '%s': reader thread crashed, "
                "sensor marked disconnected" % self.name)
            self.reactor.register_async_callback(
                lambda et: self.gcode.respond_info(
                    "SmartFilamentSensor '%s': WARNING - serial reader "
                    "crashed! Run FIRMWARE_RESTART to reconnect."
                    % self.name))

        # Check sensor connection timeout
        if self._connected:
            now = time.monotonic()
            if now - self._last_response_time > self._connection_timeout:
                self._connected = False
                logging.warning(
                    "SmartFilamentSensor '%s': sensor disconnected "
                    "(no response for %.0fs)"
                    % (self.name, self._connection_timeout))
                self.reactor.register_async_callback(
                    lambda et: self.gcode.respond_info(
                        "SmartFilamentSensor '%s': WARNING - sensor "
                        "disconnected! Check USB connection." % self.name))

        # Calibration check at print start (#5)
        if self._is_printing() and self._connected:
            if self._magnet_state == 'unknown':
                # First print since boot, haven't received HEALTH yet
                self._send("HEALTH")

        # Request magnet health from ESP32
        self._send("HEALTH")

        return eventtime + self.health_check_interval

    # ── Gcode Actions ────────────────────────────────────────────────────────

    def _action_clog(self, eventtime):
        logging.info("SmartFilamentSensor '%s': executing runout gcode" % self.name)
        try:
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': CLOG/RUNOUT detected! "
                "Underextrusion %.1f%% for %.1fs. Pausing print..."
                % (self.name, self._underextrusion_rate * 100,
                   self.underextrusion_period))
            self.gcode.run_script(self.runout_gcode)
        except Exception as e:
            logging.error("SmartFilamentSensor '%s': runout gcode failed: %s"
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
            'port': self._active_port or self.serial_port,
            'magnet_state': self._magnet_state,
            'magnet_agc': self._magnet_agc,
            'underextrusion_rate': round(self._underextrusion_rate, 4),
            'underextrusion_max_rate': self.underextrusion_max_rate,
            'underextrusion_alarming': self._underextrusion_start_time is not None,
            'detection_length': self.detection_length,
            'is_printing': self._is_printing(),
            'is_homing': self._homing,
        }

    # ── GCode Commands ───────────────────────────────────────────────────────

    def _require_connected(self, gcmd):
        """Check if sensor is connected. Returns True if OK, False if not."""
        if self._serial and not self._serial.is_open:
            self._connected = False
        if not self._connected:
            port_info = self._active_port or self.serial_port
            gcmd.respond_info(
                "SmartFilamentSensor '%s': ERROR - sensor not connected! "
                "(port: %s)\nPlug in the sensor and run FIRMWARE_RESTART."
                % (self.name, port_info))
            return False
        return True

    def cmd_STATUS(self, gcmd):
        # Update connection state
        if self._serial and not self._serial.is_open:
            self._connected = False
        port_info = self._active_port or self.serial_port
        e     = self._get_e_pos()
        last  = self._last_e_pos or 0.0
        since = (e - last) if e is not None else 0.0
        gcmd.respond_info(
            "SmartFilamentSensor '%s':\n"
            "  port=%s  sensor_connected=%s\n"
            "  enabled=%s  printing=%s  homing=%s\n"
            "  magnet=%s (AGC:%d)\n"
            "  detection_length=%.1fmm\n"
            "  underextrusion=%.1f%% (max:%.0f%%, period:%.0fs)\n"
            "  underextrusion_timer=%s\n"
            "  since_last_check=%.2fmm"
            % (self.name, port_info, self._connected,
               self._enabled, self._is_printing(),
               self._homing, self._magnet_state,
               self._magnet_agc, self.detection_length,
               self._underextrusion_rate * 100,
               self.underextrusion_max_rate * 100,
               self.underextrusion_period,
               ("%.1fs" % (time.monotonic() - self._underextrusion_start_time)
                if self._underextrusion_start_time else "idle"),
               since))

    def cmd_ENABLE(self, gcmd):
        if not self._require_connected(gcmd):
            return
        self._enabled = True
        self._last_e_pos = self._get_e_pos()
        self._send("RESET_MM")
        gcmd.respond_info("SmartFilamentSensor '%s': enabled" % self.name)

    def cmd_DISABLE(self, gcmd):
        self._enabled = False
        gcmd.respond_info("SmartFilamentSensor '%s': disabled" % self.name)

    def cmd_RESET(self, gcmd):
        if not self._require_connected(gcmd):
            return
        self._last_e_pos = self._get_e_pos()
        self._pending_expected = None
        self._extrusion_samples = []
        self._underextrusion_rate = 0.0
        self._underextrusion_start_time = None
        self._runout_triggered = False
        self._send("RESET_MM")
        gcmd.respond_info("SmartFilamentSensor '%s': re-synced" % self.name)

    def cmd_CALIBRATE(self, gcmd):
        if not self._require_connected(gcmd):
            return
        length = gcmd.get_float('LENGTH', 10.0, above=0.)
        self._calibrating = True
        self._send("START %.1f" % length)
        gcmd.respond_info(
            "SmartFilamentSensor '%s': calibration started (target=%.1fmm).\n"
            "Now extrude exactly %.1fmm of filament, then wait 5s for auto-save\n"
            "or run SFS_CALIBRATE_APPLY to save immediately."
            % (self.name, length, length))

    def cmd_CALIBRATE_APPLY(self, gcmd):
        if not self._require_connected(gcmd):
            return
        if not self._calibrating:
            gcmd.respond_info(
                "SmartFilamentSensor '%s': no calibration active" % self.name)
            return
        self._send("APPLY")
        gcmd.respond_info(
            "SmartFilamentSensor '%s': apply sent, waiting for result..."
            % self.name)

    def cmd_CALIBRATE_STOP(self, gcmd):
        if not self._require_connected(gcmd):
            return
        self._calibrating = False
        self._send("STOP")
        gcmd.respond_info(
            "SmartFilamentSensor '%s': calibration cancelled" % self.name)

    def cmd_SET(self, gcmd):
        if not self._require_connected(gcmd):
            return
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
            gcmd.respond_info(
                "SmartFilamentSensor '%s': set %s"
                % (self.name, ", ".join(sent)))
        else:
            gcmd.respond_info(
                "SmartFilamentSensor '%s': no parameters given.\n"
                "Usage: SFS_SET [SENS=] [NOISE=] [BRIGHT=] [DIR=] [CAL=]"
                % self.name)


def load_config(config):
    return SmartFilamentSensor(config)

def load_config_prefix(config):
    return SmartFilamentSensor(config)
