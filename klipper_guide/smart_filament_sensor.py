# smart_filament_sensor.py — Klipper klippy extra module
#
# ═══════════════════════════════════════════════════════════════════════════
#  SMART FILAMENT SENSOR — Klipper Native Module
#  Version: see __version__ below (reported by SFS_STATUS)
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
#     SFS_AUTO_CALIBRATE [TEMP=240] [LENGTH=50] [SPEED=100]
#                                  One-command calibration: heats hotend,
#                                  extrudes, saves automatically.
#     SFS_CALIBRATE [LENGTH=10] Manual calibration start. Extrude LENGTH mm
#                                  then wait 5s or run SFS_CALIBRATE_APPLY.
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

__version__ = "2.7.1"

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
        self._measure_active   = False  # True between MEASURE start and read
        self._measure_pending  = False  # True while waiting for MEASURE mm reply
        self._measure_deadline = 0.0    # give up waiting after this, so a lost
                                        # reply can't swallow a detection sample
        self._calibrating      = False  # True while calibration is active
        self._active_port      = None   # Actual port we connected to
        self._last_good_port   = None   # Port that passed PING/PONG before —
                                        # reconnects reuse it instead of
                                        # re-probing every serial device

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

        # Grace period after a reconnect/resync — detection stays off until
        # this deadline so stale or post-reset encoder data can't fire a clog.
        self._resync_grace_until = 0.0
        self._resync_grace_period = 5.0

        # Homing awareness
        self._homing           = False

        # Print-state edge detection: on the not-printing -> printing
        # transition both sides resync (see _extrusion_check)
        self._was_printing     = False

        # Calibration validation — expected range for deg/mm
        self._cal_min = 3.0   # below this = something very wrong
        self._cal_max = 50.0  # above this = something very wrong

        # Reader thread alive tracking (#3 fix)
        self._reader_alive     = False
        self._reader_died      = False
        self._reader_thread    = None

        # Auto-reconnect. Port probing blocks (readline with timeout), so it
        # runs on a worker thread — never on the reactor thread.
        self._reconnecting     = False
        self._shutting_down    = False

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
        self.gcode.register_command('SFS_AUTO_CALIBRATE', self.cmd_AUTO_CALIBRATE,
            desc="Auto calibrate: heat, extrude, save. Usage: SFS_AUTO_CALIBRATE [TEMP=240] [LENGTH=50] [SPEED=100]")
        self.gcode.register_command('SFS_MEASURE', self.cmd_MEASURE,
            desc="Measure mode: 1st call resets, extrude, 2nd call shows measured mm")

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def _klipper_mcu_ports(self):
        """Serial paths Klipper uses for its own MCUs.

        Probing a port means opening it and writing "PING" to it. Doing that
        to an MCU injects garbage into its protocol stream and takes the
        printer down ("Missed scheduling of next digital out event"), which
        also kills any CAN board bridged through it. These are never probed.
        """
        ports = set()
        try:
            settings = self.printer.lookup_object(
                'configfile').get_status(None)['settings']
        except Exception:
            # Can't enumerate: assume nothing is safe to blind-probe
            return None
        for section, values in settings.items():
            if section != 'mcu' and not section.startswith('mcu '):
                continue
            try:
                path = values.get('serial')
            except Exception:
                continue
            if path:
                ports.add(path)
                try:
                    ports.add(os.path.realpath(path))
                except Exception:
                    pass
        return ports

    def _auto_detect_port(self, full_scan=True):
        """Find the ESP32 serial port. Uses a PING/PONG handshake.

        full_scan=False restricts this to the port we already verified once.
        Reconnects use that: rescanning would PING every candidate again, and
        the sensor's by-id path is stable across re-enumeration anyway.
        """
        exclude = self._klipper_mcu_ports()
        if exclude is None:
            logging.warning(
                "SmartFilamentSensor '%s': cannot enumerate Klipper MCU "
                "ports, skipping probe to avoid disturbing them" % self.name)
            return None

        def usable(port):
            if port in exclude:
                return False
            try:
                return os.path.realpath(port) not in exclude
            except Exception:
                return False

        # A port we verified before is the only one worth reopening.
        if self._last_good_port and usable(self._last_good_port):
            if self._verify_sensor(self._last_good_port):
                return self._last_good_port

        if not full_scan:
            return None

        # 1. /dev/serial/by-id/ symlinks that look like an ESP32
        candidates = [p for p in sorted(glob.glob('/dev/serial/by-id/*'))
                      if any(k in p.lower()
                             for k in ('esp', 'cp210', 'ch340'))]
        # 2. Fallback: raw tty devices
        candidates += sorted(
            glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))

        for port in candidates:
            if not usable(port):
                logging.info(
                    "SmartFilamentSensor '%s': not probing %s (Klipper MCU)"
                    % (self.name, port))
                continue
            if self._verify_sensor(port):
                self._last_good_port = port
                return port

        return None

    def _verify_sensor(self, port):
        """Send PING and expect PONG — confirms this is our ESP32 sensor."""
        test = None
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
                    logging.info(
                        "SmartFilamentSensor '%s': verified sensor on %s"
                        % (self.name, port))
                    return True
        except Exception:
            pass
        finally:
            # Must close on every path — a leaked handle keeps the port busy
            # and blocks the next probe (and Klipper's own open).
            if test is not None:
                try:
                    test.close()
                except Exception:
                    pass
        return False

    def _teardown_serial(self):
        """Close the port and wind down the reader thread.

        Closing the handle unblocks a reader parked in readline(); joining it
        before we touch connection state stops the dying thread's cleanup from
        clobbering a fresh connection.
        """
        self._reader_alive = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._reader_thread = None
        self._reader_died = False

    def _attempt_connect(self, full_scan=True):
        """Open the sensor port and start reading. Returns True on success.

        Safe to call repeatedly. Blocks while probing ports, so callers must
        keep it off the reactor thread once Klipper is running.

        full_scan=False limits auto-detect to the already-known port. Used for
        reconnects, which happen while the printer is live and must not probe
        anything else.
        """
        self._teardown_serial()

        port = self.serial_port
        if port == 'auto':
            port = self._auto_detect_port(full_scan=full_scan)
            if not port:
                logging.warning(
                    "SmartFilamentSensor '%s': auto-detect found no sensor"
                    % self.name)
                return False

        try:
            self._serial = serial.Serial(port, self.baud_rate, timeout=0.1)
        except Exception as e:
            logging.warning("SmartFilamentSensor '%s': cannot open %s: %s"
                            % (self.name, port, e))
            self._serial = None
            return False

        self._active_port = port
        self._last_good_port = port
        self._last_response_time = time.monotonic()
        self._connected = True
        # A fresh port means a possibly-rebooted sensor with a zeroed counter
        self._reset_detection_state()
        self.reactor.register_async_callback(self._resync_after_reconnect)

        self._reader_alive = True
        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        # Ping now so we have a response before the first health check
        self._send("HEALTH")
        logging.info("SmartFilamentSensor '%s': connected to %s"
                     % (self.name, port))
        return True

    def _handle_connect(self):
        if not self._attempt_connect():
            self._connected = False
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': sensor not found. "
                "Plug it in — reconnect is automatic, no restart needed."
                % self.name)

        # Extrusion check timer (250ms for faster response)
        self.reactor.register_timer(
            self._extrusion_check, self.reactor.monotonic() + 2.0)

        # Health check timer (magnet + connection monitoring + reconnect)
        self.reactor.register_timer(
            self._health_check, self.reactor.monotonic() + self.health_check_interval)

    def _start_reconnect(self):
        """Kick off a reconnect attempt on a worker thread."""
        if self._reconnecting or self._shutting_down:
            return
        self._reconnecting = True

        def worker():
            try:
                # full_scan=False: the printer is live, so probing anything
                # beyond the known port risks PINGing an MCU
                if self._attempt_connect(full_scan=False):
                    self.reactor.register_async_callback(
                        lambda et: self.gcode.respond_info(
                            "SmartFilamentSensor '%s': sensor reconnected "
                            "automatically (%s)"
                            % (self.name, self._active_port)))
            except Exception as e:
                logging.error("SmartFilamentSensor '%s': reconnect failed: %s"
                              % (self.name, e))
            finally:
                self._reconnecting = False

        threading.Thread(target=worker, daemon=True).start()

    def _handle_disconnect(self):
        self._shutting_down = True
        self._teardown_serial()
        self._connected = False

    # ── Homing Awareness ─────────────────────────────────────────────────────

    def _handle_homing_begin(self, hmove):
        self._homing = True

    def _handle_homing_end(self, hmove):
        self._homing = False
        # Re-sync after homing to avoid false triggers
        self._reset_detection_state()
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
                        self._reset_detection_state()
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
            # The sensor may have rebooted, which zeroes its encoder counter
            # while Klipper kept accumulating expected extrusion. Throw away
            # everything from before the gap and re-sync both sides, or the
            # mismatch reads as underextrusion and pauses the print.
            self._reset_detection_state()
            self.reactor.register_async_callback(self._resync_after_reconnect)
            logging.info("SmartFilamentSensor '%s': sensor reconnected, "
                         "detection state reset" % self.name)
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

            # Measure mode reply — just report raw mm, no detection logic.
            # Expires: a stale flag would otherwise eat the next detection
            # reply and stall clog detection indefinitely.
            if self._measure_pending:
                if time.monotonic() <= self._measure_deadline:
                    self._measure_pending = False
                    # Leave the odometer at zero so the next detection
                    # window starts clean instead of inheriting the
                    # measurement session's total
                    self._send("RESET_MM")
                    self.reactor.register_async_callback(
                        lambda et, v=actual_mm: self.gcode.respond_info(
                            "SFS MEASURE: %.2f mm measured" % v))
                    return
                self._measure_pending = False

            expected = self._pending_expected
            self._pending_expected = None
            if expected is None:
                return

            # Record sample for underextrusion averaging
            now = time.monotonic()
            # A long gap since the previous sample (travel moves, pauses)
            # means the alarm timer was frozen, not accumulating evidence.
            # Without this reset, one bad sample after a >period gap would
            # satisfy "elapsed >= period" instantly and pause the print off
            # a single reading — the exact thing the timer exists to prevent.
            if (self._extrusion_samples and
                    now - self._extrusion_samples[-1][0]
                    > self.underextrusion_period):
                self._extrusion_samples = []
                self._underextrusion_start_time = None
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
            # Skipped right after a resync: the first samples span the gap
            # and would read as underextrusion.
            if self._enabled and not self._in_resync_grace():
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
            # underextrusion_rate: 0.0 = perfect, 1.0 = total clog.
            # Clamped to 1.0: a rate above 100% is arithmetically impossible
            # for real filament movement (it needs a negative total_actual),
            # so it always means the accounting has desynced rather than the
            # filament having stopped. Reporting "260% underextrusion" hid
            # that. Clamping keeps the number meaningful; the log line below
            # is where a desync shows up.
            rate = 1.0 - (total_actual / total_expected)
            if rate > 1.0:
                logging.warning(
                    "SmartFilamentSensor '%s': encoder read negative net "
                    "movement (expected=%.2f actual=%.2f) - accounting "
                    "desync, not a clog" % (self.name, total_expected,
                                            total_actual))
                rate = 1.0
            self._underextrusion_rate = max(0.0, rate)
        else:
            self._underextrusion_rate = 0.0

    # ── Extrusion Tracker ────────────────────────────────────────────────────

    def _reset_detection_state(self, grace=True):
        """Drop accumulated samples and clear the clog timer.

        Called whenever the ESP32 counter and Klipper's expected extrusion
        can no longer be trusted to line up: sensor disconnect, reconnect
        (a reboot zeroes the encoder counter), or a lost GET_MM_RESET reply
        (that reply's filament movement is gone for good, since the counter
        was already reset on the sensor side).
        """
        self._extrusion_samples = []
        self._underextrusion_rate = 0.0
        self._underextrusion_start_time = None
        self._runout_triggered = False
        self._pending_expected = None
        if grace:
            self._resync_grace_until = (time.monotonic()
                                        + self._resync_grace_period)

    def _in_resync_grace(self):
        return time.monotonic() < self._resync_grace_until

    def _resync_after_reconnect(self, eventtime):
        """Re-align Klipper's E tracking with the sensor counter.

        Runs on the reactor thread — the toolhead must not be queried from
        the serial reader thread.
        """
        self._last_e_pos = self._get_e_pos()
        self._send("RESET_MM")

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

        # No sensor = no data, which is not the same as no filament movement.
        # Keep tracking E so we don't hand a huge stale delta to the next
        # GET_MM_RESET once the sensor comes back.
        if not self._connected:
            self._last_e_pos = self._get_e_pos()
            self._pending_expected = None
            return eventtime + 0.25

        # Measure mode owns the odometer: a detection window would send
        # GET_MM_RESET and wipe the measurement the user is in the middle of.
        # Track E so nothing stale accumulates, open no windows.
        if self._measure_active or self._measure_pending:
            self._last_e_pos = self._get_e_pos()
            return eventtime + 0.25

        if not self._is_printing():
            self._was_printing = False
            self._last_e_pos = self._get_e_pos()
            return eventtime + 0.25

        current_e = self._get_e_pos()
        if current_e is None:
            return eventtime + 0.25

        # Entering the printing state (fresh print, or resume after pause).
        # Anything the encoder counted meanwhile — manual extrudes during a
        # filament change, purge at resume — never entered `expected`, so
        # start both sides from zero instead of comparing across the gap.
        if not self._was_printing:
            self._was_printing = True
            self._reset_detection_state()
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.25

        if self._last_e_pos is None:
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.25

        delta = current_e - self._last_e_pos

        # A large negative jump is a G92 E0 / slicer E reset, not filament
        # moving backwards. Nothing physical happened, so resync both sides.
        if delta < -20.0:
            self._last_e_pos = current_e
            self._send("RESET_MM")
            return eventtime + 0.25

        # Retractions are deliberately NOT special-cased. delta is left to
        # accumulate as *net* movement since the last check, which is exactly
        # what the encoder counts, so a retract and its unretract cancel on
        # both sides.
        #
        # The previous code advanced _last_e_pos on every negative delta,
        # which dropped the retraction from `expected` while leaving it in the
        # encoder's count. The next window then compared, say, -6mm+7mm=+1mm
        # actual against 7mm expected: a phantom ~86% underextrusion that
        # paused prints with no clog present.
        if delta >= self.detection_length:
            # A still-pending request means the previous reply never arrived.
            # The sensor already zeroed its counter for that request, so that
            # window's filament movement is lost — pairing it with the next
            # reply would read as underextrusion. Drop it and resync instead.
            if self._pending_expected is not None:
                logging.warning(
                    "SmartFilamentSensor '%s': GET_MM_RESET reply lost, "
                    "resyncing (no clog decision from this window)"
                    % self.name)
                self._reset_detection_state()
                self._send("RESET_MM")
                self._last_e_pos = current_e
                return eventtime + 0.25

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
            self._reset_detection_state()
            logging.error(
                "SmartFilamentSensor '%s': reader thread crashed, "
                "sensor marked disconnected" % self.name)
            self.reactor.register_async_callback(
                lambda et: self.gcode.respond_info(
                    "SmartFilamentSensor '%s': WARNING - serial reader "
                    "crashed, attempting automatic reconnect..."
                    % self.name))

        # Not connected: retry on a worker thread. A USB re-enumeration
        # invalidates the old handle, so reopening is the only way back —
        # otherwise the sensor stays dead until FIRMWARE_RESTART.
        if not self._connected:
            self._start_reconnect()
            return eventtime + self.health_check_interval

        # Check sensor connection timeout
        if self._connected:
            now = time.monotonic()
            if now - self._last_response_time > self._connection_timeout:
                self._connected = False
                # Samples from before/during the dropout can't be trusted
                self._reset_detection_state()
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
            'version': __version__,
            'enabled': self._enabled,
            'sensor_connected': self._connected,
            'reconnecting': self._reconnecting,
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
                "(port: %s)\nPlug it in — reconnect is automatic (retry every "
                "%.0fs)." % (self.name, port_info, self.health_check_interval))
            return False
        return True

    def cmd_STATUS(self, gcmd):
        # Update connection state
        if self._serial and not self._serial.is_open:
            self._connected = False
        port_info = self._active_port or self.serial_port
        e     = self._get_e_pos()
        last  = self._last_e_pos
        if e is None or last is None:
            since = 0.0
        else:
            since = e - last
        gcmd.respond_info(
            "SmartFilamentSensor '%s'  (v%s):\n"
            "  port=%s  sensor_connected=%s\n"
            "  enabled=%s  printing=%s  homing=%s\n"
            "  magnet=%s (AGC:%d)\n"
            "  detection_length=%.1fmm\n"
            "  underextrusion=%.1f%% (max:%.0f%%, period:%.0fs)\n"
            "  underextrusion_timer=%s\n"
            "  since_last_check=%.2fmm"
            % (self.name, __version__, port_info, self._connected,
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
        self._reset_detection_state()
        self._last_e_pos = self._get_e_pos()
        self._send("RESET_MM")
        gcmd.respond_info("SmartFilamentSensor '%s': re-synced" % self.name)

    def cmd_MEASURE(self, gcmd):
        # Manual measure - works without printing state.
        # 1st call: reset encoder. Extrude/pull filament. 2nd call: show mm.
        if not self._require_connected(gcmd):
            return
        if not self._measure_active:
            # RESET_MM zeroes the same odometer detection windows read from.
            # Clear detection state (with grace) so a window opened before
            # this command can't pair its expected mm with a zeroed counter.
            self._reset_detection_state()
            self._send("RESET_MM")
            self._measure_active = True
            gcmd.respond_info(
                "SFS MEASURE: started (encoder zeroed). Extrude/feed filament, "
                "then run SFS_MEASURE again to read measured mm.")
        else:
            self._measure_active = False
            self._measure_pending = True
            self._measure_deadline = time.monotonic() + 5.0
            # The GET_MM reply must pair with the measure request, not with a
            # detection window that opened in between
            self._reset_detection_state()
            self._send("GET_MM")
            gcmd.respond_info("SFS MEASURE: reading encoder...")

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

    def cmd_AUTO_CALIBRATE(self, gcmd):
        """One-command calibration: heat hotend, extrude, auto-save."""
        if not self._require_connected(gcmd):
            return
        temp = gcmd.get_float('TEMP', 240., above=0.)
        length = gcmd.get_float('LENGTH', 50., above=0.)
        speed = gcmd.get_float('SPEED', 100., above=0.)  # mm/min

        gcmd.respond_info(
            "SmartFilamentSensor '%s': AUTO CALIBRATE\n"
            "  Heating to %.0fC, then extruding %.0fmm at F%.0f\n"
            "  Please wait..."
            % (self.name, temp, length, speed))

        try:
            # Heat and wait
            self.gcode.run_script_from_command(
                "M109 S%.0f" % temp)

            # Start calibration on ESP32
            self._calibrating = True
            self._send("START %.1f" % length)
            gcmd.respond_info(
                "SmartFilamentSensor '%s': hotend ready, extruding %.0fmm..."
                % (self.name, length))

            # Extrude (relative mode)
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(
                "G1 E%.1f F%.0f" % (length, speed))

            # Wait for filament to stop + auto-save (6s should be enough)
            self.gcode.run_script_from_command("G4 P6000")

            # If still calibrating (auto-save didn't trigger), force apply
            if self._calibrating:
                self._send("APPLY")
                gcmd.respond_info(
                    "SmartFilamentSensor '%s': extrusion complete, "
                    "applying calibration..." % self.name)
                # Give ESP32 time to process
                self.gcode.run_script_from_command("G4 P2000")

            gcmd.respond_info(
                "SmartFilamentSensor '%s': auto calibration finished!\n"
                "Check the result above. Run SFS_STATUS to verify."
                % self.name)
        except Exception as e:
            self._calibrating = False
            gcmd.respond_info(
                "SmartFilamentSensor '%s': auto calibration failed: %s"
                % (self.name, e))


def load_config(config):
    return SmartFilamentSensor(config)

def load_config_prefix(config):
    return SmartFilamentSensor(config)
