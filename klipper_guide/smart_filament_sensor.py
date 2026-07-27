# smart_filament_sensor.py — Klipper klippy extra module
#
# ═══════════════════════════════════════════════════════════════════════════
#  SMART FILAMENT SENSOR — Klipper Native Module
#  Version: see __version__ below (reported by SFS_STATUS)
# ═══════════════════════════════════════════════════════════════════════════
#
#  ESP32 is a pure measurement device. It only reports how many mm of
#  filament the encoder measured. All clog detection logic lives here.
#
# ─── WHY THE DETECTION WINDOW IS MEASURED IN mm, NOT SECONDS ──────────────
#
#   This sensor is frame-mounted and connected to the toolhead by a bowden
#   tube (PC4-M6 fittings). The filament between the sensor and the extruder
#   is therefore NOT rigidly coupled: as the toolhead moves, the path length
#   changes and the filament is dragged back and forth through the encoder.
#   Buckling inside the tube stores and releases even more length.
#
#   Measured on real prints (klippy logs, July 2026): individual detection
#   windows read anywhere from -13.8mm to +17.2mm while the commanded
#   extrusion was ~7mm and the configured retraction was only 0.8mm. This
#   drag is bounded and averages to zero over distance, but over a handful of
#   windows it looks exactly like a clog.
#
#   The old detector averaged over `underextrusion_period` SECONDS. Window
#   spacing on a real print is ~2s median, so a 5s average contained a median
#   of THREE samples — and 9% of the time a single one. A pause was therefore
#   decided on 2-3 readings. In the log that triggered a false clog, the
#   deciding average was literally two windows: expected=15.62 actual=-9.07.
#
#   Averaging over EXTRUDED DISTANCE fixes this at the root. Drag amplitude
#   is bounded in mm, so its relative contribution shrinks as the window
#   grows, while a real clog reads ~100% deficit at every window size.
#   Replaying the same logs with perfect calibration, the worst drag-only
#   excursion was:
#
#        window    worst false underextrusion
#         25 mm            260 %
#         50 mm            106 %
#        100 mm             88 %
#        150 mm             58 %
#        200 mm             41 %
#        300 mm             26 %
#
#   At 200mm the 50% threshold has real margin, and a total clog is still
#   caught after ~115mm of commanded extrusion (~25s at typical flow).
#
# ─── CALIBRATION MATTERS MORE THAN IT USED TO ─────────────────────────────
#
#   A distance-averaged detector integrates a steady calibration error
#   instead of hiding it. Both analysed logs showed the encoder reading only
#   61-63% of the commanded extrusion — a permanent 37-39% "underextrusion"
#   that ate three quarters of the headroom to the trigger threshold.
#
#   This module now measures that ratio continuously and reports it
#   (SFS_STATUS -> encoder_gain). If it is not close to 1.00, detection is
#   NOT armed and the module says so, instead of pausing prints for a
#   calibration problem. Fix it with SFS_AUTO_CALIBRATE, or compensate a
#   known steady bias with `encoder_scale`.
#
# ─── PROTOCOL ─────────────────────────────────────────────────────────────
#
#   Klipper → ESP32:  "GET_MM_RESET\n"   read encoder mm + atomically reset
#   ESP32   → Klipper: "MM:<float>\n"    mm measured since last reset
#
#   Klipper → ESP32:  "GET_MM\n"         read encoder mm (no reset)
#   Klipper → ESP32:  "RESET_MM\n"       reset counter only
#   Klipper → ESP32:  "HEALTH\n"         query magnet health
#   ESP32   → Klipper: "HEALTH:<state>:<agc>\n"
#                                        ok, too_weak, too_strong, no_magnet
#   Klipper → ESP32:  "GET_CAL\n"        query calibration factor  (fw >=2.4)
#   Klipper → ESP32:  "GET_STEPS\n"      raw encoder steps         (fw >=2.4)
#   ESP32   → Klipper: "CAL:<float>\n"   deg/mm currently stored
#   Klipper → ESP32:  "DIAG\n"           dump AS5600 registers     (fw >=2.4)
#   ESP32   → Klipper: "DIAG:<k=v,...>\n"
#
#   Klipper → ESP32:  "START <mm>\n"     start calibration with target mm
#   Klipper → ESP32:  "APPLY\n"          force-save calibration immediately
#   Klipper → ESP32:  "STOP\n"           cancel calibration / measure
#   Klipper → ESP32:  "SET SENS|NOISE|BRIGHT|DIR|CAL <v>\n"
#
# ─── INSTALLATION ─────────────────────────────────────────────────────────
#
#   1. Upload this file via Mainsail/Fluidd (Machine tab)
#   2. SSH:  mv ~/printer_data/config/smart_filament_sensor.py \
#                ~/klipper/klippy/extras/
#      mv, not cp -- a leftover copy in the config folder shows up in the
#      Mainsail/Fluidd file list as if it were a config file, and editing
#      that copy does nothing: Klipper loads the one in klippy/extras/.
#   3. Add the config section below to printer.cfg
#   4. sudo systemctl restart klipper
#
# ─── printer.cfg ──────────────────────────────────────────────────────────
#
#   [smart_filament_sensor sfs]
#   serial: /dev/serial/by-id/usb-Espressif_...   # fixed path recommended
#   baud: 115200
#   detection_length: 7.0          # mm of extrusion per encoder read
#   detection_window: 200.0        # mm of extrusion averaged per decision
#   underextrusion_max_rate: 0.5   # 0.0-1.0 deficit that counts as a clog
#   min_window_samples: 8          # reads required before any decision
#   encoder_scale: 1.0             # multiplies encoder mm (bias correction)
#   gain_tolerance: 0.25           # |gain-1| above this disarms detection
#   pause_on_runout: True
#   runout_gcode: PAUSE
#   health_check_interval: 30.0
#
# ─── GCODE COMMANDS ───────────────────────────────────────────────────────
#
#     SFS_STATUS          Show state, health, encoder gain, window fill
#     SFS_ENABLE          Enable clog detection
#     SFS_DISABLE         Disable clog detection (sensor stays connected)
#     SFS_RESET           Re-sync extruder position and reset ESP32 odometer
#     SFS_DIAG            Dump ESP32 calibration + AS5600 registers
#
#     SFS_AUTO_CALIBRATE [TEMP=240] [LENGTH=50] [SPEED=100]
#     SFS_CALIBRATE [LENGTH=10] / SFS_CALIBRATE_APPLY / SFS_CALIBRATE_STOP
#     SFS_SET SENS= NOISE= BRIGHT= DIR= CAL=
#     SFS_MEASURE         1st call resets, extrude, 2nd call shows measured mm
#     SFS_SUSPEND / SFS_RESUME   Silence the module during probing (nestable)
#
# ═══════════════════════════════════════════════════════════════════════════

import serial
import threading
import logging
import time
import glob
import os

__version__ = "3.1.3"

# Expected PING→PONG identifier for auto-detect (avoids grabbing CAN/EBB ports)
_SFS_IDENTITY = "PONG"


class SmartFilamentSensor:
    def __init__(self, config):
        self.printer  = config.get_printer()
        self.reactor  = self.printer.get_reactor()
        self.gcode    = self.printer.lookup_object('gcode')
        self.name     = config.get_name().split()[-1]

        # ── Config ───────────────────────────────────────────────────────
        self.serial_port      = config.get('serial', 'auto')
        self.baud_rate        = config.getint('baud', 115200)
        self.detection_length = config.getfloat('detection_length', 7.0, above=0.)
        self.pause_on_runout  = config.getboolean('pause_on_runout', True)
        self.runout_gcode     = config.get('runout_gcode', 'PAUSE')
        self.underextrusion_max_rate = config.getfloat(
            'underextrusion_max_rate', 0.5, minval=0., maxval=1.)
        self.health_check_interval = config.getfloat(
            'health_check_interval', 30.0, above=0.)

        # Rolling decision window, measured in mm of COMMANDED extrusion.
        # See the header for why this is not a time window.
        self.detection_window = config.getfloat(
            'detection_window', 200.0, above=0.)
        # Independent floor on evidence count. detection_window alone can be
        # satisfied by very few reads if detection_length is large.
        self.min_window_samples = config.getint(
            'min_window_samples', 8, minval=2)

        # Steady multiplicative correction applied to every encoder reading.
        # Use only for a bias you have measured and cannot remove by
        # calibrating the ESP32 itself.
        self.encoder_scale = config.getfloat(
            'encoder_scale', 1.0, above=0.)
        # How far the measured long-run gain may drift from 1.0 before the
        # module refuses to make clog decisions.
        self.gain_tolerance = config.getfloat(
            'gain_tolerance', 0.25, above=0., maxval=1.)

        # Accepted but no longer used for the averaging window; kept so old
        # configs keep loading. Reused as the staleness gap below.
        self.sample_gap_timeout = config.getfloat(
            'underextrusion_period', 30.0, above=0.)

        # ── Runtime state ────────────────────────────────────────────────
        self._serial           = None
        self._serial_lock      = threading.Lock()
        self._enabled          = True
        self._last_e_pos       = None
        self._pending_expected = None
        self._measure_active   = False
        self._measure_pending  = False
        self._measure_deadline = 0.0
        self._calibrating      = False
        self._active_port      = None
        self._last_good_port   = None

        self._connected        = False
        self._last_response_time = 0.0
        self._connection_timeout = self.health_check_interval + 10.0

        self._magnet_state     = 'unknown'
        self._magnet_agc       = 0
        self._magnet_warned    = False
        self._esp_cal          = None   # deg/mm reported by the ESP32
        self._esp_fw           = None   # firmware version string from DIAG
        self._diag_pending     = False  # waiting for a DIAG reply

        # ── Detection window ─────────────────────────────────────────────
        # Each entry is (monotonic_time, expected_mm, actual_mm).
        self._window            = []
        self._window_expected   = 0.0   # running sum, kept in step with list
        self._window_actual     = 0.0
        self._underextrusion_rate = 0.0
        self._runout_triggered  = False
        self._last_sample_time  = 0.0

        # ── Long-run gain measurement ────────────────────────────────────
        # Lifetime totals since the last SFS_RESET / print start. This is the
        # number that tells you whether the ESP32 calibration is right.
        self._gain_expected    = 0.0
        self._gain_actual      = 0.0
        self._measured_gain    = None   # None until enough data
        self._gain_warned      = False
        # Enough commanded extrusion for the ratio to be meaningful.
        #
        # Tied to detection_window on purpose. Gain accumulates from the same
        # readings as the decision window and is never cleared by a resync, so
        # matching the two guarantees the gain guard is live the moment the
        # window first arms. When this was a larger fixed number there was a
        # band -- window full at 200mm, gain still "measuring" until 300mm --
        # where _gain_ok() returned True for lack of data and a grossly wrong
        # reading could pause the print. A reversed encoder direction (gain
        # -1.0) walked straight through that gap.
        self._gain_min_mm      = self.detection_window * 0.9

        # Grace period after reconnect/resync
        self._resync_grace_until = 0.0
        self._resync_grace_period = 5.0

        # Homing / probing awareness
        self._homing           = False
        self._probe_clear_at   = None
        self._probe_grace      = 0.75
        self._suspend_depth    = 0

        self._was_printing     = False

        # Calibration sanity range for deg/mm
        self._cal_min = 3.0
        self._cal_max = 50.0

        # Reader thread tracking
        self._reader_alive     = False
        self._reader_died      = False
        self._reader_thread    = None

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
        self.gcode.register_command('SFS_DIAG',    self.cmd_DIAG,
            desc="Dump ESP32 calibration factor and AS5600 registers")
        self.gcode.register_command('SFS_CALIBRATE', self.cmd_CALIBRATE,
            desc="Start ESP32 encoder calibration. Usage: SFS_CALIBRATE [LENGTH=10]")
        self.gcode.register_command('SFS_CALIBRATE_APPLY', self.cmd_CALIBRATE_APPLY,
            desc="Force-save calibration result immediately")
        self.gcode.register_command('SFS_CALIBRATE_STOP', self.cmd_CALIBRATE_STOP,
            desc="Cancel active calibration")
        self.gcode.register_command('SFS_SET', self.cmd_SET,
            desc="Change ESP32 settings. Usage: SFS_SET [SENS=] [NOISE=] [BRIGHT=] [DIR=] [CAL=]")
        self.gcode.register_command('SFS_AUTO_CALIBRATE', self.cmd_AUTO_CALIBRATE,
            desc="Auto calibrate: heat, extrude, save.")
        self.gcode.register_command('SFS_MEASURE', self.cmd_MEASURE,
            desc="Measure mode: 1st call resets, extrude, 2nd call shows measured mm")
        self.gcode.register_command('SFS_SUSPEND', self.cmd_SUSPEND,
            desc="Fully silence the module (no serial I/O) during probing. Nestable.")
        self.gcode.register_command('SFS_RESUME', self.cmd_RESUME,
            desc="Lift SFS_SUSPEND; resyncs when the last nested suspend clears")

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
        """Find the ESP32 serial port via a PING/PONG handshake."""
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

        if self._last_good_port and usable(self._last_good_port):
            if self._verify_sensor(self._last_good_port):
                return self._last_good_port

        if not full_scan:
            return None

        candidates = [p for p in sorted(glob.glob('/dev/serial/by-id/*'))
                      if any(k in p.lower()
                             for k in ('esp', 'cp210', 'ch340'))]
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
            test.reset_input_buffer()
            time.sleep(0.1)
            test.reset_input_buffer()
            test.write(b'PING\n')
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
            if test is not None:
                try:
                    test.close()
                except Exception:
                    pass
        return False

    def _teardown_serial(self):
        """Close the port and wind down the reader thread.

        Order matters: clear _reader_alive first so the reader stops looping,
        then close the handle (which unblocks a parked readline()), then join
        before touching connection state — otherwise the dying thread's
        cleanup clobbers a fresh connection.
        """
        self._reader_alive = False
        ser = self._serial
        self._serial = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._reader_thread = None
        self._reader_died = False

    def _attempt_connect(self, full_scan=True):
        """Open the sensor port and start reading. Returns True on success."""
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
            ser = serial.Serial(port, self.baud_rate, timeout=0.1)
        except Exception as e:
            logging.warning("SmartFilamentSensor '%s': cannot open %s: %s"
                            % (self.name, port, e))
            self._serial = None
            return False

        self._serial = ser
        self._active_port = port
        self._last_good_port = port
        self._last_response_time = time.monotonic()
        self._connected = True
        self._reset_detection_state()
        self.reactor.register_async_callback(self._resync_after_reconnect)

        self._reader_alive = True
        self._reader_thread = threading.Thread(
            target=self._serial_reader, args=(ser,), daemon=True)
        self._reader_thread.start()

        self._send("HEALTH")
        self._send("GET_CAL")
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

        self.reactor.register_timer(
            self._extrusion_check, self.reactor.monotonic() + 2.0)
        self.reactor.register_timer(
            self._health_check, self.reactor.monotonic() + self.health_check_interval)
        self.reactor.register_timer(
            self._probe_clear_check, self.reactor.monotonic() + 1.0)

    def _start_reconnect(self):
        """Kick off a reconnect attempt on a worker thread."""
        if self._reconnecting or self._shutting_down:
            return
        self._reconnecting = True

        def worker():
            try:
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
        # Suspend ALL module activity for the whole homing/probing window.
        # During a probe the host must service trsync within microseconds; any
        # serial write or get_position() from this module can delay the
        # reactor enough to cause "Unable to obtain trsync_state response".
        self._homing = True
        self._probe_clear_at = None

    def _handle_homing_end(self, hmove):
        # z_tilt / bed mesh fire begin/end per probe point. Stay suspended
        # through the gaps; a reactor timer lifts it once moves actually stop.
        self._probe_clear_at = time.monotonic() + self._probe_grace

    def _probe_clear_check(self, eventtime):
        if self._homing and self._probe_clear_at is not None:
            if time.monotonic() >= self._probe_clear_at:
                self._homing = False
                self._probe_clear_at = None
                if self._suspend_depth == 0:
                    self._reset_detection_state()
                    self._last_e_pos = self._get_e_pos()
                    self._send("RESET_MM")
        return eventtime + 0.1

    # ── Serial I/O ───────────────────────────────────────────────────────────

    def _send(self, cmd):
        ser = self._serial
        if ser is None:
            return
        with self._serial_lock:
            try:
                if ser.is_open:
                    ser.write((cmd + '\n').encode())
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

    def _serial_reader(self, ser):
        """Background thread — reads lines from the ESP32.

        Takes its own serial handle as an argument rather than reading
        self._serial each iteration. _teardown_serial() sets that attribute
        to None, and the old code could evaluate `self._serial.is_open` as
        True and then call readline() on None — the source of the recurring
        "read error: 'NoneType' object cannot be interpreted as an integer"
        followed by "reader thread died!" on every FIRMWARE_RESTART.
        """
        try:
            while self._reader_alive:
                try:
                    if not ser.is_open:
                        break
                    raw = ser.readline()
                except (serial.SerialException, OSError, TypeError,
                        ValueError, AttributeError) as e:
                    # Port closed under us during shutdown — expected.
                    if self._reader_alive:
                        logging.error(
                            "SmartFilamentSensor '%s': serial error: %s"
                            % (self.name, e))
                    break
                if not raw:
                    continue
                try:
                    line = raw.decode('utf-8', errors='replace').strip()
                except Exception:
                    continue
                if line:
                    try:
                        self._handle_line(line)
                    except Exception as e:
                        logging.error(
                            "SmartFilamentSensor '%s': line handler error: %s"
                            % (self.name, e))
        finally:
            self._reader_alive = False
            # Only flag a crash if nobody asked us to stop.
            if self._connected and not self._shutting_down:
                self._connected = False
                self._reader_died = True
                logging.error(
                    "SmartFilamentSensor '%s': reader thread died!" % self.name)

    def _handle_line(self, line):
        """Called from the reader thread. Keep it cheap — no disk logging per
        line (that stalls the reactor thread if it lands during a probe)."""
        self._last_response_time = time.monotonic()
        if not self._connected:
            self._connected = True
            self._reader_died = False
            # The sensor may have rebooted, which zeroes its encoder counter
            # while Klipper kept accumulating expected extrusion. Throw away
            # everything from before the gap and re-sync both sides.
            self._reset_detection_state()
            self.reactor.register_async_callback(self._resync_after_reconnect)
            logging.info("SmartFilamentSensor '%s': sensor reconnected, "
                         "detection state reset" % self.name)
            self.reactor.register_async_callback(
                lambda et: self.gcode.respond_info(
                    "SmartFilamentSensor '%s': sensor reconnected" % self.name))

        if line.startswith("HEALTH:"):
            self._handle_health(line[7:])
            return

        if line.startswith("CAL:"):
            try:
                self._esp_cal = float(line[4:])
            except ValueError:
                pass
            return

        if line.startswith("DIAG:"):
            self._diag_pending = False
            # The dump starts with fw=<version>; remember it so SFS_STATUS can
            # show which firmware is actually running.
            for field in line[5:].split(','):
                key, _, value = field.partition('=')
                if key == 'fw':
                    self._esp_fw = value
                    break
            msg = line
            self.reactor.register_async_callback(
                lambda et, m=msg: self.gcode.respond_info(
                    "SmartFilamentSensor '%s': %s" % (self.name, m)))
            return

        # Forward calibration output to the Klipper console
        if self._calibrating:
            if (line.startswith("[CAL]") or
                    line.startswith(">>> CALIBRATION") or
                    line.startswith(">>> Measured:") or
                    line.startswith(">>> New Cal Factor:") or
                    line.startswith(">>> ERROR:")):
                msg = line
                self.reactor.register_async_callback(
                    lambda et, m=msg: self.gcode.respond_info(m))
                if ">>> New Cal Factor:" in line:
                    self._validate_calibration(line)
                if "CALIBRATION SUCCESS" in line or ">>> ERROR:" in line:
                    self._calibrating = False
                    # A new cal factor invalidates every accumulated ratio
                    self._reset_gain_tracking()
                    self._send("GET_CAL")
                return

        if line.startswith("MM:") and line != "MM:RESET":
            try:
                actual_mm = float(line[3:])
            except ValueError:
                return
            self._handle_measurement(actual_mm)

    def _handle_health(self, health):
        # Firmware >= 2.5.0 sends "<state>:<agc>" for every state; older
        # firmware sent a bare "too_weak" and no too_strong state at all.
        state, _, agc_txt = health.partition(':')
        if agc_txt:
            try:
                self._magnet_agc = int(agc_txt)
            except ValueError:
                self._magnet_agc = 0

        if state == 'ok':
            self._magnet_state = 'ok'
        elif state == 'no_magnet':
            self._magnet_state = 'no_magnet'
            self._magnet_agc = 0
            logging.warning(
                "SmartFilamentSensor '%s': NO MAGNET DETECTED!" % self.name)
            if self._enabled and self._is_printing():
                self.reactor.register_async_callback(self._action_magnet_error)
        elif state in ('too_weak', 'too_strong'):
            self._magnet_state = state
            logging.warning("SmartFilamentSensor '%s': magnet %s (AGC %d)"
                            % (self.name, state, self._magnet_agc))
            if not self._magnet_warned:
                self._magnet_warned = True
                hint = ("Move the magnet CLOSER." if state == 'too_weak'
                        else "Increase the AIR GAP -- the magnet is too close.")
                self.reactor.register_async_callback(
                    lambda et, s=state, a=self._magnet_agc, h=hint:
                    self.gcode.respond_info(
                        "SmartFilamentSensor '%s': magnet %s (AGC %d).\n"
                        "  %s\n"
                        "  Ideal AGC is the middle of its range: ~64 at 3.3V "
                        "(0-128), ~128 at 5V (0-255).\n"
                        "  Both extremes degrade angle accuracy."
                        % (self.name, s, a, h)))
            return
        self._magnet_warned = False

    def _handle_measurement(self, actual_mm):
        """A GET_MM / GET_MM_RESET reply arrived."""
        actual_mm *= self.encoder_scale

        # Measure mode reply — just report raw mm, no detection logic.
        # Expires: a stale flag would otherwise eat the next detection reply
        # and stall clog detection indefinitely.
        if self._measure_pending:
            if time.monotonic() <= self._measure_deadline:
                self._measure_pending = False
                # Leave the odometer at zero so the next detection window
                # starts clean instead of inheriting the measurement total
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

        now = time.monotonic()

        # A long gap means the window spans a pause, a filament change, or a
        # travel-only stretch. Evidence from before the gap is not comparable
        # to evidence after it, so start clean.
        if self._window and now - self._last_sample_time > self.sample_gap_timeout:
            self._clear_window()
        self._last_sample_time = now

        self._window.append((now, expected, actual_mm))
        self._window_expected += expected
        self._window_actual += actual_mm

        # Trim from the front until the window holds at most detection_window
        # mm of commanded extrusion — but never drop below min_window_samples.
        while (self._window_expected > self.detection_window
               and len(self._window) > self.min_window_samples):
            _, e_old, a_old = self._window.pop(0)
            self._window_expected -= e_old
            self._window_actual -= a_old

        # Long-run gain tracking (independent of the decision window)
        self._gain_expected += expected
        self._gain_actual += actual_mm
        if self._gain_expected >= self._gain_min_mm:
            self._measured_gain = self._gain_actual / self._gain_expected

        self._update_underextrusion_rate()

        logging.info(
            "SmartFilamentSensor '%s': expected=%.2fmm actual=%.2fmm "
            "rate=%.0f%% | window %.0f/%.0fmm n=%d underextrusion=%.1f%% "
            "gain=%s"
            % (self.name, expected, actual_mm,
               (actual_mm / expected * 100) if expected > 0 else 100.0,
               self._window_expected, self.detection_window, len(self._window),
               self._underextrusion_rate * 100,
               ("%.3f" % self._measured_gain) if self._measured_gain
               else "n/a"))

        if self._enabled and not self._in_resync_grace():
            self._check_underextrusion()

    # ── Detection ────────────────────────────────────────────────────────────

    def _update_underextrusion_rate(self):
        if self._window_expected <= 0:
            self._underextrusion_rate = 0.0
            return
        rate = 1.0 - (self._window_actual / self._window_expected)
        if rate > 1.0:
            # Net negative encoder movement across the whole window. With a
            # 200mm window this needs sustained backward drag; log it and
            # clamp so the reported number stays physical.
            logging.warning(
                "SmartFilamentSensor '%s': window net movement negative "
                "(expected=%.2f actual=%.2f)"
                % (self.name, self._window_expected, self._window_actual))
            rate = 1.0
        self._underextrusion_rate = max(0.0, rate)

    def _gain_ok(self):
        """True when the encoder tracks the extruder well enough to judge.

        A steady calibration error is indistinguishable from a partial clog
        inside a single window, so refuse to decide rather than guess.
        """
        if self._measured_gain is None:
            return True   # not enough data yet; window fill gates us anyway
        return abs(self._measured_gain - 1.0) <= self.gain_tolerance

    def _window_ready(self):
        return (len(self._window) >= self.min_window_samples
                and self._window_expected >= self.detection_window * 0.9)

    def _check_underextrusion(self):
        """Distance-averaged clog detection.

        No time-based persistence timer: the window itself IS the evidence.
        It only makes a decision once it holds detection_window mm of
        commanded extrusion across at least min_window_samples reads, which
        on a real print is tens of seconds of averaging — long enough for
        bowden drag to cancel out.
        """
        if not self._window_ready():
            return

        # Never decide before the gain is known. _window_ready() uses a 0.9
        # factor, so without this an arming window could briefly outrun the
        # gain measurement and judge a reading the guard below would reject.
        if self._measured_gain is None:
            return

        if not self._gain_ok():
            if not self._gain_warned:
                self._gain_warned = True
                g = self._measured_gain
                # Three distinct faults land here and they need different
                # first moves, so name the likely one instead of always
                # saying "calibration".
                if g < -0.5:
                    hint = ("The encoder is counting BACKWARDS. Reverse it "
                            "with SFS_SET DIR=-1 (or DIR=1), then SFS_RESET.")
                elif abs(g) < 0.1:
                    hint = ("The encoder is reading almost nothing. Check "
                            "wiring, magnet position and wheel contact with "
                            "SFS_DIAG before trusting any reading.")
                else:
                    hint = ("Recalibrate with SFS_AUTO_CALIBRATE LENGTH=100, "
                            "then SFS_RESET. If it comes back low again, the "
                            "wheel is slipping — check spring pressure and "
                            "O-ring grip.")
                self.reactor.register_async_callback(
                    lambda et, v=g, h=hint: self.gcode.respond_info(
                        "SmartFilamentSensor '%s': clog detection NOT ARMED.\n"
                        "  Encoder reads %.0f%% of commanded extrusion "
                        "(should be ~100%%).\n"
                        "  This is a sensor/calibration problem, not a clog — "
                        "the module will not pause the print for it.\n"
                        "  %s" % (self.name, v * 100, h)))
            return
        self._gain_warned = False

        if self._underextrusion_rate <= self.underextrusion_max_rate:
            self._runout_triggered = False
            return

        if self._runout_triggered:
            return
        self._runout_triggered = True
        logging.warning(
            "SmartFilamentSensor '%s': CLOG CONFIRMED - underextrusion "
            "%.1f%% averaged over %.0fmm of commanded extrusion (%d reads)"
            % (self.name, self._underextrusion_rate * 100,
               self._window_expected, len(self._window)))
        if self.pause_on_runout:
            self.reactor.register_async_callback(self._action_clog)

    def _validate_calibration(self, line):
        """Check that a calibration result is physically plausible."""
        try:
            parts = line.split(':')
            if len(parts) >= 2:
                val_str = parts[-1].strip().split()[0]
                cal_value = float(val_str)
                if cal_value < self._cal_min or cal_value > self._cal_max:
                    self.reactor.register_async_callback(
                        lambda et, v=cal_value: self.gcode.respond_info(
                            "SmartFilamentSensor '%s': WARNING - calibration "
                            "result %.2f deg/mm is unusual (expected %.0f-%.0f).\n"
                            "Check: magnet position, encoder direction "
                            "(SFS_SET DIR=-1), or re-calibrate with longer "
                            "extrusion."
                            % (self.name, v, self._cal_min, self._cal_max)))
        except (ValueError, IndexError):
            pass

    # ── Extrusion Tracker ────────────────────────────────────────────────────

    def _clear_window(self):
        self._window = []
        self._window_expected = 0.0
        self._window_actual = 0.0
        self._underextrusion_rate = 0.0

    def _reset_gain_tracking(self):
        self._gain_expected = 0.0
        self._gain_actual = 0.0
        self._measured_gain = None
        self._gain_warned = False

    def _reset_detection_state(self, grace=True):
        """Drop accumulated evidence.

        Called whenever the ESP32 counter and Klipper's expected extrusion can
        no longer be trusted to line up: disconnect, reconnect (a reboot
        zeroes the encoder counter), or a lost GET_MM_RESET reply (that
        reply's movement is gone for good — the counter was already reset on
        the sensor side).

        Gain tracking deliberately survives this: it is a property of the
        hardware calibration, not of window alignment, and throwing it away
        on every probe would mean it never accumulates enough data.
        """
        self._clear_window()
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

    def _silenced(self):
        """True while the module must not touch serial or the toolhead."""
        return self._homing or self._suspend_depth > 0

    def _extrusion_check(self, eventtime):
        # Fully inert during homing/probing/suspension: no get_position(), no
        # serial. This is the reactor-safety guard that stops the module from
        # delaying trsync during a probe (EBBCan "Missed scheduling").
        if self._silenced():
            return eventtime + 0.25

        # No sensor = no data, which is not the same as no filament movement.
        # Keep tracking E so we don't hand a huge stale delta to the next
        # GET_MM_RESET once the sensor comes back.
        if not self._connected:
            self._last_e_pos = self._get_e_pos()
            self._pending_expected = None
            return eventtime + 0.25

        # Measure mode owns the odometer: a detection window would send
        # GET_MM_RESET and wipe the measurement in progress.
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
        # filament change, purge at resume — never entered `expected`.
        if not self._was_printing:
            self._was_printing = True
            self._reset_detection_state()
            self._reset_gain_tracking()
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

        # Retractions are deliberately NOT special-cased. delta accumulates as
        # *net* movement since the last check, which is exactly what the
        # encoder counts, so a retract and its unretract cancel on both sides.
        if delta >= self.detection_length:
            # A still-pending request means the previous reply never arrived.
            # The sensor already zeroed its counter for that request, so that
            # window's movement is lost — pairing it with the next reply would
            # read as underextrusion. Drop it and resync instead.
            if self._pending_expected is not None:
                logging.warning(
                    "SmartFilamentSensor '%s': GET_MM_RESET reply lost, "
                    "resyncing (no clog decision from this window)"
                    % self.name)
                self._reset_detection_state()
                self._send("RESET_MM")
                self._last_e_pos = current_e
                return eventtime + 0.25

            self._pending_expected = delta
            self._send("GET_MM_RESET")
            self._last_e_pos = current_e

        return eventtime + 0.25

    # ── Health Check Timer ───────────────────────────────────────────────────

    def _health_check(self, eventtime):
        if self._silenced():
            return eventtime + 1.0

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
                    "crashed, attempting automatic reconnect..." % self.name))

        # Not connected: retry on a worker thread. A USB re-enumeration
        # invalidates the old handle, so reopening is the only way back.
        if not self._connected:
            self._start_reconnect()
            return eventtime + self.health_check_interval

        now = time.monotonic()
        if now - self._last_response_time > self._connection_timeout:
            self._connected = False
            self._reset_detection_state()
            logging.warning(
                "SmartFilamentSensor '%s': sensor disconnected "
                "(no response for %.0fs)"
                % (self.name, self._connection_timeout))
            self.reactor.register_async_callback(
                lambda et: self.gcode.respond_info(
                    "SmartFilamentSensor '%s': WARNING - sensor "
                    "disconnected! Check USB connection." % self.name))

        if self._is_printing() and self._connected:
            if self._magnet_state == 'unknown':
                self._send("HEALTH")
            if self._esp_cal is None:
                self._send("GET_CAL")

        self._send("HEALTH")
        return eventtime + self.health_check_interval

    # ── Gcode Actions ────────────────────────────────────────────────────────

    def _action_clog(self, eventtime):
        logging.info("SmartFilamentSensor '%s': executing runout gcode"
                     % self.name)
        try:
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': CLOG/RUNOUT detected!\n"
                "  Underextrusion %.1f%% averaged over %.0fmm of commanded "
                "extrusion (%d reads).\n"
                "  Pausing print..."
                % (self.name, self._underextrusion_rate * 100,
                   self._window_expected, len(self._window)))
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
            'detection_length': self.detection_length,
            'detection_window': self.detection_window,
            'window_filled_mm': round(self._window_expected, 1),
            'window_samples': len(self._window),
            'detection_armed': self._window_ready() and self._gain_ok(),
            'encoder_gain': (round(self._measured_gain, 4)
                             if self._measured_gain is not None else None),
            'encoder_scale': self.encoder_scale,
            'esp_cal_deg_per_mm': self._esp_cal,
            'esp_firmware': self._esp_fw,
            'is_printing': self._is_printing(),
            'is_homing': self._homing,
        }

    # ── GCode Commands ───────────────────────────────────────────────────────

    def _require_connected(self, gcmd):
        if self._serial is not None and not self._serial.is_open:
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
        if self._serial is not None and not self._serial.is_open:
            self._connected = False
        port_info = self._active_port or self.serial_port
        e     = self._get_e_pos()
        last  = self._last_e_pos
        since = 0.0 if (e is None or last is None) else e - last

        if self._measured_gain is None:
            gain_txt = ("measuring... (%.0f/%.0f mm)"
                        % (self._gain_expected, self._gain_min_mm))
        else:
            gain_txt = "%.3f" % self._measured_gain
            if not self._gain_ok():
                gain_txt += "  <== OUT OF RANGE, detection disarmed"

        if not self._window_ready():
            armed = ("filling window (%.0f/%.0f mm, %d/%d reads)"
                     % (self._window_expected, self.detection_window,
                        len(self._window), self.min_window_samples))
        elif not self._gain_ok():
            armed = "NO - encoder gain out of range"
        elif not self._enabled:
            armed = "NO - detection disabled"
        else:
            armed = "YES"

        gcmd.respond_info(
            "SmartFilamentSensor '%s'  (v%s):\n"
            "  port=%s  sensor_connected=%s  esp_fw=%s\n"
            "  enabled=%s  printing=%s  homing=%s\n"
            "  magnet=%s (AGC:%d, ideal ~64 at 3.3V)  esp_cal=%s deg/mm\n"
            "  detection_length=%.1fmm  detection_window=%.0fmm\n"
            "  encoder_gain=%s  (encoder_scale=%.3f)\n"
            "  underextrusion=%.1f%%  (trips above %.0f%%)\n"
            "  armed=%s\n"
            "  since_last_check=%.2fmm"
            % (self.name, __version__, port_info, self._connected,
               self._esp_fw or "?",
               self._enabled, self._is_printing(), self._homing,
               self._magnet_state, self._magnet_agc,
               ("%.4f" % self._esp_cal) if self._esp_cal else "?",
               self.detection_length, self.detection_window,
               gain_txt, self.encoder_scale,
               self._underextrusion_rate * 100,
               self.underextrusion_max_rate * 100,
               armed, since))

    def cmd_DIAG(self, gcmd):
        if not self._require_connected(gcmd):
            return
        self._diag_pending = True
        self._send("GET_CAL")
        self._send("DIAG")
        gcmd.respond_info(
            "SmartFilamentSensor '%s': reading diagnostics..." % self.name)
        # Warn only if the sensor actually stays silent. The old code printed
        # "if nothing follows, your firmware is too old" unconditionally,
        # which is confusing to read directly above a DIAG dump that did
        # arrive — and doubly so once the firmware is newer than the version
        # named in the warning.
        self.reactor.register_timer(
            self._diag_timeout, self.reactor.monotonic() + 2.0)

    def _diag_timeout(self, eventtime):
        if self._diag_pending:
            self._diag_pending = False
            self.gcode.respond_info(
                "SmartFilamentSensor '%s': no DIAG reply from the sensor.\n"
                "  DIAG was added in ESP32 firmware 2.4.0 — flash a current "
                "build to use it." % self.name)
        return self.reactor.NEVER

    def cmd_SUSPEND(self, gcmd):
        self._suspend_depth += 1
        logging.info("SmartFilamentSensor '%s': suspended (depth %d)"
                     % (self.name, self._suspend_depth))

    def cmd_RESUME(self, gcmd):
        if self._suspend_depth > 0:
            self._suspend_depth -= 1
        if self._suspend_depth == 0:
            self._reset_detection_state()
            self._last_e_pos = self._get_e_pos()
            if self._connected:
                self._send("RESET_MM")
            logging.info("SmartFilamentSensor '%s': resumed, resynced"
                         % self.name)

    def cmd_ENABLE(self, gcmd):
        if not self._require_connected(gcmd):
            return
        self._enabled = True
        self._reset_detection_state()
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
        self._reset_gain_tracking()
        self._last_e_pos = self._get_e_pos()
        self._send("RESET_MM")
        self._send("GET_CAL")
        gcmd.respond_info(
            "SmartFilamentSensor '%s': re-synced, gain measurement restarted"
            % self.name)

    def cmd_MEASURE(self, gcmd):
        # Manual measure - works without printing state.
        # 1st call: reset encoder. Extrude/pull filament. 2nd call: show mm.
        if not self._require_connected(gcmd):
            return
        if not self._measure_active:
            # RESET_MM zeroes the same odometer detection windows read from.
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
            # Flipping direction negates every future reading, so anything
            # accumulated under the old sign is worse than useless -- averaging
            # the two together hides the change instead of showing it.
            self._reset_detection_state()
            self._reset_gain_tracking()
            sent.append("DIR=%d" % direction)
        cal = gcmd.get_float('CAL', None)
        if cal is not None:
            self._send("SET CAL %.4f" % cal)
            self._reset_gain_tracking()
            sent.append("CAL=%.4f" % cal)
        if sent:
            self._send("GET_CAL")
            gcmd.respond_info(
                "SmartFilamentSensor '%s': set %s"
                % (self.name, ", ".join(sent)))
        else:
            gcmd.respond_info(
                "SmartFilamentSensor '%s': no parameters given.\n"
                "Usage: SFS_SET [SENS=] [NOISE=] [BRIGHT=] [DIR=] [CAL=]"
                % self.name)

    def cmd_AUTO_CALIBRATE(self, gcmd):
        """One-command calibration: heat hotend, extrude, auto-save.

        LENGTH defaults to 100mm rather than 50: the encoder wheel is ~29mm
        of filament per revolution, so a short extrude samples only part of a
        turn and any per-revolution error folds straight into the result.
        """
        if not self._require_connected(gcmd):
            return
        temp = gcmd.get_float('TEMP', 240., above=0.)
        length = gcmd.get_float('LENGTH', 100., above=0.)
        speed = gcmd.get_float('SPEED', 100., above=0.)  # mm/min

        gcmd.respond_info(
            "SmartFilamentSensor '%s': AUTO CALIBRATE\n"
            "  Heating to %.0fC, then extruding %.0fmm at F%.0f\n"
            "  Please wait..."
            % (self.name, temp, length, speed))

        try:
            self.gcode.run_script_from_command("M109 S%.0f" % temp)

            self._calibrating = True
            self._send("START %.1f" % length)
            gcmd.respond_info(
                "SmartFilamentSensor '%s': hotend ready, extruding %.0fmm..."
                % (self.name, length))

            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(
                "G1 E%.1f F%.0f" % (length, speed))
            self.gcode.run_script_from_command("G4 P6000")

            if self._calibrating:
                self._send("APPLY")
                gcmd.respond_info(
                    "SmartFilamentSensor '%s': extrusion complete, "
                    "applying calibration..." % self.name)
                self.gcode.run_script_from_command("G4 P2000")

            self._reset_gain_tracking()
            self._send("GET_CAL")
            gcmd.respond_info(
                "SmartFilamentSensor '%s': auto calibration finished.\n"
                "  Gain measurement restarted — run a print and check\n"
                "  SFS_STATUS: encoder_gain should settle near 1.000."
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
