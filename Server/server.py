#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
The ``Server`` facade: command ingestion, telemetry fan-out, transports.

This class used to *be* the robot -- it constructed the motors, the encoders, the
gyro, the camera and the drive controller, and ran a dozen threads over them. It
is now a facade in P_web that owns no hardware at all. Its job is:

  * parse an incoming command (legacy ``CMD_*#...`` text or JSON) once, decide
    which subsystem owns it, and enqueue it;
  * compose a telemetry snapshot from what the children publish;
  * serve the legacy TCP transports and the battery monitor.

What that buys, concretely: the aiohttp event loop no longer shares a GIL with
the pigpio encoder callbacks or the 20 Hz control loop. What it costs: one queue
hop (~50-200 us) on the command path, which is noise against the 50 ms control
period -- but it does mean ``command_timeout`` now measures web-to-control
latency too, so commands are timestamped at ingress.

Mode ownership stays here. P_control applies what it is told and does not
second-guess it; refusing a drive command because the kart is in line-following
mode is this layer's call, made before the command is ever enqueued.
"""
import fcntl
import math
import socket
import struct
import threading
import time

from Command import COMMAND as cmd
from config import CONFIG
import ipc as ipc_mod
from protocol import Command, CommandRouter
from supervisor import Supervisor


class FrameReader:
    """A viewer's cursor into the shared camera ring.

    Each consumer keeps its own sequence number, so a slow viewer falls behind
    and skips frames instead of throttling the camera or the other viewers --
    which is what happened when everyone waited on one shared Condition and read
    one shared ``frame`` attribute.
    """

    def __init__(self, server: "Server", ring: ipc_mod.FrameRing):
        self._server = server
        self._ring = ring
        self.seq = 0
        self._closed = False

    def read(self, timeout: float = 2.0):
        """Next frame as JPEG bytes, or None on timeout (so the caller can
        re-check whether its client is still there)."""
        data, self.seq = self._ring.read(self.seq, timeout=timeout)
        return data

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._server.release_camera()


class Server:
    def __init__(self, *, with_camera: bool = True, with_aux: bool = True,
                 config=CONFIG):
        self.config = config
        self.tcp_Flag = True
        self.sonic = False
        self.Light = False
        self.Line = False
        self.Mode = 'one'
        self.endChar = '\n'
        self.intervalChar = '#'
        self.rotation_flag = False
        self.cmd_lock = threading.Lock()     # guards this facade's own flags

        # --- listening sockets (created once, never closed at runtime) ---
        self.server_socket = None            # video    (port 8000)
        self.server_socket1 = None           # commands (port 5000)
        self.connection1 = None              # active command connection

        # --- camera reference counting (the count belongs to whoever has the
        #     viewers; P_camera only ever sees start/stop) ---
        self.camera_lock = threading.Lock()
        self.camera_refcount = 0

        # --- latest telemetry envelope from P_control ---
        self._telemetry_lock = threading.Lock()
        self._last_drive = None
        self._last_servo = {}
        self._last_signs = dict(config.sides.signs)
        self._last_control_ts = 0.0

        # ------------------------------------------------------------------
        # Fork the children BEFORE starting any thread in this process.
        # Forking a threaded parent inherits locks in whatever state they were
        # in, so a child can deadlock on a mutex whose owner does not exist in
        # it. Everything below this point may start threads; nothing above may.
        # ------------------------------------------------------------------
        self.ipc = ipc_mod.IPC(config)
        self.supervisor = Supervisor(self.ipc, config, with_camera=with_camera,
                                     with_aux=with_aux)
        self.supervisor.start()

        self._stop_evt = threading.Event()
        self.supervisor.start_monitors()
        threading.Thread(target=self._drain_telemetry, daemon=True,
                         name="TelemetryDrain").start()
        threading.Thread(target=self._legacy_sensor_publisher, daemon=True,
                         name="LegacySensors").start()

        # --- extensible command routing ---
        self.router = CommandRouter()
        self._build_router()

    # ------------------------------------------------------------------
    # Command handler registration.
    # Every handler now ENQUEUES to the process that owns the device rather
    # than touching it. Adding a command is still "register one handler here",
    # plus a matching handler in that process's applier.
    # ------------------------------------------------------------------
    def _build_router(self):
        r = self.router
        r.register('motor',          self._h_motor)
        r.register('mecanum',        self._h_mecanum)
        r.register('car_rotate',     self._h_car_rotate)
        r.register('drive',          self._h_drive)
        r.register('drive_distance', self._h_drive_distance)
        r.register('turn',           self._h_turn)
        r.register('goto',           self._h_goto)
        r.register('raw_turn_schedule', self._h_raw_turn_schedule)
        r.register('reset_odometry', self._h_reset_odometry)
        r.register('calibrate_imu',  self._h_calibrate_imu)
        r.register('set_sign',       self._h_set_sign)
        r.register('servo',          self._h_servo)
        r.register('led',            self._h_led)
        r.register('led_mode',       self._h_led_mode)
        r.register('buzzer',         self._h_buzzer)
        r.register('sonic',          self._h_sonic)
        r.register('light',          self._h_light)
        r.register('power',          self._h_power)
        r.register('mode',           self._h_mode)

    # ------------------------------------------------------------------
    # Queue helpers
    # ------------------------------------------------------------------
    def _to_control(self, name: str, **kwargs) -> None:
        # Stamped at ingress: the control loop's staleness test
        # (control.command_timeout) should measure the age of the operator's
        # intent, not the moment the queue happened to be drained.
        kwargs.setdefault('ts', time.monotonic())
        ipc_mod.put_drop_oldest(self.ipc.control_q,
                                Command(name=name, kwargs=kwargs))

    def _to_aux(self, name: str, **kwargs) -> None:
        ipc_mod.put_drop_oldest(self.ipc.aux_q, Command(name=name, kwargs=kwargs))

    def _to_sensors(self, name: str, **kwargs) -> None:
        ipc_mod.put_drop_oldest(self.ipc.sensors_q,
                                Command(name=name, kwargs=kwargs))

    # ------------------------------------------------------------------
    # Networking
    # ------------------------------------------------------------------
    def get_interface_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(),
                                            0x8915,
                                            struct.pack('256s', b'wlan0'[:15])
                                            )[20:24])

    def StartTcpServer(self):
        """Cria os sockets de escuta uma única vez. Idempotente."""
        HOST = str(self.get_interface_ip())

        if self.server_socket1 is None:
            self.server_socket1 = socket.socket()
            self.server_socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket1.bind((HOST, 5000))
            self.server_socket1.listen(1)

        if self.server_socket is None:
            self.server_socket = socket.socket()
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((HOST, 8000))
            self.server_socket.listen(1)

        print('Server address: ' + HOST)

    def StopTcpServer(self):
        """Fecha os sockets de escuta. Usado apenas no desligamento."""
        self.tcp_Flag = False
        for sock in (self.server_socket, self.server_socket1, self.connection1):
            try:
                if sock is not None:
                    sock.close()
            except OSError:
                pass

    def send(self, data):
        conn = self.connection1
        if conn is None:
            return
        try:
            conn.send(data.encode('utf-8'))
        except OSError:
            pass

    # ------------------------------------------------------------------
    # Câmera (refcounted here; the device lives in P_camera)
    # ------------------------------------------------------------------
    def acquire_camera(self) -> FrameReader:
        """Ask P_camera to run and return a private cursor into the frame ring."""
        with self.camera_lock:
            if self.camera_refcount == 0:
                ipc_mod.put_drop_oldest(self.ipc.camera_q, {"action": "start"})
            self.camera_refcount += 1
        return FrameReader(self, self.ipc.frames)

    def release_camera(self) -> None:
        with self.camera_lock:
            if self.camera_refcount > 0:
                self.camera_refcount -= 1
            if self.camera_refcount == 0:
                ipc_mod.put_drop_oldest(self.ipc.camera_q, {"action": "stop"})

    # Compat with the old API.
    def start_camera(self) -> FrameReader:
        return self.acquire_camera()

    def stop_camera(self) -> None:
        """Force the camera off regardless of the reference count (shutdown)."""
        with self.camera_lock:
            self.camera_refcount = 0
        ipc_mod.put_drop_oldest(self.ipc.camera_q, {"action": "stop"})

    @property
    def camera_running(self) -> bool:
        return bool(self.ipc.camera_state[0])

    # ------------------------------------------------------------------
    # Laços de aceitação persistentes
    # ------------------------------------------------------------------
    def sendvideo(self):
        """Laço persistente: aceita um cliente de vídeo, transmite até cair,
        e volta a aceitar o próximo. Nunca fecha o socket de escuta."""
        while self.tcp_Flag:
            try:
                conn, client_address = self.server_socket.accept()
            except OSError:
                break                     # listening socket closed -> shutdown

            print("socket video connected ...")
            stream = conn.makefile('wb')
            reader = self.acquire_camera()
            try:
                while self.tcp_Flag:
                    frame = reader.read(timeout=2.0)
                    if frame is None:
                        continue
                    stream.write(struct.pack('<I', len(frame)))
                    stream.write(frame)
            except (OSError, BrokenPipeError):
                print("End transmit ...")
            finally:
                reader.close()
                try:
                    stream.close()
                    conn.close()
                except OSError:
                    pass

    def readdata(self):
        """Laço persistente: aceita um cliente de comandos, processa até cair,
        e volta a aceitar o próximo. Nunca fecha o socket de escuta."""
        while self.tcp_Flag:
            try:
                self.connection1, self.client_address1 = self.server_socket1.accept()
                print("Client connection successful !")
            except OSError:
                break

            restCmd = ""
            try:
                while self.tcp_Flag:
                    try:
                        chunk = self.connection1.recv(1024).decode('utf-8')
                    except OSError:
                        break
                    if chunk == '':
                        break             # clean client disconnect

                    AllData = restCmd + chunk
                    restCmd = ""

                    cmdArray = AllData.split("\n")
                    if cmdArray[-1] != "":
                        restCmd = cmdArray[-1]
                        cmdArray = cmdArray[:-1]

                    for oneCmd in cmdArray:
                        self.dispatch_command(oneCmd)
            except Exception as e:
                print(e)
            finally:
                try:
                    if self.connection1 is not None:
                        self.connection1.close()
                except OSError:
                    pass
                self.connection1 = None
                print("Client disconnected, waiting for new connection ...")

    # ------------------------------------------------------------------
    # Modes
    # ------------------------------------------------------------------
    def stopMode(self):
        """Leave whatever autonomous mode is running and hand the motors back.

        The old version called ``stop_thread()`` on each mode thread -- ctypes
        injection of SystemExit, seven times, which could land mid-I2C-write. The
        modes now live in P_aux and stop on an event, and P_control is told to
        release and brake regardless of whether P_aux answered.
        """
        self._stop_rotation()
        self._to_aux('stop_mode')
        self._to_control('release')
        self._to_control('motor', duty=[0, 0, 0, 0])
        self.sonic = False
        self.Light = False
        self.Line = False
        self.send('CMD_MODE' + '#1' + '#' + '0' + '#' + '0' + '\n')
        self.send('CMD_MODE' + '#3' + '#' + '0' + '\n')
        self.send('CMD_MODE' + '#2' + '#' + '000' + '\n')

    def _stop_rotation(self):
        """Stop a CMD_CAR_ROTATE spin."""
        self.rotation_flag = False
        self._to_control('car_rotate', stop=True)

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------
    def dispatch_command(self, oneCmd):
        """Handle a single command (legacy text or JSON), via the router.

        Called by readdata() (TCP) and the web/WebSocket handler.
        """
        with self.cmd_lock:
            try:
                self.router.dispatch(oneCmd)
            except Exception as e:
                print(f"dispatch error for {oneCmd!r}: {e}")

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------
    def _drain_telemetry(self):
        """Keep the newest control snapshot handy for the broadcaster.

        The control loop pushes to a bounded queue and never blocks on it; this
        thread keeps only the latest envelope. A queue rather than a packed
        shared struct because at 20 Hz the pickle costs well under a millisecond
        and nested telemetry (phase names, None-able fields) would otherwise need
        a hand-rolled binary layout maintained in two places.
        """
        while not self._stop_evt.is_set():
            latest = None
            for item in ipc_mod.drain(self.ipc.telemetry_q, limit=8):
                latest = item
            if latest is not None:
                with self._telemetry_lock:
                    self._last_drive = latest.get('drive')
                    self._last_servo = latest.get('servo') or {}
                    self._last_signs = latest.get('signs') or self._last_signs
                    self._last_control_ts = latest.get('ts', 0.0)
            self._stop_evt.wait(1.0 / max(1.0, self.config.control.telemetry_hz))

    def get_telemetry(self):
        """Snapshot of everything a client may want to display."""
        battery, light_l, light_r, adc_ts = self.ipc.read_adc()
        with self._telemetry_lock:
            drive = self._last_drive
            servo = dict(self._last_servo)
            signs = dict(self._last_signs)
            control_ts = self._last_control_ts

        if drive is None:
            from drive_controller import DriveController
            drive = DriveController._blank_telemetry()

        # Age of the control data, so a UI can tell "stopped" from "not reporting".
        # Without it a frozen snapshot looks exactly like a stationary kart.
        drive = dict(drive)
        drive['stale'] = (control_ts == 0.0 or
                          (time.monotonic() - control_ts) >
                          max(0.5, 5.0 / self.config.control.loop_hz))

        return {
            "battery": round(battery, 2),
            "mode": self.Mode,
            "drive": drive,
            "signs": signs,
            "servo": servo,
            "light": {"left": light_l, "right": light_r},
            "line": self.ipc.read_line(),
            "processes": self.supervisor.status(),
        }

    def _legacy_sensor_publisher(self):
        """Push the legacy ``CMD_MODE#...`` sensor lines to a TCP client.

        Replaces three self-rearming ``threading.Timer`` chains (sendUltrasonic /
        sendLight / sendLine), each of which created a BRAND NEW THREAD every
        0.17-0.23 s for as long as its sensor was enabled. One thread reading
        shared memory does the same job; the readings themselves now come from
        P_sensors instead of this process touching I2C and GPIO.
        """
        period = 0.2
        while not self._stop_evt.is_set():
            if self.connection1 is not None:
                if self.Light:
                    _b, light_l, light_r, _ts = self.ipc.read_adc()
                    self.send(f"CMD_MODE#1#{light_l}#{light_r}\n")
                if self.sonic:
                    distance, _guard, _healthy, _ts = self.ipc.read_distance()
                    if distance is not None:
                        self.send(f"{cmd.CMD_MODE}#3#{int(distance)}\n")
                if self.Line:
                    bits = self.ipc.read_line()
                    if bits is not None:
                        self.send(f"CMD_MODE#2#{bits[0]}{bits[1]}{bits[2]}\n")
            self._stop_evt.wait(period)

    def Power(self):
        """Battery monitor: report the voltage and beep when it gets low.

        Reads what P_sensors publishes instead of doing its own I2C, and asks
        P_aux to sound the buzzer instead of owning the pin.
        """
        while not self._stop_evt.is_set():
            battery, _l, _r, ts = self.ipc.read_adc()
            if ts == 0.0:                      # no reading published yet
                self._stop_evt.wait(1.0)
                continue
            try:
                self.send(cmd.CMD_POWER + '#' + str(round(battery, 2)) + '\n')
            except Exception:
                pass
            self._stop_evt.wait(3.0)

            if battery < 10:
                beeps = 4
            elif battery < 10.5:
                beeps = 2
            else:
                self._to_aux('buzzer', on='0')
                continue
            for _ in range(beeps):
                self._to_aux('buzzer', on='1')
                self._stop_evt.wait(0.1)
                self._to_aux('buzzer', on='0')
                self._stop_evt.wait(0.1)

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------
    def _h_mode(self, c: Command):
        mode = str(c.get('mode', c.arg(0)))
        if mode in ('one', '0'):
            self.stopMode()
            self.Mode = 'one'
        elif mode in ('two', '1'):
            self.stopMode()
            self.Mode = 'two'
            self._to_aux('start_mode', mode='two')
            self.Light = True
        elif mode in ('three', '3'):
            self.stopMode()
            self.Mode = 'three'
            self._to_aux('start_mode', mode='three')
            self.sonic = False
        elif mode in ('four', '2'):
            self.stopMode()
            self.Mode = 'four'
            self._to_aux('start_mode', mode='four')
            self.Line = True

    def _h_motor(self, c: Command):
        """Raw skid duty (bypasses PID). Legacy CMD_MOTOR / JSON {duty:[...]}."""
        if self.Mode != 'one':
            return
        try:
            duty = c.get('duty')
            if duty and len(duty) >= 4:
                d = [int(x) for x in duty[:4]]
            else:
                d = [c.arg_int(i) for i in range(4)]
            self._to_control('motor', duty=d)
        except Exception:
            pass

    def _h_drive(self, c: Command):
        """Closed-loop velocity command (m/s, rad/s) -> engages PID."""
        if self.Mode != 'one':
            return
        self._to_control('drive', linear=c.num('linear', 0, 0.0),
                         angular=c.num('angular', 1, 0.0))

    def _h_drive_distance(self, c: Command):
        """Drive straight a set distance (m) and stop. Closed-loop on odometry."""
        if self.Mode != 'one':
            return
        self._to_control('drive_distance', distance=c.num('distance', 0, 0.0),
                         speed=c.num('speed', 1, 0.2))

    def _h_turn(self, c: Command):
        """Turn in place by a set angle (deg) and stop."""
        if self.Mode != 'one':
            return
        self._to_control('turn', angle=c.num('angle', 0, 0.0),
                         speed=c.num('speed', 1, 1.0))

    def _h_goto(self, c: Command):
        """Go to world pose (x, y[, theta_deg]) -- turn, drive, turn."""
        if self.Mode != 'one':
            return
        theta = c.get('theta')
        self._to_control('goto', x=c.num('x', 0, 0.0), y=c.num('y', 1, 0.0),
                         theta=float(theta) if theta is not None else None)

    def _h_raw_turn_schedule(self, c: Command):
        """Open-loop PWM turn run on the Pi (no per-step network latency)."""
        if self.Mode != 'one':
            return
        params = c.get('fn_params')
        self._to_control('raw_turn_schedule',
                         turn_fn=str(c.get('turn_fn', 'trapezoid')),
                         ccw=bool(c.get('ccw', True)),
                         pwm=int(c.num('pwm', 0, 2000)),
                         min_pwm=int(c.num('min_pwm', 1, 1000)),
                         final_turn_angle=c.num('final_turn_angle', 2, 90.0),
                         fn_params=params if isinstance(params, dict) else {})

    def _h_reset_odometry(self, c: Command):
        self._to_control('reset_odometry')

    def _h_calibrate_imu(self, c: Command):
        """Re-estimate the gyro bias (kart must be still).

        Used to spawn a thread here to keep the command non-blocking. It is now
        just a queue put: P_sensors owns the sensor and does the ~1.5 s of
        sampling on its own loop.
        """
        self._to_sensors('calibrate_imu')

    def _h_set_sign(self, c: Command):
        """Flip encoder count sign(s) at runtime (calibration).

        JSON forms:
          {"type":"set_sign","motor":"M3","sign":-1}
          {"type":"set_sign","signs":{"M1":1,"M2":1,"M3":-1,"M4":-1}}

        The signs are applied in P_control (P_sensors publishes raw counts), so
        this forwards rather than mutating a dict that the reader would never see.
        """
        bulk = c.get('signs')
        if isinstance(bulk, dict):
            self._to_control('set_sign', signs=bulk)
        else:
            self._to_control('set_sign', motor=str(c.get('motor', c.arg(0))),
                             sign=c.num('sign', 1, 1))

    def _h_mecanum(self, c: Command):
        """Legacy mecanum joystick mix (CMD_M_MOTOR).

        The mix is computed here, where the joystick geometry arrives, and the
        result crosses the queue as four plain duties.
        """
        if self.Mode != 'one':
            return
        try:
            a1, m1, a2, m2 = (c.arg_int(0), c.arg_int(1),
                              c.arg_int(2), c.arg_int(3))
            LX = int(m1 * math.sin(math.radians(a1)))
            LY = int(m1 * math.cos(math.radians(a1)))
            RX = int(m2 * math.sin(math.radians(a2)))

            FR = LY - LX + RX
            FL = LY + LX - RX
            BL = LY - LX - RX
            BR = LY + LX + RX
            self._to_control('motor', duty=[FL, BL, FR, BR])
        except Exception:
            pass

    def _h_car_rotate(self, c: Command):
        if self.Mode != 'one':
            return
        try:
            a1, m1, a2, m2 = (c.arg_int(0), c.arg_int(1),
                              c.arg_int(2), c.arg_int(3))
            if m2 == 0:
                self._stop_rotation()
                LX = int(m1 * math.sin(math.radians(a1)))
                LY = int(m1 * math.cos(math.radians(a1)))
                FR = LY - LX
                FL = LY + LX
                BL = LY - LX
                BR = LY + LX
                self._to_control('motor', duty=[FL, BL, FR, BR])
            elif not self.rotation_flag:
                self.rotation_flag = True
                self._to_control('car_rotate', angle=a2)
        except Exception:
            pass

    def _h_servo(self, c: Command):
        """Servos share the PCA9685 with the motors, so P_control applies them."""
        try:
            self._to_control('servo', channel=str(c.get('channel', c.arg(0))),
                             angle=int(c.num('angle', 1, 90)))
        except Exception:
            pass

    def _h_led(self, c: Command):
        self._to_aux('led', index=int(c.num('index', 0, 255)),
                     r=int(c.num('r', 1, 0)), g=int(c.num('g', 2, 0)),
                     b=int(c.num('b', 3, 0)))

    def _h_led_mode(self, c: Command):
        self.LedMoD = str(c.get('mode', c.arg(0)))
        self._to_aux('led_mode', mode=self.LedMoD)

    def _h_sonic(self, c: Command):
        on = str(c.get('on', c.arg(0)))
        self.sonic = on in ('1', 'True', 'true')

    def _h_buzzer(self, c: Command):
        on = c.get('on')
        if on is None:
            on = c.arg(0)
        self._to_aux('buzzer', on='1' if on in (True, '1', 'true', 'True') else '0')

    def _h_light(self, c: Command):
        on = str(c.get('on', c.arg(0)))
        self.Light = on in ('1', 'True', 'true')

    def _h_power(self, c: Command):
        battery, _l, _r, _ts = self.ipc.read_adc()
        self.send(cmd.CMD_POWER + '#' + str(round(battery, 2)) + '\n')

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        """Stop everything, in the order that leaves the kart safe.

        Replaces the old ``drive.shutdown() / stop_camera() / PWM.setMotorModel(0…)``
        sequence that every entry point open-coded.
        """
        self._stop_evt.set()
        self.tcp_Flag = False
        try:
            self._to_control('stop')
            self._to_aux('stop_mode')
        except Exception:
            pass
        # Give the control loop a couple of ticks to actually apply that brake
        # before the stop event ends its loop. Without the pause the graceful
        # path is decorative: stop_evt would cut the loop before it ever drained
        # the command, and only P_control's finally block (or, worse, the
        # supervisor's emergency stop) would halt the motors.
        time.sleep(3.0 / max(1.0, self.config.control.loop_hz))
        self.StopTcpServer()
        self.supervisor.shutdown()


if __name__ == '__main__':
    # Smoke test: bring the stack up, print telemetry, shut it down.
    srv = Server()
    try:
        while True:
            time.sleep(1.0)
            tel = srv.get_telemetry()
            pose = tel['drive']['pose']
            print(f"battery={tel['battery']}V mode={tel['mode']} "
                  f"pose=({pose['x']:.3f},{pose['y']:.3f},{pose['theta_deg']:.1f}) "
                  f"stale={tel['drive']['stale']} "
                  f"procs={ {k: v['alive'] for k, v in tel['processes']['children'].items()} }")
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        srv.shutdown()
