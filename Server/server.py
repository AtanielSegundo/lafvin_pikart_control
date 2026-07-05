#!/usr/bin/python
# -*- coding: utf-8 -*-
import io
import math
import socket
import numpy as np
import struct
import time
from picamera2 import Picamera2, Preview
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from picamera2.encoders import Quality
from threading import Condition
import fcntl
import sys
import threading
from Motor import *
from servo import *
from Led import *
from Buzzer import *
from ADC import *
from Thread import *
from Light import *
from Ultrasonic import *
from Line_Tracking import *
from threading import Timer
from threading import Thread
from Command import COMMAND as cmd
import RPi.GPIO as GPIO

from config import CONFIG
from protocol import CommandRouter, Command
from encoders import WheelEncoders
from drive_controller import DriveController


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class Server:
    def __init__(self):
        self.PWM        = Motor()
        self.servo      = Servo()
        self.led        = Led()
        self.ultrasonic = Ultrasonic()
        self.buzzer     = Buzzer()
        self.adc        = Adc()
        self.light      = Light()
        self.infrared   = Line_Tracking()
        self.tcp_Flag   = True
        self.sonic            = False
        self.Light            = False
        self.Line             = False
        self.Mode             = 'one'
        self.endChar          = '\n'
        self.intervalChar     = '#'
        self.rotation_flag    = False
        self.cmd_lock         = threading.Lock()

        # --- estado da câmera, protegido por lock ---
        self.camera           = None
        self.streaming_output = None
        self.camera_lock      = threading.Lock()
        self.camera_refcount  = 0          # quantos consumidores querem a câmera ligada

        # --- sockets de escuta (criados uma vez, nunca fechados em runtime) ---
        self.server_socket    = None       # vídeo  (porta 8000)
        self.server_socket1   = None       # comandos (porta 5000)

        # --- conexão de comandos ativa (usada por send()) ---
        self.connection1      = None

        # --- closed-loop drivetrain (encoders + odometry + PID) ---
        self.encoders  = WheelEncoders(CONFIG.sides)
        self.drive     = DriveController(self.PWM, self.encoders, CONFIG)
        self._rotate_thread = None
        try:
            self.drive.start()   # begins encoder listening + control loop
        except Exception as e:
            print(f"DriveController failed to start: {e}")

        # --- extensible command routing ---
        self.router = CommandRouter()
        self._build_router()

    # ------------------------------------------------------------------
    # Command handler registration (replaces the old if/elif chain).
    # Adding a command = register one handler here.
    # ------------------------------------------------------------------
    def _build_router(self):
        r = self.router
        r.register('motor',          self._h_motor)
        r.register('mecanum',        self._h_mecanum)
        r.register('car_rotate',     self._h_car_rotate)
        r.register('drive',          self._h_drive)
        r.register('drive_distance', self._h_drive_distance)
        r.register('turn',           self._h_turn)
        r.register('reset_odometry', self._h_reset_odometry)
        r.register('servo',          self._h_servo)
        r.register('led',            self._h_led)
        r.register('led_mode',       self._h_led_mode)
        r.register('buzzer',         self._h_buzzer)
        r.register('sonic',          self._h_sonic)
        r.register('light',          self._h_light)
        r.register('power',          self._h_power)
        r.register('mode',           self._h_mode)

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
    # Câmera com contagem de referência
    # ------------------------------------------------------------------
    def acquire_camera(self):
        """Liga a câmera se ainda não estiver ligada e incrementa o refcount.
        Retorna o StreamingOutput compartilhado."""
        with self.camera_lock:
            if self.camera is None:
                self.camera = Picamera2()
                self.camera.configure(
                    self.camera.create_video_configuration(main={"size": (400, 300)}))
                self.streaming_output = StreamingOutput()
                encoder = JpegEncoder(q=90)
                self.camera.start_recording(
                    encoder, FileOutput(self.streaming_output), quality=Quality.VERY_HIGH)
                print("Camera started")
            self.camera_refcount += 1
            return self.streaming_output

    def release_camera(self):
        """Decrementa o refcount; desliga a câmera só quando o último sair."""
        with self.camera_lock:
            if self.camera_refcount > 0:
                self.camera_refcount -= 1
            if self.camera_refcount == 0 and self.camera is not None:
                try:
                    self.camera.stop_recording()
                    self.camera.close()
                except Exception:
                    pass
                self.camera = None
                self.streaming_output = None
                print("Camera stopped")

    # Compatibilidade com web.py (mantém a API antiga)
    def start_camera(self):
        """Compat: garante câmera ligada sem mexer no refcount de longa duração.
        web.py chama isto a cada requisição /video; o release ocorre lá."""
        return self.acquire_camera()

    def stop_camera(self):
        """Compat: força desligamento total (usado no shutdown)."""
        with self.camera_lock:
            self.camera_refcount = 0
            if self.camera is not None:
                try:
                    self.camera.stop_recording()
                    self.camera.close()
                except Exception:
                    pass
                self.camera = None
                self.streaming_output = None
                print("Camera stopped")

    # ------------------------------------------------------------------
    # Laços de aceitação persistentes (substituem o antigo Reset)
    # ------------------------------------------------------------------
    def sendvideo(self):
        """Laço persistente: aceita um cliente de vídeo, transmite até cair,
        e volta a aceitar o próximo. Nunca fecha o socket de escuta."""
        while self.tcp_Flag:
            try:
                conn, client_address = self.server_socket.accept()
            except OSError:
                # socket de escuta foi fechado (shutdown) -> encerra o laço
                break

            print("socket video connected ...")
            stream = conn.makefile('wb')
            output = self.acquire_camera()
            try:
                while self.tcp_Flag:
                    with output.condition:
                        output.condition.wait(timeout=2.0)
                        frame = output.frame
                    if frame is None:
                        continue
                    lengthBin = struct.pack('<I', len(frame))
                    stream.write(lengthBin)
                    stream.write(frame)
            except (OSError, BrokenPipeError):
                print("End transmit ...")
            finally:
                self.release_camera()
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
                        # cliente desconectou de forma limpa
                        break

                    AllData = restCmd + chunk
                    print(AllData)
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

    def stopMode(self):
        # Hand motor control back to the loop-free state and stop any rotation.
        self._stop_rotation()
        self.drive.release()
        try:
            stop_thread(self.infraredRun)
            self.PWM.setMotorModel(0, 0, 0, 0)
        except:
            pass
        try:
            stop_thread(self.lightRun)
            self.PWM.setMotorModel(0, 0, 0, 0)
        except:
            pass
        try:
            stop_thread(self.ultrasonicRun)
            self.PWM.setMotorModel(0, 0, 0, 0)
            self.servo.setServoPwm('0', 90)
            self.servo.setServoPwm('1', 90)
        except:
            pass
        self.sonic = False
        self.Light = False
        self.Line = False
        self.send('CMD_MODE' + '#1' + '#' + '0' + '#' + '0' + '\n')
        self.send('CMD_MODE' + '#3' + '#' + '0' + '\n')
        self.send('CMD_MODE' + '#2' + '#' + '000' + '\n')

    def dispatch_command(self, oneCmd):
        """Handle a single command (legacy text or JSON), via the router.

        Called by readdata() (TCP) and the web/WebSocket handler.
        """
        with self.cmd_lock:
            try:
                self.router.dispatch(oneCmd)
            except Exception as e:
                print(f"dispatch error for {oneCmd!r}: {e}")

    def get_telemetry(self):
        """Snapshot of everything a client may want to display."""
        try:
            battery = round(self.adc.recvADC(2) * 5, 2)
        except Exception:
            battery = 0.0
        return {
            "battery": battery,
            "mode": self.Mode,
            "drive": self.drive.telemetry(),
        }

    def _stop_rotation(self):
        """Cooperatively stop the CMD_CAR_ROTATE spin thread, if running."""
        try:
            self.PWM.stop_rotate()
        except Exception:
            pass
        self.rotation_flag = False
        t = self._rotate_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._rotate_thread = None

    # ------------------------------------------------------------------
    # Command handlers (registered in _build_router)
    # ------------------------------------------------------------------
    def _h_mode(self, c: Command):
        mode = str(c.get('mode', c.arg(0)))
        if mode in ('one', '0'):
            self.stopMode()
            self.Mode = 'one'
        elif mode in ('two', '1'):
            self.stopMode()
            self.Mode = 'two'
            self.lightRun = Thread(target=self.light.run, daemon=True)
            self.lightRun.start()
            self.Light = True
            self.lightTimer = threading.Timer(0.3, self.sendLight)
            self.lightTimer.start()
        elif mode in ('three', '3'):
            self.stopMode()
            self.Mode = 'three'
            self.ultrasonicRun = Thread(target=self.ultrasonic.run, daemon=True)
            self.ultrasonicRun.start()
            self.sonic = False
            self.ultrasonicTimer = threading.Timer(5, self.sendUltrasonic)
            self.ultrasonicTimer.start()
        elif mode in ('four', '2'):
            self.stopMode()
            self.Mode = 'four'
            self.infraredRun = Thread(target=self.infrared.run, daemon=True)
            self.infraredRun.start()
            self.Line = True
            self.lineTimer = threading.Timer(0.4, self.sendLine)
            self.lineTimer.start()

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
            self.drive.release()          # stop PID fighting the raw duty
            self.PWM.setMotorModel(*d)
        except Exception:
            pass

    def _h_drive(self, c: Command):
        """Closed-loop velocity command (m/s, rad/s) -> engages PID."""
        if self.Mode != 'one':
            return
        v = c.num('linear', 0, 0.0)
        w = c.num('angular', 1, 0.0)
        self.drive.set_twist(v, w)

    def _h_drive_distance(self, c: Command):
        """Drive straight a set distance (m) and stop. Closed-loop on odometry."""
        if self.Mode != 'one':
            return
        distance = c.num('distance', 0, 0.0)
        speed = c.num('speed', 1, 0.2)
        self.drive.drive_distance(distance, speed)

    def _h_turn(self, c: Command):
        """Turn in place by a set angle (deg) and stop."""
        if self.Mode != 'one':
            return
        angle = c.num('angle', 0, 0.0)
        speed = c.num('speed', 1, 1.0)
        self.drive.turn_in_place(angle, speed)

    def _h_reset_odometry(self, c: Command):
        self.drive.reset_odometry()

    def _h_mecanum(self, c: Command):
        """Legacy mecanum joystick mix (CMD_M_MOTOR)."""
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
            self.drive.release()
            self.PWM.setMotorModel(FL, BL, FR, BR)
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
                self.drive.release()
                self.PWM.setMotorModel(FL, BL, FR, BR)
            elif not self.rotation_flag:
                self.angle = a2
                self._stop_rotation()
                self.drive.release()
                self.rotation_flag = True
                self._rotate_thread = Thread(target=self.PWM.Rotate,
                                             args=(a2,), daemon=True)
                self._rotate_thread.start()
        except Exception:
            pass

    def _h_servo(self, c: Command):
        try:
            channel = str(c.get('channel', c.arg(0)))
            angle = int(c.num('angle', 1, 90))
            self.servo.setServoPwm(channel, angle)
        except Exception:
            pass

    def _h_led(self, c: Command):
        try:
            index = int(c.num('index', 0, 255))
            r = int(c.num('r', 1, 0))
            g = int(c.num('g', 2, 0))
            b = int(c.num('b', 3, 0))
            self.led.ledIndex(index, r, g, b)
        except Exception:
            pass

    def _h_led_mode(self, c: Command):
        self.LedMoD = str(c.get('mode', c.arg(0)))
        if self.LedMoD == '0':
            self._stop_led_mode()
        elif self.LedMoD == '1':
            self._stop_led_mode()
            self.led.ledMode(self.LedMoD)
            time.sleep(0.1)
            self.led.ledMode(self.LedMoD)
        else:
            self._stop_led_mode()
            time.sleep(0.1)
            self._led_mode = Thread(target=self.led.ledMode,
                                    args=(self.LedMoD,), daemon=True)
            self._led_mode.start()

    def _stop_led_mode(self):
        try:
            stop_thread(self._led_mode)
        except Exception:
            pass

    def _h_sonic(self, c: Command):
        on = str(c.get('on', c.arg(0)))
        if on in ('1', 'True', 'true'):
            self.sonic = True
            self.ultrasonicTimer = threading.Timer(0.5, self.sendUltrasonic)
            self.ultrasonicTimer.start()
        else:
            self.sonic = False

    def _h_buzzer(self, c: Command):
        try:
            on = c.get('on')
            if on is None:
                on = c.arg(0)
            value = '1' if on in (True, '1', 'true', 'True') else '0'
            self.buzzer.run(value)
        except Exception:
            pass

    def _h_light(self, c: Command):
        on = str(c.get('on', c.arg(0)))
        if on in ('1', 'True', 'true'):
            self.Light = True
            self.lightTimer = threading.Timer(0.3, self.sendLight)
            self.lightTimer.start()
        else:
            self.Light = False

    def _h_power(self, c: Command):
        try:
            ADC_Power = self.adc.recvADC(2) * 5
            self.send(cmd.CMD_POWER + '#' + str(round(ADC_Power, 2)) + '\n')
        except Exception:
            pass

    def sendUltrasonic(self):
        if self.sonic == True:
            ADC_Ultrasonic = self.ultrasonic.get_distance()
            try:
                self.send(cmd.CMD_MODE + "#" + "3" + "#" + str(ADC_Ultrasonic) + '\n')
            except:
                self.sonic = False
            self.ultrasonicTimer = threading.Timer(0.23, self.sendUltrasonic)
            self.ultrasonicTimer.start()

    def sendLight(self):
        if self.Light == True:
            ADC_Light1 = self.adc.recvADC(0)
            ADC_Light2 = self.adc.recvADC(1)
            try:
                self.send("CMD_MODE#1" + '#' + str(ADC_Light1) + '#' + str(ADC_Light2) + '\n')
            except:
                self.Light = False
            self.lightTimer = threading.Timer(0.17, self.sendLight)
            self.lightTimer.start()

    def sendLine(self):
        if self.Line == True:
            Line1 = 1 if GPIO.input(14) else 0
            Line2 = 1 if GPIO.input(15) else 0
            Line3 = 1 if GPIO.input(23) else 0
            try:
                self.send("CMD_MODE#2" + '#' + str(Line1) + str(Line2) + str(Line3) + '\n')
            except:
                self.Line = False
            self.LineTimer = threading.Timer(0.20, self.sendLine)
            self.LineTimer.start()

    def Power(self):
        while True:
            ADC_Power = self.adc.recvADC(2) * 5
            try:
                self.send(cmd.CMD_POWER + '#' + str(round(ADC_Power, 2)) + '\n')
            except:
                pass
            time.sleep(3)
            if ADC_Power < 10:
                for i in range(4):
                    self.buzzer.run('1')
                    time.sleep(0.1)
                    self.buzzer.run('0')
                    time.sleep(0.1)
            elif ADC_Power < 10.5:
                for i in range(2):
                    self.buzzer.run('1')
                    time.sleep(0.1)
                    self.buzzer.run('0')
                    time.sleep(0.1)
            else:
                self.buzzer.run('0')

if __name__ == '__main__':
    pass