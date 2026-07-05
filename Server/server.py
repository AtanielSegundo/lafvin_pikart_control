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
        """Handle a single command string. Called by readdata() and web handler."""
        with self.cmd_lock:
            data = oneCmd.split("#")
            if data is None:
                return
            elif cmd.CMD_MODE in data:
                if data[1] == 'one' or data[1] == "0":
                    self.stopMode()
                    self.Mode = 'one'
                
                elif data[1] == 'two' or data[1] == "1":
                    self.stopMode()
                    self.Mode = 'two'
                    self.lightRun = Thread(target=self.light.run)
                    self.lightRun.start()
                    self.Light = True
                    self.lightTimer = threading.Timer(0.3, self.sendLight)
                    self.lightTimer.start()
                
                elif data[1] == 'three' or data[1] == "3":
                    self.stopMode()
                    self.Mode = 'three'
                    self.ultrasonicRun = threading.Thread(target=self.ultrasonic.run)
                    self.ultrasonicRun.start()
                    self.sonic = False
                    self.ultrasonicTimer = threading.Timer(5, self.sendUltrasonic)
                    self.ultrasonicTimer.start()
                
                elif data[1] == 'four' or data[1] == "2":
                    self.stopMode()
                    self.Mode = 'four'
                    self.infraredRun = threading.Thread(target=self.infrared.run)
                    self.infraredRun.start()
                    self.Line = True
                    self.lineTimer = threading.Timer(0.4, self.sendLine)
                    self.lineTimer.start()

            elif (cmd.CMD_MOTOR in data) and self.Mode == 'one':
                try:
                    data1=int(data[1])
                    data2=int(data[2])
                    data3=int(data[3])
                    data4=int(data[4])
                    self.PWM.setMotorModel(data1, data2, data3, data4)
                except:
                    pass
            elif (cmd.CMD_M_MOTOR in data) and self.Mode == 'one':
                try:
                    data1 = int(data[1])
                    data2 = int(data[2])
                    data3 = int(data[3])
                    data4 = int(data[4])

                    LX = int(data2 * math.sin(math.radians(data1)))
                    LY = int(data2 * math.cos(math.radians(data1)))
                    RX = int(data4 * math.sin(math.radians(data3)))
                    RY = int(data4 * math.cos(math.radians(data3)))

                    FR = LY - LX + RX
                    FL = LY + LX - RX
                    BL = LY - LX - RX
                    BR = LY + LX + RX

                    self.PWM.setMotorModel(FL, BL, FR, BR)
                except:
                    pass
            elif (cmd.CMD_CAR_ROTATE in data) and self.Mode == 'one':
                try:
                    data1 = int(data[1])
                    data2 = int(data[2])
                    data3 = int(data[3])
                    data4 = int(data[4])
                    if data4 == 0:
                        try:
                            stop_thread(self._rotate_mode)
                            self.rotation_flag = False
                        except:
                            pass
                        LX = int(data2 * math.sin(math.radians(data1)))
                        LY = int(data2 * math.cos(math.radians(data1)))
                        RX = int(data4 * math.sin(math.radians(data3)))
                        RY = int(data4 * math.cos(math.radians(data3)))

                        FR = LY - LX + RX
                        FL = LY + LX - RX
                        BL = LY - LX - RX
                        BR = LY + LX + RX

                        self.PWM.setMotorModel(FL, BL, FR, BR)
                    elif self.rotation_flag == False:
                        self.angle = data[3]
                        try:
                            stop_thread(self._rotate_mode)
                        except:
                            pass
                        self.rotation_flag = True
                        self._rotate_mode = Thread(target=self.PWM.Rotate, args=(data3,))
                        self._rotate_mode.start()
                except:
                    pass
            elif cmd.CMD_SERVO in data:
                try:
                    data1 = data[1]
                    data2 = int(data[2])
                    self.servo.setServoPwm(data1, data2)
                except:
                    pass

            elif cmd.CMD_LED in data:
                try:
                    data1=int(data[1])
                    data2=int(data[2])
                    data3=int(data[3])
                    data4=int(data[4])
                    self.led.ledIndex(data1, data2, data3, data4)
                except:
                    pass
            elif cmd.CMD_LED_MOD in data:
                self.LedMoD = data[1]
                if self.LedMoD == '0':
                    try:
                        stop_thread(self._led_mode)
                    except:
                        pass
                if self.LedMoD == '1':
                    try:
                        stop_thread(self._led_mode)
                    except:
                        pass
                    self.led.ledMode(self.LedMoD)
                    time.sleep(0.1)
                    self.led.ledMode(self.LedMoD)
                else:
                    try:
                        stop_thread(self._led_mode)
                    except:
                        pass
                    time.sleep(0.1)
                    self._led_mode = Thread(target=self.led.ledMode, args=(data[1],))
                    self._led_mode.start()
            elif cmd.CMD_SONIC in data:
                if data[1] == '1':
                    self.sonic = True
                    self.ultrasonicTimer = threading.Timer(0.5, self.sendUltrasonic)
                    self.ultrasonicTimer.start()
                else:
                    self.sonic = False
            elif cmd.CMD_BUZZER in data:
                try:
                    self.buzzer.run(data[1])
                except:
                    pass
            elif cmd.CMD_LIGHT in data:
                if data[1] == '1':
                    self.Light = True
                    self.lightTimer = threading.Timer(0.3, self.sendLight)
                    self.lightTimer.start()
                else:
                    self.Light = False
            elif cmd.CMD_POWER in data:
                ADC_Power = self.adc.recvADC(2) * 5
                try:
                    self.send(cmd.CMD_POWER + '#' + str(round(ADC_Power, 2)) + '\n')
                except:
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