import io
import os
import socket
import struct
import time
import picamera2
import sys, getopt
from Thread import *
from threading import Thread
from server import Server
from server_ui import Ui_server_ui
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

class MyWindow(QMainWindow, Ui_server_ui):
    def __init__(self):
        self.user_ui = True
        self.start_tcp = False
        self.port = 8000
        self.parseOpt()

        self.TCP_Server = Server()

        if self.user_ui:
            self.app = QApplication(sys.argv)
            super(MyWindow, self).__init__()
            self.setupUi(self)
            self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
            self.Button_Server.clicked.connect(self.on_server_button_clicked)
            self.pushButton_Close.clicked.connect(self.close)
            self.pushButton_Min.clicked.connect(self.showMinimized)
            
            # 添加鼠标事件以实现窗口拖动
            self.mousePressEvent = self.on_mouse_press
            self.mouseMoveEvent = self.on_mouse_move
            self.mouseReleaseEvent = self.on_mouse_release
        
        if self.start_tcp:
            self.start_server()

    def parseOpt(self):
        self.opts, self.args = getopt.getopt(sys.argv[1:], "tnp")
        for o, a in self.opts:
            if o in ('-t'):
                print("Open TCP")
                self.start_tcp = True
            elif o in ('-n'):
                self.user_ui = False
            elif o in ('-p'):
                self.port = 5001

    def start_server(self):
        self.TCP_Server.StartTcpServer()
        self.ReadData = Thread(target=self.TCP_Server.readdata)
        self.SendVideo = Thread(target=self.TCP_Server.sendvideo)
        self.power = Thread(target=self.TCP_Server.Power)
        self.SendVideo.start()
        self.ReadData.start()
        self.power.start()
        if self.user_ui:
            self.label_status.setText("Server On")
            self.Button_Server.setText("Stop Server")

    def stop_server(self):
        try:
            stop_thread(self.SendVideo)
            stop_thread(self.ReadData)
            stop_thread(self.power)
        except:
            pass
        try:
            self.TCP_Server.server_socket.shutdown(2)
            self.TCP_Server.server_socket1.shutdown(2)
            self.TCP_Server.StopTcpServer()
        except:
            pass
        print("Close TCP")
        if self.user_ui:
            self.label_status.setText("Server Off")
            self.Button_Server.setText("Start Server")

    def on_server_button_clicked(self):
        if self.label_status.text() == "Server Off":
            self.start_server()
        elif self.label_status.text() == 'Server On':
            self.stop_server()

    def close(self):
        self.stop_server()
        if self.user_ui:
            QCoreApplication.instance().quit()
        os._exit(0)

    # 以下三个方法用于实现窗口拖动
    def on_mouse_press(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_press_pos = event.globalPos() - self.pos()
            event.accept()

    def on_mouse_move(self, event):
        if event.buttons() == Qt.LeftButton:
            self.move(event.globalPos() - self.mouse_press_pos)
            event.accept()

    def on_mouse_release(self, event):
        self.mouse_press_pos = None

if __name__ == '__main__':
    try:
        myshow = MyWindow()
        if myshow.user_ui:
            myshow.show()
            sys.exit(myshow.app.exec_())
        else:
            try:
                while True:
                    pass
            except KeyboardInterrupt:
                myshow.close()
    except KeyboardInterrupt:
        myshow.close()