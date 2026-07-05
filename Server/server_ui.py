from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_server_ui(object):
    def setupUi(self, server_ui):
        server_ui.setObjectName("server_ui")
        server_ui.resize(400, 300)
        font = QtGui.QFont()
        font.setFamily("Arial")
        server_ui.setFont(font)
        
        server_ui.setStyleSheet("""
        QWidget {
            background: #F0F8FF;
            color: #333333;
        }
        QPushButton {
            border-style: none;
            border-radius: 5px;
            padding: 5px;
            color: #FFFFFF;
            background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #4169E1, stop:1 #1E90FF);
        }
        QPushButton:hover {
            background-color: #6495ED;
        }
        QPushButton:pressed {
            background-color: #4682B4;
        }
        QLabel {
            color: #333333;
            border: none;
            background: none;
        }
        """)

        self.centralwidget = QtWidgets.QWidget(server_ui)
        self.centralwidget.setObjectName("centralwidget")
        server_ui.setCentralWidget(self.centralwidget)

        # 主布局
        self.main_layout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(10)

        # 顶部布局（用于放置最小化和关闭按钮）
        self.top_layout = QtWidgets.QHBoxLayout()
        self.top_layout.addStretch(1)  # 添加弹性空间，将按钮推到右侧

        # 最小化和关闭按钮
        self.pushButton_Min = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_Min.setText("-")
        self.pushButton_Min.setFixedSize(30, 30)
        self.pushButton_Close = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_Close.setText("×")
        self.pushButton_Close.setFixedSize(30, 30)
        
        self.top_layout.addWidget(self.pushButton_Min)
        self.top_layout.addWidget(self.pushButton_Close)

        self.main_layout.addLayout(self.top_layout)

        # 标题 "Dddd"
        self.label_title = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(24)
        font.setBold(True)
        self.label_title.setFont(font)
        self.label_title.setAlignment(QtCore.Qt.AlignCenter)
        self.label_title.setText("Lafvin")
        self.main_layout.addWidget(self.label_title)

        # 添加一些空间
        self.main_layout.addStretch(1)

        # 服务器状态标签
        self.label_status = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_status.setFont(font)
        self.label_status.setAlignment(QtCore.Qt.AlignCenter)
        self.label_status.setText("Server Off")
        self.main_layout.addWidget(self.label_status)

        # 服务器按钮
        self.Button_Server = QtWidgets.QPushButton(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Button_Server.setFont(font)
        self.Button_Server.setText("Start Server")
        self.Button_Server.setFixedSize(150, 40)
        self.main_layout.addWidget(self.Button_Server, 0, QtCore.Qt.AlignCenter)

        # 添加一些空间
        self.main_layout.addStretch(1)

        self.retranslateUi(server_ui)
        QtCore.QMetaObject.connectSlotsByName(server_ui)

    def retranslateUi(self, server_ui):
        _translate = QtCore.QCoreApplication.translate
        server_ui.setWindowTitle(_translate("server_ui", "RaspberryCar Server"))