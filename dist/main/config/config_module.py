import sys, serial, glob
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtGui import QIcon

from gui.config import Ui_Configuration as config

class configWindow(object):
    def __init__(self, app):
        self.app = app
        self.window = QMainWindow()
        self.config_window = config()
        self.config_window.setupUi(self.window)
        self.config_window.retranslateUi(self.window)
        self.connect_signals()
        self.scan_serial_ports()
        self.window.show()
        self.data = {"name": "",
                     "port": "",
                     "baudrate": 0,
                    }
        
        icon = QIcon("resources/icon_inebranlable.JPG")
        self.app.setWindowIcon(icon)
    
    def connect_signals(self):
        self.config_window.pushButton_start.clicked.connect(self.start_gui_app)

    def start_gui_app(self):
        from main import open_frame_zigbee
        self.update_dict()
        open_frame_zigbee(self.data, self.app)
        self.window.close()

    def scan_serial_ports(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                self.config_window.comboBox_port.addItem(port)
            except (OSError, serial.SerialException):
                pass

    def update_dict(self):
        self.data["name"] = self.config_window.lineEdit_name.text()
        self.data["port"] = self.config_window.comboBox_port.currentText()
        self.data["baudrate"] = self.config_window.lineEdit_baudrate.text()
