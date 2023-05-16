import sys
from PyQt6.QtWidgets import QApplication
from config.config_module import configWindow
from frame.frameZigBee import frameZigBee

__version__ = 1.0

frame = None
app = None

def open_frame_zigbee(_data, app):
    global frame
    frame = frameZigBee(_data, app)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    config = configWindow(app)
    sys.exit(app.exec())