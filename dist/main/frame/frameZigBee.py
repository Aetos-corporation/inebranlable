
from time import sleep
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtGui import QPixmap, QTransform, QIcon
from PyQt6.QtCore import QRect
from threading import Thread, Event

from gui.ihm_inebranlable_v3 import Ui_MainWindow as Inebranlable
from zigbee.zigBee import zigBeeComDevice
from log.log import Log

TIME_UPDATE_GUI_S = 0.1


class frameZigBee(object):
    def __init__(self, data, app):
        """
        Initialization of the frame for zigbee use
        ``Args:``
            ``data:`` dictionnary, contains the port, name and baudrate of the zigbee device
            ``app:``  Qapplication, application to execute PyQt6
        ``Raises:``
        ``Return:``
        """
        self.app = app
        self.data = data
        self.window = QMainWindow()
        self.gui = Inebranlable()
        self.data = data

        # Variables to know if each level of log is available
        self.log_info = True
        self.log_warning = True
        self.log_error = True

        # device to read the serial port for the xbee module
        self.device = zigBeeComDevice(self, self.data['port'])

        # used to store logs in a file
        self.trace = Log()

        # Thread to update the gui with the elements in the gui_objects dictionnary
        self.thread_refresh_gui = Thread(target=self.update_gui)
        self.thread_refresh_gui_alive = Event()
        self.thread_refresh_gui_alive.set()

        # Dictionnary keeping the value of all the widgets in the gui
        self.gui_objects = { "latitude": "",
                             "longitude": "",
                             "dest_latitude": "0",
                             "dest_longitude": "0",
                             "init_latitude": "",
                             "init_longitude": "",
                             "wind_direction": "",
                             "yaw": "",
                             "servo_safran": "",
                             "servo_voile": "",
                             "roll": "",
                             "pitch": ""
                           }

        # Coordinates for the right top corner of the map
        self.right_top_corner_latitude = 47.256710
        self.right_top_corner_longitude = -1.364716

        # Coordinates of the bottom left corner of the map
        self.left_bottom_corner_latitude = 47.250532
        self.left_bottom_corner_longitude = -1.378632

        # Pixmap for the red arrow in the compass
        self.pixmap_arrow = QPixmap('resources/red_arrow.png')

        # Pixmap for the red triangle representing the boat on the map
        self.pixmap_boat = QPixmap('resources/red_triangle.png')

        # Initialization of the gui
        self.gui.setupUi(self.window)
        self.gui.retranslateUi(self.window)
        self.window.show()
        self.connect_events()
        self.set_pictures()

        # Starts the thread to update the gui
        self.thread_refresh_gui.start()

        self.trace.log("SYSTEM", "Frame initialized and ready")
    
    def set_pictures(self):
        """
        Method to set the pictures in the interface
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        # set the pixmap for the map
        pixmap = QPixmap('resources/maps/lake_resized.png')
        self.gui.label_image_map.setPixmap(pixmap)

        # Set the pixmap for the boat
        t = QTransform()
        t.rotate(180)
        self.pixmap_boat = self.pixmap_boat.transformed(t)
        self.gui.label_boat.setPixmap(self.pixmap_boat)
        self.gui.label_boat.setScaledContents(True)
        self.gui.label_boat.setGeometry(QRect(541-334, 361-147, 20, 20))

        # Set the pixmap for the compass
        compass = QPixmap('resources/compass.png')
        self.gui.label_image_compass.setPixmap(compass)
        self.gui.label_image_compass.setScaledContents(True)

        # Set the pixmap for the red arrow in the compass
        t = QTransform()
        t.rotate(180)
        self.pixmap_arrow = self.pixmap_arrow.transformed(t)
        self.gui.label_red_arrow.setPixmap(self.pixmap_arrow)
        self.gui.label_red_arrow.setScaledContents(True)

        # Set the icon of the application
        icon = QIcon("resources/icon_inebranlable.JPG")
        self.app.setWindowIcon(icon)

        self.trace.log("SYSTEM", "Pictures init done")

    def connect_events(self):
        """
        Connects all the events for the user interface to work
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        # Connects the pushbutton pushButton_change_dest_coordinates to method change_dest_location 
        self.gui.pushButton_change_dest_coordinates.clicked.connect(self.change_dest_location)
        # Connects the checkbox checkBox_log_level_info to method log_level_info
        self.gui.checkBox_log_level_info.stateChanged.connect(self.log_level_info)
        # Connects the checkbox checkBox_log_level_warnings to method log_level_warning
        self.gui.checkBox_log_level_warnings.stateChanged.connect(self.log_level_warning)
        # Connects the checkbox checkBox_log_level_errors to method log_level_error
        self.gui.checkBox_log_level_errors.stateChanged.connect(self.log_level_error)
        # Connects the button pushButton_clear_console to method clear_console
        self.gui.pushButton_clear_console.clicked.connect(self.clear_console)
        # Connects the button to kill the application to method stop_application
        self.app.aboutToQuit.connect(self.stop_application)

        self.trace.log("SYSTEM", "Connect events done")
    
    def stop_application(self):
        """
        Stops the application and the logs
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        self.device.close_device()
        self.trace.log("SYSTEM", "Device ZigBee closed")
        self.thread_refresh_gui_alive.clear()
        self.trace.log("SYSTEM", "Thread to update gui ended")
        self.trace.close_log()

    def change_dest_location(self):
        """
        Sends the new coordinates to the boat
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        self.dest_location_N = self.gui.lineEdit_new_dest_location_N.text()
        self.dest_location_W = self.gui.lineEdit_new_dest_location_W.text()
        self.gui_objects["dest_location_N"] = self.dest_location_N
        self.gui_objects["dest_location_W"] = self.dest_location_W
        
        self.trace.log("COMMAND", "Change destination coordinates")

        #TODO: complete sending method

    def log_level_info(self):
        """
        Changes the state of the log info to know if they can be displayed
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        if self.gui.checkBox_log_level_info.isChecked():
            self.log_info = True
        else:
            self.log_info = False

        self.trace.log("SYSTEM", "Log level info set to {}".format(self.log_info))

    def log_level_warning(self):
        """
        Changes the state of the log wrning to know if they can be displayed
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        if self.gui.checkBox_log_level_warnings.isChecked():
            self.log_warning = True
        else:
            self.log_warning = False
        
        self.trace.log("SYSTEM", "Log level warning set to {}".format(self.log_info))

    def log_level_error(self):
        """
        Changes the state of the log error to know if they can be displayed
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        if self.gui.checkBox_log_level_errors.isChecked():
            self.log_error = True
        else:
            self.log_error = False
        
        self.trace.log("SYSTEM", "Log level error set to {}".format(self.log_info))

    def clear_console(self):
        """
        Clears the log console when pushbutton pushButton_clear_console is clicked
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        self.gui.textEdit_logs.clear()

    def update_gui_object_dict(self, func, wr, size, data):
        """
        Updates the dictionnary used to store all the text in the widgets
        See frame documentation for the precise values for func, wr and size
        ``Args:``
            ``func:`` string, number to know what sensor or level of log
            ``wr:``   string, mode (read/write)
            ``size:`` string, size of the data (string representing an int)
            ``data:`` string, data received
        ``Raises:``
        ``Return:``
        """
        if int(func) == 1:
            self.gui_objects["latitude"] = data
            self.trace.log("SENSOR", "Latitude value received: {}".format(data))
        elif int(func) == 2:
            self.gui_objects["longitude"] = data
            self.trace.log("SENSOR", "Longitude value received: {}".format(data))
        elif int(func) == 3:
            self.gui_objects["wind_direction"] = data
            self.trace.log("SENSOR", "Wind direction value received: {}".format(data))
        elif int(func) == 4:
            self.gui_objects["roll"] = data
            self.trace.log("SENSOR", "Roll value received: {}".format(data))
        elif int(func) == 5:
            self.gui_objects["pitch"] = data
            self.trace.log("SENSOR", "Pitch value received: {}".format(data))
        elif int(func) == 6:
            self.gui_objects["yaw"] = data
            self.trace.log("SENSOR", "Yaw value received: {}".format(data))
            self.update_compass_arrow_pos()
        elif int(func) == 7:
            self.gui_objects["servo_safran"] = data
            self.trace.log("SENSOR", "Servo safran value received: {}".format(data))
        elif int(func) == 8:
            self.gui_objects["servo_voile"] = data
            self.trace.log("SENSOR", "Servo voile value received: {}".format(data))
        elif int(func) >= 9 and int(func) <= 11:
            self.update_log_console(func, data)
        
    def update_log_console(self, func, data):
        """
        Updates the text edit if the data received is a log
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        if int(func) == 9:
            if self.log_info:
                self.gui.textEdit_logs.append("[INFO]   : " + data)
            self.trace.log("LOG", "[INFO]: received: {}".format(data))
        elif int(func) == 10:
            if self.log_warning:
                self.gui.textEdit_logs.append("[WARNING]: " + data)
            self.trace.log("LOG", "[WARNING]: received: {}".format(data))
        elif int(func) == 11:
            if self.log_error:
                self.gui.textEdit_logs.append("[ERROR]  : " + data)
            self.trace.log("LOG", "[ERROR]: received: {}".format(data))

    def update_compass_arrow_pos(self):
        """
        Updates the orientation of the red arrow on the compass
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        def try_float(val):
            """
            Tries to convert val into float
            ``Args:``
                ``val:`` string
            ``Raises:``
                ``ValueError:`` if val cannot be converted to float
            ``Return:``
                ``True:`` if val can be converted to float
                ``False:`` if val cannot be converted to float
            """
            try:
                float(val)
                return True
            except ValueError:
                self.trace.log("DEBUG", "Yaw value is not float")
                return False

        if try_float(self.gui_objects["yaw"]):
            t = QTransform()
            t.rotate(float(self.gui_objects["yaw"]))
            pixmap = self.pixmap_arrow.transformed(t)
            self.gui.label_red_arrow.setPixmap(pixmap)
            self.gui.label_red_arrow.setScaledContents(True)

            self.trace.log("SYSTEM", "Changed compass orientation")

    def update_gui(self):
        """
        Takes the values from the dictionnary gui_objects and updates the graphic interface
        Function only called in a thread
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        while self.thread_refresh_gui_alive.is_set():
            self.gui.lineEdit_location_N.setText(self.gui_objects["latitude"])
            self.gui.lineEdit_location_W.setText(self.gui_objects["longitude"])
            self.gui.lineEdit_dest_location_N.setText(self.gui_objects["dest_latitude"])
            self.gui.lineEdit_dest_location_W.setText(self.gui_objects["dest_longitude"])
            self.gui.lineEdit_init_location_N.setText(self.gui_objects["init_latitude"])
            self.gui.lineEdit_init_location_W.setText(self.gui_objects["init_longitude"])
            self.gui.lineEdit_wind_direction.setText(self.gui_objects["wind_direction"])
            self.gui.lineEdit_yaw.setText(self.gui_objects["yaw"])
            self.gui.lineEdit_servo_safran.setText(self.gui_objects["servo_safran"])
            self.gui.lineEdit_servo_voile.setText(self.gui_objects["servo_voile"])
            self.gui.lineEdit_roll.setText(self.gui_objects["roll"])
            self.gui.lineEdit_pitch.setText(self.gui_objects["pitch"])

            sleep(TIME_UPDATE_GUI_S)
