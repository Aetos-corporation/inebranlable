import time
import struct
from queue import Queue
from serial import Serial
from threading import Thread, Event

from zigbee.utils import *

DEFAULT_BAUDRATE = 9600

QUEUE = Queue()

TIME_SLEEP_RECEIVE_S = 0.1
TIME_SLEEP_READ_DATA_FROM_QUEUE = 0.1

def read_message_from_queue():
    """
        Function to read messages from the boat
        ``Args:``
        ``Raises:``
        ``Return:``
                Returns the func, wr, size and data if there's a message in the queue, else returns empty strings
    """
    if not QUEUE.empty():
        while not QUEUE.empty():
            boat_message = QUEUE.get()
            return boat_message.func, boat_message.wr, boat_message.size, boat_message.data
    else:
        return "", "", "", ""


class messageFromBoat(object):
    """
        Class to create a message received from the zigbee
        ``Args:``
        ``Raises:``
        ``Return:``
    """
    def __init__(self, func, wr, size, data):
        """
        Initialization of the message
        ``Args:``
            ``func:`` string, number to know what sensor or level of log
            ``wr:``   string, mode (read/write)
            ``size:`` string, size of the data (string representing an int)
            ``data:`` string, data received
        ``Raises:``
        ``Return:``
        """
        self.func = func
        self.wr = wr
        self.size = size
        self.data = data


class zigBeeComDevice(object):
    """
        Class to handle the serial port on which the zigbee is connected
        ``Args:``
        ``Raises:``
        ``Return:``
    """
    def __init__(self, frame, port_com, baudrate=DEFAULT_BAUDRATE):
        """
        Initialization of the port to use the zigbee module
        ``Args:``
            ``frame``: frameZigBee, contains an instance of the frame to call method update_gui_object_dict
            ``port_com``: string, port on which the zigbee device is connected
            ``baudrate``: int, baudrate to use the serial port (def: DEFAULT_BAUDRATE)
        ``Raises:``
        ``Return:``
        """
        self.port = port_com
        self.baudrate = baudrate
        self.frame = frame
        if self.port != "":
            self.device = Serial(str(self.port), int(self.baudrate))

        # Thread to read on the serial port the data received
        self.thread_receive = Thread(target=self.receive)
        self.thread_receive_alive = Event()
        self.thread_receive_alive.set()

        # Thread to read the data received and added to the queue
        self.thread_read = Thread(target=self.read_data_in_queue)
        self.thread_read_alive = Event()
        self.thread_read_alive.set()

        # Start the threads
        self.thread_receive.start()
        self.thread_read.start()

    def close_device(self):
        """
        Method to close the device
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        self.stop_threads()
        self.device.close()
    
    def stop_threads(self):
        """
        Method to end properly the threads and kill them
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        self.thread_receive_alive.clear()
        self.thread_read_alive.clear()

    def receive(self):
        """
        Method called only in a thread to read the serial port, create the messageBoat and put it in a queue
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        while self.thread_receive_alive.is_set():
            if self.device.in_waiting > 0:
                func, wr, size, data = self.callback()
                boat_message = messageFromBoat(func, wr, size, data)
                QUEUE.put(boat_message)
            time.sleep(TIME_SLEEP_RECEIVE_S)

    def callback(self):
        """
        Method called to read the different informations from the serial port
        ``Args:``
        ``Raises:``
        ``Return:``
            Returns the func, mode, size and data according to the doc
        """
        message = self.device.read(3)
        func = message[0]
        mode = message[1]
        size = message[2]
        data = self.device.read(int(size))
        if 1 <= func <= 2 and len(data) == 4:
            data = str(round(struct.unpack('f', data)[0], 6))
        elif 3 <= func <= 8 and len(data) == 4:
            data = str(round(struct.unpack('f', data)[0], 3))
        else:
            data = str(data)[2:]
            data = data[:-1]
        self.frame.trace.log("DEBUG", "func: {}| mode: {}| size: {}| data: {}".format(func, mode, size, data))
        print("func: " + str(func) + " | wr: " + str(mode) + " | size: " + str(size) + "| data: " + str(data))
        return func, mode, size, data
    
    def read_data_in_queue(self):
        """
        Method only called in a thread to read messages from the queue and call update_gui_object_dict to update the gui
        ``Args:``
        ``Raises:``
        ``Return:``
        """
        while self.thread_read_alive.is_set():
            func, wr, size, data = read_message_from_queue()
            if func != "":
                self.frame.update_gui_object_dict(func, wr, size, data)
            time.sleep(TIME_SLEEP_READ_DATA_FROM_QUEUE)
