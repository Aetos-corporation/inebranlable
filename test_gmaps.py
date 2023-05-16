from serial import Serial
import time
import struct

device = None

def callback():
    print("=========================")
    message = device.read(3)
    print("message: " + str(message))
    func = message[0]
    mode = message[1]
    size = message[2]
    data = device.read(int(size))
    if 1 <= int(func) <= 8:
        print("data: " + str(round(struct.unpack('f', data)[0], 3)))
    else:
        data = str(data)[2:]
        data = data[:-1]
        print("data: " + str(data))
    print("=========================")
    return func, mode, size, data

def receive():
    if device.in_waiting > 0:
        func, wr, size, data = callback()
        print("func: " + str(func) + " | wr: " + str(wr) + " | size: " + str(size) + "| data: " + str(data))

if __name__ == '__main__':
    device = Serial('COM3', 9600)
    while True:
        receive()
        time.sleep(1)
