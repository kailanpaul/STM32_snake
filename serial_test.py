import serial
import os
import serial.tools.list_ports
from time import sleep
import struct

var = 0

# sniff for devices
while True:
    try:
        if os.name == 'nt':
            com_list = [comport.device for comport in serial.tools.list_ports.comports()]
            if len(com_list) == 0:
                raise IndexError
            else:
                print("Picking first com port from list: ", com_list)
                sleep(3)
                try:
                    ser = serial.Serial(str(com_list[0]), 9600)
                    break
                except serial.serialutil.SerialException:
                    print("ACCESS DENIED WAH WAH WAH (remember to close other shells)")
                    sleep(2)
                    exit()
        else:
            ser = serial.Serial('/dev/ttyACM0')
            break
    except IndexError:
        print("No devices detected. Retrying...")
        sleep(2)
        continue

# ser.write(b'x')

# print what is read from USB until port is closed
# while True:
#     try:
#         print(ser.read())
#     except serial.serialutil.SerialException:
#         print("Device lost :( Exiting...")
#         break

# string = b''

# angle_command = int(input("Give an angle command between 0 and 50 degrees: "))

# string += struct.pack('!B', angle_command)

# ser.write(string)
# print(string)
while True:
    try:
        var = ser.read()
        print(int(var.decode())+10)
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        break