import serial
import os
import serial.tools.list_ports
from time import sleep

var = 0

# sniff for devices
while True:
    try:
        if os.name == 'nt':
            com_list = [comport.device for comport in serial.tools.list_ports.comports()]
            if len(com_list) == 0:
                raise IndexError
            else:
                print("Using port: ", com_list[0])
                try:
                    ser = serial.Serial(str(com_list[0]), write_timeout=0)
                    break
                except serial.serialutil.SerialException:
                    print("ACCESS DENIED WAH WAH WAH (remember to close other shells)")
                    exit()
        else:
            try:
                ser = serial.Serial('/dev/ttyACM0')
            except serial.serialutil.SerialException:
                raise IndexError
    except IndexError:
        print("No devices detected. Retrying...")
        sleep(2)
        continue

# only send
while True:
    try:
        ser.write(str(input(">> ")).encode())
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        break
ser.close()

# only listen
# while True:
#     try:
#         var = ser.read()
#         print(var.decode())
#     except serial.serialutil.SerialException:
#         print("Device lost :( Exiting...")
#         break
# ser.close()

# send and listen
# while True:
#     ser.write(str(input(">> ")).encode())
#     try:
#         var = ser.read()
#         print(var.decode())
#     except serial.serialutil.SerialException:
#         print("Device lost :( Exiting...")
#         break
# ser.close()