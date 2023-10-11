import serial
import sys
import os
import serial.tools.list_ports
from time import sleep

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
                    ser = serial.Serial(str(com_list[0]), write_timeout=0, parity=serial.PARITY_NONE)
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
        command_deg = float(input(">> ")) # accept angle command as float degrees
        command_raw = (int(command_deg / 0.326 + 513)).to_bytes(2, 'little') # convert to positional command represented by 2 bytes
        ser.write(command_raw) 
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        break
ser.close()

# only listen
# while True:
#     try:
#         data = ser.read(2) # read 2 bytes
#         pos_packet = [b for b in data]
#         servo_pos = (((pos_packet[1] & 0x03) << 8 | pos_packet[0]) - 513) * 0.326 # convert bytes to raw 10-bit and then angle, in deg
#         print(servo_pos)
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