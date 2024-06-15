import serial
import sys
import os
import serial.tools.list_ports
from time import sleep
import numpy as np

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
                    print("ACCESS DENIED WAH WAH WAH")
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
# while True:
#     try:
#         command_deg = float(input(">> ")) # accept angle command as float degrees
#         command_raw = (int(command_deg / 0.326 + 513)).to_bytes(2, 'little') # convert to positional command represented by 2 bytes
#         ser.write(command_raw) 
#     except serial.serialutil.SerialException:
#         print("Device lost :( Exiting...")
#         break
# ser.close()

# data = ser.read(2) # read 2 bytes
# packet = [b for b in data]
# # servo_pos = (((pos_packet[1] & 0x03) << 8 | pos_packet[0]) - 513) * 0.326
# offset = (packet[1] << 8) | packet [0]

# only listen
# while True:
#     try:
#         data = ser.read(8) 
#         packet = [b for b in data]
#         print(packet)
#         for i in range(0, 8, 2):                                         
#             if ((packet[i+1] & 0x80) == 0x80):              
#                 servo_pos = (((packet[1] & 0x03) << 8 | packet[0]) - 513) * 0.326
#                 print(servo_pos, "deg servo")
#             else:
#                 encoder_raw = (packet[i+1] << 8) | packet[i]
#                 calibrated_angle = ((encoder_raw) * 360) / 4095
#                 if (calibrated_angle > 180.0):
#                     calibrated_angle -= 360
#                 print(calibrated_angle, "encoder")
#     except serial.serialutil.SerialException:
#         print("Device lost :( Exiting...")
#         break
# ser.close()

# convert servo position data from bytes to raw position value and then to angle in rad 
def bytes2pos(pos_data_packet):
    return np.deg2rad((((pos_data_packet[1] & 0x03) << 8 | pos_data_packet[0]) - 513) * 0.326)

# convert SEA encoder data from bytes to angle (radians)
def bytes2ang(sea_data_packet):
    sea_raw = (sea_data_packet[1] << 8) | sea_data_packet[0]
    calibrated_angle = (sea_raw * 2 * np.pi) / 4095
    if (calibrated_angle > np.pi):
        calibrated_angle -= 2*np.pi
    return calibrated_angle

sea_data = np.zeros(3)
servo_pos = np.zeros(3)

# send and listen
while True:
    # ser.write(str(input(">> ")).encode())
    try:
        data = ser.read(5)                                                     # read 2 bytes
        serial_packet = [b for b in data]                                                       # put bytes in array
        print("Serial packet: ", serial_packet)
        servo_pos[serial_packet[0]] = bytes2pos(serial_packet[1:3])
        sea_data[serial_packet[0]] = bytes2ang(serial_packet[3:5])
        print("Servo data: ", np.rad2deg(servo_pos))
        print("SEA data: ", np.rad2deg(sea_data))
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        break
ser.close()