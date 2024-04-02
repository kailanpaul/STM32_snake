#------------------------------------------------------------------------------------------------------------
#
#   Behold, THE Noodle controller
#
#   Made with love, by Gordon and Kailan
#
#   Oct. 2023
#
#------------------------------------------------------------------------------------------------------------

import os
# import serial
# import serial.tools.list_ports
import numpy as np
import time
import asyncio
from serial_asyncio import open_serial_connection
from sshkeyboard import listen_keyboard_manual

#------------------------------------------------------------------------------------------------------------

# constants

#-----------------------------------------------------------------------------------------------------
N_JOINTS = 8                                                                        # number of joints
#-----------------------------------------------------------------------------------------------------

DATA_SIZE = 2                                                                       # all data (SEA and servo) is 2 bytes in size                                            
SERIAL_PACKET_SIZE = 2 * DATA_SIZE + 1                                              # total number of bytes in a serial packet
SERIAL_DECODE_MASK = 0x80                                                           # decode serial data encoded on STM32 side

COMMAND_FREQ = 15                                                                   # frequency of commands (Hz)
COMMAND_PERIOD = 1/COMMAND_FREQ                                                     # time between commands in seconds

A = np.pi/8                                                                         # sine amplitude
OMEGA = np.pi                                                                       # temporal freq.
PHI = ((-2*np.pi-0.4))/5                                                            # spatial freq.

K = 3.54                                                                            # torsional stiffness constant of SEE (Nm/rad)
K_D = 0.5*K                                                                        # admittance/impedance constant
K_AI = (K - K_D) / (K * K_D)                                                        # admittance/impedance gain

ROM_P = np.deg2rad(50)                                                              # range of motion positive limit
ROM_M = -np.deg2rad(50)                                                             # range of motion negative limit
JOINT_DATA_ID = 6                                                                   # joint to have servo and SEA data saved
TORQUE_CONTROL_MODE = 2                                                             # 1 for admittance, 2 for impedance

RUN_TIME = 45

go = 0

#------------------------------------------------------------------------------------------------------------

# # sniff for devices
# async def sniff():
#     while True:
#         try:
#             if os.name == 'nt':
#                 com_list = [comport.device for comport in serial.tools.list_ports.comports()]
#                 if len(com_list) == 0:
#                     raise IndexError
#                 else:
#                     print("Using port: ", com_list[0])
#                     try:
#                         ser = serial.Serial(str(com_list[0]), write_timeout=0, parity=serial.PARITY_NONE)
#                         return ser
#                     except serial.serialutil.SerialException:
#                         print("ACCESS DENIED WAH WAH WAH")
#                         exit()
#             else:
#                 try:
#                     ser = serial.Serial('/dev/ttyACM0')
#                     return ser
#                 except serial.serialutil.SerialException:
#                     raise IndexError
#         except IndexError:
#             print("No devices detected. Retrying...")
#             time.sleep(2)
#             continue

# gait pattern generator: generates joint angles (rad) for 'n' joints, to achieve pose corresponding to time step
# approximates a sine wave
def gpg(time, n):

    # initialise empty arrays to store link end points and desired joint angles
    points = np.zeros(n+2)
    desired_pos = np.zeros(n)

    # get points on sine wave for n links
    for i in range(1, (n+3)):
        points_i = A*np.sin(time*OMEGA + PHI*(i-1))
        points[i-1] = points_i

    # get gradients of links
    m = np.zeros(n+1)
    for l in range(0, len(points)-1):
        m[l] = points[l+1]-points[l]

    # get desired joint angles in radians from link gradients using trig identity
    for k in range(0, len(m)-1):
        desired_pos[k] = np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k])
    
    # return joint angles in radians
    return desired_pos

# send 'command' (bytes) over serial to port 'com'
# async def send_command(com, command):
#     try:
#         com.write(command)                                                                     
#     except serial.serialutil.SerialException:
#         print("Device lost :( Exiting...")
#         com.close()
#         return  

# convert servo position in degrees to raw position value and then to bytes
def pos2bytes(pose):
    bytes_array = [0] * (len(pose) * 2)
    for i in range(0, len(pose)):
        bytes_array[(2*i):((2*i)+2)] = (int((pose[i] / 0.326) + 512)).to_bytes(2, 'little')
    return bytes_array

# convert servo position data from bytes to raw position value and then to angle in rad 
def bytes2pos(pos_data_packet):
    return np.deg2rad((((pos_data_packet[1] & 0x03) << 8 | pos_data_packet[0]) - 513) * 0.326)

# convert SEA encoder data from bytes to angle (radians)
def bytes2ang(sea_data_packet):
    sea_raw = (sea_data_packet[1] << 8) | sea_data_packet[0]
    converted_angle = (sea_raw * 2 * np.pi) / 4095
    if (converted_angle > np.pi):
        curated_angle = converted_angle - 2*np.pi
    else:
        curated_angle = converted_angle
    if (curated_angle > np.deg2rad(30)):
        curated_angle = 0
    return curated_angle

async def press(key):
    if key == "up":
        global go
        go = 1

async def release(key):
    if key == "up":
        global go
        go = 0

#------------------------------------------------------------------------------------------------------------

async def main():

    # initialize variables

    sea_data = np.zeros(N_JOINTS)
    servo_pos = np.zeros(N_JOINTS)
    desired_pos = np.zeros(N_JOINTS) 
    desired_pos_fb = np.zeros(N_JOINTS)  

    # data storage arrays
    joint_data = [0, 0, 0]
    header = ['time', 'servo data', 'SEA data']                              

    reader, writer = await open_serial_connection(url='/dev/ttyACM0', baudrate=115200)                                              

    # print("PlEAsE wAiT")

    # wait for servo to init and zero
    # serial.close()
    # time.sleep(1) 
    # serial.open()

    writer.write(bytes(pos2bytes(np.rad2deg(gpg(0.2, 8)))))

    # time.sleep(1)

    start_time = time.time()                                                                           
    prev_command_time = time.time()
    prev_time = time.time()

    await asyncio.sleep(0.1)

    print("Start")

    while (1):
        await asyncio.sleep(0.1)

        current_time = time.time()                                                    

        try:
            # FEEDBACK..

            # data = await reader.read(SERIAL_PACKET_SIZE)                                                      # read 2 bytes
            # serial_packet = [b for b in data]                                                           # put bytes in array
            # servo_pos[serial_packet[0]] = bytes2pos(serial_packet[1:3])                                 # update servo data in index indicated by packet
            # sea_data[serial_packet[0]] = bytes2ang(serial_packet[3:5])                                  # update SEA data in index indicated by packet

            # # useful for checking if SEA data is rubbish or not
            # # if(serial_packet[0] == 5):
            # #     print(serial_packet)

            # if (serial_packet[0] == JOINT_DATA_ID):
            #     joint_data = np.vstack([joint_data, ([current_time - start_time,
            #                                           servo_pos[JOINT_DATA_ID],
            #                                           sea_data[JOINT_DATA_ID]
            #                                           ])])

            # # ..CONTROL
            if go:
                effective_time = current_time - off_time
                off_time = 0
                if (effective_time - prev_command_time) >= COMMAND_PERIOD:                        # check if command period has passed since last command                                       
                    desired_pos = gpg(round((effective_time - start_time), 3), N_JOINTS)                      # calculate desired pose command using GPG for current time (in seconds)
                    # desired_pos = np.zeros(8)
                    prev_command_time = current_time                                                        # update time of previous command

                    # print the time stamp
                    # print("t =", int((current_time - start_time) / 60), "min", round((current_time - start_time)  % 60, 3), "sec")


                # apply torque feedback and soft limit

                #-----------------------------------------------------------------------------------------------------------------------------------------------
                # if ((current_time - start_time) < 10):
                #     K_AI = 0
                # elif ((current_time - start_time) < 20):
                #     K_AI = (K - 0.666*K) / (K * 0.666*K)
                # else:
                #     K_AI = (K - 0.5*K) / (K * 0.5*K)  

                # admittance
                if (TORQUE_CONTROL_MODE == 1):
                    desired_pos_fb = desired_pos + sea_data*K*K_AI 
                #impedance                                             
                elif (TORQUE_CONTROL_MODE == 2):
                    desired_pos_fb = desired_pos - sea_data*K*K_AI
                else:
                    desired_pos_fb = desired_pos

                # desired_pos_fb = desired_pos_fb * np.array([0.01,0,0,0,0,0,0,1])
                #-----------------------------------------------------------------------------------------------------------------------------------------------

                for idx, x in np.ndenumerate(desired_pos_fb):
                    if (x > ROM_P):
                        desired_pos_fb[idx] = ROM_P
                    elif (x < ROM_M):
                        desired_pos_fb[idx] = ROM_M
                    else:
                        continue

                # send_tcommand(serial, pos2bytes(np.rad2deg(desired_pos_fb)))                                    # send command to STM32 over serial
                writer.write(bytes(pos2bytes(np.rad2deg(desired_pos_fb))))
            else:
                off_time += current_time - prev_time
            prev_time = current_time

        except: #serial.serialutil.SerialException:
            # print("Device lost :( Exiting...")
            break

    # serial.close()
      
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.create_task(main())
    loop.create_task(listen_keyboard_manual(on_press=press,on_release=release))
    loop.run_forever()                    