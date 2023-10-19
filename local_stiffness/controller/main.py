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
import serial
import serial.tools.list_ports
import numpy as np
import time

#------------------------------------------------------------------------------------------------------------

# constants

N_JOINTS = 3                                                                        # number of joints
DATA_SIZE = 2                                                                       # all data (SEA and servo) is 2 bytes in size                                            
SERIAL_PACKET_SIZE = 2 * DATA_SIZE + 1                                              # total number of bytes in a serial packet
SERIAL_DECODE_MASK = 0x80                                                           # decode serial data encoded on STM32 side

COMMAND_FREQ = 20                                                                   # frequency of commands (Hz)
COMMAND_PERIOD = 1/COMMAND_FREQ                                                     # time between commands in seconds

A = np.pi/6                                                                         # sine amplitude
omega = np.pi                                                                       # temporal freq.
phi = ((-2*np.pi)-0.4)/5                                                            # spatial freq.

K = 3.38                                                                            # torsional stiffness constant of SEE (Nm/rad)
K_D = 0.4*K                                                                         # admittance/impedance constant
K_AI = (K - K_D) / (K * K_D)                                                        # admittance/impedance gain

ROM_P = np.deg2rad(40)                                                              # range of motion positive limit
ROM_M = -np.deg2rad(40)                                                             # range of motion negative limit

#------------------------------------------------------------------------------------------------------------

# sniff for devices
def sniff():
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
                        return ser
                    except serial.serialutil.SerialException:
                        print("ACCESS DENIED WAH WAH WAH")
                        exit()
            else:
                try:
                    ser = serial.Serial('/dev/ttyACM0')
                    return ser
                except serial.serialutil.SerialException:
                    raise IndexError
        except IndexError:
            print("No devices detected. Retrying...")
            time.sleep(2)
            continue

# gait pattern generator: generates joint angles (rad) for 'n' joints, to achieve pose corresponding to time step
# approximates a sine wave
def gpg(time, n):

    # initialise empty arrays to store link end points and desired joint angles
    points = np.zeros(n+2)
    desired_pos = np.zeros(n)

    # get points on sine wave for n links
    for i in range(1, (n+3)):
        points_i = A*np.sin(time*omega + phi*(i-1))
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
def send_command(com, command):
    try:
        com.write(command)                                                                     
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        com.close()
        return  

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
    calibrated_angle = (sea_raw * 2 * np.pi) / 4095
    if (calibrated_angle > np.pi):
        calibrated_angle -= 2*np.pi
    return calibrated_angle

#------------------------------------------------------------------------------------------------------------

def main():

    # initialize variables

    sea_data = np.zeros(N_JOINTS)
    servo_pos = np.zeros(N_JOINTS)
    desired_pos = np.zeros(N_JOINTS) 
    desired_pos_fb = np.zeros(N_JOINTS)                                

    ser = sniff()                                                  

    print("PlEAsE wAiT")

    # wait for servo to init and zero
    ser.close()
    time.sleep(3) 
    ser.open()

    start_time = round(time.time(), 2)                                                                            
    prev_command_time_sec = round(time.time(), 2)   

    time.sleep(0.1)

    print("Start")

    while (1):

        current_time_sec = round(time.time(), 2)                                                    

        try:

            # FEEDBACK..

            data = ser.read(SERIAL_PACKET_SIZE)                                                     # read 2 bytes
            serial_packet = [b for b in data]                                                       # put bytes in array

            # print(serial_packet)
            # for i in range(0, SERIAL_PACKET_SIZE-1, DATA_SIZE):                                     # iterate through serial packet in 2-byte chunks
            #     if ((serial_packet[i+1] & SERIAL_DECODE_MASK) == SERIAL_DECODE_MASK):               # mask MSB with 0b1000000 - treat as pos data if result is 1, else SEA data
            #         idx = int(i/4)
            #         servo_pos[idx] = bytes2pos(serial_packet[i:i+DATA_SIZE])                        # convert byte type to raw 10-bit position and then angle
            #     else:
            #         idx = int((i-2)/4)
            #         sea_data[idx] = bytes2ang(serial_packet[i:i+DATA_SIZE])                         # convert byte type to raw 12-bit position and then deflection angle
            servo_pos[serial_packet[0]] = bytes2pos(serial_packet[1:3])
            sea_data[serial_packet[0]] = bytes2ang(serial_packet[3:5])

            # ..CONTROL

            if ((current_time_sec - prev_command_time_sec) >= COMMAND_PERIOD):                      # check if command period has passed since last command                                       
                desired_pos = gpg((current_time_sec - start_time), N_JOINTS)                        # calculate desired pose command using GPG for current time (in seconds)
                prev_command_time_sec = current_time_sec                                            # update time of previous command

            desired_pos_fb = desired_pos + sea_data*K*K_AI                                          # addition for admittance, subtraction for impedance

            # soft limit of +/- 50 deg
            for idx, x in np.ndenumerate(desired_pos_fb):
                if (x > ROM_P):
                    desired_pos_fb[idx] = ROM_P
                elif (x < ROM_M):
                    desired_pos_fb[idx] = ROM_M
                else:
                    continue

            print(np.rad2deg(desired_pos_fb))
            send_command(ser, pos2bytes(np.rad2deg(desired_pos_fb)))                                # send command to STM32 over serial

        except serial.serialutil.SerialException:
            print("Device lost :( Exiting...")
            break

    ser.close()
      
if __name__ == "__main__":
    main()                      