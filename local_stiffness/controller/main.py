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

N_JOINTS = 1                                                                        # number of joints
DATA_SIZE = 2                                                                        
POSITION_DATA_SIZE = 2                                                              # number of bytes in servo position reading
SEA_DATA_SIZE = 2                                                                   # number of bytes in SEA encoder reading                                                
SERIAL_PACKET_SIZE = N_JOINTS*(SEA_DATA_SIZE+POSITION_DATA_SIZE)                    # total number of bytes in a serial packet

COMMAND_FREQ = 1                                                                    # frequency of commands (Hz)
COMMAND_PERIOD = 1/COMMAND_FREQ                                                     # time between commands in seconds

K = 3.38                                                                            # torsional stiffness constant of SEE (Nm/rad)
K_D = 0.4*K                                                                         # admittance/impedance constant
K_AI = (K-K_D)/(K*K_D)                                                              # admittance/impedance gain

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
                        print("ACCESS DENIED WAH WAH WAH (remember to close other shells)")
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

# function to iterate over range with floats
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step
        
# C_p block: takes servo position and velocity, and desired position and velocity, to calculate U, the control input torque
def force_controller(servo_pos, servo_vel, desired_pos, desired_vel):
    # PD controller gains
    return np.array(-K_P*(servo_pos-desired_pos)-K_D*(servo_vel-desired_vel))

# gait pattern generator: generates joint angles (rad) for 'n' joints, to achieve pose corresponding to time step
def gpg(time, n):

    # gait parameters
    A = np.pi/6                     # sine amplitude
    omega = np.pi                   # temporal freq.
    phi = -4.0144/9 * np.pi         # spatial freq.

    # initialise empty arrays to store link end points and desired joint angles
    points = [0] * (n+2)
    desired_pos = [0] * n

    # get points on sine wave for n links
    for i in range(1, (n+3)):
        points_i = A*np.sin(time*omega + phi*(i-1))
        points[i-1] = round(points_i, 3)

    # get gradients of links
    m = [0] * (n+1)
    for l in range(0, len(points)-1):
        m[l] = round(points[l+1]-points[l], 3)

    # get desired joint angles in radians from link gradients using trig identity
    for k in range(0, len(m)-1):
        desired_pos[k] = round(np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k]), 3)
    
    # return joint angles in radians
    desired_pos = np.array(desired_pos)
    return desired_pos

# send 'command' (bytes) over serial to port 'com'
def send_command(com, command):
    try:
        com.write(command)                                                                      # send command to STM32
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

# convert servo position data from bytes to raw position value and then to angle in rad (3 DP)
def bytes2pos(pos_data_packet):
    return np.deg2rad(round((((pos_data_packet[1] & 0x03) << 8 | pos_data_packet[0]) - 513) * 0.326, 3))

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
    desired_pos = np.zeros(N_JOINTS)                                

    # initial servo measurements
    current_servo_pos = np.zeros(N_JOINTS)

    ser = sniff()                                                  

    print("PlEAsE wAiT")

    # wait for servo to zero
    ser.close()
    time.sleep(3) 
    ser.open()

    start_time = round(time.time(), 2)                                                                            
    prev_command_time_sec = round(time.time(), 2)   

    time.sleep(0.1)

    while (1):

        current_time_sec = round(time.time(), 2)                                                    

        try:

            # FEEDBACK

            data = ser.read(SERIAL_PACKET_SIZE)                                                     # read 2 bytes
            serial_packet = [b for b in data]                                                       # put bites in array
            for i in range(0, (2*N_JOINTS)+1, DATA_SIZE):                                           # iterate through array in 2-byte chunks
                if ((serial_packet[i+1] & 0x80) == 0x80):                                           # mask with 0b1000000 - treat as pos data if result is 1, else SEA data
                    idx = int(i/4)
                    current_servo_pos[idx] = bytes2pos(serial_packet[i:i+DATA_SIZE])                # convert byte pairs to raw 10-bit position and then angle
                else:
                    idx = int((i-2)/4)
                    sea_data[idx] = bytes2ang(serial_packet[i:i+DATA_SIZE])

            # CONTROL

            if ((current_time_sec - prev_command_time_sec) >= COMMAND_PERIOD):                      # check if command period has passed since last command                                       
                desired_pos = np.round(gpg((current_time_sec - start_time), N_JOINTS), 1)           # calculate desired pose command using GPG for current time (in seconds)

            desired_pos_prime = desired_pos - sea_data[0]*K*K_AI                                    # addition for admittance, subtraction for impedance
            send_command(ser, pos2bytes(np.rad2deg(desired_pos_prime)))                             # send command to STM32 over serial

        except serial.serialutil.SerialException:
            print("Device lost :( Exiting...")
            break
      
if __name__ == "__main__":
    main()                      