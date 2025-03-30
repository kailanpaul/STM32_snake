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
import csv
from xlsxwriter.workbook import Workbook

#------------------------------------------------------------------------------------------------------------

# constants

#-----------------------------------------------------------------------------------------------------
N_JOINTS = 12                                                                        # number of joints
#-----------------------------------------------------------------------------------------------------

DATA_SIZE = 2                                                                       # all data (SEA and servo) is 2 bytes in size                                            
SERIAL_PACKET_SIZE = 2 * DATA_SIZE + 1                                              # total number of bytes in a serial packet
SERIAL_DECODE_MASK = 0x80                                                           # decode serial data encoded on STM32 side

COMMAND_FREQ = 15                                                                  # frequency of commands (Hz)
COMMAND_PERIOD = 1/COMMAND_FREQ                                                     # time between commands in seconds

# A = np.pi/8                                                                         # sine amplitude
# OMEGA = np.pi                                                                       # temporal freq.
# PHI = ((-2*np.pi-0.4))/5                                                            # spatial freq.

A = 0.6                                                                         # sine amplitude
OMEGA = 1.5*np.pi                                                                       # temporal freq.
PHI = ((2*np.pi-0.3))/6                                                                         # spatial freq.

K = 3.54                                                                            # torsional stiffness constant of SEE (Nm/rad)
K_D = 0.5*K                                                                        # admittance/impedance constant
K_AI = (K - K_D) / (K * K_D)                                                        # admittance/impedance gain

ROM_P = np.deg2rad(65)                                                              # range of motion positive limit
ROM_M = -np.deg2rad(65)                                                             # range of motion negative limit
JOINT_DATA_ID = 5                                                                   # joint to have servo and SEA data saved
TORQUE_CONTROL_MODE = 0                                                             # 1 for admittance, 2 for impedance

RUN_TIME = 10

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
# approximates two sine waves
def gpg(time, n):
    n2 = int(n/2)
    # initialise empty arrays to store link end points and desired joint angles
    points_v = np.zeros(n2+2) # vertical wave
    points_h = np.zeros(n2+2) # horizontal wave
    desired_pos = np.zeros(n)

    # get points on sine wave for n links
    for i in range(1, (n2+3)):
        points_v[n2-i+2] = A*np.sin(time*OMEGA + PHI*(i-1))
        points_h[n2-i+2] = 2*A*np.sin(time*OMEGA + PHI*(i-1)-np.pi/2)
        # points_v[i-1] = 2*points_h[i-1]
    # print(points_v)
    # print(points_h)
    # get gradients of links
    m_v = np.zeros(n2+1)
    m_h = np.zeros(n2+1)
    for l in range(0, n2+1):
        m_v[l] = points_v[l+1]-points_v[l]
        m_h[l] = points_h[l+1]-points_h[l]

    # get desired joint angles in radians from link gradients using trig identity
    # assumes first node is vertical
    for k in range(0, n2):
        j = 2*k
        desired_pos[j] = np.arctan(m_v[k+1]-m_v[k])/(1 + m_v[k+1]*m_v[k])
        desired_pos[j+1] = np.arctan(m_h[k+1]-m_h[k])/(1 + m_h[k+1]*m_h[k])
    
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
    converted_angle = (sea_raw * 2 * np.pi) / 4095
    if (converted_angle > np.pi):
        curated_angle = converted_angle - 2*np.pi
    else:
        curated_angle = converted_angle
    if (curated_angle > np.deg2rad(30)):
        curated_angle = 0
    return curated_angle

def pos2bytes2(pose):
    bytes_array = [0] * (int(len(pose)/2+1))
    pose += 65
    pose *= (15/130)
    bytes_array[0] = round(pose[0])
    for i in range(1, int(len(pose)/2+1)):
        left = round(pose[2*i-1])<<4
        if((2*i) == N_JOINTS):
            right = 0
        else:
            right = round(pose[2*i])
        bytes_array[i] = left | right
    return bytes_array

def test():

    # initialize variables

    sea_data = np.zeros(N_JOINTS)
    servo_pos = np.zeros(N_JOINTS)
    desired_pos = np.zeros(N_JOINTS) 
    desired_pos_fb = np.zeros(N_JOINTS)  

    # data storage arrays
    joint_data = [0, 0, 0]
    header = ['time', 'servo data', 'SEA data']                              
                                               

    print("PlEAsE wAiT")

    # wait for servo to init and zero

    # time.sleep(1)

    start_time = time.time()                                                                           
    prev_command_time = time.time()   

    time.sleep(0.1)

    print("Start")

    while (1):

        current_time = time.time()                                                    

        if (current_time - prev_command_time) >= COMMAND_PERIOD:                        # check if command period has passed since last command                                       
            desired_pos = gpg(round((current_time - start_time), 3), N_JOINTS)                      # calculate desired pose command using GPG for current time (in seconds)
            # desired_pos = np.zeros(8)
            prev_command_time = current_time                                                        # update time of previous command

            # print the time stamp
            print("t =", int((current_time - start_time) / 60), "min", round((current_time - start_time)  % 60, 3), "sec")

        # admittance
        if (TORQUE_CONTROL_MODE == 1):
            desired_pos_fb = desired_pos + sea_data*K*K_AI 
        #impedance                                             
        elif (TORQUE_CONTROL_MODE == 2):
            desired_pos_fb = desired_pos - sea_data*K*K_AI
        else:
            desired_pos_fb = desired_pos

        for idx, x in np.ndenumerate(desired_pos_fb):
            if (x > ROM_P):
                desired_pos_fb[idx] = ROM_P
            elif (x < ROM_M):
                desired_pos_fb[idx] = ROM_M
            else:
                continue

        # print(np.rad2deg(desired_pos_fb))
        # print(pos2bytes(np.rad2deg(desired_pos_fb)))
        results = pos2bytes2(np.rad2deg(desired_pos_fb))    # send command to STM32 over serial
        table = {
            0:-49,
            1:-43.3,
            2:-36.7,
            3:-30,
            4:-23.3,
            5:-16.7,
            6:-10,
            7:-3.3,
            8:3.3,
            9:10,
            10:16.7,
            11:23.3,
            12:30,
            13:36.7,
            14:43.3,
            15:49,
        }
        l = []
        f = 0
        for result in results:
            if (f):
                left = (result & 0b11110000)>>4
                right = result & 0b1111
                l.append(table[left])
                l.append(table[right])
            else:
                l.append(table[result])
                f = 1
        # print((int((pose[i] / 0.326) + 512)))
        for i in range(len(desired_pos_fb)):
            if (abs(np.rad2deg(desired_pos_fb[i]) - l[i]) > 3.5):
                print("fail")
        
        for i in range(16):
            print(round((table[i]/ 0.326) + 512))
        # for srge in l:
        #     print(round((srge/ 0.326) + 512))
        # print(round((0/ 0.326) + 512))
        break



def test2():

    # initialize variables

    sea_data = np.zeros(N_JOINTS)
    servo_pos = np.zeros(N_JOINTS)
    desired_pos = np.zeros(N_JOINTS) 
    desired_pos_fb = np.zeros(N_JOINTS)  

    # data storage arrays
    joint_data = [0, 0, 0]
    header = ['time', 'servo data', 'SEA data']                              

    ser = sniff()                                                  

    print("PlEAsE wAiT")

    # wait for servo to init and zero
    ser.close()
    time.sleep(1) 
    ser.open()

    # send_command(ser, pos2bytes(np.rad2deg(gpg(0.2, N_JOINTS))))

    # time.sleep(1)

    start_time = time.time()                                                                           
    prev_command_time = time.time()   

    time.sleep(0.1)

    print("Start")

    while (1):

        current_time = time.time()                                                    

        try:

            if (current_time - prev_command_time) >= COMMAND_PERIOD:                        # check if command period has passed since last command                                       
                prev_command_time = current_time                                                        # update time of previous command

                # print the time stamp
                print("t =", int((current_time - start_time) / 60), "min", round((current_time - start_time)  % 60, 3), "sec")


                angs = np.rad2deg(gpg(1, N_JOINTS))
                print(angs)
                send_command(ser, pos2bytes2(angs))

        except serial.serialutil.SerialException:
            print("Device lost :( Exiting...")
            break

    ser.close()


#------------------------------------------------------------------------------------------------------------

def main():

    # initialize variables

    sea_data = np.zeros(N_JOINTS)
    servo_pos = np.zeros(N_JOINTS)
    desired_pos = np.zeros(N_JOINTS) 
    desired_pos_fb = np.zeros(N_JOINTS)  

    # data storage arrays
    joint_data = [0, 0, 0]
    header = ['time', 'joint', 'SEA data']                              

    ser = sniff()                                                  

    print("PlEAsE wAiT")

    # wait for servo to init and zero
    ser.close()
    time.sleep(1) 
    ser.open()

    # send_command(ser, pos2bytes(np.rad2deg(gpg(0.2, N_JOINTS))))

    # time.sleep(1)

    start_time = time.time()                                                                           
    prev_command_time = time.time()   

    time.sleep(0.1)

    print("Start")

    while (1):

        current_time = time.time()                                                    

        try:

            # FEEDBACK..

            data = ser.read(SERIAL_PACKET_SIZE)                                                         # read 2 bytes
            serial_packet = [b for b in data]                                                           # put bytes in array
            # servo_pos[serial_packet[0]] = bytes2pos(serial_packet[1:3])                                 # update servo data in index indicated by packet
            sea_data[serial_packet[0]] = bytes2ang(serial_packet[1:3])                                  # update SEA data in index indicated by packet

            # useful for checking if SEA data is rubbish or not
            # if(serial_packet[0] == 5):
            #     print(serial_packet)

            if (serial_packet[0] == 4):
                joint_data = np.vstack([joint_data, ([current_time - start_time,
                                                      serial_packet[0],
                                                      sea_data[4]
                                                      ])])
                
            if (serial_packet[0] == 5):
                joint_data = np.vstack([joint_data, ([current_time - start_time,
                                                      serial_packet[0],
                                                      sea_data[5]
                                                      ])])

            # ..CONTROL

            if (current_time - prev_command_time) >= COMMAND_PERIOD:                        # check if command period has passed since last command                                       
                desired_pos = gpg(round((current_time - start_time), 3), N_JOINTS)                      # calculate desired pose command using GPG for current time (in seconds)
                # desired_pos = np.zeros(8)
                prev_command_time = current_time                                                        # update time of previous command

                # print the time stamp
                print("t =", int((current_time - start_time) / 60), "min", round((current_time - start_time)  % 60, 3), "sec")


            # apply torque feedback and soft limit

            #-----------------------------------------------------------------------------------------------------------------------------------------------
            # if ((current_time - start_time) < 10):
            #     K_AI = 0
            # elif ((current_time - start_time) < 20):
            #     K_AI = (K - 0.666*K) / (K * 0.666*K)
            # else:
            #     K_AI = (K - 0.5*K) / (K * 0.5*K)  

            # admittance
            # if (TORQUE_CONTROL_MODE == 1):
            #     desired_pos_fb = desired_pos + sea_data*K*K_AI 
            # #impedance                                             
            # elif (TORQUE_CONTROL_MODE == 2):
            #     desired_pos_fb = desired_pos - sea_data*K*K_AI
            # else:
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

            # print(desired_pos_fb)

            # if sampling duration has elapsed, save the data and end program
            if (current_time - start_time >= RUN_TIME):
                csvfile = str(int(round(current_time, 0))) + '_' + 'joint_' + str(JOINT_DATA_ID) + '_' + str(TORQUE_CONTROL_MODE) + '_' + str(round(K_AI*K, 2)) + '_data.csv'
                with open(csvfile, 'x', newline='') as f:
                    print("saving data...")
                    writer = csv.writer(f)
                    writer.writerow(header)
                    print(joint_data)
                    writer.writerows(joint_data)
                    f.close()
                    workbook = Workbook(csvfile[:-4] + '.xlsx')
                    worksheet = workbook.add_worksheet()
                    with open(csvfile, 'rt', encoding='utf8') as ff:
                        reader = csv.reader(ff)
                        for r, row in enumerate(reader):
                            for c, col in enumerate(row):
                                worksheet.write(r, c, col)
                    workbook.close()
                    os.remove(csvfile) 
                    print("saved")

                    # if (TORQUE_CONTROL_MODE == 1):
                    #     print("joint", JOINT_DATA_ID, "admittance with torque gain", K_AI*K)                                         
                    # elif (TORQUE_CONTROL_MODE == 2):
                    #     print("joint", JOINT_DATA_ID, "impedance with torque gain", K_AI*K)
                    # else:
                    #     print("joint", JOINT_DATA_ID, "with no torque gain")
                    ser.close()
                    exit()
            send = pos2bytes2(np.rad2deg(desired_pos_fb))
            print(send)
            send_command(ser, send)                                    # send command to STM32 over serial

        except serial.serialutil.SerialException:
            print("Device lost :( Exiting...")
            break

    ser.close()
      
if __name__ == "__main__":
    # print(pos2bytes(np.rad2deg(gpg(0.2, N_JOINTS))))
    # print(pos2bytes2(np.rad2deg(gpg(0.2, N_JOINTS))))
    main()                      
    # test2()