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

# PD controller gains
K_P = 50                                                        # Nm/rad
K_D = 20                                                        # Nms/rad

POSITION_BYTES = 2                                              # number of bytes in servo position reading
N = 1                                                           # number of joints
J = 1                                                           # moment of inertia in kgm**2 (estimate based on 87 mm joint-joint lengths and servo-dominated segment mass)

COMMAND_FREQ = 0.2                                              # frequency of commands (Hz)
COMMAND_PERIOD = 1/COMMAND_FREQ                                 # time between commands in seconds

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

# convert angle in deg to raw position value and then to bytes
def deg2bytes(pose):
    bytes_array = [0] * (len(pose) * 2)
    for i in range(0, len(pose)):
        bytes_array[(2*i):((2*i)+2)] = (int((pose[i] / 0.326) + 512)).to_bytes(2, 'little')
    return bytes_array

# convert bytes to raw position value and then to angle in deg (3 DP)
def bytes2deg(pos_bytes_packet):
    return round((((pos_bytes_packet[1] & 0x03) << 8 | pos_bytes_packet[0]) - 513) * 0.326, 3)

#------------------------------------------------------------------------------------------------------------

def main():

    # initialize variables

    osc_flag = 0

    # initial desired position and velocity
    prev_desired_pos = np.zeros(N)
    desired_pos = np.zeros(N)                                                                   # pose is 0 rad initially
    desired_vel = (desired_pos - prev_desired_pos) / COMMAND_PERIOD                             # velocity is 0 rad/s initially

    # initial control block position (rad), velocity (rad/s) and acceleration (rad/s/s)
    C_p_pos = np.zeros(N)
    C_p_vel = np.zeros(N)
    C_p_acc = np.zeros(N)

    # initial servo measurements
    current_servo_pos = np.zeros(N)
    prev_servo_pos = np.zeros(N)
    servo_vel = np.zeros(N)

    ser = sniff()                                                                               # set port

    print("PlEAsE wAiT")

    # wait for servo to zero
    time.sleep(3) 
    send_command(ser, deg2bytes(prev_desired_pos))

    start_time = round(time.time(), 2)                                                          # time stamp in seconds to 2 DP
    prev_read_time_sec = round(time.time(), 2)                                                
    prev_command_time_sec = round(time.time(), 2)   

    time.sleep(0.1)

    #   init 
    current_time_sec = round(time.time(), 2)                                            # update time
    data = ser.read(POSITION_BYTES)                                                     # read 2 bytes
    dt = current_time_sec - prev_read_time_sec                                          # calculate time step size for this iteration, for numerical integration
    prev_read_time_sec = current_time_sec
    pos_packet = [b for b in data]                                                      # put bites in array
    for i in range(0, 2*N, POSITION_BYTES):                                             # iterate through array in 2-byte chunks 
        current_servo_pos[i-1] = np.deg2rad(bytes2deg(pos_packet[i:i+POSITION_BYTES]))  # convert byte pairs to raw 10-bit position and then angle, in deg
    print("servo_pos = ", np.rad2deg(current_servo_pos[0]), " deg")

    servo_vel = (current_servo_pos - prev_servo_pos) / dt                               # compute servo velocity numerically
    servo_vel[np.isnan(servo_vel)] = 0
    prev_servo_pos = current_servo_pos                                                  # update servo pose

    while (1):

        current_time_sec = round(time.time(), 2)                                                # update time

        time.sleep(0.1)

        # TO DO: READ FROM ENCODER

        try:
            # reading position data from servo (~10 Hz)
            data = ser.read(POSITION_BYTES)                                                     # read 2 bytes
            dt = current_time_sec - prev_read_time_sec                                          # calculate time step size for this iteration, for numerical integration
            prev_read_time_sec = current_time_sec
            pos_packet = [b for b in data]                                                      # put bites in array
            for i in range(0, 2*N, POSITION_BYTES):                                             # iterate through array in 2-byte chunks 
                current_servo_pos[i-1] = np.deg2rad(bytes2deg(pos_packet[i:i+POSITION_BYTES]))  # convert byte pairs to raw 10-bit position and then angle, in deg
            print("servo_pos = ", np.rad2deg(current_servo_pos[0]), " deg")

            servo_vel = (current_servo_pos - prev_servo_pos) / dt                               # compute servo velocity numerically
            servo_vel[np.isnan(servo_vel)] = 0
            prev_servo_pos = current_servo_pos                                                  # update servo pose

            # compute control signal:

            # check if command period has passed since last command
            if ((current_time_sec - prev_command_time_sec) >= COMMAND_PERIOD):                                          
                # desired_pos = np.round(gpg((current_time_sec - start_time), N), 1)              # calculate desired pose command using GPG for current time (in seconds)
                if (osc_flag == 0):
                    desired_pos = np.array([np.deg2rad(50)])
                    osc_flag = 1
                else:
                    desired_pos = np.array([np.deg2rad(-50)])
                    osc_flag = 0
                print("desired_pos = ", np.rad2deg(desired_pos), " deg")
                desired_vel = (desired_pos - prev_desired_pos) / COMMAND_PERIOD                 # update desired velocity with current and previous pose commands (rad/s)
                prev_command_time_sec = current_time_sec                                        # update time of last command
                prev_desired_pos = desired_pos                                                  # update previous pose

                with np.errstate(divide='ignore'):
                    # err_norm = ((np.sum(abs(desired_pos-current_servo_pos)))/(np.sum(abs(np.mean(current_servo_pos)-(current_servo_pos)))))
                    # err_norm = (((desired_pos-current_servo_pos))/(np.sum(abs(np.mean(current_servo_pos)-(current_servo_pos)))))
                    err_norm = np.divide(abs(desired_pos - current_servo_pos), abs(desired_pos - prev_desired_pos))   # i think this expression is correct ..
                err_norm[np.isnan(err_norm)] = 0
                err_norm[np.isinf(err_norm)] = 0

                scaled_desired_vel = err_norm * desired_vel                                         # make the speed proportional to the (normalised?) error 
                tau_U = force_controller(current_servo_pos, servo_vel, desired_pos, scaled_desired_vel)     # compute control input U [Nm]

                print("U = ", tau_U[0], " Nm")
                # convert U to position command for servo
                # calculate angular acceleration using the (simple) motor dynamics equation
                C_p_acc = tau_U / J
                C_p_acc[np.isnan(C_p_acc)] = 0

                # calculate C_p output angular velocity and position using numerical integrations (beware of drift)
                C_p_vel += C_p_acc * dt
                C_p_pos += np.round((C_p_vel * dt), 3)

                print("PD control position =", np.rad2deg(C_p_pos[0]), " deg")
                command_raw = deg2bytes(np.rad2deg(C_p_pos))                                        # convert to positional command represented by 2 bytes
                send_command(ser, deg2bytes(np.rad2deg(desired_pos)))                               # send command to STM32 over serial

        except serial.serialutil.SerialException:
            print("Device lost :( Exiting...")
            break
      
if __name__ == "__main__":
    main()                      