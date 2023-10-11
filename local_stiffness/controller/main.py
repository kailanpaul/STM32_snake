import numpy as np
import matplotlib.pyplot as plt
import serial
import os
import serial.tools.list_ports
from time import sleep
import time


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

#------------------------------------------------------------------------------------------------------------

# define constants
# gait parameters
A = np.pi/6
omega = np.pi
phi = -4.0144/9 * np.pi

# simulation parameters
dt = 0.1
t_start = 0
t_max = 10
t = np.linspace(t_start, t_max, int(t_max/dt)+1)
# x_points = np.linspace(0, 13, 14)

K_p = 60 # Nm/rad
K_D = 25 # Nms/rad
# J_l = 1.52e-5 kgm**2
# J_r = 2.45e-7 kgm**2 for eg

# iterate over range with floats
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step
        
# takes servo position and angular velocity, and desired position and velocity, to calculate U, the control input torque
def force_controller(servo_pos, servo_vel, desired_pos, desired_vel):
    return np.array(-K_p*(servo_pos-desired_pos)-K_D*(servo_vel-desired_vel))


# gait pattern generator: generates joint angles (rad) to achieve pose corresponding to time step
def gpg(t):

    # initialise empty arrays to store link end points and desired joint angles
    points = [0] * 14
    desired_pos = [0] * 12

    # get points on sine wave for 12 links
    for i in range(1, 15):
        points_i = A*np.sin(t*omega + phi*(i-1))
        points[i-1] = round(points_i, 3)

    # get gradients of links
    m = [0] * 13
    for l in range(0, len(points)-1):
        m[l] = round(points[l+1]-points[l], 3)

    # get desired joint angles in radians from link gradients using trig identity
    for k in range(0, len(m)-1):
        desired_pos[k] = round(np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k]), 3)
        
    return desired_pos

#------------------------------------------------------------------------------------------------------------

# Motor parameters
J = 1  # Moment of inertia in kgm**2

# Simulation parameters
duration = 5  # Duration of the simulation (seconds)
sampling_rate = 50  # Number of time steps per second
time_steps = int(duration * sampling_rate)
time_ = np.linspace(0, duration, time_steps)
delta_t = time_[1] - time_[0]

initial_pos = np.array(gpg(t[10])) # initial pose at one second
desired_pos = np.array(gpg(t[15])) # pose at 1.5 sec (after 5*dt seconds)
desired_vel = (desired_pos - initial_pos) / (dt*5) # rad/s

# Initialize variables
# Initial servo position (rad), initial servo velocity (rad/s) and acceleration
servo_pos = initial_pos # set initial servo position to the initial position
servo_vel = np.zeros(12)
alpha = np.zeros(12)

# Numpy arrays to store angular position and velocity values
servo_positions = np.array([servo_pos])
servo_velocities = np.array([servo_vel])

#------------------------------------------------------------------------------------------------------------
prev_time_millis = int(round(time.time() * 1000))
while (1):

    current_time_millis = int(round(time.time() * 1000))

    if ((current_time_millis - prev_time_millis) >= 1e6):
        # send a command to switch position (+/- 50 deg), must have a flag to remember which position its currently in
        # if osc_flag==1 etc etc
        try:
            command_deg = float(90) # accept angle command as float degrees
            command_raw = (int(command_deg / 0.326 + 513)).to_bytes(2, 'little') # convert to positional command represented by 2 bytes
            ser.write(command_raw) 
        except serial.serialutil.SerialException:
            print("Device lost :( Exiting...")
            ser.close()
            break


    # must listen to serial and unpack the servo position readings (2 bytes each)

    try:
        data = ser.read(2) # read 2 bytes
        pos_packet = [b for b in data]
        servo_pos = (((pos_packet[1] & 0x03) << 8 | pos_packet[0]) - 513) * 0.326 # convert bytes to raw 10-bit and then angle, in deg
        # integrate servo_pos reading into control loop
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        break

    # err_norm = ((np.sum(abs(desired_pos-servo_pos)))/(np.sum(abs(np.mean(servo_pos)-(servo_pos)))))
    # err_norm = (((desired_pos-servo_pos))/(np.sum(abs(np.mean(servo_pos)-(servo_pos)))))
    err_norm = np.divide((desired_pos - servo_pos), abs(desired_pos - initial_pos))
    # print(err_norm)
    new_desired_vel = err_norm * desired_vel    # make the speed proportional to the normalised error 
                                                # (max vel when err is max, 0 vel when err is 0)
    tau = force_controller(servo_pos, servo_vel, desired_pos, new_desired_vel)  

    # Calculate angular acceleration using the (simple) motor dynamics equation
    alpha = tau / J

    # Update angular velocity and position using numerical integration
    servo_vel += alpha * delta_t
    servo_pos += servo_vel * delta_t

    # Append values to the lists
    servo_positions = np.vstack([servo_positions, servo_pos]) # this would actually be the position command
    servo_velocities = np.vstack([servo_velocities, servo_vel])

#------------------------------------------------------------------------------------------------------------

# Simulate 
# for t in time[1:]:
#     # Calculate torque command: heavyside step at 3 seconds
#     if t < 3:
#         tau = np.zeros(12)
#     else:
#         # err_norm = ((np.sum(abs(desired_pos-servo_pos)))/(np.sum(abs(np.mean(servo_pos)-(servo_pos)))))
#         # err_norm = (((desired_pos-servo_pos))/(np.sum(abs(np.mean(servo_pos)-(servo_pos)))))
#         err_norm = np.divide((desired_pos - servo_pos), abs(desired_pos - initial_pos))
#         # print(err_norm)
#         new_desired_vel = err_norm * desired_vel    # make the speed proportional to the normalised error 
#                                                     # (max vel when err is max, 0 vel when err is 0)
#         tau = force_controller(servo_pos, servo_vel, desired_pos, new_desired_vel)  

#     # Calculate angular acceleration using the (simple) motor dynamics equation
#     alpha = tau / J
    
#     # Update angular velocity and position using numerical integration
#     servo_vel += alpha * delta_t
#     servo_pos += servo_vel * delta_t
    
#     # Append values to the lists
#     servo_positions = np.vstack([servo_positions, servo_pos]) # this would actually be the position command
#     servo_velocities = np.vstack([servo_velocities, servo_vel])

# print(initial_pos)
# print(desired_pos)
# print(np.round(servo_pos, 4))
# print("Error: ", desired_pos-servo_pos)

# plt.plot(time[:-3], servo_positions[3:])
# plt.xlabel('Time (s)')
# plt.ylabel('Angular Position (radians)')
# plt.title('Angular Position vs. Time')
# plt.grid(True)
# plt.show()