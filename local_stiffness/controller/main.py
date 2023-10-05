import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
import serial
import os
import sys
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

while True:
    try:
        print(ser.read())
    except serial.serialutil.SerialException:
        print("Device lost :( Exiting...")
        break

# ser.write(b'x')
# print(ser.read())

# define constants
# gait parameters
A = np.pi/6
omega = np.pi
phi = -4.0144/9 * np.pi

# simulation parameters
dt = 0.1 
t = 0
# t_max = 10
# t = np.linspace(dt, t_max, int(t_max/dt))
# x_points = np.linspace(0, 13, 14)

# iterate over range with floats
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step

# max_curr = 0
# max_prev = 0

# iterate over time steps
# for t in range_with_floats(0, t_max, dt):

    # gait generation from parameters

    # ax = plt.gca()
    # ax.set_aspect('equal', adjustable='box')
    # ax.clear()

points = [0] * 14
alpha_desired = [0] * 12

# get points on sine wave for 12 links
for i in range(1, 15):
    points_i = A*np.sin(t*omega + phi*(i-1))
    points[i-1] = round(points_i, 3)

# print(points)

# get gradients of links
m = [0] * 13
for l in range(0, len(points)-1):
    m[l] = round(points[l+1]-points[l], 3)

# print(m)

# get desired joint angles from link gradients using trig identity
for k in range(0, len(m)-1):
    alpha_desired[k] = round(np.rad2deg(np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k])), 3)

# print(alpha_desired)
    # print("\n")

    # check and update greatest joint angle
    # max_curr = np.max(np.abs(alpha_desired))
    # if max_curr > max_prev:
    #     max_prev = max_curr

    # plotting
    # plt.plot(x_points, points, 'bo--')
    # plt.ylim(-1, 1)
    # plt.ylabel('amplitude')
    # plt.xlabel('joint number (0 and 13 are not joints)')

    # plt.show()

# print("Max angle experienced is: ", max_curr, "degrees")

