import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
from matplotlib import rc
import time 
import csv
from xlsxwriter.workbook import Workbook
import os
import pandas as pd

# import serial

# ser = serial.Serial('/dev/ttyACM0')
# ser.write(b'x')

# while True:
#     print(ser.read())

# define constants

# gait parameters
A = np.pi/6
omega = 7*np.pi/12
phi = ((-2 * np.pi) - 0.4)/5

# simulation parameters
dt = 0.05 
t_max = 30
t = np.linspace(dt, t_max, int(t_max/dt))
x_points = np.linspace(0, 13, 14)

# iterate over range with floats
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step

max_curr = 0
max_prev = 0

joint_7_data = [0,0]
header = ['time', 'command']  

# iterate over time steps
for t in range_with_floats(0, t_max, dt):

    # gait generation from parameters

    # ax = plt.gca()
    # ax.set_aspect('equal', adjustable='box')
    # ax.clear()
    points = [0] * 14
    desired_pos = [0] * 12

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
        desired_pos[k] = round(np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k]), 3) # rad

    joint_7_data = np.vstack([joint_7_data, ([t, desired_pos[6]])])

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

csvfile = 'C:/Users/gal65/masters/STM32_snake/local_stiffness/data/sim_commands.csv'
with open(csvfile, 'w', newline='') as f:
        print("saving data...")
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(joint_7_data)
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
        exit()

# print("Max angle experienced is: ", max_curr, "degrees")

