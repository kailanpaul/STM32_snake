import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft

# Define constants
A = np.pi/6
omega = np.pi
phi = -1/3 * np.pi

time_step = 0.1 # seconds
t_max = 10

def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step

for t in range_with_floats(0, t_max, time_step):

    points = [0] * 14
    alpha_desired = [0] * 12

    # get points on sine wave for 12 links
    for i in range(1, 15):
        points_i = A*np.sin(t*omega + phi*(i-1))
        points[i-1] = round(points_i, 3)

    # get gradients of links
    m = [0] * 13
    for l in range(0, len(points)-1):
        m[l] = round(points[l+1]-points[l], 3)

    # get desired joint angles from link gradients using trig identity
    for k in range(0, len(m)-1):
        alpha_desired[k] = round(np.rad2deg(np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k])), 3)

    x_points = np.linspace(0, 13, 14)
    x_joints = np.linspace(0, 11, 12)
    x = np.linspace(0, 13, 200)

    plt.plot(x_points, points, 'bo--')
    plt.ylim(-1, 1)
    plt.ylabel('amplitude')
    plt.xlabel('joint number (0 and 13 are not joints)')
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    # plt.plot(x, A*np.sin(x*omega*0.333 - 0.5*phi), 'r--')
    plt.show()