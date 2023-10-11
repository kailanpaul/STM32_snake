import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter
plt.rcParams['animation.ffmpeg_path'] = r'C:\Users\gal65\FFMpeg\bin\ffmpeg'

# Define constants
A = np.pi/6
omega = np.pi
phi = -4.0144/9 * np.pi
fig = plt.figure()
dt = 0.1 
t_max = 10
t = np.linspace(dt, t_max, int(t_max/dt))
x_points = np.linspace(0, 13, 14)

def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step

# marking the x-axis and y-axis
axis = plt.axes(xlim =(0, 13),
                ylim =(-1, 1))
 
# initializing a line variable
line, = axis.plot([], [], lw = 3)
 
# data which the line will
# contain (x, y)
def init():
    line.set_data([], [])
    return line

# frame iterable
def animate(i):
    points = [0] * 14
    # get points on sine wave for 12 links
    for j in range(1, 15):
        points_j = A*np.sin(t[i]*omega + phi*(j-1))
        points[j-1] = np.round(points_j, 3)
    line.set_data(x_points, points)
    return line

ani = FuncAnimation(fig, animate, init_func=init, frames=100,
                interval=500, repeat=False)

FFwriter = FFMpegWriter(fps=10, extra_args=['-vcodec', 'libx264'])
ani.save('snakey.mp4', writer=FFwriter)

#------------------------------------------------------------------------------------------------------------

# max_curr = 0
# max_prev = 0

# iterate over time steps
# for t in range_with_floats(0, t_max, dt):

    # gait generation from parameters

    # ax = plt.gca()
    # ax.set_aspect('equal', adjustable='box')
    # ax.clear()

# points = [0] * 14
# alpha_desired = [0] * 12

# # get points on sine wave for 12 links
# for i in range(1, 15):
#     points_i = A*np.sin(t*omega + phi*(i-1))
#     points[i-1] = round(points_i, 3)

# # print(points)

# # get gradients of links
# m = [0] * 13
# for l in range(0, len(points)-1):
#     m[l] = round(points[l+1]-points[l], 3)

# # print(m)

# # get desired joint angles from link gradients using trig identity
# for k in range(0, len(m)-1):
#     alpha_desired[k] = round(np.rad2deg(np.arctan(m[k+1]-m[k])/(1 + m[k+1]*m[k])), 3)

# print("Joint angles for t=0s (degrees): ", gpg(0))
# print("Joint angles for t=0s (degrees): ", gpg(0.1))
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

# print("Controller output (Nm): ", U)