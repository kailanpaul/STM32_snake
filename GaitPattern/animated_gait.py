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