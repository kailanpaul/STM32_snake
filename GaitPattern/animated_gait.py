import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter
plt.rcParams['animation.ffmpeg_path'] = r'C:\Users\gal65\FFMpeg\bin\ffmpeg'

# Define constants
A = np.pi/6
omega = np.pi
phi = (-2 * np.pi - 0.4)/5
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
mark = list(np.array([1,2,3,4,5,6,7,8,9,10,11,12]))
line, = axis.plot([], [], markevery=mark, marker="o", lw = 3)
 
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
ani.save('none.mp4', writer=FFwriter)

#------------------------------------------------------------------------------------------------------------
# simulation parameters
# dt = 0.1
# t_start = 0
# t_max = 10
# t = np.linspace(t_start, t_max, int(t_max/dt)+1)
# x_points = np.linspace(0, 13, 14)

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

#------------------------------------------------------------------------------------------------------------

# Motor parameters
# J = 1  # Moment of inertia in kgm**2

# Simulation parameters
# duration = 5  # Duration of the simulation (seconds)
# sampling_rate = 50  # Number of time steps per second
# time_steps = int(duration * sampling_rate)
# time_ = np.linspace(0, duration, time_steps)
# delta_t = time_[1] - time_[0]

# initial_pos = np.array(gpg(t[10])) # initial pose at one second
# desired_pos = np.array(gpg(t[15])) # pose at 1.5 sec (after 5*dt seconds)
# desired_vel = (desired_pos - initial_pos) / (dt*5) # rad/s

# # Initialize variables
# # Initial servo position (rad), initial servo velocity (rad/s) and acceleration
# servo_pos = initial_pos # set initial servo position to the initial position
# servo_vel = np.zeros(12)
# alpha = np.zeros(12)

# # Numpy arrays to store angular position and velocity values
# servo_positions = np.array([servo_pos])
# servo_velocities = np.array([servo_vel])

# Simulate 
# for t in time[1:]:
#     # Calculate torque command: heavyside step at 3 seconds
#     if t < 3:
#         tau = np.zeros(12)
#     else:
#         # err_norm = ((np.sum(abs(desired_pos-servo_pos)))/(np.sum(abs(np.mean(servo_pos)-(servo_pos)))))
#         # err_norm = (((desired_pos-servo_pos))/(np.sum(abs(np.mean(servo_pos)-(servo_pos)))))
#         err_norm = np.divide((desired_pos - servo_pos), abs(desired_pos - initial_pos))
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