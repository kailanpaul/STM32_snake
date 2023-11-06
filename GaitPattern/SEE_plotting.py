import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import pearsonr

# delta_L = np.array([0.00,0.02,0.04,0.06,0.09,0.12,0.15,0.18,0.22,0.26,0.29,0.32,0.36,0.40,0.43,0.47])
delta_L = np.array([0.00,0.02,0.04,0.06,0.09,0.12,0.15,0.18,0.22,0.26,0.29,0.32,0.36])
# delta_U = np.array([0.07,0.11,0.14,0.17,0.20,0.24,0.26,0.30,0.32,0.36,0.38,0.41,0.43,0.45,0.46,0.47])
torques = np.linspace(0, 1.2, 13)
torques_extend = np.linspace(-1.2, 1.2, 25)
delta_L_extend = np.concatenate([np.sort(np.negative(delta_L[1:])),(delta_L)])

z = np.polyfit(delta_L_extend, torques_extend, 1)
p = np.poly1d(z)

print(p)

print(pearsonr(p(delta_L), torques))

plt.plot(torques,np.rad2deg(delta_L), 'ro--')
# plt.plot(delta_U, torques, 'ro--')
plt.plot(p(delta_L),np.rad2deg(delta_L))
# plt.title("K_tau=%.2f Nm/rad"%(z[0])) 
plt.ylabel("Deflection (deg)")
plt.xlabel("Torque (Nm)")
# plt.legend(['Loading', 'Unloading', 'K_tau'])
plt.legend(['Raw data', 'Linear approximation'], loc=4)
plt.show()