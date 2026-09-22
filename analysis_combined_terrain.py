from subfunctions import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
### Ash and Arnav install if needed, useful for built-in root finder functions ###
from scipy.optimize import brentq


rover = define_rover()
planet = define_planet()
Ng = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
r = rover['wheel_assembly']['wheel']['radius']
speed_noload = rover['wheel_assembly']['motor']['speed_noload']

Crr_array = np.linspace(0.01,0.5,25)
slope_array_deg = np.linspace(-15,35,25)

CRR, SLOPE = np.meshgrid(Crr_array, slope_array_deg)

v_max = np.zeros(np.shape(CRR), dtype = float)
N = np.shape(CRR)[0]

def F_net_at_omega(omega, angle, rover, planet, Crr):
    return F_net(omega, angle, rover, planet, Crr)

for i in range(N):
    for j in range(N):
        try:
            Crr_sample = float(CRR[i,j])
            slope_sample = float(SLOPE[i,j])
            omega_root = brentq(F_net_at_omega, 0.0, speed_noload, args=(slope_sample, rover, planet, Crr_sample))
            v_max[i,j] = omega_root * r / Ng

        except ValueError:
            v_max[i,j] = np.nan



# # 1. Prepopulate mock 2D matrices using meshgrid
# crr_axis = np.linspace(0.01, 0.2, 30)
# slope_axis = np.linspace(0, 25, 30)
# CRR, SLOPE = np.meshgrid(crr_axis, slope_axis)

# # Mock calculation for Max Rover Speed
# v_max = np.maximum(0.1, 5.0 - (CRR * 10) - (np.sin(np.radians(SLOPE)) * 8))

# # 2. Initialize using your exact legacy syntax
# figure = plt.figure(figsize=(8, 6))
# ax = Axes3D(figure, elev=1, azim=1)

# # Fix: If it's a layout issue, plt.tight_layout() can sometimes collapse
# # legacy Axes3D instances to a size of 0. We specify the geometry explicitly here.
# figure.add_axes(ax)

# # 3. Plot the surface
# ax.plot_surface(CRR, SLOPE, v_max, cmap='viridis')

# # 4. Add labels and title
# ax.set_xlabel('Crr')
# ax.set_ylabel('Slope [deg]')
# ax.set_zlabel('Max Rover Speed [m/s]')
# ax.set_title('LEGACY TEST: Max Rover Speed vs Crr and Slope')

# plt.show()
figure = plt.figure()

ax = Axes3D(figure, elev = 5, azim = 5)

figure.add_axes(ax)
ax.plot_surface(CRR, SLOPE, v_max, cmap = 'viridis')

ax.set_xlabel('Crr')
ax.set_ylabel('Slope [deg]')
ax.set_zlabel('Max Rover Speed [m/s]')
ax.set_title('Max Rover Speed vs Crr and Slope')
ax.legend()

plt.tight_layout()
plt.show()