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

figure = plt.figure()

ax = Axes3D(figure, elev = 5, azim = 5)

ax.plot_surface(CRR, SLOPE, v_max)

ax.plot()