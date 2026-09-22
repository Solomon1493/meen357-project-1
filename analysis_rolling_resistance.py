from subfunctions import *
import numpy as np
import matplotlib.pyplot as plt
### Ash and Arnav install if needed, useful for built-in root finder functions ###
from scipy.optimize import brentq

rover = define_rover()
planet = define_planet()
Ng = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
r = rover['wheel_assembly']['wheel']['radius']
speed_noload = rover['wheel_assembly']['motor']['speed_noload']
Crr_array = np.linspace(0.01, 0.5, 25)
terrain_angle = 0.0

v_max = np.zeros(len(Crr_array))

def F_net_at_omega(omega, angle, rover, planet, Crr):
    return F_net(omega, angle, rover, planet, Crr)

for i, Crr in enumerate(Crr_array):
    try:
        Crr_root = brentq(F_net_at_omega, 0.0, speed_noload, args=(terrain_angle, rover, planet, Crr))
        v_max[i] = Crr_root * r / Ng
    except ValueError:
        v_max[i] = np.nan

plt.figure()
plt.plot(Crr_array, v_max)
plt.xlabel("Crr Value")
plt.ylabel("Max Rover Speed [m/s]")
plt.title("Max Rover Speed vs Crr Value")

plt.tight_layout()

plt.show()
