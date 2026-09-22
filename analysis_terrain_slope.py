from subfunctions import *
import numpy as np
import matplotlib.pyplot as plt
### Ash and Arnav install if needed, useful for built-in root finder functions ###
from scipy.optimize import brentq


'''
Crete a py-file script called analysis_terrain_slope.py in which you use a root-finding method (e.g.,
bisection method, secant method, etc.) to determine the speed of the rover at various terrain slopes.
• Assume a coefficient of rolling resistance of Crr = 0.15.
• Generate terrain angles to test with the following line of code:
o slope_array_deg = numpy.linspace(-15,35,25);
o Note that this gives you angles in DEGREES.
• Store the maximum velocity [m/s] at each angle in a vector called v_max.
• Plot v_max versus slope_array_deg. Make sure to label the axes and indicate their units.
MEEN 357 Engineering Analysis for Mechanical Engineers Fall 2026
10
• Do not display anything to the console
• *** Hint: Since we are not using a speed controller in this model, your rover will travel at the
fastest speed it can in any given situation. Its top speed is the velocity it is at when it stops
accelerating. This means you must look for the operating point of the motor at which the net force
acting on the rover is zero. ***
• Hint: You can use the no-load and stall speeds of the motor to define an initial bracket for a root-
finding method. Alternatively, provide an open method with any value on this range.
'''

Crr = 0.15
rover = define_rover()
planet = define_planet()
Ng = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
r = rover['wheel_assembly']['wheel']['radius']
speed_noload = rover['wheel_assembly']['motor']['speed_noload']
slope_array_deg = np.linspace(-15,35,25)
v_max = np.zeros(len(slope_array_deg))

def F_net_at_omega(omega, angle, rover, planet, Crr):
    return F_net(omega, angle, rover, planet, Crr)

for i, angle in enumerate(slope_array_deg):
    try:
        omega_root = brentq(F_net_at_omega, 0.0, speed_noload, args=(angle, rover, planet, Crr))
        v_max[i] = omega_root * r / Ng
    except ValueError:
        v_max[i] = np.nan

plt.figure()
plt.plot(slope_array_deg, v_max)
plt.xlabel("Terrain Slope [deg]")
plt.ylabel("Max Rover Speed [m/s]")
plt.title("Max Rover Speed vs Terrain Slope")

plt.tight_layout()

plt.show()
