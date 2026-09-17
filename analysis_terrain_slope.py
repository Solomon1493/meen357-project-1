from subfunctions import *
import numpy as np
import matplotlib.pyplot as plt

'''
Crete a py-file script called analysis_terrain_slope.py in which you use a root-finding method (e.g.,
bisection method, secant method, etc.) to determine the speed of the rover at various terrain slopes.
• Assume a coefficient of rolling resistance of 𝐶𝑟𝑟 = 0.15.
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

crr = 0.15
slope_array_deg = np.linspace(-15,35,25)
v_max = np.zeros(len(slope_array_deg))

print(v_max)
# for i, slope in enumerate(slope_array_deg):
#     slope_rad = np.deg2rad(slope)
#     F_net = F_net(crr, slope_rad)
#     v_max[i] = F_net / mass_rover

# plt.plot(slope_array_deg, v_max)
# plt.xlabel('Slope [deg]')
# plt.ylabel('Velocity [m/s]')
# plt.title('Maximum Velocity vs. Slope')
# plt.show()
