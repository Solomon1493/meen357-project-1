import matplotlib.pyplot as plt
import numpy as np
from subfunctions import *

'''
Create a py-file script called graphs_motor.py according to the following specifications:
• It does not display anything to the console
• It plots the following three graphs in a 3x1 array (use the matplotlib.pyplot.subplot
command to achieve this) in the following order top to bottom:
o motor shaft speed [rad/s] vs. motor shaft torque [Nm] (use torque on the x-axis)
o motor power [W] vs. motor shaft torque [Nm] (use torque on the x-axis)
o motor power [W] vs. motor shaft speed [rad/s] (use speed on the x-axis)
• All graphs should have both axes labeled clearly (with units indicated). Use the
matplotlib.pyplot.xlabel and matplotlib.pyplot.ylabel commands.
• Use the functions you created to generate the graphs
'''

rover = define_rover()
speed_noload = rover['wheel_assembly']['motor']['speed_noload']

omega_array = np.linspace(0, speed_noload, 100)

tau_array = tau_dcmotor(omega_array, rover['wheel_assembly']['motor'])
power_array = tau_array * omega_array

plt.subplot(3,1,1)
plt.plot(tau_array, omega_array)
plt.xlabel('Torque [Nm]')
plt.ylabel('Speed [rad/s]')
plt.title('Motor Shaft Speed vs. Motor Shaft Torque')

plt.subplot(3,1,2)
plt.plot(tau_array, power_array)
plt.xlabel('Torque [Nm]')
plt.ylabel('Power [W]')
plt.title('Motor Power vs. Motor Shaft Torque')

plt.subplot(3,1,3)
plt.plot(omega_array, power_array)
plt.xlabel('Speed [rad/s]')
plt.ylabel('Power [W]')
plt.title('Motor Power vs. Motor Shaft Speed')

plt.tight_layout()

plt.show()