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
print(rover)

omega_array = np.linspace(0, 100, 100)

tau_array = tau_dcmotor(omega_array, rover['wheel_assembly']['motor'])
power_array = tau_array * omega_array

plt.subplot(3,1,1)
plt.plot(omega_array, tau_array)
plt.xlabel('Speed [rad/s]')
plt.ylabel('Torque [Nm]')
plt.title('Motor Shaft Speed vs. Motor Shaft Torque')

plt.subplot(3,1,2)
plt.plot(power_array, tau_array)
plt.xlabel('Power [W]')
plt.ylabel('Torque [Nm]')
plt.title('Motor Power vs. Motor Shaft Torque')

plt.subplot(3,1,3)
plt.plot(power_array, omega_array)
plt.xlabel('Power [W]')
plt.ylabel('Speed [rad/s]')
plt.title('Motor Power vs. Motor Shaft Speed')

plt.tight_layout()

plt.show()