from subfunctions import *
import matplotlib.pyplot as plt
import numpy as np

"""
Create a py-file script called graphs_sr.py that is similar to graphs_motor.py EXCEPT that it uses the
speed, torque, and power of the speed reducer output shaft (the motor shaft now is the input to the
speed reducer; your graphs should be of the torque, speed, and power of the speed reducer output). As
in the previous case, please label your axes properly.
"""

rover = define_rover()
gear_ratio = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])

omega_array = np.linspace(0, 100, 100)
tau_array = tau_dcmotor(omega_array, rover['wheel_assembly']['motor']) * gear_ratio
power_array = tau_array * omega_array

plt.subplot(3,1,1)
plt.plot(tau_array, omega_array)
plt.xlabel('Torque [Nm]')
plt.ylabel('Speed [rad/s]')
plt.title('Speed Reducer Speed vs. Speed Reducer Torque')

plt.subplot(3,1,2)
plt.plot(tau_array, power_array)
plt.xlabel('Torque [Nm]')
plt.ylabel('Power [W]')
plt.title('Speed Reducer Power vs. Speed Reducer Torque')

plt.tight_layout()

plt.show()