import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from subfunctions import define_rover

rover = define_rover()
motor = rover['wheel_assembly']['motor']
effcy_tau = motor['effcy_tau']
effcy = motor['effcy']

effcy_fun = interp1d(effcy_tau, effcy, kind = 'cubic') # fit the cubic spline

tau = np.linspace(np.min(effcy_tau), np.max(effcy_tau), 100)
effcy_fit = effcy_fun(tau)

plt.plot(tau, effcy_fit, linestyle='-', label='Cubic spline')
plt.plot(effcy_tau, effcy, marker="*", linestyle='None', markersize=10, label='Rover data')

plt.title('Efficiency vs. Torque')
plt.xlabel('Motor torque (N m)')
plt.ylabel('Efficiency (fraction)')
plt.legend()

plt.show()