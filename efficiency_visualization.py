import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from subfunctions import *
from define_experiment import *

experiment = experiment1()
effcy_tau = experiment[0]['effcy_tau']
effcy = experiment[0]['effcy']

effcy_fun = interp1d(effcy_tau, effcy, kind = 'cubic') # fit the cubic spline

x = np.linspace(0, 1000, 1001)
y = effcy_fun(x)

plt.plot(x, y, linestyle='-', label='Fitted Curve')
plt.plot(effcy_tau, effcy, marker="*", linestyle='None', markersize=10, label='Experiment Data')

plt.title('Efficiency vs. Torque')
plt.xlabel('Torque (N-m)')
plt.ylabel('Efficiency')
plt.legend()