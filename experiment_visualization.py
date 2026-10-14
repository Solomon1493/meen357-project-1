import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from subfunctions import *
from define_experiment import *


experiment = experiment1()
alpha_dist = experiment[0]['alpha_dist']
alpha_deg = experiment[0]['alpha_deg']

alpha_fun = interp1d(alpha_dist, alpha_deg, kind = 'cubic', fill_value = 'extrapolate') # fit the cubic spline

x = np.linspace(0, 1000, 1001)
y = alpha_fun(x)

print(x)

print(y)

plt.plot(x, y, linestyle='-', label='Line Plot')

# Customize the chart
plt.title('Line Plot vs. Star Symbols')
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.legend()

# Display the plot
plt.show()