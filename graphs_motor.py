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
motor = rover["wheel_assembly"]["motor"]

tau = tau_dcmotor(omega_array, motor)

