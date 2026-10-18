import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from subfunctions import define_rover


def main():
    rover = define_rover()
    motor = rover["wheel_assembly"]["motor"]
    effcy_tau = motor["effcy_tau"]
    effcy = motor["effcy"]

    effcy_fun = interp1d(effcy_tau, effcy, kind="cubic", fill_value="extrapolate")

    x = np.linspace(effcy_tau.min(), effcy_tau.max(), 1001)
    y = effcy_fun(x)

    plt.plot(x, y, linestyle='-', label='Fitted Curve')
    plt.plot(effcy_tau, effcy, marker="*", linestyle='None', markersize=10, label='Experiment Data')

    plt.title('Efficiency vs. Torque')
    plt.xlabel('Torque (N-m)')
    plt.ylabel('Efficiency')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()