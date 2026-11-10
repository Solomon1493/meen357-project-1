import matplotlib.pyplot as plt

from define_experiment import experiment1
from subfunctions import define_planet, define_rover, simulate_rover


def main():
    experiment, end_event = experiment1()
    end_event["max_distance"] = 1000
    end_event["max_time"] = 10000
    end_event["min_velocity"] = 0.01

    rover = simulate_rover(
        define_rover(),
        define_planet(),
        experiment,
        end_event,
    )
    telemetry = rover["telemetry"]
    time = telemetry["Time"]

    figure, axes = plt.subplots(3, 1, sharex=True)
    axes[0].plot(time, telemetry["position"])
    axes[0].set_title("Position vs. Time")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Position (m)")

    axes[1].plot(time, telemetry["velocity"])
    axes[1].set_title("Velocity vs. Time")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity (m/s)")

    axes[2].plot(time, telemetry["power"])
    axes[2].set_title("Power vs. Time")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Power (W)")

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
