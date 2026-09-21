import math
import numpy as np

def define_rover():
    rover = {
        'wheel_assembly': {
            'wheel': {
                'radius': 0.30, 
                'mass': 1.0
            },
            'speed_reducer': {
                'type': 'reverted',
                'diam_pinion': 0.04,
                'diam_gear': 0.07,
                'mass': 1.5
            },
            'motor': {
                'torque_stall': 170,
                'torque_noload': 0,
                'speed_noload': 3.80,
                'mass': 5.0
            }
        },
        'chassis': {'mass': 659},
        'science_payload': {'mass': 75},
        'power_subsys': {'mass': 90}
    }
    return rover

rover = define_rover()

def define_planet():
    return {'g': 3.72}

def tau_dcmotor(omega, motor):
    if not (np.isscalar(omega) or np.ndim(omega) <= 1):
        raise Exception("Omega must be a scalar or vector")
    if not isinstance(motor, dict):
        raise Exception("Motor must be a dictionary")

    torque_stall = motor["torque_stall"]
    torque_noload = motor["torque_noload"]
    speed_noload = motor["speed_noload"]
    
    tau = np.zeros(len(omega))

    for i in range(len(omega)):
        if omega[i] > speed_noload:
            tau[i] = 0
        elif omega[i] < 0:
            tau[i] = torque_stall
        else:
            tau[i] = torque_stall - (((torque_stall - torque_noload) / speed_noload) * omega[i])
    return tau

def get_gear_ratio(speed_reducer):
    if not isinstance(speed_reducer, dict):
        raise Exception("Speed reducer must be a dictionary")

    reducer_type = speed_reducer.get("type", "").lower()
    if reducer_type != "reverted":
        raise Exception(f"Invalid speed reducer type: '{reducer_type}'. Only 'reverted' gear sets are supported.")

    pinion_d = speed_reducer["diam_pinion"]
    gear_d = speed_reducer["diam_gear"]
    Ng = (gear_d/pinion_d)**2
    return Ng

def get_mass(rover):
    if not isinstance(rover, dict):
        raise Exception("Rover must be a dictionary")
    
    mass_wheel = rover['wheel_assembly']['wheel']['mass'] + rover['wheel_assembly']['speed_reducer']['mass'] + rover['wheel_assembly']['motor']['mass']
    mass_chassis = rover['chassis']['mass']
    mass_science_payload = rover['science_payload']['mass']
    mass_power_subsys = rover['power_subsys']['mass']
    m = mass_wheel + mass_chassis + mass_science_payload + mass_power_subsys
    return m

def F_rolling(omega, terrain_angle, rover, planet, Crr):
    if len(omega) != len(terrain_angle):
        raise Exception("Omega and terrain angle must have the same length")

    if any(angle > 75 or angle < -75 for angle in terrain_angle):
        raise Exception("Slope angle must be between -75 and 75 degrees")

    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("Rover and planet must be a dictionary")

    if not (np.isscalar(Crr) or Crr >= 0):
        raise Exception("Crr must be a scalar and greater than or equal to 0")

    mass_rover = get_mass(rover)
    gear_ratio = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])

    Frr = np.zeros(len(terrain_angle))

    for x in range(len(Frr)):
        Frr[x] = Crr * mass_rover * planet["g"] * math.cos(math.degrees(terrain_angle[x]))
        Frr[x] = Frr[x] * math.erf(40*omega[x])
    
    return Frr

def F_gravity(terrain_angle, rover, planet):
    if len(terrain_angle) != len(rover) or len(terrain_angle) != len(planet):
        raise Exception("Terrain angle, rover, and planet must have the same length")

    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("Rover and planet must be a dictionary")

    if any(angle > 75 or angle < -75 for angle in terrain_angle):
        raise Exception("Slope angle must be between -75 and 75 degrees")

    mass_rover = get_mass(rover)
    Fgt = np.zeros(len(terrain_angle))

    for x in range(len(Fgt)):
        Fgt[x] = mass_rover * planet["g"] * math.sin(math.degrees(terrain_angle[x]))

    return Fgt

planet = define_planet()
print(F_gravity([0, 10, 20, 30, 40, 50, 60, 65, 70, 75], rover, planet), "N")

def F_drive():
    return F_drive
def F_net():
    return F_net
