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

def tau_dcmotor(wheel_assembly, speed):
    torque_stall =  wheel_assembly["motor"]["torque_stall"]
    torque_noload = wheel_assembly["motor"]["torque_noload"]
    speed_noload = wheel_assembly["motor"]["speed_noload"]
    current_speed = speed

    tau = torque_stall - (((torque_stall - torque_noload)/speed_noload) * current_speed)
    
    return tau

def get_gear_ratio(speed_reducer):
    pinion_d = speed_reducer["diam_pinion"]
    gear_d = speed_reducer["diam_gear"]
    ratio = (gear_d/pinion_d)**2
    return ratio

def get_mass(wheel_assembly, chassis, science_payload, power_subsys):
    mass_wheel = wheel_assembly["wheel"]["mass"] + wheel_assembly["speed_reducer"]["mass"] + wheel_assembly["motor"]["mass"]
    mass_chassis = chassis["mass"]
    mass_science_payload = science_payload["mass"]
    mass_power_subsys = power_subsys["mass"]
    mass_rover = mass_wheel + mass_chassis + mass_science_payload + mass_power_subsys
    return mass_rover


def F_rolling(omega, terrain_angle, rover, planet, Crr):
    mass_rover = get_mass(rover['wheel_assembly'], rover['chassis'], rover['science_payload'], rover['power_subsys'])
    gear_ratio = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])

    if len(omega) != len(terrain_angle):
        raise Exception("Omega and terrain angle must have the same length")

    if any(angle > 75 or angle < -75 for angle in terrain_angle):
        raise Exception("Slope angle must be between -75 and 75 degrees")

    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("Rover and planet must be a dictionary")

    if not (np.isscalar(Crr) or Crr >= 0):
        raise Exception("Crr must be a scalar and greater than or equal to 0")

    frr = np.zeros(len(terrain_angle))

    for x in range(len(frr)):
        degree_result = math.cos(math.radians(terrain_angle[x]))
        frr_simple = Crr*mass_rover*planet["g"]*degree_result
        frr = math.erf(40*omega[x])*frr_simple
        frr
    return frr


planet = {"g": 9.81}

print(F_rolling([1,2,3],[-10,60,75], rover, planet, 3))

def F_gravity():
    return F_gravity
def F_drive():
    return F_drive
def F_net():
    return F_net
