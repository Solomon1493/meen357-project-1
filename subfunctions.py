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

def define_planet():
    return {'g': 3.72}

def tau_dcmotor():
    return tau_dcmotor

def get_gear_ratio():
    pinion_d = speed_reducer["diam_pinion"]
    gear_d = speed_reducer["diam_gear"]
    ratio = (gear_d/pinion_d)**2
    return ratio

def get_mass():
    mass_wheel = wheel_assembly["wheel"]["mass"] + wheel_assembly["speed_reducer"]["mass"] + wheel_assembly["motor"]["mass"]
    mass_chassis = chassis["mass"]
    mass_science_payload = science_payload["mass"]
    mass_power_subsys = power_subsys["mass"]
    mass_rover = mass_wheel + mass_chassis + mass_science_payload + mass_power_subsys
    return mass_rover


def F_rolling():
    return F_rolling
def F_gravity():
    return F_gravity
def F_drive():
    return F_drive
def F_net():
    return F_net


