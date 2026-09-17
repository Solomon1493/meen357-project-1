def tau_dcmotor():
    
def get_gear_ratio():
    
def get_mass():
    mass_wheel = wheel_assembly["wheel"]["mass"] + wheel_assembly["speed_reducer"]["mass"] + wheel_assembly["motor"]["mass"]
    mass_chassis = chassis["mass"]
    mass_science_payload = science_payload["mass"]
    mass_power_subsys = power_subsys["mass"]
    mass_rover = mass_wheel + mass_chassis + mass_science_payload + mass_power_subsys
    return mass_rover
def F_rolling():
    
def F_gravity():
    
def F_drive():
    
def F_net():
    