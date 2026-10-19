import math
import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from define_experiment import *
#from end_of_mission_event import end_of_mission_event

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
                'mass': 5.0,

                #PHASE 2____________________________________________________________________
                'effcy_tau': np.array([0, 10, 20, 40, 70, 165]),
                'effcy': np.array([0,0.55, 0.75, 0.71, 0.50, 0.05])
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

planet = define_planet()

def tau_dcmotor(omega, motor):
    if not (np.isscalar(omega) or np.ndim(omega) <= 1):
        raise Exception("Omega must be a scalar or vector")
    
    if isinstance(omega, str):
        raise Exception("Omega must be a scalar or vector")
    
    if not isinstance(motor, dict):
        raise Exception("Motor must be a dictionary")

    torque_stall = motor["torque_stall"]
    torque_noload = motor["torque_noload"]
    speed_noload = motor["speed_noload"]

    is_scalar = np.isscalar(omega)
    omega_arr = np.atleast_1d(omega)
    
    tau = np.zeros(len(omega_arr))

    for i in range(len(omega_arr)):
        if omega_arr[i] > speed_noload:
            tau[i] = 0.0
        elif omega_arr[i] < 0:
            tau[i] = torque_stall
        else:
            tau[i] = torque_stall - ((torque_stall - torque_noload) / speed_noload) * omega_arr[i]

    return tau[0] if is_scalar else tau

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
    
    mass_wheel_assembly = (rover['wheel_assembly']['wheel']['mass'] + rover['wheel_assembly']['speed_reducer']['mass'] + rover['wheel_assembly']['motor']['mass'])
    
    mass_chassis = rover['chassis']['mass']
    mass_science_payload = rover['science_payload']['mass']
    mass_power_subsys = rover['power_subsys']['mass']
    
    m = (6* mass_wheel_assembly + mass_chassis + mass_science_payload + mass_power_subsys)
    
    return m

def F_rolling(omega, terrain_angle, rover, planet, Crr):
    is_scalar = np.isscalar(omega) and np.isscalar(terrain_angle)
    omega = np.atleast_1d(omega)
    terrain_angle = np.atleast_1d(terrain_angle)
    
    if len(omega) != len(terrain_angle):
        raise Exception("Omega and terrain angle must have the same length")

    if any(angle > 75 or angle < -75 for angle in terrain_angle):
        raise Exception("Slope angle must be between -75 and 75 degrees")

    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("Rover and planet must be a dictionary")

    if not np.isscalar(Crr) or isinstance(Crr, str) or Crr <= 0:
        raise Exception("Crr must be a positive scalar")

    mass_rover = get_mass(rover)
    gear_ratio = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
    wheel_radius = rover['wheel_assembly']['wheel']['radius']

    Frr = np.zeros(len(terrain_angle))

    for x in range(len(Frr)):
        #convert motor speed to wheel speed
        omega_wheel = omega[x] / gear_ratio
        
        #convert wheel angular speed to rover velocity
        v_rover = wheel_radius * omega_wheel
        
        #calculate normal force
        Fn = mass_rover * planet["g"] * math.cos(np.radians(terrain_angle[x]))
        
        #rolling resis
        Frr[x] = -Crr * Fn *math.erf(40 * v_rover)
    
    return Frr[0] if is_scalar else Frr

def F_gravity(terrain_angle, rover, planet):
    if isinstance(terrain_angle, str) or not (np.isscalar(terrain_angle) or np.ndim(terrain_angle) <= 1):
        raise Exception("Terrain angle must be a scalar or vector")

    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("Rover and planet must be a dictionary")

    is_scalar = np.isscalar(terrain_angle)
    terrain_angle = np.atleast_1d(terrain_angle)

    if any(angle > 75 or angle < -75 for angle in terrain_angle):
        raise Exception("Slope angle must be between -75 and 75 degrees")

    mass_rover = get_mass(rover)
    Fgt = np.zeros(len(terrain_angle))

    for x in range(len(Fgt)):
        Fgt[x] = -mass_rover * planet["g"] * math.sin(np.radians(terrain_angle[x]))

    return Fgt[0] if is_scalar else Fgt

def F_drive(omega, rover):
    #check if omega data type is valid
    omega_type_check = False
    if isinstance(omega, np.ndarray) and omega.ndim == 1:
        omega_type_check = True
    if np.isscalar(omega):
        omega_type_check = True
    
    if omega_type_check == False:
        raise Exception("Omega must be scalar or vector")
        
    if isinstance(rover, dict) == False:
        raise Exception("Rover must be a dictionary")
    
    motor = rover["wheel_assembly"]["motor"]
    sr = rover["wheel_assembly"]["speed_reducer"]
    wheel = rover["wheel_assembly"]["wheel"]
    
    #use formula 
    tau = tau_dcmotor(omega, motor)
    Ng = get_gear_ratio(sr)
    Fd = 6 * Ng * tau / wheel["radius"]

    return Fd

def F_net(omega, terrain_angle, rover, planet, Crr):
    is_scalar = np.isscalar(omega) and np.isscalar(terrain_angle)
    omega = np.atleast_1d(omega)
    terrain_angle = np.atleast_1d(terrain_angle)

    if not (np.isscalar(omega) or np.ndim(omega) <= 1):
        raise Exception("Omega must be a scalar or vector")
    if not (np.isscalar(terrain_angle) or np.ndim(terrain_angle) <= 1):
        raise Exception("Terrain angle must be a scalar or vector")

    if len(omega) != len(terrain_angle):
        raise Exception("Omega and terrain angle must have the same length")
        
    if any(angle > 75 or angle < -75 for angle in terrain_angle):
        raise Exception("Slope angle must be between -75 and 75 degrees")
    
    if not isinstance(rover, dict) or not isinstance(planet, dict):
        raise Exception("Rover and planet must be dictionaries")
        
    if not np.isscalar(Crr) or Crr <= 0:
        raise Exception("Crr must be a positive scalar")
        
    Fd = F_drive(omega, rover)
    Fg = F_gravity(terrain_angle, rover, planet)
    Frr = F_rolling(omega, terrain_angle, rover, planet, Crr)
    
    Fnet = Fd + Fg + Frr
    return Fnet[0] if is_scalar else Fnet

#PROJECT PHASE 2 ________________________________________________________________

def motorW(v, rover): #NEEDS TESTING
    """
    Calculate the Angular Velocity of the DC motor shaft based on the rover's velocity, wheel radius, and speed reducer gear ratio'

    Parameters
    ----------
    v : np.ndarray or scalar (float or integer) value that represents the translational velocity of the rover in meters per second.
    rover : Dictionary that contains rover characteristics.

    Returns
    -------
    W : np.ndarray or scalar (float or integer) value that is the calculated angular velocity of the shaft in radians per second.
    
    Raises
    ------
    Excpetion if velocity is not a np.ndarray or scalar value
    Exception if rover is not a dictionary
    -------

    """
    if not (isinstance(v, np.ndarray) or np.isscalar(v)):
        raise Exception("v must be a scalar or vector")
    if not isinstance(rover, dict):
        raise Exception("Rover must be a dictionary")

    sr = rover["wheel_assembly"]["speed_reducer"]
    wheel = rover["wheel_assembly"]["wheel"]

    ratio = get_gear_ratio(sr)
    r = wheel["radius"]
    W = (v / r) * ratio

    return W


def rover_dynamics(t, y, rover, planet, experiment): # ex. t = 10,  y = [v, x]
    """
    Finds values for the derivative of the rover state vector.
    
    Parameters
    ----------
    t : Current simulation time in seconds
    y : State vector [velocity, position]
    rover : Dictionary with Rover characteristics
    planet : Dictionary with Planet characteristics
    experiment : Dictionary with expirement parameters

    Returns
    -------
    dydt : Array with [acceleration, velocity]

    """
    v = y[0]
    x = y[1]

    alpha_fun = interp1d(experiment['alpha_dist'], experiment['alpha_deg'], kind = 'cubic', fill_value = 'extrapolate')

    terrain_angle = float(alpha_fun(x))
    W = motorW(v, rover)
    m = get_mass(rover)
    Crr = experiment['Crr']
    
    Fnet = F_net(W, terrain_angle, rover, planet, Crr)

    dydt = np.array([Fnet / m, v])

    return dydt


#print(rover_dynamics(10, [10, 10], rover, planet, experiment1()))


def mechpower(v, rover):
    """
    Compute the instantaneous mech power of the DC motor.
    
    Parameters:
        v: Rover Velocity in meters/second as a scalar or 1D array
        rover: Dictionary containing rover parameters
    
    Returns
        P: Mechanical Power W, in Scalar or 1d array
    """
    
    #Confirm valid parameter values
    if isinstance(v, np.ndarray):
        if v.ndim != 1 or not np.issubdtype(v.dtype, np.number):
            raise Exception("Velocity must be a numerical scalar or 1D numPy array ")
    elif not np.isscalar(v) or isinstance(v, (str, bytes, bool)) or not isinstance(v, (int, float, np.number)):
        raise Exception("Velocity must be a numerical scalar or 1D numPy array")
    
    #Validate rover dict value
    if not isinstance(rover, dict):
        raise Exception("Rover must be a dictionary")
        
    #Find motor value
    motor = rover["wheel_assembly"]["motor"]
    
    #Convert rover velocity into angular speed in radians/second
    omega = motorW(v,rover)
    
    #Calculate motor shaft torque in Newton-Meters
    tau = tau_dcmotor(omega, motor)
    
    #calculate mechanical power in Watts
    P = tau * omega
    
    return P


def battenergy(t, v, rover):
    """
    Computes the total battery energy used by all 6 motors in the rover.
    
    Parameters:
        t: 1d array of time values in seconds
        v: 1d array of rover velocities in meters per second
        rover: Dictionary of rover parameters
    
    Returns:
        E: Total battery energy consumed by the motors in Joules
    """
    
    #Confirm values are valid
    if not isinstance(t, np.ndarray) or t.ndim != 1:
        raise Exception("Time must be a 1d NumPy array")
        
    if not isinstance(v, np.ndarray) or v.ndim != 1:
        raise Exception("Velocity must be a 1d numPy array")
        
    if len(t) != len(v):
        raise Exception("Time and velocity must have equal array lengths")
        
    if not np.issubdtype(t.dtype, np.number) or not np.issubdtype(v.dtype, np.number):
        raise Exception("Time and velocity must contain numerical, float, or scalar values")
        
    if not isinstance(rover, dict):
        raise Exception("Rover must be a dictionary")
        
    motor = rover["wheel_assembly"]["motor"]
    
    omega = motorW(v,rover)
    tau = tau_dcmotor(omega, motor)
    
    P_mech = mechpower(v, rover)
    
    effcy_tau = motor["effcy_tau"]
    effcy = motor["effcy"]
    
    eff_func = interp1d(effcy_tau, effcy, kind = "cubic")
    
    eta = eff_func(tau)
    
    P_battery = 6 * P_mech / eta
    
    #Integrate power
    E = np.trapezoid(P_battery, t)
    
    return E

def end_of_mission_event(end_event):
    """
    Defines an event that terminates the mission simulation. Mission is over
    when rover reaches a certain distance, has moved for a maximum simulation 
    time or has reached a minimum velocity.            
    """
    
    mission_distance = end_event['max_distance']
    mission_max_time = end_event['max_time']
    mission_min_velocity = end_event['min_velocity']
    
    # Assume that y[1] is the distance traveled
    distance_left = lambda t,y: mission_distance - y[1]
    distance_left.terminal = True
    
    time_left = lambda t,y: mission_max_time - t
    time_left.terminal = True
    
    velocity_threshold = lambda t,y: y[0] - mission_min_velocity;
    velocity_threshold.terminal = True
    velocity_threshold.direction = -1
    
    # terminal indicates whether any of the conditions can lead to the
    # termination of the ODE solver. In this case all conditions can terminate
    # the simulation independently.
    
    # direction indicates whether the direction along which the different
    # conditions is reached matters or does not matter. In this case, only
    # the direction in which the velocity treshold is arrived at matters
    # (negative)
    
    events = [distance_left, time_left, velocity_threshold]
    
    return events

def simulate_rover(rover, planet, experiment, end_event):
    """
    

    Parameters
    ----------
    rover : dict that contains rover specifications.
    planet : dictionary containing planetary specifications.
    experiment : dictionary containing initial conditions and variables such as time range, terrain distance, slope, and rolling resistance coefficient.
    end_event : dictionary containing maximum values for distance and time, and the minimum velocity conditions. 

    Raises
    ------
    Exception
        if any parameters are not a dictionary.

    Returns
    -------
    rover : updated rover characteristics.

    """
    if not isinstance(rover, dict):
        raise Exception("Rover must be a dictionary")
    if not isinstance(planet, dict):
        raise Exception("Planet must be a dictionary")
    if not isinstance(experiment, dict):
        raise Exception("Experiment must be a dictionary")
    if not isinstance(end_event, dict):
        raise Exception("End event must be a dictionary")
        
    #Old code
    # initial_conditions = experiment[0]['initial_conditions']
    # v0 = initial_conditions[0]
    # x0 = initial_conditions[1]
    
    # alpha_fun = interp1d(experiment['alpha_dist'], experiment['alpha_deg'], kind = 'cubic', fill_value = 'extrapolate')
    # terrain_angle = float(alpha_fun(x0))
    # m = get_mass(rover)
    # Crr = experiment['Crr']
    # W = motorW(v0, rover)
    # Fnet = F_net(W, terrain_angle, rover, planet, Crr)
    # dydt = np.array([Fnet / m, v0])
    # rover_dynamics = solve_ivp(rover_dynamics, t_span, initial_conditions, args=(rover, planet, experiment), t_eval=t_eval)
    # v = rover_dynamics.y[0]
    
    #New code
    #Get ICs and time span
    initial_conditions = experiment["initial_conditions"]
    t_span = experiment["time_range"]
    
    #Get mission stopping conditions
    events = end_of_mission_event(end_event)
    
    #Solve Rover EOM

    sol = solve_ivp(rover_dynamics, t_span, initial_conditions, args=(rover, planet, experiment), events=events, max_step=1)

    
    #check if solution passed
    if not sol.success:
        raise Exception(sol.message)
        
    #Get time, speed, and position values
    t = sol.t
    v = sol.y[0]
    x = sol.y[1]
    
    #Calculate mechanical power and battery energy
    P = mechpower(v, rover)
    E = battenergy(t, v, rover)
    
    #Calculate telemetry values
    completion_time = t[-1] - t[0]
    distance_traveled = x[-1] - x[0]
    max_velocity = np.max(v)
    
    if completion_time > 0:
        average_velocity = np.trapezoid(v, t) / completion_time
    else:
        average_velocity = 0
        
    if distance_traveled > 0:
        energy_per_distance = E / distance_traveled
    else:
        energy_per_distance = 0
    
    #Simulation results of the rover
    rover["telemetry"] = {"Time": t, "completion_time": completion_time, "velocity": v, "position": x, "distance_traveled": distance_traveled, "max_velocity": max_velocity, "average_velocity": average_velocity, "power": P, "battery_energy": E, "energy_per_distance": energy_per_distance}
    
    return rover

#print(simulate_rover(rover, planet, experiment1(), experiment1()))

