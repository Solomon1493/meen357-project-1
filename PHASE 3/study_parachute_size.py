#Task 2.5 - MEEN357 Project 3
#study the effect of parachute diameter in meters (0.5 meter intervals from 14 to 19 meters) on EDL performance

import numpy as np
import matplotlib.pyplot as plt

from define_edl_system import define_edl_system_1
from define_planet import define_planet
from define_mission_events import define_mission_events
from subfunctions_EDL import simulate_edl


#change to np.nan
if not hasattr(np, "NaN"):
    np.NaN = np.nan
    
#make diamtere ranges from 14 to 19 m in 0.5m intervals
diameters = np.arange(14, 19.5, 0.5)

#Create empty arrays to store results
termination_times = []
rover_speeds = []
landing_success = []

#max time
tmax = 2000 

#test each parachute diameter in simulation
for diameter in diameters:
    edl_system = define_edl_system_1()
    mars = define_planet()
    mission_events = define_mission_events()
    
    #make ICs
    edl_system["altitude"] = 11000 #meters
    edl_system["velocity"] = -590 #meters/second
    
    edl_system["parachute"]["diameter"] = diameter
    edl_system["parachute"]["deployed"] = True
    edl_system["parachute"]["ejected"] = False
    
    edl_system["heat_shield"]["ejected"] = False
    edl_system["rocket"]["on"] = False
    edl_system["sky_crane"]["on"] = False
    edl_system["speed_control"]["on"] = False
    edl_system["position_control"]["on"] = False
    edl_system["rover"]["on_ground"] = False
    
    #Run simulation test
    t, Y, edl_system = simulate_edl(edl_system, mars, mission_events, tmax, False)
    
    #Record the time the simulation ends
    time_end = t[-1]
    #Rover speed
    rover_speed = abs(Y[0,-1] + Y[5,-1])
    #altitude at trial termination
    sky_crane_altitude = Y[1,-1]
    
    #Determine if landing was a success and store results
    safe_landing = (edl_system["rover"]["on_ground"] and sky_crane_altitude >= edl_system["sky_crane"]["danger_altitude"] and abs(rover_speed) <=abs(edl_system["sky_crane"]["danger_speed"]))
    termination_times.append(time_end)
    rover_speeds.append(rover_speed)
    landing_success.append(int(safe_landing))
    
    #results
    print(f"Diameter: {diameter:.1f} m")
    print(f"Termination time: {time_end:.2f} s")
    print(f"Rover velocity: {rover_speed:.3f} m/s")
    print(f"Landing success: {int(safe_landing)}")
    print()
    
#plot all trial results
fix, axs = plt.subplots(3,1,figsize = (9,11))

#plot 1: Termination time (seconds) vs Parachute Diameter (meters)
axs[0].plot(diameters, termination_times, "o-")
axs[0].set_title("Termination Time vs Parachute Diameter")
axs[0].set_xlabel("Parachute Diameter (m)")
axs[0].set_ylabel("Termination Time (s)")
axs[0].grid(True)

#Plot 2: Rover velocity vs parachute diameter
axs[1].plot(diameters, rover_speeds, "o-")
axs[1].set_title("Rover Speed vs Parachute Diameter")
axs[1].set_xlabel("Parachute Diameter (m)")
axs[1].set_ylabel("Rover Speed (m/s)")
axs[1].grid(True)

#Plot 3: Landing success vs paracute diameter
axs[2].plot(diameters, landing_success, "o-")
axs[2].set_title("Landing Success vs Parachute Diameter")
axs[2].set_xlabel("Parachute Diameter (m)")
axs[2].set_ylabel("Landing Success, 1 is Yes and 0 is No")
axs[2].set_yticks([0,1])
axs[2].grid(True)

plt.tight_layout()
plt.show()



    
    
