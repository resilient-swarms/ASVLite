import sys
import toml
import collections
import math
from dataclasses import dataclass
sys.path.insert(0, "../wrapper")
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions
from rudder_controller_pid import Rudder_controller
from cyclone import Cyclone, Time, Location, Data

@dataclass
class _Simulation_object:
    '''
    A class to hold the simulation data related to a single ASV.
    '''
    id: int
    asv: Asv
    waypoints: list
    current_waypoint_index: int
    simulation_data : list # Hold the following simulation data - [cog_x, cog_y, cog_z, asv_heading, v_surge].
    controller : Rudder_controller

class Simulation:
    '''
    Class to perform the simulation of vehicle and waves.
    '''
    def __init__(self, asv_input_file, hs_files, dp_files):
        toml_file = asv_input_file
        # Create cyclone
        self.cyclone = Cyclone()
        self.cyclone.init(hs_files, dp_files)
        # Get the time range of the simulation.
        count_files = self.cyclone.count_sets
        count_time_steps_in_last_file = self.cyclone.hs[count_files - 1].count_time_steps
        self.simulation_start_time = self.cyclone.hs[0].time_steps[0] # hrs
        self.simulation_end_time = self.cyclone.hs[count_files-1].time_steps[count_time_steps_in_last_file-1] # hrs
        # TODO: set the start position of the wave glider
        # TODO: set the current centre of the storm
        # Create wave
        # TODO: change the initial sea state to the first sea state at the start position.
        significant_wave_ht = 1.1
        wave_heading        = 30
        rand_seed           = 1
        self.wave = Wave(significant_wave_ht, wave_heading, rand_seed)
        # Create ASVs 
        self.asvs = [] # collection of simulation objects, ie asvs
        toml_data = toml.load(toml_file)
        if not "asv" in toml_data:
            raise ValueError("Table [[asv]] missing in input file.")
        for item in toml_data["asv"]:
            asv = Asv()
            id = item["id"]
            asv.spec.L_wl = item["L_wl"]
            asv.spec.B_wl = item["B_wl"]
            asv.spec.D = item["D"]
            asv.spec.T = item["T"]
            asv.spec.max_speed = item["max_speed"]
            asv.spec.disp = item["displacement"]
            asv.spec.r_roll  = item["radius_of_gyration"][0]
            asv.spec.r_pitch = item["radius_of_gyration"][1]
            asv.spec.r_yaw   = item["radius_of_gyration"][2]
            asv.spec.cog = Dimensions(*item["cog"])
            asv.origin_position = Dimensions(*item["asv_position"])
            asv.attitude = Dimensions(*[angle * math.pi/180.0 for angle in item["asv_attitude"]])
            thrusters = item["thrusters"]
            for i in range(len(thrusters)):
                asv.propellers[i] = Asv_propeller()
                asv.propellers[i].position = Dimensions(*thrusters[i])  
            _waypoints = item["waypoints"]
            waypoints = []
            for waypoint in _waypoints:
                waypoints.append(Dimensions(*waypoint))
            self.asvs.append(_Simulation_object(id, asv, waypoints, 0, [], Rudder_controller(asv.spec, [25,1,9])))
        if "clock" in toml_data:
            self.time_step_size = toml_data["clock"]["time_step_size"] # millisec
        else:
            self.time_step_size = 40 # millisec
        for item in self.asvs:
            # Set the clock
            item.asv.dynamics.time_step_size = self.time_step_size/1000.0 # sec
            item.asv.dynamics.time = 0.0 
            # Initialise the ASV
            item.asv.init(self.wave)        
    
    def run(self):
        f = open("./path.txt", "w")  
        # Initialise time to 0.0 sec
        time = 0.0 # sec
        # loop till all asvs reach destination
        while all([item.current_waypoint_index < len(item.waypoints) for item in self.asvs]):
            for item in self.asvs:
                # Current waypoint index
                i = item.current_waypoint_index
                # Increment time
                time += self.time_step_size/1000.0
                # Check if reached the waypoint
                proximity_margin = 25.0
                x = item.asv.cog_position.x - item.waypoints[i].x
                y = item.asv.cog_position.y - item.waypoints[i].y
                distance = math.sqrt(x**2 + y**2)
                if distance <= proximity_margin:
                    # Reached waypoint
                    item.current_waypoint_index += 1
                else:
                    if i < len(item.waypoints):
                        # TODO: Get the sea state
                        # TODO: Compare the sea state with the current sea state
                        # TODO: if the sea state has changed then, set the new sea state in the asv object
                        # Set rudder angle
                        rudder_angle = item.controller.get_rudder_angle(item.asv, item.waypoints[i])
                        # Compute the dynamics for the current time step
                        #print(rudder_angle)
                        item.asv.compute_dynamics(rudder_angle, time)
                        # Save simulation data
                        item.simulation_data.append([item.asv.cog_position.x, 
                                                    item.asv.cog_position.y, 
                                                    item.asv.cog_position.z, 
                                                    item.asv.attitude.z, 
                                                    item.asv.dynamics.V[0]])
                        f.write("{} {} \n".format(item.asv.cog_position.x, item.asv.cog_position.y))
        f.close()

if __name__ == '__main__':   
    asv_input_file = "./sample_files/wave_glider"
    hs_files = ["./sample_files/hs1.nc", "./sample_files/hs2.nc", "./sample_files/hs3.nc"]
    dp_files = ["./sample_files/dp1.nc", "./sample_files/dp2.nc", "./sample_files/dp3.nc"] 
    simulation = Simulation(asv_input_file, hs_files, dp_files)
    simulation.run()