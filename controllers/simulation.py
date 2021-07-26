import sys
import toml
import collections
import math
from dataclasses import dataclass
sys.path.insert(0, "../wrapper")
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions

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

class Simulation:
    '''
    Class to perform the simulation of vehicle and waves.
    '''
    def __init__(self, command_line_args):
        toml_file = command_line_args[0]
        output_name = command_line_args[1]
        significant_wave_ht = float(command_line_args[2])
        wave_heading = float(command_line_args[3])
        rand_seed = int(command_line_args[4])
        # Create wave
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
            asv.attitude = Dimensions(*item["asv_attitude"])
            thrusters = item["thrusters"]
            for i in range(len(thrusters)):
                asv.propellers[i] = Asv_propeller()
                asv.propellers[i].position = Dimensions(*thrusters[i])  
            _waypoints = item["waypoints"]
            waypoints = []
            for waypoint in _waypoints:
                waypoints.append(Dimensions(*waypoint))
            self.asvs.append(_Simulation_object(id, asv, waypoints, 0, []))
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
        # Initialise time to 0.0 sec
        time = 0.0 # sec
        # loop till all asvs reach destination
        while all([item.current_waypoint_index < len(item.waypoints) for item in self.asvs]):
            for item in self.asvs:
                # Current waypoint index
                i = item.current_waypoint_index
                if i < len(item.waypoints):
                    # Set rudder angle
                    # TODO: rudder_angle = controller.get_rudder_angle(time, )
                    rudder_angle = 0.0
                    # Compute the dynamics for the current time step
                    item.asv.compute_dynamics(rudder_angle, time)
                    # Save simulation data
                    item.simulation_data.append([item.asv.cog_position.x, 
                                                 item.asv.cog_position.y, 
                                                 item.asv.cog_position.z, 
                                                 item.asv.attitude.z, 
                                                 item.asv.dynamics.V[0]])
                    # Increment time
                    time += self.time_step_size/1000.0
                    # Check if reached the waypoint
                    proximity_margin = 2.0
                    x = item.asv.cog_position.x - item.waypoints[i].x
                    y = item.asv.cog_position.y - item.waypoints[i].y
                    distance = math.sqrt(x**2 + y**2)
                    if distance <= proximity_margin:
                        item.current_waypoint_index += 1

if __name__ == '__main__':    
    simulation = Simulation(sys.argv[1:])
    simulation.run()