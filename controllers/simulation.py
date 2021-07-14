import sys
import toml
sys.path.insert(0, "../wrapper")
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions

class Simulation:
    def __init__(self, command_line_args):
        toml_file = command_line_args[0]
        output_name = command_line_args[1]
        significant_wave_ht = float(command_line_args[2])
        wave_heading = float(command_line_args[3])
        rand_seed = int(command_line_args[4])
        # Create wave
        self.wave = Wave(significant_wave_ht, wave_heading, rand_seed)
        # Create ASVs 
        self.asvs = {} # collection of asvs --> id:(Asv, waypoints)
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
            waypoints = item["waypoints"]
            self.asvs[id] = (asv, waypoints)
        if "clock" in toml_data:
            self.time_step_size = toml_data["clock"]["time_step_size"] # millisec
        else:
            self.time_step_size = 40 # millisec
        for key, value in self.asvs.items():
            asv, waypoints = value
            # Set the clock
            asv.dynamics.time_step_size = self.time_step_size/1000.0 # sec
            asv.dynamics.time = 0.0 
            # Initialise the ASV
            asv.init(self.wave)
    
    def run(self):
        # Initialise time to 0.0 sec
        time = 0.0
        while time < 10000:
            for key, value in self.asvs.items():
                asv, waypoints = value
                time += self.time_step_size/1000.0 # sec
                # Set rudder angle
                rudder_angle = 0.0
                # Compute the dynamics for the current time step
                asv.compute_dynamics(rudder_angle, time)
                print(asv.cog_position.x)



if __name__ == '__main__':    
    simulation = Simulation(sys.argv[1:])
    simulation.run()