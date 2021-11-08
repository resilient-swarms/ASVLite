import sys
import toml
from collections import namedtuple
import math
import csv
from dataclasses import dataclass
sys.path.insert(0, "../wrapper")
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions
from rudder_controller_pid import Rudder_controller
from wave_data import Wave_data
from storm_track import Storm_track
from datetime import datetime, timedelta
import cartopy.crs as ccrs
import matplotlib.pyplot as plt
from tqdm import tqdm

Simulation_data = namedtuple("Simulation_data", ["time", "cog_x", "cog_y", "cog_z", "asv_heading", "v_surg", "hs", "distance_to_storm"])

@dataclass
class _Simulation_object:
    '''
    A class to hold the simulation data related to a single ASV.
    '''
    id: int
    wave: Wave
    asv: Asv
    waypoints: list
    current_waypoint_index: int
    simulation_data : list # Hold simulation data
    controller : Rudder_controller

class Simulation:
    '''
    Class to perform the simulation of vehicle and waves.
    '''
    def __init__(self, asv_input_file, nc_file_path, storm_track, simulation_start_time, simulation_end_time):
        toml_file = asv_input_file
        # Wave data
        self.wave_data = Wave_data(nc_file_path)
        # Cyclone track
        self.storm_track = Storm_track(storm_track)
        # Get the time range of the simulation.
        self.simulation_start_time = simulation_start_time
        self.simulation_end_time = simulation_end_time
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
            #Create wave
            start_latitude = asv.origin_position.x
            start_longitude = asv.origin_position.y
            wave_hs, wave_dp = self.wave_data.get_wave_data_at(start_latitude, start_longitude, self.simulation_start_time)
            rand_seed = 1
            wave = Wave(wave_hs, wave_dp, rand_seed)
            self.asvs.append(_Simulation_object(id, wave, asv, waypoints, 0, [], Rudder_controller(asv.spec, [25,1,9])))
        if "clock" in toml_data:
            self.time_step_size = toml_data["clock"]["time_step_size"] # millisec
        else:
            self.time_step_size = 40 # millisec
        for item in self.asvs:
            # Set the clock
            item.asv.dynamics.time_step_size = self.time_step_size/1000.0 # sec
            item.asv.dynamics.time = 0.0 
            # Initialise the ASV
            item.asv.init(item.wave) 
    
    def run(self):
        record_asv_path = [] # To record the asv path to a text file
        record_distance_to_storm = [] # To record the distance between asv and storm to a text file
        record_hs = [] # To record the significant wave height experianced by the asv to a text file
        time = self.simulation_start_time
        time_step_size = self.time_step_size/1000.0 # sec
        times = [] # List of time steps to simulate
        while time <= self.simulation_end_time:
            times.append(time)           
            time = time + timedelta(seconds=time_step_size)
        for current_time in tqdm(times):
            # Find the position of the storm for the current time
            current_time = current_time + timedelta(seconds=time_step_size)
            cyclone_eye = self.storm_track.get_eye_location(current_time)
            for item in self.asvs:
                # Get the distance to the centre of the storm
                # Ref: https://www.movable-type.co.uk/scripts/latlong.html
                lat1  = item.asv.cog_position.x * math.pi/180.0         
                long1 = item.asv.cog_position.y * math.pi/180.0
                lat2  = item.waypoints[0].x * math.pi/180.0
                long2 = item.waypoints[0].y * math.pi/180.0
                d_lat = (lat2 - lat1)
                d_long = (long2 - long1) 
                a = (math.sin(d_lat/2.0))**2 + math.cos(lat2)*math.cos(lat1)*(math.sin(d_long/2.0))**2
                c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                R = 6378000.0
                distance_to_storm = R*c / 1000.0 # Km
                # Get the sea state
                current_latitude = item.asv.cog_position.x
                current_longitude = item.asv.cog_position.y
                new_hs, new_dp = self.wave_data.get_wave_data_at(current_latitude, current_longitude, current_time)
                # Compare the sea state with the current sea state
                current_hs = item.wave.significant_wave_height
                current_dp = item.wave.heading
                is_sea_state_same = (float(new_hs) == float(current_hs) and float(new_dp == current_dp))
                # If the sea state has changed then, set the new sea state in the asv object
                if not is_sea_state_same:
                    rand_seed = 1
                    #print(wave_hs)
                    item.wave = Wave(new_hs, new_dp, rand_seed)
                    item.asv.set_sea_state(item.wave)
                # TODO: Update waypoint if required
                item.waypoints[0] = Dimensions(*cyclone_eye)
                # Set rudder angle
                using_earth_coordinate_system = True
                rudder_angle = item.controller.get_rudder_angle(item.asv, item.waypoints[0], using_earth_coordinate_system)
                # Compute the dynamics for the current time step
                seconds = (current_time - self.simulation_start_time).total_seconds()
                item.asv.compute_dynamics(rudder_angle, seconds)
                # Save simulation data
                item.simulation_data.append(Simulation_data(current_time,
                                                            item.asv.cog_position.x, 
                                                            item.asv.cog_position.y, 
                                                            item.asv.cog_position.z, 
                                                            item.asv.attitude.z, 
                                                            item.asv.dynamics.V[0],
                                                            current_hs,
                                                            distance_to_storm))
                record_asv_path.append([seconds, current_time, item.asv.cog_position.x, item.asv.cog_position.y])
        
    def write_simulation_data(self, dir_path):
        for asv in self.asvs:
            file_path = dir_path + "/" + asv.id
            f = open(file_path, "w")  
            for data in asv.simulation_data:
                f.write("{} {} {} {} {} {} {} {} \n".format(data.time, data.cog_x, data.cog_y, data.cog_z, data.asv_heading, data.v_surg, data.hs, data.distance_to_storm))
            f.close()

    def plot_path(self):
        ax = plt.axes(projection=ccrs.PlateCarree())
        # ax.stock_img()
        ax.coastlines()
        # Set the extent of the plot
        longitude_min = min(self.wave_data.longitudes[0], self.wave_data.longitudes[-1])
        longitude_max = max(self.wave_data.longitudes[0], self.wave_data.longitudes[-1])
        latitude_min = min(self.wave_data.latitudes[0], self.wave_data.latitudes[-1])
        latitude_max = max(self.wave_data.latitudes[0], self.wave_data.latitudes[-1])
        ax.set_xlim(longitude_min, longitude_max)
        ax.set_ylim(latitude_min, latitude_max)
        # Plot map
        plt.plot(color='blue', linewidth=2, marker='o',
                transform=ccrs.Geodetic())
        # Plot storm track
        times      = [data[0] for data in self.storm_track.track]
        latitudes  = [data[1] for data in self.storm_track.track]
        longitudes = [data[2] for data in self.storm_track.track]
        plt.plot(longitudes, latitudes, color='red', linestyle='--', transform=ccrs.Geodetic())
        # Set storm markers
        # ax.scatter(longitudes, latitudes, s=80, marker=".", color='red',)
        # Plot vehicle path
        # Plot ASV path
        for asv in self.asvs:
            times = [data[0] for data in asv.simulation_data]
            latitudes = [data[1] for data in asv.simulation_data]
            longitudes = [data[2] for data in asv.simulation_data]
            plt.plot(longitudes, latitudes, linestyle='-', transform=ccrs.Geodetic())
        plt.show()

if __name__ == '__main__':   
    asv_input_file = "./sample_files/katrina/wave_glider"
    nc_file_path = "./sample_files/katrina/wave_data.nc"
    storm_track = "./sample_files/katrina/track.csv"
    simulation_start_time = datetime(2005, 8, 26, 9)
    simulation_end_time = datetime(2005, 8, 29, 20)
    simulation = Simulation(asv_input_file, nc_file_path, storm_track, simulation_start_time, simulation_end_time)
    simulation.run()
    simulation.write_simulation_data(".")
    simulation.plot_path()

# p "../sample_files/cyclone_path.txt" u 2:1 w l, "../sample_files/world_10m.txt" u 1:2 w l, "path_01.txt" u 4:3 w l, "path_02.txt" u 4:3 w l, "path_03.txt" u 4:3 w l, "path_04.txt" u 4:3 w l, "path_05.txt" u 4:3 w l, "path_06.txt" u 4:3 w l, "path_07.txt" u 4:3 w l
# p "distance_01.txt" w l, "distance_02.txt" w l, "distance_03.txt" w l, "distance_04.txt" w l, "distance_05.txt" w l, "distance_06.txt" w l, "distance_07.txt" w l
