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
import multiprocessing as mp

Simulation_data = namedtuple("Simulation_data", ["time", "cog_x", "cog_y", "cog_z", "asv_heading", "v_surg", "hs", "distance_to_storm"])

@dataclass
class _Simulation_object:
    '''
    A class to hold the simulation data related to a single ASV.
    '''
    id: str
    wave: Wave
    asv: Asv
    waypoints: list
    current_waypoint_index: int
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
        self.simulation_objects = [] # collection of simulation objects, ie asvs
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
            self.simulation_objects.append(_Simulation_object(id, wave, asv, waypoints, 0, Rudder_controller(asv.spec, [25,1,9])))
        if "clock" in toml_data:
            self.time_step_size = toml_data["clock"]["time_step_size"] # millisec
        else:
            self.time_step_size = 40 # millisec
        for simulation_object in self.simulation_objects:
            # Set the clock
            simulation_object.asv.dynamics.time_step_size = self.time_step_size/1000.0 # sec
            simulation_object.asv.dynamics.time = 0.0 
            # Initialise the ASV
            simulation_object.asv.init(simulation_object.wave) 
    
    def __simulate_asv(self, simulation_object, tqdm_progress_bar, dir_path):
        file_path = dir_path + "/" + simulation_object.id
        f = open(file_path, "w")  
        time = self.simulation_start_time
        time_step_size = self.time_step_size/1000.0 # sec
        times = [] # List of time steps to simulate
        while time <= self.simulation_end_time:
            times.append(time)           
            time = time + timedelta(seconds=time_step_size)
        tqdm_progress_bar.iterable = times
        tqdm_progress_bar.total = len(times)
        tqdm_progress_bar.set_description("Simulating " + str(simulation_object.id))
        for current_time in tqdm_progress_bar:
            # Find the position of the storm for the current time
            current_time = current_time + timedelta(seconds=time_step_size)
            cyclone_eye = self.storm_track.get_eye_location(current_time)
            # Get the distance to the centre of the storm
            # Ref: https://www.movable-type.co.uk/scripts/latlong.html
            lat1  = simulation_object.asv.cog_position.x * math.pi/180.0         
            long1 = simulation_object.asv.cog_position.y * math.pi/180.0
            lat2  = simulation_object.waypoints[0].x * math.pi/180.0
            long2 = simulation_object.waypoints[0].y * math.pi/180.0
            d_lat = (lat2 - lat1)
            d_long = (long2 - long1) 
            a = (math.sin(d_lat/2.0))**2 + math.cos(lat2)*math.cos(lat1)*(math.sin(d_long/2.0))**2
            c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            R = 6378000.0
            distance_to_storm = R*c / 1000.0 # Km
            # Get the sea state
            current_latitude = simulation_object.asv.cog_position.x
            current_longitude = simulation_object.asv.cog_position.y
            new_hs, new_dp = self.wave_data.get_wave_data_at(current_latitude, current_longitude, current_time)
            # Compare the sea state with the current sea state
            current_hs = simulation_object.wave.significant_wave_height
            current_dp = simulation_object.wave.heading
            is_sea_state_same = (float(new_hs) == float(current_hs) and float(new_dp == current_dp))
            # If the sea state has changed then, set the new sea state in the asv object
            if not is_sea_state_same:
                rand_seed = 1
                #print(wave_hs)
                simulation_object.wave = Wave(new_hs, new_dp, rand_seed)
                simulation_object.asv.set_sea_state(simulation_object.wave)
            # TODO: Update waypoint if required
            simulation_object.waypoints[0] = Dimensions(*cyclone_eye)
            # Set rudder angle
            using_earth_coordinate_system = True
            rudder_angle = simulation_object.controller.get_rudder_angle(simulation_object.asv, simulation_object.waypoints[0], using_earth_coordinate_system)
            # Compute the dynamics for the current time step
            seconds = (current_time - self.simulation_start_time).total_seconds()
            simulation_object.asv.compute_dynamics(rudder_angle, seconds)
            # Save simulation data
            f.write("{date} {x} {y} {z} {heading} {v} {hs} {dist} \n".format( 
                                                    date=current_time.strftime("%Y-%m-%d %H:%M:%S.%f"),
                                                    x=simulation_object.asv.cog_position.x, 
                                                    y=simulation_object.asv.cog_position.y, 
                                                    z=simulation_object.asv.cog_position.z, 
                                                    heading=simulation_object.asv.attitude.z, 
                                                    v=simulation_object.asv.dynamics.V[0],
                                                    hs=current_hs,
                                                    dist=distance_to_storm))

    def run(self, dir_path):
        processes = [] # To store the data per process/simulation_object
        # tqdm 
        progress_bars = [tqdm(leave=False) for item in self.simulation_objects]
        index = 0
        # Create the processes
        for simulation_object in self.simulation_objects:
            process = mp.Process(target=self.__simulate_asv, args=(simulation_object, progress_bars[index], dir_path))
            processes.append(process)
            index += 1
        # Start the simulations
        for process in processes:
            process.start()
        # Close processes
        for process in processes:
            process.join()
    
    def plot_path(self, dir_path):
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
        # Create markers
        markers = times[::5]
        legend = [ str(i)+" | "+markers[i].strftime("%d-%b-%y %H:%M") for i in range(len(markers))]
        legend = "\n".join(legend)
        plt.figtext(0.5, 0.01, legend, ha="center", fontsize=6,)
        # Create markers for storm track
        marker_longitudes = longitudes[::5]
        marker_latitudes = latitudes[::5]
        for i in range(len(marker_latitudes)):
            plt.text(marker_longitudes[i], marker_latitudes[i], str(i), color="red", fontsize=6, label=markers[i])
        # Plot ASV path
        for simulation_object in tqdm(self.simulation_objects, desc="Ploting data"):
            file_path = dir_path + "/" + simulation_object.id
            f = open(file_path, "r") 
            # Ploting all the points from the file can be too much,therefor plot the position 
            # of the vehicle for each min instead of each simulation time step
            n = int(1000/self.time_step_size * 60)
            simulation_data = [line.strip().split(" ") for line in f.readlines()[::n]] 
            times = [datetime.strptime(row[0] + " " + row[1], "%Y-%m-%d %H:%M:%S.%f") for row in simulation_data]
            latitudes = [float(row[2]) for row in simulation_data]
            longitudes = [float(row[3]) for row in simulation_data]
            asv_path_plot = plt.plot(longitudes, latitudes, linestyle='-', transform=ccrs.Geodetic())
            # Create markers for ASV path
            color = asv_path_plot[0].get_color()
            indices = []
            for time in markers:
                index = min(range(len(times)), key=lambda i: abs((times[i]-time).total_seconds()))
                indices.append(index)
            for i in range(len(indices)):
                if indices[i] != 0:
                    plt.text(longitudes[indices[i]], latitudes[indices[i]], str(i), fontsize=6, color=color)
        # Add legend
        plt.savefig(dir_path + "/plot.png", bbox_inches='tight', dpi=300)

if __name__ == '__main__':   
    asv_input_file = "./sample_files/katrina/wave_glider"
    nc_file_path = "./sample_files/katrina/wave_data.nc"
    storm_track = "./sample_files/katrina/track.csv"
    simulation_start_time = datetime(2005, 8, 26, 9)
    simulation_end_time = datetime(2005, 8, 29, 20)
    simulation = Simulation(asv_input_file, nc_file_path, storm_track, simulation_start_time, simulation_end_time)
    #simulation.run("./temp")
    simulation.plot_path("./temp")

# p "../sample_files/cyclone_path.txt" u 2:1 w l, "../sample_files/world_10m.txt" u 1:2 w l, "path_01.txt" u 4:3 w l, "path_02.txt" u 4:3 w l, "path_03.txt" u 4:3 w l, "path_04.txt" u 4:3 w l, "path_05.txt" u 4:3 w l, "path_06.txt" u 4:3 w l, "path_07.txt" u 4:3 w l
# p "distance_01.txt" w l, "distance_02.txt" w l, "distance_03.txt" w l, "distance_04.txt" w l, "distance_05.txt" w l, "distance_06.txt" w l, "distance_07.txt" w l
