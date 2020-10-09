#!/usr/bin/env python3

# Data processing
import numpy as np 
import pandas as pd 
from pandas import DataFrame
import subprocess
import os
import sys
import shutil
import time

asv_input_string = str("""
[[asv]]
id = "asv{id}"
L_wl = 0.3 # m
B_wl = 0.3 # m
D = 0.3  # m
T = 0.1 # m
displacement = 0.007 # m3
max_speed = 2.0 # m/s
cog = [0.15, 0.0, -0.2] # [x(m), y(m), z(m)]
radius_of_gyration = [0.08, 0.08, 0.106] # [r_roll(m), r_pitch(m), r_yaw(m)]
thrusters = [[0.065, -0.085, -0.0485], 
			 [0.235, -0.085, -0.0485], 
			 [0.235, 0.085, -0.0485], 
			 [0.065, 0.085, -0.0485]] # [[x(m), y(m), z(m)]]
asv_position = [{x_1}, {y_1}] # [X(m), Y(m)]
asv_attitude = [0.0, 0.0, 0.0] #[heel(deg), trim(deg), heading(deg)]
waypoints = [[{x_2}, {y_2}]] # [[X(m), Y(m)]]
""")

clock_input_string = str("""
[clock]
time_step_size = 40 # milli-sec
""")

visualisation_input_string = str("""
[visualisation]
sea_surface_edge_length = {length} #m
count_mesh_cells_along_edge = 20 # Number of mesh cells along one edge of the sea surface. A larger number means a finer sea surface mesh.
""")

out_dir = "./asv_out"
out_dir_swarm = out_dir + "/{size}"
build_dir = "./build"
build_dir_threading_disabled = build_dir + "/threading_disabled"
build_dir_threading_with_sync = build_dir + "/threading_with_sync"
build_dir_threading_without_sync = build_dir + "/threading_without_sync"
swarm_sizes = [10, 50, 100, 150, 200, 250, 500, 1000]


def crete_output_dir():
	os.mkdir(out_dir)
	for swarm_size in swarm_sizes:
		os.mkdir(out_dir_swarm.format(size = swarm_size))

def create_build_dir():
	os.mkdir(build_dir)
	os.mkdir(build_dir_threading_disabled)
	os.mkdir(build_dir_threading_with_sync)
	os.mkdir(build_dir_threading_without_sync)

def create_input_file(swarm_size):
	file_name = out_dir_swarm + "/asv"
	file = open(file_name.format(size=swarm_size),"w")
	for i in range(swarm_size):
		x_start = float(i)
		y_start = 0.0
		x_end = float(i) 
		y_end = 20.0
		field_length = max((y_end - y_start), swarm_size)
		file.write(asv_input_string.format(id=i, x_1=x_start, y_1=y_start, x_2=x_end, y_2=y_end))
	file.write(clock_input_string)
	file.write(visualisation_input_string.format(length = field_length))

def create_input_files():
	for swarm_size in swarm_sizes:
		create_input_file(swarm_size)

# Function to configure and build the simulator.
# This test rely on the asv_swarm binaries created in this director.
# Create the binary file if it does not exist by calling this function.
# 3 binaries are created
# - threading_disabled
# - threading_with_sync
# - threading_without_sync
def build():
	project_dir = "../../../../../"
	create_build_dir()
	# Build with threading disabled
	ps = subprocess.Popen(["cmake", "-DENABLE_MULTI_THREADING=OFF", "--config", "Release", project_dir], cwd=build_dir_threading_disabled)
	ps.wait()
	ps = subprocess.Popen(["make"], cwd=build_dir_threading_disabled)
	ps.wait()
	# Build with sync enabled
	ps = subprocess.Popen(["cmake", "-DENABLE_TIME_SYNC=ON", "--config", "Release", project_dir], cwd=build_dir_threading_with_sync)
	ps.wait()
	ps = subprocess.Popen(["make"], cwd=build_dir_threading_with_sync)
	ps.wait()
	# Build with sync disabled
	ps = subprocess.Popen(["cmake", "-DENABLE_TIME_SYNC=OFF", "--config", "Release", project_dir], cwd=build_dir_threading_without_sync)
	ps.wait()
	ps = subprocess.Popen(["make"], cwd=build_dir_threading_without_sync)
	ps.wait()


# remove build dir
def clean_bin():
	# check if build dir exist
	if(os.path.isdir(build_dir)):
		print("removing " + build_dir)
		shutil.rmtree(build_dir)

# remove asv_out dir
def clean_output():
	# check if asv_out dir exist
	if(os.path.isdir(out_dir)):
		print("removing " + out_dir)
		shutil.rmtree(out_dir)
	# remove the file with time recording
	time_file = "./run_time"
	if(os.path.isfile(time_file)):
		print("removing " + time_file)
		os.remove(time_file)

def clean_all():
	clean_bin()
	clean_output()

def write_summary(time_file, summary_file, build_type, swarm_size):
	data = pd.read_csv(time_file, sep=" ", header=None)
	df = DataFrame(data)
	summary_file.write("{size} {build} {real_time} {sim_time} {ratio} \n".format(
		size=swarm_size, 
		build=build_type,
		real_time=df[1].mean(), 
		sim_time=df[3].mean(), 
		ratio=df[5].mean()).encode())

# Run simulation for all swarm sizes
def run_all():
	crete_output_dir()
	create_input_files()
	app_threading_disabled = build_dir_threading_disabled + "/asv_simulator"
	app_threading_with_sync = build_dir_threading_with_sync + "/asv_simulator"
	app_threading_without_sync = build_dir_threading_without_sync + "/asv_simulator"
	wave_ht = 1.0
	wave_heading = 180.0
	rand = 1
	summary_file = open("./run_time", "wb", buffering=0)
	summary_file.write("swarm_size real_time(s) sim_time(s) real_to_sim_ratio\n".encode())
	for swarm_size in swarm_sizes:
		print("Simulating swarm of size {}".format(swarm_size))
		input_file_name = out_dir_swarm.format(size=swarm_size) + "/asv"
		out_file_name = out_dir_swarm.format(size=swarm_size) + "/asv_out"
		
		time_file_name = out_dir_swarm.format(size=swarm_size) + "/time_threading_disabled"
		time_file = open(time_file_name, "wb", buffering=0)
		ps = subprocess.Popen([app_threading_disabled, input_file_name, out_file_name, str(wave_ht), str(wave_heading), str(rand)], stdout=time_file)
		ps.wait()
		write_summary(time_file_name, summary_file, "threading_disabled", swarm_size)

		time_file_name = out_dir_swarm.format(size=swarm_size) + "/time_threading_with_sync"
		time_file = open(time_file_name, "wb", buffering=0)
		ps = subprocess.Popen([app_threading_with_sync, input_file_name, out_file_name, str(wave_ht), str(wave_heading), str(rand)], stdout=time_file)
		ps.wait()
		write_summary(time_file_name, summary_file, "sync_enabled", swarm_size)

		time_file_name = out_dir_swarm.format(size=swarm_size) + "/time_threading_without_sync"
		time_file = open(time_file_name, "wb", buffering=0)
		ps = subprocess.Popen([app_threading_without_sync, input_file_name, out_file_name, str(wave_ht), str(wave_heading), str(rand)], stdout=time_file)
		ps.wait()
		write_summary(time_file_name, summary_file, "sync_disable", swarm_size)


def print_error_msg():
	print('''\nError! Incorrect command. Valid options are:

Build all binaries:
python3 batch_run.py build

Run all simulations: 
python3 batch_run.py run_all

Clean all binaries and output files: 
python3 batch_run.py clean_all

Clean only output file: 
python3 batch_run.py clean_output

Clean only binaries: 
python3 batch_run.py clean_bin''')

# Extract the command line args
if len(sys.argv) == 2:
	if sys.argv[1] == "clean_all":
		clean_all()
	elif sys.argv[1] == "clean_bin":
		clean_bin()
	elif sys.argv[1] == "clean_output":
		clean_output()
	elif sys.argv[1] == "build_all":
		build()
	elif sys.argv[1] == "run_all":
		run_all()
	else:
		print_error_msg()
else:
	print_error_msg()
		
