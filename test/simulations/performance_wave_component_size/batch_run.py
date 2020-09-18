# Data processing
import numpy as np 
import pandas as pd 
from pandas import DataFrame
import subprocess
import os
import sys
import shutil


# Function to configure and build the binaries.
# This test rely on the asv_swarm binaries created in each of the subforders.
# Create the binaries if it does not exist by calling this function.
def build_all():
	subfolders = [ f.path for f in os.scandir(".") if f.is_dir() ]
	for dir in subfolders:
		ps = subprocess.Popen(["mkdir", "build"], cwd=dir)
		ps.wait()
		ps = subprocess.Popen(["cmake", "--config", "Release", "../"], cwd=dir+"/build")
		ps.wait()
		ps = subprocess.Popen(["make"], cwd=dir+"/build")
		ps.wait()

# remove all asv_out dir
def clean_bin():
	# remove asv_out from sub-directories
	subfolders = [ f.path for f in os.scandir(".") if f.is_dir() ]
	for dir in subfolders:
		build_dir = dir + "/build"
		# check if asv_out dir exist
		if(os.path.isdir(build_dir)):
			print("removing " + build_dir)
			shutil.rmtree(build_dir)

# remove all asv_out dir
def clean_output():
	# remove the file with time recording
	time_file = "./run_time"
	if(os.path.isfile(time_file)):
		print("removing " + time_file)
		os.remove(time_file)
	# remove asv_out from sub-directories
	subfolders = [ f.path for f in os.scandir(".") if f.is_dir() ]
	for dir in subfolders:
		path_to_asv_out = dir + "/asv_out"
		# check if asv_out dir exist
		if(os.path.isdir(path_to_asv_out)):
			print("removing " + path_to_asv_out)
			shutil.rmtree(path_to_asv_out)

def clean_all():
	clean_bin()
	clean_output()

# Run the simulation defined in each subdirectory
def run_all():
	wave_ht = 1.0
	wave_heading = 90
	rand = 1
	subfolders = [ f.path for f in os.scandir(".") if f.is_dir() ]
	for dir in subfolders:
		print("Run simulation in directory - " + dir)
		ps = subprocess.Popen(["python3", "simulate.py", str(wave_ht), str(wave_heading), str(rand)], cwd=dir)
		# Run each simulation one after the other
		ps.wait()

# Iterate through each subdirectory and find the average run time
def get_simulation_time(file):
	subfolders = [ f.path for f in os.scandir(".") if f.is_dir() ]
	for dir in subfolders:
		time_file = dir + "/asv_out/run_time"
		data = pd.read_csv(time_file, sep=" ", header=None)
		df = DataFrame(data)
		file.write("{directory} {real_time} {sim_time} {ratio} \n".format(
			directory=dir, 
			real_time=df[0].mean(), 
			sim_time=df[2].mean(), 
			ratio=df[4].mean()))
		#file.write(dir + str(df[0].mean()) + " " + str(df[2].mean()) + " " + str(df[4].mean()) + "\n")

def print_error_msg():
	print('''\nError! Incorrect command. Valid options are:

Build all binaries:
python3 batch_run.py build_all

Run all simulations: 
python3 batch_run.py wave_height wave_heading rand_seed

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
		build_all()
	elif sys.argv[1] == "run_all":
		run_all()
		# Write the simulation run times to file
		run_time_data = open("./run_time", "w")
		run_time_data.write("dir_name real_time(s) sim_time(s) real_to_sim_ratio \n")
		get_simulation_time(run_time_data)
	else:
		print_error_msg()
else:
	print_error_msg()
		
