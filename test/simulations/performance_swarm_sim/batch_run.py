# Data processing
import numpy as np 
import pandas as pd 
from pandas import DataFrame
import subprocess
import os
import sys
import shutil
import time


# Function to configure and build the binaries.
# This test rely on the asv_swarm binaries created in this director.
# Create the binary file if it does not exist by calling this function.
def build():
	dir = "."
	ps = subprocess.Popen(["mkdir", "build"], cwd=dir)
	ps.wait()
	ps = subprocess.Popen(["cmake", "--config", "Release", "../"], cwd=dir+"/build")
	ps.wait()
	ps = subprocess.Popen(["make"], cwd=dir+"/build")
	ps.wait()

# remove all asv_out dir
def clean_bin():
	dir = "."
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
	# remove asv_out dir
	dir = "."
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
	app = "./build/asv_simulator"
	input_file = "./asv"
	wave_ht = 1.0
	wave_heading = 90
	rand = 1
	swarm_sizes = [10, 50, 100, 150, 200, 250, 500, 1000]
	# Create the output dir
	out_dir = "./asv_out"
	os.mkdir(out_dir)
	time_file = open("./run_time", "w")
	time_file.write("swarm_size real_time(s) sim_time(s) real_to_sim_ratio\n")
	for swarm_size in swarm_sizes:
		print("Simulating swarm of size {}".format(swarm_size))
		subprocesses = []
		# Write the simulation run times to file
		dir = "{}/{}".format(out_dir, swarm_size)
		os.mkdir(dir)
		swarm_time_file_name = "{}/run_time".format(dir)
		swarm_time_file = open(swarm_time_file_name, "w")
		#run_time_record.write("real_time(s) sim_time(s) real_to_sim_ratio\n")
		start_time = time.time()
		for i in range(swarm_size):
			#out_file = "./asv_out/{}/{}".format(swarm_size, i)
			out_file = "/dev/null"
			ps = subprocess.Popen([app, input_file, out_file, str(wave_ht), str(wave_heading), str(rand)], stdout=swarm_time_file)
			subprocesses.append(ps)
		[ps.wait() for ps in subprocesses]
		end_time = time.time()
		sim_time = end_time - start_time # sec
    	# Write the simulation times to file
		data = pd.read_csv(swarm_time_file_name, sep=" ", header=None)
		df = DataFrame(data)
		time_file.write("{size} {real_time} {sim_time} {ratio} \n".format(
			size=swarm_size, 
			real_time=df[0].mean(), 
			sim_time=sim_time, 
			ratio=df[0].mean()/sim_time))

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
	elif sys.argv[1] == "build":
		build()
	elif sys.argv[1] == "run_all":
		run_all()
	else:
		print_error_msg()
else:
	print_error_msg()
		
