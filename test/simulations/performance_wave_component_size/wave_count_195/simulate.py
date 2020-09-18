# Data processing
import numpy as np 
import subprocess
import os
import sys

# This file should be run as:
# python3 simulate wave_height wave_heading rand_seed

# Extract the command line args
wave_ht = sys.argv[1]
wave_heading = sys.argv[2]
rand = sys.argv[3]

# simulator app
app = "./build/asv_simulator"

# input file
input_file = "../asv"

# Create the output dir
out_dir = "./asv_out"
os.mkdir(out_dir)

# number of trials
trials = np.arange(1, 11, 1)

# simulate each trial one after the other.
time_file_name = out_dir + "/run_time"
time_file = open(time_file_name, "w")
for trial in trials:
	print("trial - " + str(trial))
	#out_file = out_dir + "/asv_out_" + str(trial)
	out_file = "/dev/null"
	ps = subprocess.run([app, input_file, out_file, str(wave_ht), str(wave_heading), str(rand)], stdout=time_file)

