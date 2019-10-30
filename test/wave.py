# Data processing
import numpy as np 
import pandas as pd 
from pandas import DataFrame
# Plot 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib import cm
# Run command line
import subprocess
import os
# Measure time for simulations
import time
# Interrupt handling
import signal
import sys
# Get user home dir
from pathlib import Path

# Simulations
# -----------
wave_heights = np.arange(0.5, 2.1, 0.5)
wave_headings = np.arange(0.0, 361.0, 15.0)
trials = np.arange(1, 2, 1)
count_wave_hts = wave_heights.shape[0]
count_wave_headings = wave_headings.shape[0]
count_trials = trials.shape[0]
count_simulations = 0
subprocesses = []
# Table to hold arry of motion amplitude for each wave hts and wave heading
heave = [[[] for i_wave_heading in range(count_wave_headings)] for i_wave_ht in range(count_wave_hts)]
pitch = [[[] for i_wave_heading in range(count_wave_headings)] for i_wave_ht in range(count_wave_hts)]
roll  = [[[] for i_wave_heading in range(count_wave_headings)] for i_wave_ht in range(count_wave_hts)]

# Create the subdirectory for output files
#out_dir = os.getcwd()
out_dir = str(Path.home())
out_dir = out_dir + "/asv_out"
os.mkdir(out_dir)

# Interrupt handler
# -----------------
def interrupt_handler(sig, frame):
	# Kill all subprocesses and also clean all intermediate files. 
	print('\nKill all simulations.')
	for ps in subprocesses:
		pid = ps.pid
		os.kill(pid, signal.SIGINT)
	print('Clean files:')
	for wave_height in wave_heights:
		for wave_heading in wave_headings:
			file_name = out_dir + "/{}_{}_{}".format(trial, wave_height, wave_heading)
			os.remove(file_name)
			print("... removed file {}".format(file_name))
	sys.exit(0)

# Register the interrupt handler. 
signal.signal(signal.SIGINT, interrupt_handler)

# Merge intermediate files into a single file
# -------------------------------------------
def merge_files():
	file_name = out_dir + "/asv_wave"
	file = open(file_name, "w")
	print("Merge files:")
	file_count = 0
	for wave_height in wave_heights:
		for wave_heading in wave_headings:
			file2_name = out_dir + "/{}_{}_{}".format(trial, wave_height, wave_heading)
			file2 = open(file2_name, "r")
			print("... merging file {}".format(file2_name))
			line_count = 0
			for line in file2:
				if(file_count != 0 and line_count == 0): 
					file.write('\n') # ignore the header
				else:
					file.write(line)
				line_count += 1
			file_count += 1
			file2.close()
			print("... remove file {}".format(file2_name))
			os.remove(file2_name)
	file.close()

# Read simulation data from file into dataframe
# ---------------------------------------------
def read_data_from_file(file_name):
	data = pd.read_csv(file_name, sep=" ")
	#print(data.info())
	#print(data.head().append(data.tail())) # Print head and tail of dataframe 
	dfs = DataFrame(data, columns=[	'sig_wave_ht(m)', 
									'wave_heading(deg)', 
									'rand_seed',
									'wave_elevation(m)', 
									'cog_z(m)', 
									'trim(deg)',
									'heel(deg)'	])
	return dfs

# Function to get array of amplitudes
# -----------------------------------
# This function creates a list motion amplitudes from the given list of instanious values. 
def get_abs_amplitudes(values):
	amplitudes = []
	is_first = True
	val_previous = 0.0
	val_max = 0.0
	for val in values:
		if(val * val_previous > 0.0):
			if(abs(val) > abs(val_max)):
				val_max = val
			else:
				pass
		else:
			# zero crossing or first entry
			if(is_first):
				is_first = False
			else:
				amplitudes.append(abs(val_max))
			val_max = val
		val_previous = val
	return amplitudes

# Function to read each file and set data into the tables for motion amplitudes.
def append_data(table, col_heading):
	for i in range(count_trials):
		for j in range(count_wave_hts):
			for k in range(count_wave_headings):
				file = out_dir + "/{}_{}_{}".format(trials[i], wave_heights[j], wave_headings[k])
				dfs = read_data_from_file(file)
				data = get_abs_amplitudes(dfs[col_heading])
				table[j][k].append(data)

# Plot
# ----
def box_plot(data, title):
	fig, axs = plt.subplots(count_wave_hts, count_wave_headings)
	fig.suptitle(title)
	for i in range(count_wave_hts):
		for j in range(count_wave_headings):
			axs[i][j].boxplot(data[i][j])
			#sub_title = "{}m,{}deg".format(wave_heights[i], wave_headings[j])
			#axs[i][j].set_title(sub_title)
	plt.show()

def contour_plot(data, title):
	fig = plt.figure()
	fig.suptitle(title)
	X, Y = np.meshgrid(wave_heights, wave_headings)
	Z = np.zeros(shape=(count_wave_hts, count_wave_headings))
	for i in range(count_wave_hts):
		for j in range(count_wave_headings):
			amps = np.array(data[i][j])
			abs_amps = np.absolute(amps)
			Z[i][j] = abs_amps.mean()
	contour = plt.contourf(X, Y, np.transpose(Z), cmap=cm.coolwarm)
	plt.colorbar()
	plt.show()

def scatter_plot(data, title):
	fig = plt.figure()
	fig.suptitle(title)
	X, Y = np.meshgrid(wave_heights, wave_headings)
	Z = np.zeros(shape=(count_wave_hts, count_wave_headings))
	for i in range(count_wave_hts):
		for j in range(count_wave_headings):
			amps = np.array(data[i][j])
			abs_amps = np.absolute(amps)
			Z[i][j] = abs_amps.mean()
	contour = plt.scatter(X, Y, c=np.transpose(Z), cmap=cm.coolwarm)
	plt.colorbar()
	plt.show()

# Run simulation
# --------------
start_time = time.time()
for trial in trials:
	subprocesses.clear()
	for wave_height in wave_heights:
		for wave_heading in wave_headings:
			count_simulations += 1
			out_file = out_dir + "/{}_{}_{}".format(trial, wave_height, wave_heading)
			print("count simulations = {} [trial={}, wave_ht={}, wave_heading={}]".format(count_simulations, trial, wave_height, wave_heading))
			ps = subprocess.Popen(["../build/asv_simulator", "asv", out_file, str(wave_height), str(wave_heading), str(trial)])
			subprocesses.append(ps)
	exit_codes = [p.wait() for p in subprocesses]
print("Simulations completed.")
end_time = time.time()
sim_time_sec = end_time - start_time
sim_time_min = (sim_time_sec/60.0)
print("\nSimulation time = {0:0.2f}sec ({0:0.2f}min)\n".format(sim_time_sec, sim_time_min))

# merge_files()

# Read each file and get motion amplitudes
append_data(heave, 'cog_z(m)')
append_data(pitch, 'trim(deg)')
append_data(roll, 'heel(deg)')

box_plot(heave, 'Heave(m)')
contour_plot(heave, 'Heave(m)')
scatter_plot(heave, 'Heave(m)')

box_plot(pitch, 'Pitch(deg)')
contour_plot(pitch, 'Pitch(deg)')
scatter_plot(pitch, 'Pitch(deg)')

box_plot(roll, 'Roll(deg)')
contour_plot(roll, 'Roll(deg)')
scatter_plot(roll, 'Roll(deg)')