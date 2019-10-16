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

# Simulations
# -----------
wave_heights = np.arange(0.5, 2.51, 0.5)
wave_headings = np.arange(0.0, 361.0, 45.0)
trials = np.arange(1, 5, 1)
count_simulations = 0
subprocesses = []

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
			file_name = "asv_{}_{}".format(wave_height, wave_heading)
			os.remove(file_name)
			print("... removed file {}".format(file_name))
	sys.exit(0)

# Register the interrupt handler. 
signal.signal(signal.SIGINT, interrupt_handler)

# Run simulation
# --------------
start_time = time.time()
for trial in trials:
	subprocesses.clear()
	for wave_height in wave_heights:
		for wave_heading in wave_headings:
			count_simulations += 1
			out_file = "asv_{}_{}".format(wave_height, wave_heading)
			print("count simulations = {} [wave_ht={}, wave_heading={}, trial={}]".format(count_simulations, wave_height, wave_heading, trial))
			ps = subprocess.Popen(["../build/asv_simulator", "asv", out_file, str(wave_height), str(wave_heading), str(trial)])
			subprocesses.append(ps)
	exit_codes = [p.wait() for p in subprocesses]
print("Simulations completed.")
end_time = time.time()
sim_time_sec = end_time - start_time
sim_time_min = (sim_time_sec/60.0)
print("\nSimulation time = {0:0.2f}sec ({0:0.2f}min)\n".format(sim_time_sec, sim_time_min))

# Merge intermediate files into a single file
# -------------------------------------------
file = open("asv_wave", "w")
print("Merge files:")
file_count = 0
for wave_height in wave_heights:
	for wave_heading in wave_headings:
		file2_name = "asv_{}_{}".format(wave_height, wave_heading)
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

# Function to get array of amplitudes
# -----------------------------------
def get_amplitudes(values):
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
				amplitudes.append(val_max)
			val_max = val
		val_previous = val
	return amplitudes

# Read simulation data from file into dataframe
# ---------------------------------------------
data = pd.read_csv('asv_wave', sep=" ")
print(data.info())
print(data.head().append(data.tail())) # Print head and tail of dataframe 
dfs = DataFrame(data, columns=[	'sig_wave_ht(m)', 
								'wave_heading(deg)', 
								'rand_seed',
								'wave_elevation(m)', 
								'cog_z(m)', 
								'trim(deg)',
								'heel(deg)'	])
# Group the data:
# ---------------
# Group the data into 4 dimensions:
# dim 1 -> wave heights
# dim 2 -> wave headings
# dim 3 -> trials 
# dim 4 -> amplitudes in a trial
def group_data(col_heading):
	data = []
	df_wave_hts = dfs.groupby('sig_wave_ht(m)')
	for (wave_ht, df_wave_ht) in df_wave_hts:
		data_wave_headings = []
		df_wave_headings = df_wave_ht.groupby('wave_heading(deg)')
		for(wave_heading, df_heading) in df_wave_headings:
			data_trials = []
			df_trials = df_heading.groupby('rand_seed')
			for(trial_id, df_trial) in df_trials:
				data_trial	= get_amplitudes(df_trial[col_heading])
				data_trials.append(data_trial)
			data_wave_headings.append(data_trials)
		data.append(data_wave_headings)
	return data

wave 	= group_data('wave_elevation(m)')
heave 	= group_data('cog_z(m)')
pitch 	= group_data('trim(deg)')
roll 	= group_data('heel(deg)')

# Plot
# ----
n_wave_hts = wave_heights.shape[0]
n_wave_headings = wave_headings.shape[0]
n_trials = trials.shape[0]

def box_plot(data, title):
	fig = plt.figure()
	fig.suptitle(title)
	outer = gridspec.GridSpec(n_wave_hts, n_wave_headings)
	for i in range(n_wave_hts * n_wave_headings):
		inner = gridspec.GridSpecFromSubplotSpec(1, n_trials, subplot_spec=outer[i])
		for j in range(n_trials):
			n = int(i/n_wave_headings)
			m = i%n_wave_headings
			ax = plt.Subplot(fig, inner[j])
			bp = ax.boxplot(data[n][m])
			fig.add_subplot(ax)
	plt.show()

def contour_plot(data, title):
	fig = plt.figure()
	fig.suptitle(title)
	X, Y = np.meshgrid(wave_heights, wave_headings)
	Z = np.zeros(shape=(n_wave_hts, n_wave_headings))
	for i in range(n_wave_hts):
		for j in range(n_wave_headings):
			Z_trials = np.zeros(shape=(n_trials,1)) # mean amplitude of each trial
			for k in range(n_trials):
				amps = np.array(data[i][j][k])
				abs_amps = np.absolute(amps)
				Z_trials[k] = np.median(abs_amps)
			Z[i][j] = Z_trials.mean()
	contour = plt.contourf(X, Y, np.transpose(Z), cmap=cm.coolwarm)
	plt.colorbar()
	plt.show()

box_plot(wave, 'wave_elevation(m)')
contour_plot(wave, 'wave_elevation(m)')
box_plot(heave, 'Heave(m)')
contour_plot(heave, 'Heave(m)')
box_plot(pitch, 'Pitch(deg)')
contour_plot(pitch, 'Pitch(deg)')
box_plot(roll, 'Roll(deg)')
contour_plot(roll, 'Roll(deg)')