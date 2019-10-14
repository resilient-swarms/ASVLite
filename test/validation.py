# Data processing
import numpy as np 
import pandas as pd 
from pandas import DataFrame
# Plot 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
# Run command line
import subprocess
import os
# Measure time for simulations
import time
# Interrupt handling
import signal
import sys

count_simulations = 0
subprocesses = []

def interrupt_handler(sig, frame):
	print('\nKill all simulations.')
	for ps in subprocesses:
		pid = ps.pid
		os.kill(pid, signal.SIGINT)
	sys.exit(0)

# Register the interrupt handler. 
signal.signal(signal.SIGINT, interrupt_handler)

# Motion is still water
# ---------------------
# Run simulation
count_simulations += 1
print("count simulations = {}".format(count_simulations))
subprocess.run(["../build/asv_simulator", "asv", "asv_still_water", "0.0", "0.0", "1"])

# Read simulation data from file into dataframe
data = pd.read_csv('asv_still_water', sep=" ")
print(data.info())
print(data.head().append(data.tail())) # Print head and tail of dataframe 

# Plot
fig_still_water = plt.figure()
# Path
df = DataFrame(data, columns=['cog_x(m)', 'cog_y(m)'])
ax1 = fig_still_water.add_subplot(141)
ax1.title.set_text('Path')
ax1.set_xlabel('X(m)')
ax1.set_ylabel('Y(m)')
ax1.plot('cog_x(m)', 'cog_y(m)', data=df)
# Pitch
df = DataFrame(data, columns=['time(sec)', 'trim(deg)'])
ax2 = fig_still_water.add_subplot(142)
ax2.title.set_text('Trim')
ax2.set_xlabel('time(sec)')
ax2.set_ylabel('trim(deg)')
ax2.plot('time(sec)', 'trim(deg)', data=df)
# Heel
df = DataFrame(data, columns=['time(sec)', 'heel(deg)'])
ax3 = fig_still_water.add_subplot(143)
ax3.title.set_text('Heel')
ax3.set_xlabel('time(sec)')
ax3.set_ylabel('heel(deg)')
ax3.plot('time(sec)', 'heel(deg)', data=df)
# Surge velocity and acceleration
df = DataFrame(data, columns=['time(sec)', 'surge_vel(m/s)', 'surge_acc(m/s2)'])
ax4 = fig_still_water.add_subplot(144)
ax4.title.set_text('Surge velocity and acceleration')
ax4.set_xlabel('time(sec)')
ax4.plot('time(sec)', 'surge_vel(m/s)', data=df, color='skyblue')
ax4.plot('time(sec)', 'surge_acc(m/s2)', data=df, color='orange')
plt.show()

# Motion in waves
# ---------------
# Run simulation
start_time = time.time()
wave_heights = np.arange(0.5, 2.51, 0.5)
wave_headings = np.arange(0.0, 360.0, 30.0)
rand_seeds = range(1, 11)
for rand_seed in rand_seeds:
	subprocesses.clear()
	for wave_height in wave_heights:
		for wave_heading in wave_headings:
			count_simulations += 1
			out_file = "asv_{}_{}".format(wave_height, wave_heading)
			print("count simulations = {} [wave_ht={}, wave_heading={}, rand_seed={}]".format(count_simulations, wave_height, wave_heading, rand_seed))
			ps = subprocess.Popen(["../build/asv_simulator", "asv", out_file, str(wave_height), str(wave_heading), str(rand_seed)])
			subprocesses.append(ps)
	exit_codes = [p.wait() for p in subprocesses]
print("Simulations completed.")

# Merge all output files to a single file
file1 = open("asv_wave", "w")
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
				file1.write('\n') # ignore the header
			else:
				file1.write(line)
			line_count += 1
		file_count += 1
		file2.close()
		print("... remove file {}".format(file2_name))
		os.remove(file2_name)
file1.close()

# Clock
end_time = time.time()
sim_time = end_time - start_time
print("\nSimulation time = {0:0.2f}sec ({0:0.2f}min)\n".format(sim_time, sim_time/60.0))

# Read simulation data from file into dataframe
data = pd.read_csv('asv_wave', sep=" ")
print(data.info())
print(data.head().append(data.tail())) # Print head and tail of dataframe 

# Plot
# Relative heave
fig_heave = plt.figure()
fig_heave.suptitle('Heave, in m, with respect to sea surface. (wave_ht, wave_heading)')
dfs = DataFrame(data, columns=['sig_wave_ht(m)', 'wave_heading(deg)', 'wave_elevation(m)', 'cog_z(m)'])
dfs['rel_heave(m)'] = dfs['cog_z(m)'] - dfs['wave_elevation(m)']
dfs['rel_heave(m)'] = dfs['rel_heave(m)']
dfs = dfs.groupby('sig_wave_ht(m)')
nrows = len(wave_heights)
ncols = len(wave_headings)
n = 0
heave_mean = []
for (wave_ht, df) in dfs:
	df = df.groupby('wave_heading(deg)')
	heave_mean_row = []
	for(wave_heading, table) in df:
		n += 1
		heave = table['rel_heave(m)'].abs()
		heave_mean_row.append(heave.mean())
		heave = heave.values
		ax = fig_heave.add_subplot(nrows, ncols, n)
		ax.title.set_text('({}, {})'.format(wave_ht, wave_heading))
		bp_dict = ax.boxplot(heave, vert=False)
		for line in bp_dict['medians']:
			# get position data for median line
			(x, y) = line.get_xydata()[1] # top of median line
			# overlay median value
			ax.text(x, y, "%.3f" % x, horizontalalignment='center') # draw above, centered
		for line in bp_dict['boxes']:
			(x, y) = line.get_xydata()[0] # bottom of left line
			ax.text(x, y, "%.3f" % x, horizontalalignment='right', verticalalignment='top') # centered, below
			(x, y) = line.get_xydata()[3] # bottom of right line
			ax.text(x, y, "%.3f" % x, horizontalalignment='left', verticalalignment='top') # centered, below
	heave_mean.append(heave_mean_row)
plt.show()
# Surface plot
heave_3D = plt.figure()
heave_3D.suptitle('Heave')
ax = heave_3D.gca(projection='3d')
X, Y = np.meshgrid(wave_heights, wave_headings)
Z = np.array(heave_mean)
surf = ax.plot_surface(X, Y, np.transpose(Z), cmap=cm.coolwarm, linewidth=0, antialiased=False)
heave_3D.colorbar(surf, shrink=0.5, aspect=5)
plt.show()

# Trim
fig_trim = plt.figure()
fig_trim.suptitle('Trim, in deg. (wave_ht, wave_heading)')
dfs = DataFrame(data, columns=['sig_wave_ht(m)', 'wave_heading(deg)', 'trim(deg)'])
dfs = dfs.groupby('sig_wave_ht(m)')
nrows = len(wave_heights)
ncols = len(wave_headings)
n = 0
trim_mean = []
for (wave_ht, df) in dfs:
	df = df.groupby('wave_heading(deg)')
	trim_mean_row = []
	for(wave_heading, table) in df:
		n += 1
		trim = table['trim(deg)'].abs()
		trim_mean_row.append(trim.mean())
		trim = trim.values
		ax = fig_trim.add_subplot(nrows, ncols, n)
		ax.title.set_text('({}, {})'.format(wave_ht, wave_heading))
		bp_dict = ax.boxplot(trim, vert=False)
		for line in bp_dict['medians']:
			# get position data for median line
			(x, y) = line.get_xydata()[1] # top of median line
			# overlay median value
			ax.text(x, y, "%.3f" % x, horizontalalignment='center') # draw above, centered
		for line in bp_dict['boxes']:
			(x, y) = line.get_xydata()[0] # bottom of left line
			ax.text(x, y, "%.3f" % x, horizontalalignment='right', verticalalignment='top') # centered, below
			(x, y) = line.get_xydata()[3] # bottom of right line
			ax.text(x, y, "%.3f" % x, horizontalalignment='left', verticalalignment='top') # centered, below
	trim_mean.append(trim_mean_row)
plt.show()
# Surface plot
trim_3D = plt.figure()
trim_3D.suptitle('Trim')
ax = trim_3D.gca(projection='3d')
X, Y = np.meshgrid(wave_heights, wave_headings)
Z = np.array(trim_mean)
surf = ax.plot_surface(X, Y, np.transpose(Z), cmap=cm.coolwarm, linewidth=0, antialiased=False)
heave_3D.colorbar(surf, shrink=0.5, aspect=5)
plt.show()

# Heel
fig_heel = plt.figure()
fig_heel.suptitle('Heel, in deg. (wave_ht, wave_heading)')
dfs = DataFrame(data, columns=['sig_wave_ht(m)', 'wave_heading(deg)', 'heel(deg)'])
dfs = dfs.groupby('sig_wave_ht(m)')
nrows = len(wave_heights)
ncols = len(wave_headings)
n = 0
heel_mean = []
for (wave_ht, df) in dfs:
	df = df.groupby('wave_heading(deg)')
	heel_mean_row = []
	for(wave_heading, table) in df:
		n += 1
		heel = table['heel(deg)'].abs()
		heel_mean_row.append(heel.mean())
		heel = heel.values
		ax = fig_heel.add_subplot(nrows, ncols, n)
		ax.title.set_text('({}, {})'.format(wave_ht, wave_heading))
		bp_dict = ax.boxplot(heel, vert=False)
		for line in bp_dict['medians']:
			# get position data for median line
			(x, y) = line.get_xydata()[1] # top of median line
			# overlay median value
			ax.text(x, y, "%.3f" % x, horizontalalignment='center') # draw above, centered
		for line in bp_dict['boxes']:
			(x, y) = line.get_xydata()[0] # bottom of left line
			ax.text(x, y, "%.3f" % x, horizontalalignment='right', verticalalignment='top') # centered, below
			(x, y) = line.get_xydata()[3] # bottom of right line
			ax.text(x, y, "%.3f" % x, horizontalalignment='left', verticalalignment='top') # centered, below
	heel_mean.append(heel_mean_row)
plt.show()
# Surface plot
heel_3D = plt.figure()
heel_3D.suptitle('Heel')
ax = trim_3D.gca(projection='3d')
X, Y = np.meshgrid(wave_heights, wave_headings)
Z = np.array(heel_mean)
surf = ax.plot_surface(X, Y, np.transpose(Z), cmap=cm.coolwarm, linewidth=0, antialiased=False)
heave_3D.colorbar(surf, shrink=0.5, aspect=5)
plt.show()
