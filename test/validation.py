# Data processing
import numpy as np 
import pandas as pd 
from pandas import DataFrame
# Plot 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# Run command line
import subprocess
import os
# Measure time for simulations
import time
# Intrrupt handling
import signal
import sys

count_simulations = 0
subprocesses = []

def interrupt_handler(sig, frame):
	print('You pressed Ctrl+C!')
	for ps in subprocesses:
		pid = ps.pid
		os.kill(pid, signal.SIGINT)
	sys.exit(0)

signal.signal(signal.SIGINT, interrupt_handler)

# Motion is still water
# ---------------------
# Run simulation
count_simulations += 1
print("count simulations = {}".format(count_simulations))
subprocess.run(["../build/asv_simulator", "asv", "asv_still_water", "0.0", "0.0", "1"])

# Read simulation data from file into dataframe
data = pd.read_csv('asv_still_water', sep=" ")
print(data.head())
print(data.info())

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
wave_headings = range(0, 360, 30)
rand_seeds = range(0, 10)
for wave_height in wave_heights:
	for wave_heading in wave_headings:
		for rand_seed in rand_seeds:
			count_simulations += 1
			out_file = "asv_{}_{}".format(wave_height, wave_heading)
			print("count simulations = {} [wave_ht={}, wave_heading={}, rand_seed={}]".format(count_simulations, wave_height, wave_heading, rand_seed))
			ps = subprocess.Popen(["../build/asv_simulator", "asv", out_file, str(wave_height), str(wave_heading), str(rand_seed)])
			subprocesses.append(ps)
exit_codes = [p.wait() for p in subprocesses]
print("Simulations completed.")

# Merge all output files to a single file
print("Merge files:")
file_count = 0
for wave_height in wave_heights:
	for wave_heading in wave_headings:
		out_file = "asv_{}_{}".format(wave_height, wave_heading)
		file1 = open("asv_out_wave", "a")
		file2 = open(out_file, "r")
		print("... merging file {}".format(out_file))
		line_count = 0
		for line in file2:
			if(file_count != 0 and line_count == 0):
				# ignore the header 
				continue
			else:
				file1.write(line)
				line_count += 1
		file_count += 1
		print("... remove file {}".format(out_file))
		file1.close()
		os.remove(out_file)

# Clock
end_time = time.time()
print("Simulation time = {}".format(end_time - start_time))

# Read simulation data from file into dataframe
data = pd.read_csv('asv_out_wave', sep=" ")
print(data.head())
print(data.info())

# Plot
fig_wave = plt.figure()
# Relative heave
df = DataFrame(data, columns=['sig_wave_ht(m)', 'wave_heading(deg)', 'rand_seed', 'wave_elevation(m)', 'cog_z(m)'])
df['rel_heave(m)'] = df['cog_z(m)'] - df['wave_elevation(m)']
df['rel_heave(m)'] = df['rel_heave(m)'].abs()
ax1 = fig_wave.add_subplot(131, projection='3d')
ax1.title.set_text('Heave (relative to sea surface)')
ax1.set_xlabel('sig wave ht(m)')
ax1.set_ylabel('wave heading(deg)')
ax1.set_zlabel('heave(m)')
x = df['sig_wave_ht(m)']
y = df['wave_heading(deg)']
z = df['rel_heave(m)']
ax1.scatter(x,y,z)
# Pitch
df = DataFrame(data, columns=['sig_wave_ht(m)', 'wave_heading(deg)', 'rand_seed', 'trim(deg)'])
ax2 = fig_wave.add_subplot(132, projection='3d')
ax2.title.set_text('Trim')
ax2.set_xlabel('sig wave ht(m)')
ax2.set_ylabel('wave heading(deg)')
ax2.set_zlabel('trim(deg)')
x = df['sig_wave_ht(m)']
y = df['wave_heading(deg)']
z = df['trim(deg)']
ax2.scatter(x,y,z)
# Heel
df = DataFrame(data, columns=['sig_wave_ht(m)', 'wave_heading(deg)', 'rand_seed', 'heel(deg)'])
ax3 = fig_wave.add_subplot(133, projection='3d')
ax3.title.set_text('Heel')
ax3.set_xlabel('sig wave ht(m)')
ax3.set_ylabel('wave heading(deg)')
ax3.set_zlabel('heel(deg)')
x = df['sig_wave_ht(m)']
y = df['wave_heading(deg)']
z = df['heel(deg)']
ax3.scatter(x,y,z)
plt.show()
