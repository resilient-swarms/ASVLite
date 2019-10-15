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
