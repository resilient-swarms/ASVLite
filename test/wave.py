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

# Simulator App
# -------------
app_dir = str(Path.home()) + "/build"
simulator_app = app_dir + "/asv_simulator"

# Create the subdirectory for output files
#out_dir = os.getcwd()
out_dir = str(Path.home()) + "/asv_out"
os.mkdir(out_dir)

# Simulations
# -----------
wave_heights = np.arange(0.5, 2.1, 0.5)
wave_headings = np.arange(0.0, 361.0, 22.5)
trials = np.arange(1, 101, 1)
count_wave_hts = wave_heights.shape[0]
count_wave_headings = wave_headings.shape[0]
count_trials = trials.shape[0]
count_simulations = 0
subprocesses = []
# Table to hold arry of motion amplitude for each wave hts and wave heading
heave = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'amplitude'])
pitch = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'amplitude'])
roll  = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'amplitude'])
sig_heave = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'sig_amplitude'])
sig_pitch = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'sig_amplitude'])
sig_roll  = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'sig_amplitude'])

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
            file_name = out_dir + "/{}_{}_{}".format(trial, 
                                                     wave_height, 
                                                     wave_heading)
            os.remove(file_name)
            print("... removed file {}".format(file_name))
    sys.exit(0)

# Register the interrupt handler. 
signal.signal(signal.SIGINT, interrupt_handler)

# Read simulation data from file into dataframe
# ---------------------------------------------
def read_data_from_file(file_name):
    data = pd.read_csv(file_name, sep=" ")
    dfs = DataFrame(data, columns=[ 'sig_wave_ht(m)', 
                                    'wave_heading(deg)', 
                                    'rand_seed',
                                    'wave_elevation(m)',
                                    'cog_z(m)', 
                                    'trim(deg)',
                                    'heel(deg)' ])
    return dfs

# Function to get array of amplitudes
# -----------------------------------
# This function creates a list motion amplitudes from the given list of 
# instantaneous values. 
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
def append_data(col_heading):
    table = pd.DataFrame(columns=['wave_ht', 'wave_heading', 'amplitude'])
    for i in range(count_trials):
        for j in range(count_wave_hts):
            for k in range(count_wave_headings):
                print("Append data for {} {} {}".format(trials[i], 
							wave_heights[j], 
							wave_headings[k]))
                file = out_dir + "/{}_{}_{}".format(trials[i], 
                                                    wave_heights[j], 
                                                    wave_headings[k])
                dfs = read_data_from_file(file)
                data = get_abs_amplitudes(dfs[col_heading])
                for n in range(len(data)):
                    table = table.append({'wave_ht':wave_heights[j], 
                                          'wave_heading': wave_headings[k], 
                                          'amplitude':data[n]}, 
                                         ignore_index=True)
    return(table)

# Function to compute the significant motion amptitudes
def set_significant_amplitudes(table):
    sig_table = pd.DataFrame(columns=['wave_ht','wave_heading','sig_amplitude'])
    for i in range(count_wave_hts):
        for j in range(count_wave_headings):
            (count_rows, count_cols) = table.shape
            top_third = round(count_rows/3)
            table.sort_values(by=['amplitude'], ascending=False, inplace=True)
            for n in range(top_third):
                amp = table.iloc[n]['amplitude']
                sig_table = sig_table.append({'wave_ht':wave_heights[i], 
                                              'wave_heading': wave_headings[j], 
                                              'sig_amplitude':amp}, 
                                             ignore_index=True)
    return(sig_table)

# Plot
# ----
def box_plot(data, title, column):
    fig, axs = plt.subplots(count_wave_hts)
    data = data.groupby('wave_ht')
    i = 0
    for (ht, df) in data:
        print("Making box_plot for wave height {}".format(ht))
        df.boxplot(column=[column], by=['wave_heading'], ax=axs[i])
        i += 1
    plt.savefig(out_dir+"/boxplot_{}.png".format(title), 
                bbox_inches='tight')

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
            out_file = out_dir + "/{}_{}_{}".format(trial, 
                                                    wave_height, 
                                                    wave_heading)
            print("count simulations:{}[trial={}, wave_ht={}, wave_heading={}]".
                  format(count_simulations, trial, wave_height, wave_heading))
            ps = subprocess.Popen([simulator_app, 
                                   "asv", 
                                   out_file, 
                                   str(wave_height), 
                                   str(wave_heading), 
                                   str(trial)])
            subprocesses.append(ps)
    exit_codes = [p.wait() for p in subprocesses]
print("Simulations completed.")
end_time = time.time()
sim_time_sec = end_time - start_time
sim_time_min = (sim_time_sec/60.0)
print("\nSimulation time = {0:0.2f}sec ({0:0.2f}min)\n".
      format(sim_time_sec, sim_time_min))
# merge_files()
# Read each file and get motion amplitudes
print("\nAppend data for heave")
heave = append_data('cog_z(m)')
print("\nAppend data for pitch")
pitch = append_data('trim(deg)')
print("\nAppend data for roll")
roll = append_data('heel(deg)')
# Se the significant motion amplitudes
#sig_heave = set_significant_amplitudes(heave)
#sig_pitch = set_significant_amplitudes(pitch)
#sig_roll = set_significant_amplitudes(roll)

box_plot(heave, 'Heave(m)', 'amplitude')
#box_plot(sig_heave, 'Significant Heave(m)', 'sig_amplitude')
#contour_plot(sig_heave, 'Heave(m)')
#scatter_plot(sig_heave, 'Heave(m)')

box_plot(pitch, 'Pitch(deg)', 'amplitude')
#box_plot(sig_pitch, 'Significant Pitch(deg)', 'sig_amplitude')
#contour_plot(sig_pitch, 'Pitch(deg)')
#scatter_plot(sig_pitch, 'Pitch(deg)')

box_plot(roll, 'Roll(deg)', 'amplitude')
#box_plot(sig_roll, 'Significant Roll(deg)', 'sig_amplitude')
#contour_plot(sig_roll, 'Roll(deg)')
#scatter_plot(sig_roll, 'Roll(deg)')
