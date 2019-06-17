#!/bin/bash

# Script to stop any ongoing ASV node mission

# TODO: for each shell/python file that prints into the screen, there should be an prefix for each message,
# so it can be easily identified during debugging / operation.
# OP: define console output prefix that should appear in all console messages
# Usage: echo $output_prefix "Put your message here"
output_prefix="[stop_mission] "

# read yaml file from configuration folder
config_dir="/home/pi/configuration"
# specify path where output data will be stored
# modify to your own local structure when running in a PC, instead in the RPi 
output_base_dir=/home/pi/data

# flag for forced stop procedure
force_flag=0

#find the newest folder, assumed to be the active one
output_dir=`ls -td ${output_base_dir}/* | head -1`

echo 
echo $output_prefix "End sequence for configuration: $config_dir"  
echo $output_prefix "Latest mission directory found: $output_dir"  
echo 

# call asv_main stop function, when trying clean shutdown
python ~/git/oplab_lcm/vector_node_asv/src/asv_main.py stop &

sleep 5s

# force lcm-logger kill
pkill lcm-logger
result=$?

if [ $result -eq 1 ]; then
	echo $output_prefix "lcm-logger not running."
elif [ $result -eq 0 ]; then
	echo $output_prefix "Stopped lcm-logger."
else
	echo $output_prefix "Unknown output of pkill ($result)"
fi

# WARNING: force kill upon all python instances
# Use process ID (PID) retrieved when launching asv_main.py from start_mission.sh

# SIGTERM in a polite way... using pkill
pkill python

# Retrieve Python PID with pgrep
pid_python=$(pgrep python)
kill -9 $pid_python

if [ $result -eq 1 ]; then
	echo $output_prefix "Mission ended normally."
elif [ $result -eq 0 ]; then	
	
	echo $output_prefix "Abnormal standard end sequence..."
	echo $output_prefix "Will attempt force end."
	force_flag=1
else
	echo $output_prefix "Unknown output of pkill ($result)"
fi

if [ $force_flag -eq 1 ]; then
	sleep 5s

	python ~/git/oplab_lcm/vector_node_asv/src/lib_safety/asv_force_end.py -c ${config_dir} -o ${output_dir} &

	sleep 10s

	# TODO: we should figure out a cleaner way to kill the specific Python process, not all existing instances.
	# Perhaps by storing PID information when launching each Python instance
	pkill python

	if [ $result -eq 1 ]; then
		echo $output_prefix "Mission force end successful."
	elif [ $result -eq 0 ]; then		
	
		echo $output_prefix "Killed Python."

	else
		echo $output_prefix "Unknown output of pkill ($result)"
	fi
fi
