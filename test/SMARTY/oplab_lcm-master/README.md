# oplab_lcm

This repository contains the [lcm](https://lcm-proj.github.io/) based implementation of the acquisition and control for the ASV Smarty platform and the Raspberry Pi 3 code required to interact with the SenseHat add-on. GPS-like position information from the Qualisys system is employed. For a general overview of the system dependencies please see the [dependencies map](DEPENDENCIES.md).

This platform setup is being employed for the lab demonstration in the **SESS6072 Maritime Robotics** course.

## Tasks
- [ ] Improve WiFi network configuration during booting process in the RPi. Former dhcp based implementation was faster but unreliable
- [ ] Expand LCM interconection map to include channels structure.
- [ ] Need to make better use of LCM interprocess calls.
- [x] Implement a motion and DR model
- [ ] Sortout abnormal shutdown > store **pid** during launch of **asv_main** to avoid shutting down other running instances of Python in the RPi

# How to use this code and test lcm
*Note: This section assumes you have successfully installed lcm in your system. If not, please refer to the [lcm building](https://lcm-proj.github.io/build_instructions.html) instructions.*

To install this repository, go to the directory you want it to be installed in and in a terminal/command prompt, type

    git clone --recursive https://github.com/ocean-perception/oplab_lcm.git
    
To push updates, stage changes, commit and push to a branch (usually master):

    git add -A
    git commit -m "Some message about the change"
    git push origin master

## Testing lcm

To test if lcm is working

	cd oplab_lcm/example

and generate the Python scripts by running

	lcm-gen -p example_t.lcm

which creates

	exlcm/example_t.py
	exlcm/__init__.py
	
In one terminal run *listener.py*

	python listener.py

In another terminal run *send_message.py*

	python send_message.py

The message should appear in the *listener.py* terminal

	VECTOR_NODE_ASV

The content of this folder should be put in the home directory

## Configuring and testing the vehicle

Configure your vehicle setup and mission name in: 

	configuration/mission.yaml  # mission name, type of action/control
	             /vehicle.yaml  # platform name, thruster & sensor config.

To start a mission, type 

	./start_mission.sh

To end a mission, type

	./stop_mission.sh

Data will be logged in 

	~/data/$mission_dir*

The *host_qualisys_client_host_rpi_client.py* contains an implementation of a dual client that connects to both the Qualisys host and the Raspberry Pi host. 

## Developing

The following libraries should be modified/employed only if you are developing:

	check/check_acquisition.sh # checks for data in $mission_dir 
	     /infinite_watchman.sh # auto software restart if camera stops 
	     /watchman_restarts.txt # if software reboots, logs the number

	parse/parse_yaml.sh #decodes yaml format for .bash
