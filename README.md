# ASV-Swarm

## Introduction
ASV-Swarm is a simulator that provides high fidelity and computationally
efficient model of ocean waves and dynamics of marine surface vehicles in waves.
The simulator is ideal for applications requiring high run-time performance,
such as with simulation of a swarm of autonomous marine vehicles, or in
developing optimal vehicle control strategies using reinforcement learning
techniques. ASV-Swarm also has a low computational overhead making it ideal for
onboard simulation for applications such as online learning for adaptation to
changes in the environment. 

## Build instruction
``` 
cd ~
git clone --recurse-submodules https://github.com/resilient-swarms/asv-swarm.git
cd asv-swarm
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
```

## Running the simulator
The simulator requires the following values as command-line arguments:
1. input file, 
2. output file,
3. significant wave height of the sea state to simulate, 
4. predominant wave direction of the wave,
5. random seed number.

Example for simulating vehicle dynamics in a sea with a significant wave height of 1.2 m, wave heading of 20<sup>o</sup>North and with a random number seed of 3:
```
asv_simulator input_file out_file 1.2 20.0 3
```

The `input_file` uses `toml` syntax and should provide the inputs required for the simulation. Given below is an example of an input file. 
```
# Physical specification of the vehicle. 
[spec]
L_wl = 0.3
B_wl = 0.3
D = 0.3
T = 0.1
displacement = 0.007
max_speed = 2.0

# Center of gravity of the vehicle.
[cog]
x = 0.15
y = 0.00
z = -0.2

# Radius of gyration of the vehicle.
[radius_of_gyration]
roll = 0.08
pitch = 0.08
yaw = 0.106

# Set the position of propellers. 
# Here we set 4 propellers for the vehicle.
# Propeller - 1 
[[propeller]]
x = 0.065
y = -0.085
z = -0.0485
# Propeller - 2
[[propeller]]
x = 0.235
y = -0.085
z = -0.0485
# Propeller - 3
[[propeller]]
x = 0.235
y = 0.085
z = -0.0485
# Propeller - 4
[[propeller]]
x = 0.065
y = 0.085
z = -0.0485

# Initial position of the vehicle.
[vehicle_position]
x = 100.0
y = 100.0

# Initial floating attitude of the vehicle. 
[vehicle_attitude]
heel = 0
trim = 0
heading = 0

# Define the waypoints for navigation. 
[[waypoint]]
x = 100.0
y = 150.0


# Following set of inputs are optional
# Set the time step size for simulation.
[clock]
time_step_size = 40
```

## Using ASV-Swarm as a software library. 

Refer the Doxygen generated documentation in directory `doc/html/index.html` for details on the 
programming interface provided by ASV-Swarm.
