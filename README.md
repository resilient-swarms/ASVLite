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
The simulator required the following values to be passed as command line arguments:
1. input file, 
2. output file,
3. significant wave height of the sea state to simulate, 
4. predominant wave direction of the wave,
5. random seed number.

Shown below is an example for a sea state of significant wave height of 1.2m and wave heading of 20deg north. 

```
./asv_simulator input_file out_file 1.2 20.0 3
```

The file `example_input.toml` shows the format of the input file. 

## Using ASV-Swarm as a software library. 

Refer the Doxygen generated documentation in directory `doc` for details on the 
programming interface provided by ASV-Swarm.
