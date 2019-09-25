# asv-swarm

## Introduction
A simulator for swarm of marine robots.

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
The simulator required three values to be passed as command line arguments:
1. The input (toml) file. 
2. Significant wave height of the sea state to simulate. 
3. The direction of the wave. 

Shown below is an example for a sea state of significant wave height of 1.2m and wave heading of 20deg north. 

```
./asv_simulator input_file 1.2 20.0
```

