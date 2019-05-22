# asv-swarm

## Introduction
A simulator for swarm of marine robots.

## Documentation 
- Reference to hydrodynamics theory available 
  [here](reference/build/hydrodynamics_reference.pdf).  
- Doxygen generated source code documentation available 
  [here](documentation/html/index.html)

## Prerequisites for compiling asv-swarm
- *CMake* for managing build process.  
``` sudo apt install cmake-qt-gui ```
- libxml2
```sudo apt install libxml2-dev```
- *VTK 7.1* for visualisation.   
``` sudo apt install libvtk7-dev libvtk-qt-dev ```

## Build instruction
``` 
cd ~
git clone --recurse-submodules https://github.com/resilient-swarms/asv-swarm.git
cd asv-swarm
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make 

# If build successfully then run application:
./asv_swarm
```

