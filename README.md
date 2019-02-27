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
- *CGAL 4.13* for computational geometry.  
``` sudo apt install libcgal-dev ```
- *Boost.Units 1.68.0* for units and dimensions.  
``` sudo apt install libboost-dev ```   
- *Google Test 1.8.1* for unit testing.  
``` sudo apt install libgtest-dev ```
- *VTK 7.1* for visualisation.   
``` sudo apt install libvtk7-dev libvtk-qt-dev ```

## Build instruction
``` 
cd ~
git clone https://github.com/resilient-swarms/asv-swarm.git
cd asv-swarm
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make 

# If build successfully then run application:
./asv_swarm
```
 
