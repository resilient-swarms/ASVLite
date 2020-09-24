# asv-swarm visualisation

## Introduction
A simple visualisation module for asv-swarm.

## Prerequisites for compiling asv-swarm
*VTK 9.0* for visualisation.   
``` sudo apt install libvtk9-dev libvtk-qt-dev ```

## Build instruction
```
mkdir build
cd build
```

If VTK is installed on your system then:
```
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
```

Or, if VTK is not installed but compiled on your system, you will need to specify the path to your VTK build:
```
cmake -DVTK_DIR:PATH=/path/to/vtk/build -DCMAKE_BUILD_TYPE=Release ..
make 
```
