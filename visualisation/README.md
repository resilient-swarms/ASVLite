# ASVLite-visualisation

## Introduction
A simple visualisation module for ASVLite.

## Prerequisites for compiling ASVLite-visualisation
Install VTK-9 using the package manager. 

``` sudo apt install libvtk9-dev libvtk-qt-dev ```

If VTK-9 is not provided by the package manager then follow the below steps to compile the VTK-9 available with this repository.

```
cd ../dependency/VTK-9.1.0
mkdir build
cd build
cmake ../
```

## Build instruction
```
mkdir build
cd build
```

If using the VTK-9 installed by package manager:
```
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
```

Or, if using the VTK-9 that is available with this repository:
```
cmake -DVTK_DIR:PATH=../dependency/VTK-9.1.0/build -DCMAKE_BUILD_TYPE=Release ..
make 
```