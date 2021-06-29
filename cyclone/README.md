# ASVLite-cyclone

## Introduction
Extend ASVLite to simulate tropical storms.

## Build instruction

Install dependency - [netCDF](https://github.com/Unidata/netcdf-c).

```
sudo apt install libnetcdf-dev
```

Build

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
```

Although not a dependency for ASVLite, having the following tools will be useful to explore netCDF (.nc) files.

```
sudo apt install ncview netcdf-bin
```

Useful commands:
`ncdump` - view information and data in the `nc` file.
`ncview` - visualisation of data in the `nc` file. 

