# ASVLite-controllers

## Introduction
Provides an implementation for ASV controllers and extends ASVLite to simulate a tropical storm and wave glider dynamics in the storm.

## Dependency

Install dependency - [netCDF](https://github.com/Unidata/netcdf-c).

```
sudo apt install python3-netcdf4 python3-cartopy
```

Although not a dependency for ASVLite, having the following tools will be useful to explore netCDF (.nc) files.

```
sudo apt install ncview netcdf-bin
```

Useful commands:
`ncdump` - view information and data in the `nc` file.
`ncview` - visualisation of data in the `nc` file. 

## Simulation data
To simulate the storm, `wave_data.py` requires data downloaded from [Climate Data Store](https://cds.climate.copernicus.eu/cdsapp#!/dataset/reanalysis-era5-single-levels?tab=overview). The data should be downloaded as a netCDF file and should contain the variables - `Significant height of combined wind waves and swell` and `Mean wave direction`.