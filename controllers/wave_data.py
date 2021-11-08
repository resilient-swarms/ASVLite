import netCDF4 as nc 
import numpy as np
from datetime import datetime, timedelta

class Wave_data:
    def __init__(self, nc_file_path):
        self.file = nc.Dataset(nc_file_path)
        self.latitudes = self.file.variables["latitude"][:].data # numpy array of dtype=float32
        self.longitudes = self.file.variables["longitude"][:].data # numpy array of dtype=float32
        self.times = self.file.variables["time"][:].data # numpy array of dtype=float32
        self.hs = self.file.variables["swh"][:].data # numpy array of dtype=float32 of shape (len_time, len_latitude, len_longitude)
        self.dp = self.file.variables["mwd"][:].data # numpy array of dtype=float32 of shape (len_time, len_latitude, len_longitude)

    def get_wave_data_at(self, latitude, longitude, time):
        epoch_time = datetime(1900, 1, 1)  
        time_diff_from_epoch = time - epoch_time # time difference w.r.t epoch time
        time_diff_from_epoch_in_sec = time_diff_from_epoch.total_seconds()
        time_diff_from_epoch_in_hrs = divmod(time_diff_from_epoch_in_sec, 3600)[0]
        # Check if latitude, longitude and time are within range of the data in the netcdf file.
        if not ((self.latitudes[0] <= latitude <= self.latitudes[-1]) or (self.latitudes[0] >= latitude >= self.latitudes[-1])):
            raise ValueError("Latitude {} is out of range [{}, {}]".format(latitude, self.latitudes[0], self.latitudes[-1]))
        if not ((self.longitudes[0] <= longitude <= self.longitudes[-1]) or (self.longitudes[0] >= longitude >= self.longitudes[-1])):
            raise ValueError("Longitude {} is out of range [{}, {}]".format(longitude, self.longitudes[0], self.longitudes[-1]))
        if not ((self.times[0] <= time_diff_from_epoch_in_hrs <= self.times[-1]) or (self.times[0] >= time_diff_from_epoch_in_hrs >= self.times[-1])):
            time_0 = epoch_time + timedelta(hours=int(self.times[0]))
            time_n = epoch_time + timedelta(hours=int(self.times[-1]))
            raise ValueError("Time {} is out of range [{}, {}]".format(time, time_0, time_n))
        # Find the index of the data
        delta_time = self.times[2] - self.times[1]
        delta_latitude = self.latitudes[2] - self.latitudes[1]
        delta_longitude = self.longitudes[2] - self.longitudes[1] 
        # Index time
        index_time = int((time_diff_from_epoch_in_hrs - self.times[0])//delta_time)
        index_latitude = int((latitude - self.latitudes[0])//delta_latitude)
        index_longitude = int((longitude - self.longitudes[0])//delta_longitude)
        hs = self.hs[index_time][index_latitude][index_longitude]
        dp = self.dp[index_time][index_latitude][index_longitude]
        return (hs,dp)

        


# temp = Wavedata("./sample_files/katrina.nc")
# print(temp.get_wave_data_at(24.4, -81.2, datetime(2005, 8, 27, 7)))



# sudo apt install python3-netcdf4

# from https://cds.climate.copernicus.eu/cdsapp#!/dataset/reanalysis-era5-single-levels?tab=form
# download dataset - (1) Significant height of combined wind waves and swell (2) Mean wave direction
