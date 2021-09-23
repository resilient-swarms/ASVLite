import ctypes
import dll

class Time(ctypes.Structure):
    _fields_ = [("year",  ctypes.c_int),
                ("month", ctypes.c_int),
                ("day",   ctypes.c_int),
                ("hour",  ctypes.c_int),]
    
class Location(ctypes.Structure):
    _fields_ = [("latitude",  ctypes.c_float),
                ("longitude", ctypes.c_float)]

class Data(ctypes.Structure):
    _fields_ = [("count_longitudes", ctypes.c_int),
                ("count_latitudes",  ctypes.c_int),
                ("count_time_steps", ctypes.c_int),
                ("longitudes", ctypes.POINTER(ctypes.c_float)),
                ("latitudes",  ctypes.POINTER(ctypes.c_float)),
                ("time_steps", ctypes.POINTER(ctypes.c_float)),
                ("map",  ctypes.POINTER(ctypes.c_int)),
                ("data",  ctypes.POINTER(ctypes.c_float))]

class Cyclone(ctypes.Structure):
    _fields_ = [("hs", ctypes.POINTER(Data)),
                ("dp", ctypes.POINTER(Data)),
                ("count_sets", ctypes.c_int)]
    
    def init(self, path_to_hs_nc_files, path_to_dp_nc_files):
        cyclone_init = dll.dll.cyclone_init
        cyclone_init.argtypes = [ctypes.POINTER(Cyclone), ctypes.c_char_p * len(path_to_hs_nc_files), ctypes.c_char_p * len(path_to_dp_nc_files), ctypes.c_int]
        path_to_hs_nc_files = [ctypes.c_char_p(file.encode('utf-8')) for file in path_to_hs_nc_files]
        path_to_dp_nc_files = [ctypes.c_char_p(file.encode('utf-8')) for file in path_to_dp_nc_files]
        path_to_hs_nc_files = (ctypes.c_char_p * len(path_to_hs_nc_files))(*path_to_hs_nc_files)
        path_to_dp_nc_files = (ctypes.c_char_p * len(path_to_dp_nc_files))(*path_to_dp_nc_files)
        cyclone_init(self, path_to_hs_nc_files, path_to_dp_nc_files, len(path_to_dp_nc_files))
    
    def __del__(self):
        dll.dll.cyclone_clean(ctypes.pointer(self))
    
    def print_data(self):
        dll.dll.cyclone_print_data(ctypes.pointer(self))

    def get_wave_height_using_time(self, location, time):
        cyclone_get_wave_height_using_time = dll.dll.cyclone_get_wave_height_using_time
        cyclone_get_wave_height_using_time.argtypes = [ctypes.POINTER(Cyclone), Location, Time]
        cyclone_get_wave_height_using_time.restype = ctypes.c_float
        result = cyclone_get_wave_height_using_time(self, location, time)
        return result

    def get_wave_height_using_days(self, location, time):
        cyclone_get_wave_height_using_days = dll.dll.cyclone_get_wave_height_using_days
        cyclone_get_wave_height_using_days.argtypes = [ctypes.POINTER(Cyclone), Location, ctypes.c_float]
        cyclone_get_wave_height_using_days.restype = ctypes.c_float
        result = cyclone_get_wave_height_using_days(self, location, time)
        return result
    
    def get_wave_heading_using_time(self, location, time):
        cyclone_get_wave_heading_using_time = dll.dll.cyclone_get_wave_heading_using_time
        cyclone_get_wave_heading_using_time.argtypes = [ctypes.POINTER(Cyclone), Location, Time]
        cyclone_get_wave_heading_using_time.restype = ctypes.c_float
        result = cyclone_get_wave_heading_using_time(self, location, time)
        return result
    
    def get_wave_heading_using_days(self, location, time):
        cyclone_get_wave_heading_using_days = dll.dll.cyclone_get_wave_heading_using_days
        cyclone_get_wave_heading_using_days.argtypes = [ctypes.POINTER(Cyclone), Location, ctypes.c_float]
        cyclone_get_wave_heading_using_days.restype = ctypes.c_float
        result = cyclone_get_wave_heading_using_days(self, location, time)
        return result
    


# cyclone = Cyclone()
# hs_file = ["../cyclone/sample_files/hs1.nc", "../cyclone/sample_files/hs2.nc", "../cyclone/sample_files/hs3.nc"]
# dp_file = ["../cyclone/sample_files/dp1.nc", "../cyclone/sample_files/dp2.nc", "../cyclone/sample_files/dp3.nc"]
# cyclone.init(hs_file, dp_file)
# cyclone.print_data()
#https://stackoverflow.com/questions/27127413/converting-python-string-object-to-c-char-using-ctypes
#https://stackoverflow.com/questions/4145775/how-do-i-convert-a-python-list-into-a-c-array-by-using-ctypes