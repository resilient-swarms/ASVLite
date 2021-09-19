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
                ("longitudes", ctypes.POINTER(float)),
                ("latitudes",  ctypes.POINTER(float)),
                ("time_steps",  ctypes.POINTER(float)),
                ("map",  ctypes.POINTER(int)),
                ("data",  ctypes.POINTER(float))]

class Cyclone(ctypes.Structure):
    _fields_ = [("hs", ctypes.POINTER(Data)),
                ("dp", ctypes.POINTER(Data)),
                ("count_sets", ctypes.c_int)]
    
    def init(self, path_to_hs_nc_files, path_to_dp_nc_files, count_sets):
        dll.dll.cyclone_init(ctypes.pointer(self), 
                            ctypes.pointer(ctypes.c_char_p(path_to_hs_nc_files)),
                            ctypes.pointer(ctypes.c_char_p(path_to_dp_nc_files)),
                            ctypes.c_int(count_sets))
    
    def __del__(self):
        dll.dll.cyclone_clean(ctypes.pointer(self))
    
    def print_data(self):
        dll.dll.cyclone_print_data(ctypes.pointer(self))

    def get_wave_height(self, location, time):
        cyclone_get_wave_height = dll.dll.cyclone_get_wave_height
        cyclone_get_wave_height.restype = ctypes.c_float
        result = cyclone_get_wave_height(ctypes.pointer(self), Location(location), Time(time))
        return result
    
    def cyclone_get_wave_heading(self, location, time):
        cyclone_get_wave_heading = dll.dll.cyclone_get_wave_heading
        cyclone_get_wave_heading.restype = ctypes.c_float
        result = cyclone_get_wave_heading(ctypes.pointer(self), Location(location), Time(time))
        return result
    
