import ctypes
import dll
import constants
import geometry
import regular_wave
      
class Wave(ctypes.Structure):
    _fields_ = [("significant_wave_height",     ctypes.c_double),
                ("heading",                     ctypes.c_double),
                ("random_number_seed",          ctypes.c_long),
                ("spectrum",                    regular_wave.Regular_wave * constants.COUNT_WAVE_SPECTRAL_DIRECTIONS * constants.COUNT_WAVE_SPECTRAL_FREQUENCIES ) ,
                ("min_spectral_frequency",      ctypes.c_double),
                ("max_spectral_frequency",      ctypes.c_double),
                ("peak_spectral_frequency",     ctypes.c_double),
                ("min_spectral_wave_heading",   ctypes.c_double),
                ("max_spectral_wave_heading",   ctypes.c_double)]

    def __init__(self, sig_wave_height, wave_heading, rand_seed):
        dll.dll.wave_init(ctypes.pointer(self), 
                          ctypes.c_double(sig_wave_height), 
                          ctypes.c_double(wave_heading), 
                          ctypes.c_double(rand_seed))

    def get_elevation(self, location, time):
        wave_get_elevation = dll.dll.wave_get_elevation
        wave_get_elevation.restype = ctypes.c_double
        result = wave_get_elevation(ctypes.pointer(self), 
                                    ctypes.pointer(location), 
                                    ctypes.c_double(time))
        return result