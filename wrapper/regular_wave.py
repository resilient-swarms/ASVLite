import ctypes
import dll
import geometry
   
class Regular_wave(ctypes.Structure):
    _fields_ = [("amplitude",   ctypes.c_double),
                ("frequency",   ctypes.c_double),
                ("phase_lag",   ctypes.c_double),
                ("direction",   ctypes.c_double),
                ("time_period", ctypes.c_double),
                ("wave_length", ctypes.c_double),
                ("wave_number", ctypes.c_double)]

    def __init__(self, amplitude, frequency, phase_lag, direction):
        dll.dll.regular_wave_init(ctypes.pointer(self), 
                                  ctypes.c_double(amplitude), 
                                  ctypes.c_double(frequency), 
                                  ctypes.c_double(phase_lag), 
                                  ctypes.c_double(direction))

    def get_phase(self, location, time):
        regular_wave_get_phase = dll.dll.regular_wave_get_phase
        regular_wave_get_phase.restype = ctypes.c_double
        result = regular_wave_get_phase(ctypes.pointer(self), 
                                        ctypes.pointer(location), 
                                        ctypes.c_double(time))
        return result

    def get_elevation(self, location, time):
        regular_wave_get_elevation = dll.dll.regular_wave_get_elevation
        regular_wave_get_elevation.restype = ctypes.c_double
        result = dll.dll.regular_wave_get_elevation(ctypes.pointer(self), 
                                                    ctypes.pointer(location), 
                                                    ctypes.c_double(time))
        return result
    
    def get_pressure_amp(self, z):
        regular_wave_get_pressure_amp = dll.dll.regular_wave_get_pressure_amp
        regular_wave_get_pressure_amp.restype = ctypes.c_double
        result = dll.dll.regular_wave_get_pressure_amp(ctypes.pointer(self), 
                                                       ctypes.c_double(z))
        return result