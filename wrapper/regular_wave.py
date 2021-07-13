import ctypes
import dll
import geometry

class _Regular_wave(ctypes.Structure):
    _fields_ = [("amplitude",   ctypes.c_double),
                ("frequency",   ctypes.c_double),
                ("phase_lag",   ctypes.c_double),
                ("direction",   ctypes.c_double),
                ("time_period", ctypes.c_double),
                ("wave_length", ctypes.c_double),
                ("wave_number", ctypes.c_double)]
    
class Regular_wave:
    def __init__(self, amplitude, frequency, phase_lag, direction):
        self.__c_object = _Regular_wave()
        dll.dll.regular_wave_init(ctypes.pointer(self.__c_object), 
                                  ctypes.c_double(amplitude), 
                                  ctypes.c_double(frequency), 
                                  ctypes.c_double(phase_lag), 
                                  ctypes.c_double(direction))

    def get_phase(self, location, time):
        regular_wave_get_phase = dll.dll.regular_wave_get_phase
        regular_wave_get_phase.restype = ctypes.c_double
        result = regular_wave_get_phase(ctypes.pointer(self.__c_object), 
                                        location.c_object, 
                                        ctypes.c_double(time))
        return result

    def get_elevation(self, location, time):
        regular_wave_get_elevation = dll.dll.regular_wave_get_elevation
        regular_wave_get_elevation.restype = ctypes.c_double
        result = dll.dll.regular_wave_get_elevation(ctypes.pointer(self.__c_object), 
                                                    location.c_object, 
                                                    ctypes.c_double(time))
        return result
    
    def get_pressure_amp(self, z):
        regular_wave_get_pressure_amp = dll.dll.regular_wave_get_pressure_amp
        regular_wave_get_pressure_amp.restype = ctypes.c_double
        result = dll.dll.regular_wave_get_pressure_amp(ctypes.pointer(self.__c_object), 
                                                       ctypes.c_double(z))
        return result

    # Getter for c_object
    @property
    def c_object(self):
        return self.__c_object

    # Getter and setter for amplitude    
    @property
    def amplitude(self):
        return self.__c_object.amplitude
    
    @amplitude.setter
    def amplitude(self, value):
        self.__c_object.amplitude = value 
    
    # Getter and setter for frequency   
    @property
    def frequency(self):
        return self.__c_object.frequency
    
    @frequency.setter
    def frequency(self, value):
        self.__c_object.frequency = value 
    
    # Getter and setter for phase_lag   
    @property
    def phase_lag(self):
        return self.__c_object.phase_lag
    
    @phase_lag.setter
    def phase_lag(self, value):
        self.__c_object.phase_lag = value 
    
    # Getter and setter for direction
    @property
    def direction(self):
        return self.__c_object.direction
    
    @direction.setter
    def direction(self, value):
        self.__c_object.direction = value 
    
    # Getter for time_period
    @property
    def time_period(self):
        return self.__c_object.time_period
    
    # Getter for wave_length 
    @property
    def wave_length(self):
        return self.__c_object.wave_length

    # Getter for wave_number
    @property
    def wave_number(self):
        return self.__c_object.wave_number
