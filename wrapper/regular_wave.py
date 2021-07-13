import ctypes
from geometry import *

_dll = ctypes.cdll.LoadLibrary("./lib/libASVLite-python.so")

class C_Regular_wave(ctypes.Structure):
    _fields_ = [("amplitude", ctypes.c_double),
                ("frequency", ctypes.c_double),
                ("phase_lag", ctypes.c_double),
                ("direction", ctypes.c_double),
                ("time_period", ctypes.c_double),
                ("wave_length", ctypes.c_double),
                ("wave_number", ctypes.c_double)]
    
class Regular_wave:
    def __init__(self, amplitude, frequency, phase_lag, direction):
        global _dll
        self.c_regular_wave = C_Regular_wave()
        _dll.regular_wave_init(ctypes.pointer(self.c_regular_wave), 
                               ctypes.c_double(amplitude), 
                               ctypes.c_double(frequency), 
                               ctypes.c_double(phase_lag), 
                               ctypes.c_double(direction))
    
    def get_phase(self, location, time):
        global _dll
        result = _dll.regular_wave_get_phase(ctypes.pointer(self.c_regular_wave), 
                                             location.c_object, 
                                             ctypes.c_double(time))
        return float(result)

    def get_elevation(self, location, time):
        global _dll
        result = _dll.regular_wave_get_elevation(ctypes.pointer(self.c_regular_wave), 
                                                                location.c_object, 
                                                                ctypes.c_double(time))
        return float(result)
    
    def get_pressure_amp(self, z):
        global _dll
        result = _dll.regular_wave_get_pressure_amp(ctypes.pointer(self.c_regular_wave), 
                                                                   ctypes.c_double(z))
        return float(result)

    # Getter and setter for amplitude    
    @property
    def amplitude(self):
        return self.c_regular_wave.amplitude
    
    @amplitude.setter
    def amplitude(self, value):
        self.c_regular_wave.amplitude = value 
    
    # Getter and setter for frequency   
    @property
    def frequency(self):
        return self.c_regular_wave.frequency
    
    @frequency.setter
    def frequency(self, value):
        self.c_regular_wave.frequency = value 
    
    # Getter and setter for phase_lag   
    @property
    def phase_lag(self):
        return self.c_regular_wave.phase_lag
    
    @phase_lag.setter
    def phase_lag(self, value):
        self.c_regular_wave.phase_lag = value 
    
    # Getter and setter for direction
    @property
    def direction(self):
        return self.c_regular_wave.direction
    
    @direction.setter
    def direction(self, value):
        self.c_regular_wave.direction = value 
    
    # Getter and setter for time_period
    @property
    def time_period(self):
        return self.c_regular_wave.time_period
    
    @time_period.setter
    def time_period(self, value):
        self.c_regular_wave.time_period = value 
    
    # Getter and setter for wave_length 
    @property
    def wave_length(self):
        return self.c_regular_wave.wave_length
    
    @wave_length.setter
    def wave_length(self, value):
        self.c_regular_wave.wave_length = value 

    # Getter and setter for wave_number
    @property
    def wave_number(self):
        return self.c_regular_wave.wave_number
    
    @wave_number.setter
    def wave_number(self, value):
        self.c_regular_wave.wave_number = value 
