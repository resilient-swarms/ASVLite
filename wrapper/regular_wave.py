import ctypes
from unittest import result
import dll
from geometry import Coordinates_3D
   
class Regular_wave(ctypes.Structure):
    '''
    Class to define a regular wave. 
    '''
    pass

    def __init__(self, amplitude, frequency, phase_lag, direction):
        '''
        Create and initialise a regular wave.
        :param float amplitude: Wave amplitude in meter.
        :param float frequency: Wave frequency in Hz.
        :param float phase_lag: Wave phase in radians. 
        :param direction: Northing of the wave in radians. 
        '''
        regular_wave_new = dll.dll.regular_wave_new
        regular_wave_new.restype = ctypes.POINTER(Regular_wave)
        result = regular_wave_new(ctypes.c_double(amplitude), 
                                  ctypes.c_double(frequency), 
                                  ctypes.c_double(phase_lag), 
                                  ctypes.c_double(direction))
        self.__c_base_object = result 
    
    def __del__(self):
        '''
        Free memory allocated for the regular wave. 
        '''
        regular_wave_delete = dll.dll.regular_wave_delete
        regular_wave_delete.restype = None 
        regular_wave_delete(self.__c_base_object)
    
    def __get_error_msg__(self):
        '''
        Returns error message related to the last function called for the instance of Regular_wave.
        '''
        regular_wave_get_error_msg = dll.dll.regular_wave_get_error_msg
        regular_wave_get_error_msg.restype = ctypes.c_char_p
        result = regular_wave_get_error_msg(self.__c_base_object)
        return result

    def __check_error_throw_exception(self):
        error_msg = self.__get_error_msg__()
        if error_msg != None:
            raise ValueError(error_msg.decode("utf-8") )
  
    def get_amplitude(self):
        '''
        Get wave amplitude in meter.
        '''
        regular_wave_get_amplitude = dll.dll.regular_wave_get_amplitude
        regular_wave_get_amplitude.restype = ctypes.c_double
        result = regular_wave_get_amplitude(self.__c_base_object)
        self.__check_error_throw_exception()
        return result

    def get_frequency(self):
        '''
        Get wave wave frequency in Hz. 
        '''
        regular_wave_get_frequency = dll.dll.regular_wave_get_frequency
        regular_wave_get_frequency.restype = ctypes.c_double
        result = regular_wave_get_frequency(self.__c_base_object)
        self.__check_error_throw_exception()
        return result

    def get_direction(self):
        '''
        Get wave heading in radians. 
        '''
        regular_wave_get_direction = dll.dll.regular_wave_get_direction
        regular_wave_get_direction.restype = ctypes.c_double
        result = regular_wave_get_direction(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_phase(self, location, time):
        '''
        Get the phase of the wave at a given point for a given time.
        :param Coordinates_3D location: Location at which the phase is to be calculated. All coordinates in meter.
        :param float time: Time for which the phase is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        regular_wave_get_phase = dll.dll.regular_wave_get_phase
        regular_wave_get_phase.restype = ctypes.c_double
        result = regular_wave_get_phase(self.__c_base_object, Coordinates_3D(location), ctypes.c_double(time))
        self.__check_error_throw_exception()
        return result
    
    def get_elevation(self, location, time):
        '''
        Get elevation of the wave at a given point for a given time.
        :param Coordinates_3D location: Location at which the eleveation is to be calculated. All coordinates in meter.
        :param float time: Time for which the elevation is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        regular_wave_get_elevation = dll.dll.regular_wave_get_elevation
        regular_wave_get_elevation.restype = ctypes.c_double
        result = regular_wave_get_elevation(self.__c_base_object, location, ctypes.c_double(time))
        self.__check_error_throw_exception()
        return result
    
    def get_pressure_amp(self, depth):
        '''
        Get wave pressure amplitude at a given depth.
        :param float depth: Depth in meter at which the pressure amplitude is to be computed. Depth should be positive value. 
        '''
        regular_wave_get_pressure_amp = dll.dll.regular_wave_get_pressure_amp
        regular_wave_get_pressure_amp.restype = ctypes.c_double
        result = regular_wave_get_pressure_amp(self.__c_base_object, ctypes.c_double(depth))
        self.__check_error_throw_exception()
        return result