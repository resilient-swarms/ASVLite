import ctypes
from unittest import result
import dll
from geometry import Coordinates_3D
from regular_wave import Regular_wave
      
class Wave(ctypes.Structure):
    '''
    Class to define a sea surface. 
    '''
    pass

    def __init__(self, sig_wave_height, wave_heading, rand_seed, count_wave_spectral_directions, count_wave_spectral_frequencies):
        '''
        Create and initialise a sea surface.
        :param float sig_wave_height: Significant wave height in meter.
        :param float wave_heading: Predominant wave heading, in radians, with respect to the geographic North.
        :param int rand_seed: Seed for random number generator 
        :param int count_wave_spectral_directions: The number of discrete direction bands in the wave spectrum. Value should be greater than 1.
        :param int count_wave_spectral_frequencies: The number of discrete frequency bands in the wave spectrum. Value should be greater than 1.
        '''
        wave_new = dll.dll.wave_new
        wave_new.restype = ctypes.POINTER(Wave)
        result = wave_new(ctypes.c_double(sig_wave_height), 
                          ctypes.c_double(wave_heading), 
                          ctypes.c_int(rand_seed), 
                          ctypes.c_int(count_wave_spectral_directions), 
                          ctypes.c_int(count_wave_spectral_frequencies))
        self.__c_base_object = result 
    
    @classmethod
    def from_c_base_object(cls, c_base_object):
        '''
        Create and initialise a wave from a c object. 
        '''
        wave = cls(0.0, 0.0, 0, 0, 0)
        wave.__delete()
        wave.__c_base_object = c_base_object
        return wave

    def __del__(self):
        self.__delete()
    
    def __delete(self):
        '''
        Free memory allocated for the wave. 
        '''
        wave_delete = dll.dll.wave_delete
        wave_delete.restype = None 
        wave_delete(self.__c_base_object)
    
    def __get_error_msg(self):
        '''
        Returns error message related to the last function called for the instance of Wave.
        '''
        wave_get_error_msg = dll.dll.wave_get_error_msg
        wave_get_error_msg.restype = ctypes.c_char_p
        result = wave_get_error_msg(self.__c_base_object)
        return result
    
    def __check_error_throw_exception(self):
        error_msg = self.__get_error_msg()
        if error_msg != None:
            raise ValueError(error_msg.decode("utf-8") )

    def get_elevation(self, location, time):
        '''
        Get sea surface elevation at the given location for the given time.
        :param Coordinates_3D location: Location at which the elevation is to be calculated. All coordinates in meter.
        :param float time: Time for which the elevation is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        wave_get_elevation = dll.dll.wave_get_elevation
        wave_get_elevation.restype = ctypes.c_double
        result = wave_get_elevation(self.__c_base_object, location, ctypes.c_double(time))
        self.__check_error_throw_exception()
        return result
    
    def get_count_wave_spectral_directions(self):
        '''
        Get the number of direction bands in the wave spectrum.
        '''
        wave_get_count_wave_spectral_directions = dll.dll.wave_get_count_wave_spectral_directions
        wave_get_count_wave_spectral_directions.restype = ctypes.c_int
        result = wave_get_count_wave_spectral_directions(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_count_wave_spectral_frequencies(self):
        '''
        Get the number of frequency bands in the wave spectrum.
        '''
        wave_get_count_wave_spectral_frequencies = dll.dll.wave_get_count_wave_spectral_frequencies
        wave_get_count_wave_spectral_frequencies.restype = ctypes.c_int
        result = wave_get_count_wave_spectral_frequencies(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_regular_wave_at(self, d, f):
        '''
        Get the regular wave at spectrum[d][f].
        '''
        wave_get_regular_wave_at = dll.dll.wave_get_regular_wave_at
        wave_get_regular_wave_at.restype = ctypes.POINTER(Regular_wave)
        result = wave_get_regular_wave_at(self.__c_base_object, ctypes.c_int(d), ctypes.c_int(f))
        self.__check_error_throw_exception()
        return Regular_wave.from_c_base_object(result)

    def get_min_spectral_frequency(self):
        '''
        Get the minimum spectral frequency, in Hz, for the wave spectrum.
        '''
        wave_get_min_spectral_frequency = dll.dll.wave_get_min_spectral_frequency
        wave_get_min_spectral_frequency.restype = ctypes.c_double
        result = wave_get_min_spectral_frequency(self.__c_base_object)
        self.__check_error_throw_exception()
        return result

    def get_max_spectral_frequency(self):
        '''
        Get the maximum spectral frequency, in Hz, for the wave spectrum.
        '''
        wave_get_max_spectral_frequency = dll.dll.wave_get_max_spectral_frequency
        wave_get_max_spectral_frequency.restype = ctypes.c_double
        result = wave_get_max_spectral_frequency(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_significant_height(self):
        '''
        Get the significant wave height, in meter, for the sea state.
        '''
        wave_get_significant_height = dll.dll.wave_get_significant_height
        wave_get_significant_height.restype = ctypes.c_double
        result = wave_get_significant_height(self.__c_base_object)
        self.__check_error_throw_exception()
        return result

    def get_predominant_heading(self):
        '''
        Get the predominant wave heading, in radians, for the sea state.
        '''
        wave_get_predominant_heading = dll.dll.wave_get_predominant_heading
        wave_get_predominant_heading.restype = ctypes.c_double
        result = wave_get_predominant_heading(self.__c_base_object)
        self.__check_error_throw_exception()
        return result