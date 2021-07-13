import ctypes
import dll
import constants
import geometry
import regular_wave

class _Wave(ctypes.Structure):
    _fields_ = [("significant_wave_height",     ctypes.c_double),
                ("heading",                     ctypes.c_double),
                ("random_number_seed",          ctypes.c_long),
                ("spectrum",                    ctypes.POINTER(regular_wave.Regular_wave)),
                ("min_spectral_frequency",      ctypes.c_double),
                ("max_spectral_frequency",      ctypes.c_double),
                ("peak_spectral_frequency",     ctypes.c_double),
                ("min_spectral_wave_heading",   ctypes.c_double),
                ("max_spectral_wave_heading",   ctypes.c_double)]
    
class Wave:
    def __init__(self, sig_wave_height, wave_heading, rand_seed):
        self.__c_object = _Wave()
        dll.dll.wave_init(ctypes.pointer(self.__c_object), 
                          ctypes.c_double(sig_wave_height), 
                          ctypes.c_double(wave_heading), 
                          ctypes.c_double(rand_seed))

    def get_elevation(self, location, time):
        wave_get_elevation = dll.dll.wave_get_elevation
        wave_get_elevation.restype = ctypes.c_double
        result = wave_get_elevation(ctypes.pointer(self.__c_object), 
                                        location.c_object, 
                                        ctypes.c_double(time))
        return result
    
    # Getter for c_object
    @property
    def c_object(self):
        return self.__c_object

    # Getter and setter for significant_wave_height    
    @property
    def significant_wave_height(self):
        return self.__c_object.significant_wave_height
    
    @significant_wave_height.setter
    def significant_wave_height(self, value):
        self.__c_object.significant_wave_height = value 

    # Getter and setter for heading    
    @property
    def heading(self):
        return self.__c_object.heading
    
    @heading.setter
    def heading(self, value):
        self.__c_object.heading = value 

    # Getter and setter for random_number_seed    
    @property
    def random_number_seed(self):
        return self.__c_object.random_number_seed
    
    @random_number_seed.setter
    def random_number_seed(self, value):
        self.__c_object.random_number_seed = value 

    # Getter for spectrum    
    @property
    def spectrum(self):
        return self.__c_object.spectrum
    
    # Getter for spectrum array shape    
    @property
    def spectrum_shape(self):
        return (constants.COUNT_WAVE_SPECTRAL_DIRECTIONS, constants.COUNT_WAVE_SPECTRAL_FREQUENCIES)
    
    # Getter for min_spectral_frequency    
    @property
    def min_spectral_frequency(self):
        return self.__c_object.min_spectral_frequency

    # Getter for max_spectral_frequency    
    @property
    def max_spectral_frequency(self):
        return self.__c_object.max_spectral_frequency

     # Getter for peak_spectral_frequency    
    @property
    def peak_spectral_frequency(self):
        return self.__c_object.peak_spectral_frequency 

     # Getter for min_spectral_wave_heading    
    @property
    def min_spectral_wave_heading(self):
        return self.__c_object.min_spectral_wave_heading

     # Getter for max_spectral_wave_heading    
    @property
    def max_spectral_wave_heading(self):
        return self.__c_object.max_spectral_wave_heading 