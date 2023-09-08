from geometry cimport py_Coordinates_3D
from regular_wave cimport py_Regular_wave

cdef class py_Sea_surface:
    '''
    Class to define a sea surface. 
    '''

    def __cinit__(self, double sig_wave_ht=1, double wave_heading=0, int rand_seed=1, int count_component_waves=21):
        self._c_object = sea_surface_new(sig_wave_ht, wave_heading, rand_seed, count_component_waves)
    
    def __dealloc__(self):
        sea_surface_delete(self._c_object)

    cdef void __check_error_throw_exception(self):
        cdef char* error = sea_surface_get_error_msg(self._c_object)
        if error != NULL:
            raise ValueError(<bytes>error)
    
    cdef double get_elevation(self, Coordinates_3D location, double time):
        '''
        Get sea surface elevation at the given location for the given time.
        :param Coordinates_3D location: Location at which the elevation is to be calculated. All coordinates in meter.
        :param float time: Time for which the elevation is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        cdef double value = sea_surface_get_elevation(self._c_object, location, time)
        self.__check_error_throw_exception()
        return value
    
    def py_get_elevation(self, py_Coordinates_3D location, double time):
        '''
        Get sea surface elevation at the given location for the given time.
        :param Coordinates_3D location: Location at which the elevation is to be calculated. All coordinates in meter.
        :param float time: Time for which the elevation is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        return self.get_elevation(location._c_object, time)

    cdef int get_count_component_waves(self):
        '''
        Get the number of regular component waves.
        '''
        cdef int value = sea_surface_get_count_component_waves(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_count_component_waves(self) -> int:
        '''
        Get the number of regular component waves.
        '''
        return self.get_count_component_waves()

    cdef double get_min_spectral_frequency(self):
        '''
        Get the minimum spectral frequency, in Hz, for the wave spectrum.
        '''
        cdef double value = sea_surface_get_min_spectral_frequency(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_min_spectral_frequency(self) -> float:
        '''
        Get the minimum spectral frequency, in Hz, for the wave spectrum.
        '''
        return self.get_min_spectral_frequency()

    cdef double get_max_spectral_frequency(self):
        '''
        Get the maximum spectral frequency, in Hz, for the wave spectrum.
        '''
        cdef double value = sea_surface_get_max_spectral_frequency(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_max_spectral_frequency(self) -> float:
        '''
        Get the maximum spectral frequency, in Hz, for the wave spectrum.
        '''
        return self.get_max_spectral_frequency()
    
    cdef double get_peak_spectral_frequency(self):
        '''
        Get the peak spectral frequency, in Hz, for the wave spectrum.
        '''
        cdef double value = sea_surface_get_peak_spectral_frequency(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_peak_spectral_frequency(self) -> float:
        '''
        Get the peak spectral frequency, in Hz, for the wave spectrum.
        '''
        return self.get_peak_spectral_frequency()

    cdef double get_significant_height(self):
        '''
        Get the significant wave height, in meter, for the sea state.
        '''
        cdef double value = sea_surface_get_significant_height(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_significant_height(self) -> float:
        '''
        Get the significant wave height, in meter, for the sea state.
        '''
        return self.get_significant_height()

    cdef double get_predominant_heading(self):
        '''
        Get the predominant wave heading, in radians, for the sea state.
        '''
        cdef double value = sea_surface_get_predominant_heading(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_predominant_heading(self) -> float:
        '''
        Get the predominant wave heading, in radians, for the sea state.
        '''
        return self.get_predominant_heading()