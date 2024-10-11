from geometry cimport py_Coordinates_3D

cdef class py_Regular_wave:
    '''
    Class to define a regular wave. 
    '''

    def __cinit__(self, double amplitude, double frequency, double phase_lag, double direction):
        '''
        Create and initialise a regular wave.
        :param float amplitude: Wave amplitude in meter.
        :param float frequency: Wave frequency in Hz.
        :param float phase_lag: Wave phase in radians. 
        :param direction: Northing of the wave in radians. 
        '''
        self._c_object = regular_wave_new(amplitude, frequency, phase_lag, direction)
    
    def __dealloc__(self):
        regular_wave_delete(self._c_object)
   
    cdef void __check_error_throw_exception(self):
        cdef char* error = regular_wave_get_error_msg(self._c_object)
        if error != NULL:
            raise ValueError(<bytes>error)
    
    cdef double get_amplitude(self):
        '''
        Get wave amplitude in meter.
        '''
        cdef double value = regular_wave_get_amplitude(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_amplitude(self) -> float:
        '''
        Get wave amplitude in meter.
        '''
        return self.get_amplitude()
    
    cdef double get_frequency(self):
        '''
        Get wave wave frequency in Hz. 
        '''
        cdef double value = regular_wave_get_frequency(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_frequency(self) -> float:
        '''
        Get wave wave frequency in Hz. 
        '''
        return self.get_frequency()
    
    cdef double get_direction(self):
        '''
        Get wave heading in radians. 
        '''
        cdef double value = regular_wave_get_direction(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_direction(self) -> float:
        '''
        Get wave heading in radians. 
        '''
        return self.get_direction()

    cdef double get_wavenumber(self):
        '''
        Get wavenumber. 
        '''
        cdef double value = regular_wave_get_wavenumber(self._c_object)
        self.__check_error_throw_exception()
        return value

    def py_get_wavenumber(self) -> float:
        '''
        Get wavenumber.
        '''
        return self.get_wavenumber()
    
    cdef double get_phase(self, Coordinates_3D location, double time):
        '''
        Get the phase of the wave at a given point for a given time.
        :param Coordinates_3D location: Location at which the phase is to be calculated. All coordinates in meter.
        :param float time: Time for which the phase is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        cdef double value = regular_wave_get_phase(self._c_object, location, time)
        self.__check_error_throw_exception()
        return value
    
    def py_get_phase(self, py_Coordinates_3D location, double time) -> float:
        '''
        Get the phase of the wave at a given point for a given time.
        :param py_Coordinates_3D location: Location at which the phase is to be calculated. All coordinates in meter.
        :param float time: Time for which the phase is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        return self.get_phase(location._c_object, time)
    
    cdef double get_elevation(self, Coordinates_3D location, double time):
        '''
        Get elevation of the wave at a given point for a given time.
        :param Coordinates_3D location: Location at which the eleveation is to be calculated. All coordinates in meter.
        :param float time: Time for which the elevation is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        cdef double value = regular_wave_get_elevation(self._c_object, location, time)
        self.__check_error_throw_exception()
        return value
    
    def py_get_elevation(self, py_Coordinates_3D location, double time) -> float:
        '''
        Get elevation of the wave at a given point for a given time.
        :param py_Coordinates_3D location: Location at which the eleveation is to be calculated. All coordinates in meter.
        :param float time: Time for which the elevation is to be calculated. Time is measured in seconds from the start of simulation. Time should be non-negative.
        '''
        return self.get_elevation(location._c_object, time)
    
    cdef double get_pressure_amp(self, double depth):
        '''
        Get wave pressure amplitude at a given depth.
        :param float depth: Depth in meter at which the pressure amplitude is to be computed. Depth should be positive value. 
        '''
        cdef double value = regular_wave_get_pressure_amp(self._c_object, depth)
        self.__check_error_throw_exception()
        return value
    
    def py_get_pressure_amp(self, double depth) -> float:
        '''
        Get wave pressure amplitude at a given depth.
        :param float depth: Depth in meter at which the pressure amplitude is to be computed. Depth should be positive value. 
        '''
        return self.get_pressure_a