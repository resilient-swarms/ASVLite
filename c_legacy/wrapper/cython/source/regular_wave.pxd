from geometry cimport Coordinates_3D

cdef extern from "regular_wave.h":
    struct Regular_wave:
        pass
    
    Regular_wave* regular_wave_new(double amplitude, double frequency, double phase_lag, double direction)
    void regular_wave_delete(Regular_wave* regular_wave)
    char* regular_wave_get_error_msg(Regular_wave* regular_wave)
    double regular_wave_get_amplitude(Regular_wave* regular_wave)
    double regular_wave_get_frequency(Regular_wave* regular_wave)
    double regular_wave_get_direction(Regular_wave* regular_wave)
    double regular_wave_get_wavenumber(Regular_wave * regular_wave)
    double regular_wave_get_phase(Regular_wave* regular_wave, Coordinates_3D location, double time)
    double regular_wave_get_elevation(Regular_wave* regular_wave, Coordinates_3D location, double time)
    double regular_wave_get_pressure_amp(Regular_wave* regular_wave, double depth)

cdef class py_Regular_wave:
    cdef Regular_wave* _c_object
    cdef void __check_error_throw_exception(self)
    cdef double get_amplitude(self)
    cdef double get_frequency(self)
    cdef double get_direction(self)
    cdef double get_wavenumber(self)
    cdef double get_phase(self, Coordinates_3D location, double time)
    cdef double get_elevation(self, Coordinates_3D location, double time)
    cdef double get_pressure_amp(self, double depth)