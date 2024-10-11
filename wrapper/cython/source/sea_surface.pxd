from geometry cimport Coordinates_3D
from regular_wave cimport Regular_wave

cdef extern from "sea_surface.h":
    struct Sea_surface:
        pass
    
    Sea_surface* sea_surface_new(double sig_wave_ht,double wave_heading, int rand_seed, int count_component_waves)
    Sea_surface* sea_surface_new_from_wind(double wind_velocity_u, double wind_velocity_v, int rand_seed, int count_component_waves);
    void sea_surface_delete(Sea_surface* sea_surface)
    char* sea_surface_get_error_msg(Sea_surface* sea_surface)
    double sea_surface_get_elevation(Sea_surface* sea_surface, Coordinates_3D location, double time)
    int sea_surface_get_count_component_waves(Sea_surface* sea_surface)
    double sea_surface_get_min_spectral_frequency(Sea_surface* sea_surface)
    double sea_surface_get_max_spectral_frequency(Sea_surface* sea_surface)
    double sea_surface_get_peak_spectral_frequency(Sea_surface* sea_surface)
    double sea_surface_get_significant_height(Sea_surface* sea_surface)
    double sea_surface_get_predominant_heading(Sea_surface* sea_surface)
    double sea_surface_get_mean_wavenumber(Sea_surface* sea_surface)

cdef class py_Sea_surface:
    cdef Sea_surface* _c_object
    cdef void __check_error_throw_exception(self)
    cdef double get_elevation(self, Coordinates_3D location, double time)
    cdef int get_count_component_waves(self)
    cdef double get_min_spectral_frequency(self)
    cdef double get_max_spectral_frequency(self)
    cdef double get_peak_spectral_frequency(self)
    cdef double get_significant_height(self)
    cdef double get_predominant_heading(self)
    cdef double get_mean_wavenumber(self)