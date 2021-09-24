import ctypes
import dll
import geometry
import constants
import regular_wave
import wave
   
class Asv_propeller(ctypes.Structure):
    _fields_ = [("position",    geometry.Dimensions),
                ("orientation", geometry.Dimensions),
                ("thrust",      ctypes.c_double)]

class Asv_specification(ctypes.Structure):
    _fields_ = [("L_wl",        ctypes.c_double),
                ("B_wl",        ctypes.c_double),
                ("D",           ctypes.c_double),
                ("T",           ctypes.c_double),
                ("max_speed",   ctypes.c_double),
                ("disp",        ctypes.c_double),
                ("r_roll",      ctypes.c_double),
                ("r_pitch",     ctypes.c_double),
                ("r_yaw",       ctypes.c_double),
                ("cog",         geometry.Dimensions)]

class Asv_dynamics(ctypes.Structure):
    _fields_ = [("time_step_size",       ctypes.c_double),
                ("time",                 ctypes.c_double),
                ("M",                    ctypes.c_double * constants.COUNT_DOF),
                ("C",                    ctypes.c_double * constants.COUNT_DOF),
                ("K",                    ctypes.c_double * constants.COUNT_DOF),
                ("X",                    ctypes.c_double * constants.COUNT_DOF),
                ("V",                    ctypes.c_double * constants.COUNT_DOF),
                ("A",                    ctypes.c_double * constants.COUNT_DOF),
                ("F",                    ctypes.c_double * constants.COUNT_DOF),
                ("F_wave",               ctypes.c_double * constants.COUNT_DOF),
                ("F_propeller",          ctypes.c_double * constants.COUNT_DOF),
                ("F_drag",               ctypes.c_double * constants.COUNT_DOF),
                ("F_restoring",          ctypes.c_double * constants.COUNT_DOF),
                ("P_unit_wave",          ctypes.c_double * constants.COUNT_ASV_SPECTRAL_FREQUENCIES * 2),
                ("P_unit_regular_wave",  ctypes.c_double),
                ("P_unit_wave_freq_min", ctypes.c_double),
                ("P_unit_wave_freq_max", ctypes.c_double)]

class Asv(ctypes.Structure):
    _fields_ = [("spec",            Asv_specification),
                ("count_propellers", ctypes.c_int),
                ("propellers",      Asv_propeller * constants.COUNT_PROPELLERS_MAX),
                ("wave",            ctypes.POINTER(wave.Wave)),
                ("origin_position", geometry.Dimensions),
                ("attitude",        geometry.Dimensions),
                ("dynamics",        Asv_dynamics),
                ("cog_position",    geometry.Dimensions)]
    
    def init(self, wave):
        dll.dll.asv_init(ctypes.pointer(self), 
                         ctypes.pointer(wave))
    
    def set_sea_state(self, wave):
        asv_set_sea_state = dll.dll.asv_set_sea_state
        asv_set_sea_state.restype = None
        result = asv_set_sea_state(ctypes.pointer(self), 
                                    ctypes.pointer(wave))
        return result

    def compute_dynamics(self, time):
        asv_compute_dynamics = dll.dll.asv_compute_dynamics
        asv_compute_dynamics.restype = None
        result = asv_compute_dynamics(ctypes.pointer(self), 
                                      ctypes.c_double(time))
        return result

    def compute_dynamics(self, rudder_angle, time):
        wave_glider_compute_dynamics = dll.dll.wave_glider_compute_dynamics
        wave_glider_compute_dynamics.restype = None
        result = wave_glider_compute_dynamics(ctypes.pointer(self), 
                                               ctypes.c_double(rudder_angle),
                                               ctypes.c_double(time))
        return result




# Example of how to access data from Asv.wave:
# >>> w = Wave(1.2, 0, 1)
# >>> a = Asv(w)
# >>> a.wave.contents.heading
# 0.0
# >>> a.wave.contents.significant_wave_height
# 1.2