import ctypes
import dll
import geometry
   
class PID_controller(ctypes.Structure):
    _fields_ = [("asv_position",        geometry.Dimensions),
                ("asv_attitude",        geometry.Dimensions),
                ("way_point",           geometry.Dimensions),
                ("kp_heading",          ctypes.c_double),
                ("ki_heading",          ctypes.c_double),
                ("kd_heading",          ctypes.c_double),
                ("kp_position",         ctypes.c_double),
                ("ki_position",         ctypes.c_double),
                ("kd_position",         ctypes.c_double),
                ("thrust_fore_ps",      ctypes.c_double),
                ("thrust_fore_sb",      ctypes.c_double),
                ("thrust_aft_ps",       ctypes.c_double),
                ("thrust_aft_sb",       ctypes.c_double),
                ("error_heading",       ctypes.c_double),
                ("error_int_heading",   ctypes.c_double),
                ("error_diff_heading",  ctypes.c_double),
                ("error_position",      ctypes.c_double),
                ("error_int_position",  ctypes.c_double),
                ("error_diff_position", ctypes.c_double)]
    
    def __init__(self):
        dll.dll.pid_controller_init(ctypes.pointer(self))

    def set_gains_position(self, p, i, d):
        pid_controller_set_gains_position = dll.dll.pid_controller_set_gains_position
        pid_controller_set_gains_position.restype = None
        result = pid_controller_set_gains_position(ctypes.pointer(self), 
                                                   ctypes.c_double(p),
                                                   ctypes.c_double(i),
                                                   ctypes.c_double(d))
        return result
    
    def set_gains_heading(self, p, i, d):
        pid_controller_set_gains_heading = dll.dll.pid_controller_set_gains_heading
        pid_controller_set_gains_heading.restype = None
        result = pid_controller_set_gains_heading(ctypes.pointer(self), 
                                                  ctypes.c_double(p),
                                                  ctypes.c_double(i),
                                                  ctypes.c_double(d))
        return result

    def set_current_state(self, position, attitude):
        pid_controller_set_current_state = dll.dll.pid_controller_set_current_state
        pid_controller_set_current_state.restype = None
        result = pid_controller_set_current_state(ctypes.pointer(self), 
                                                  position,
                                                  attitude)
        return result
    
    def set_way_point(self, way_point):
        pid_controller_set_way_point = dll.dll.pid_controller_set_way_point
        pid_controller_set_way_point.restype = None
        result = pid_controller_set_way_point(ctypes.pointer(self), way_point)
        return result

    def set_thrust(self):
        pid_controller_set_thrust = dll.dll.pid_controller_set_thrust
        pid_controller_set_thrust.restype = None
        result = pid_controller_set_thrust(ctypes.pointer(self))
        return result