import ctypes
import dll

class Coordinates_3D(ctypes.Structure):
    '''
    Cartesian coordinates for a three-dimensional space. 
    '''
    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

class Rigid_body_DOF(ctypes.Structure):
    '''
    Six degrees of freedom for a rigid body in a three-dimensional space. 
    '''
    _fields_ = [("surge", ctypes.c_double),
                ("sway", ctypes.c_double),
                ("heave", ctypes.c_double),
                ("roll", ctypes.c_double),
                ("pitch", ctypes.c_double),
                ("yaw", ctypes.c_double)]

def normalise_angle_PI(angle):
    '''
    Normalise angle so that -PI < angle <= PI.
    '''
    geometry_normalise_angle_PI = dll.dll.normalise_angle_PI
    geometry_normalise_angle_PI.restype = ctypes.c_double
    result = geometry_normalise_angle_PI(ctypes.c_double(angle))
    return result

def normalise_angle_2PI(angle):
    '''
    Normalise angle so that 0 <= angle < 2PI.
    '''
    geometry_normalise_angle_2PI = dll.dll.normalise_angle_2PI
    geometry_normalise_angle_2PI.restype = ctypes.c_double
    result = geometry_normalise_angle_2PI(ctypes.c_double(angle))
    return result