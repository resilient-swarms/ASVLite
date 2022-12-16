import ctypes
import dll
from sea_surface import Sea_surface
from geometry import Coordinates_3D, Rigid_body_DOF

   
class Asv_specification(ctypes.Structure):
    '''
    Specification of the vehicle. 
    '''
    _fields_ = [("L_wl",        ctypes.c_double),
                ("B_wl",        ctypes.c_double),
                ("D",           ctypes.c_double),
                ("T",           ctypes.c_double),
                ("max_speed",   ctypes.c_double),
                ("disp",        ctypes.c_double),
                ("r_roll",      ctypes.c_double),
                ("r_pitch",     ctypes.c_double),
                ("r_yaw",       ctypes.c_double),
                ("cog",         Coordinates_3D)]

class Thruster(ctypes.Structure):
    '''
    Class to define a thruster on the ASV.
    '''

    def __init__(self, position):
        '''
        Create and initialise a Thruster.
        '''
        thruster_new = dll.dll.thruster_new
        thruster_new.restype = ctypes.POINTER(Thruster)
        result = thruster_new(position)
        self.__c_base_object = result 

    @classmethod
    def from_c_base_object(cls, c_base_object):
        '''
        Create and initialise a thruster from a c object. 
        '''
        thruster = cls(Coordinates_3D(0.0, 0.0, 0.0))
        thruster.__delete()
        thruster.__c_base_object = c_base_object
        return thruster

    def __del__(self):
        self.__delete()
    
    def __delete(self):
        '''
        Free memory allocated for the thruster. 
        '''
        thruster_delete = dll.dll.thruster_delete
        thruster_delete.restype = None 
        thruster_delete(self.__c_base_object)
    
    def __get_error_msg(self):
        '''
        Returns error message related to the last function called for the instance of Thruster.
        '''
        thruster_get_error_msg = dll.dll.thruster_get_error_msg
        thruster_get_error_msg.restype = ctypes.c_char_p
        result = thruster_get_error_msg(self.__c_base_object)
        return result
    
    def __check_error_throw_exception(self):
        error_msg = self.__get_error_msg()
        if error_msg != None:
            raise ValueError(error_msg.decode("utf-8") )

    def get_c_base_object(self):
        '''
        Get the instance of the base class. 
        '''
        return self.__c_base_object

    def set_thrust(self, orientation, magnitude):
        '''
        Set the orientation and magnitude of thrust vector. 
        :param Coordinates_3D orientation: Orientation of the thrust vector, in radians, in body-fixed frame.
        :param float magnitude: Magnitude of the thrust in N.
        '''
        thruster_set_thrust = dll.dll.thruster_set_thrust
        thruster_set_thrust.restype = None
        result = thruster_set_thrust(self.__c_base_object, orientation, ctypes.c_double(magnitude))
        self.__check_error_throw_exception()
        return result
    
    def get_position(self):
        thruster_get_position = dll.dll.thruster_get_position
        thruster_get_position.restype = Coordinates_3D
        result = thruster_get_position(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
class Asv(ctypes.Structure):
    '''
    Class to define an ASV. 
    '''    

    def __init__(self, spec, sea_surface, position, attitude):
        '''
        Create and initialise an ASV.
        :param Asv_specification spec: Specification of the ASV.
        :param Sea_surface sea_surface: Sea surface for the ASV.
        :param Coordinate_3D position: Position of ASV on the sea surface.
        :param Coordinate_3D attitude: Floating attitude of the ASV.
        '''
        asv_new = dll.dll.asv_new
        asv_new.restype = ctypes.POINTER(Asv)
        result = asv_new(spec, sea_surface.get_c_base_object(), position, attitude)
        self.__c_base_object = result 
    
    @classmethod
    def from_c_base_object(cls, c_base_object):
        '''
        Create and initialise an Asv from a c object. 
        '''
        asv = cls(Asv_specification(0,0,0,0,0,0,0,0,0,Coordinates_3D(0,0,0)), None, Coordinates_3D(0,0,0), Coordinates_3D(0,0,0))
        asv.__delete()
        asv.__c_base_object = c_base_object
        return asv

    def __del__(self):
        self.__delete()
    
    def __delete(self):
        '''
        Free memory allocated for the ASV. 
        '''
        asv_delete = dll.dll.asv_delete
        asv_delete.restype = None 
        asv_delete(self.__c_base_object)
    
    def __get_error_msg(self):
        '''
        Returns error message related to the last function called for the instance of Asv.
        '''
        asv_get_error_msg = dll.dll.asv_get_error_msg
        asv_get_error_msg.restype = ctypes.c_char_p
        result = asv_get_error_msg(self.__c_base_object)
        return result
    
    def __check_error_throw_exception(self):
        error_msg = self.__get_error_msg()
        if error_msg != None:
            raise ValueError(error_msg.decode("utf-8") )

    def get_c_base_object(self):
        '''
        Get the instance of the base class. 
        '''
        return self.__c_base_object
    
    def set_thrusters(self, thrusters):
        '''
        Set the thrusters for the asv.
        :param list thrusters: List of thrusters for the asv.
        '''
        c_thrusters = (ctypes.POINTER(Thruster) * len(thrusters))(*thrusters)
        asv_set_thrusters = dll.dll.asv_set_thrusters
        asv_set_thrusters.restype = None
        result = asv_set_thrusters(self.__c_base_object, c_thrusters, len(thrusters))
        self.__check_error_throw_exception()
        return result 

    def get_thrusters(self):
        '''
        Get the list of thrusters.
        '''
        asv_get_thrusters = dll.dll.asv_get_thrusters
        asv_get_count_thrusters = dll.dll.asv_get_count_thrusters
        asv_get_thrusters.restype = ctypes.POINTER(ctypes.POINTER(Thruster))
        asv_get_count_thrusters.restype = ctypes.c_int
        count_thruster = asv_get_count_thrusters(self.__c_base_object)
        self.__check_error_throw_exception()
        result = asv_get_thrusters(self.__c_base_object)
        self.__check_error_throw_exception()
        thrusters = [result[i] for i in range(count_thruster)]
        return thrusters
    
    def set_sea_state(self, sea_surface):
        '''
        Modify the current sea state to a new sea state.
        '''
        asv_set_sea_state = dll.dll.asv_set_sea_state
        asv_set_sea_state.restype = None
        result = asv_set_sea_state(self.__c_base_object, sea_surface.get_c_base_object())
        self.__check_error_throw_exception()
        return result

    def compute_dynamics(self, time_step_size):
        '''
        Increment time and compute dynamics for the ASV.
        :param float time_step_size: Step size, in milliseconds, to increment the current time.
        '''
        asv_compute_dynamics = dll.dll.asv_compute_dynamics
        asv_compute_dynamics.restype = None
        result = asv_compute_dynamics(self.__c_base_object, ctypes.c_double(time_step_size))
        self.__check_error_throw_exception()
        return result

    def wg_set_thrust_tuning_factor(self, tuning_factor):
        '''
        Overwrite the default tuning factor of 1 used for computing wave glider thrust.
        :param float tuning_factor: tuning factor.
        '''
        wave_glider_set_thrust_tuning_factor = dll.dll.wave_glider_set_thrust_tuning_factor
        wave_glider_set_thrust_tuning_factor.restype = None
        result = wave_glider_set_thrust_tuning_factor(self.__c_base_object, ctypes.c_double(tuning_factor))
        self.__check_error_throw_exception()
        return result

    def wg_compute_dynamics(self, rudder_angle, time_step_size):
        '''
        Similar to function compute_dynamics but should be used only for a wave glider.
        The function computes the dynamics of the wave glider for the next time step,
        and also computes the wave thrust force generated by the underwater glider.
        :param float rudder_angle: Angle of the rudder with respect to X axis of the ASV. 
        Rudder angle must within (-PI/2, PI/2). Angle is positive when the vehicle has to turn 
        to starboard (ie. aft end of the rudder points to starboard side). 
        :param float time_step_size: Step size, in milliseconds, to increment the current time.
        '''
        wave_glider_compute_dynamics = dll.dll.wave_glider_compute_dynamics
        wave_glider_compute_dynamics.restype = None
        result = wave_glider_compute_dynamics(self.__c_base_object, ctypes.c_double(rudder_angle), ctypes.c_double(time_step_size))
        self.__check_error_throw_exception()
        return result 
    
    def wg_run(self, callback_precompute, callback_postcompute, time_step_size):
        '''
        Simulate the wave glider for multiple time steps. 
        :param callback_precompute: callback at the beginning of each time step. Can be used for setting rudder angle
        for the time step, modify the sea_surface for the time step, etc. callback_precompute takes as argument a pointer to the 
        rudder angle that is to be set. The return value of the callback is used to continue
        or exit from the simulation. If the callback returned false, simulation is terminated.
        :param callback_postcompute: callback at the end of each time step. Can be used for fetching/printing the results 
        of the time step.
        '''
        wave_glider_run = dll.dll.wave_glider_run
        wave_glider_run.restype = None
        result = wave_glider_run(self.__c_base_object, 
                                ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.POINTER(ctypes.c_double))(callback_precompute), 
                                ctypes.CFUNCTYPE(None)(callback_postcompute), 
                                ctypes.c_double(time_step_size))
        self.__check_error_throw_exception()
        return result
    
    def get_sea_surface(self):
        '''
        Get the sea surface initialised for the ASV.
        '''
        asv_get_sea_surface = dll.dll.asv_get_sea_surface
        asv_get_sea_surface.restype = ctypes.POINTER(Sea_surface)
        result = asv_get_sea_surface(self.__c_base_object)
        self.__check_error_throw_exception()
        return Sea_surface.from_c_base_object(result)
    
    def get_position_cog(self):
        '''
        Get the position of the asv using the COG of the vehicle.
        '''
        asv_get_position_cog = dll.dll.asv_get_position_cog
        asv_get_position_cog.restype = Coordinates_3D
        result = asv_get_position_cog(self.__c_base_object)
        self.__check_error_throw_exception()
        return result 
    
    def get_position_origin(self):
        '''
        Get the position of the asv using the origin of the vehicle.
        '''
        asv_get_position_origin = dll.dll.asv_get_position_origin
        asv_get_position_origin.restype = Coordinates_3D
        result = asv_get_position_origin(self.__c_base_object)
        self.__check_error_throw_exception()
        return result 

    def get_attitude(self):
        '''
        Get the floating attitude of the asv.
        '''
        asv_get_attitude = dll.dll.asv_get_attitude
        asv_get_attitude.restype = Coordinates_3D
        result = asv_get_attitude(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_F(self):
        '''
        Get force.
        '''
        asv_get_F = dll.dll.asv_get_F
        asv_get_F.restype = Rigid_body_DOF
        result = asv_get_F(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_A(self):
        '''
        Get acceleration.
        '''
        asv_get_A = dll.dll.asv_get_A
        asv_get_A.restype = Rigid_body_DOF
        result = asv_get_A(self.__c_base_object)
        self.__check_error_throw_exception()
        return result

    def get_V(self):
        '''
        Get velocity.
        '''
        asv_get_V = dll.dll.asv_get_V
        asv_get_V.restype = Rigid_body_DOF
        result = asv_get_V(self.__c_base_object)
        self.__check_error_throw_exception()
        return result
    
    def get_spec(self):
        '''
        Get ASV specification.
        '''
        asv_get_spec = dll.dll.asv_get_spec
        asv_get_spec.restype = Asv_specification
        result = asv_get_spec(self.__c_base_object)
        self.__check_error_throw_exception()
        return result   
