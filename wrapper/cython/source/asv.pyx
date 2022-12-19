from geometry cimport py_Coordinates_3D, py_Rigid_body_DOF
from regular_wave cimport py_Regular_wave
from sea_surface cimport py_Sea_surface

cdef class py_Asv_specification:
    '''
    Specification of the vehicle. 
    '''

    def __cinit__(self, double L_wl=0, 
                        double B_wl=0, 
                        double D=0, 
                        double T=0, 
                        double max_speed=0, 
                        double disp=0, 
                        double r_roll=0, 
                        double r_pitch=0, 
                        double r_yaw=0, 
                        py_Coordinates_3D cog=py_Coordinates_3D(0,0,0)):
        self._c_object.L_wl = L_wl 
        self._c_object.B_wl = B_wl 
        self._c_object.D = D 
        self._c_object.T = T 
        self._c_object.max_speed = max_speed 
        self._c_object.disp = disp 
        self._c_object.r_roll = r_roll 
        self._c_object.r_pitch = r_pitch 
        self._c_object.r_yaw = r_yaw 
        self._c_object.cog = cog._c_object
    
    @property
    def L_wl(self):
        return self._c_object.L_wl
    
    @L_wl.setter
    def L_wl(self, double value):
        self._c_object.L_wl = value
    
    @property
    def B_wl(self):
        return self._c_object.B_wl
    
    @B_wl.setter
    def B_wl(self, double value):
        self._c_object.B_wl = value
    
    @property
    def D(self):
        return self._c_object.D
    
    @D.setter
    def D(self, double value):
        self._c_object.D = value
    
    @property
    def T(self):
        return self._c_object.T
    
    @T.setter
    def T(self, double value):
        self._c_object.T = value
    
    @property
    def max_speed(self):
        return self._c_object.max_speed
    
    @max_speed.setter
    def max_speed(self, double value):
        self._c_object.max_speed = value
    
    @property
    def disp(self):
        return self._c_object.disp
    
    @disp.setter
    def disp(self, double value):
        self._c_object.disp = value
    
    @property
    def r_roll(self):
        return self._c_object.r_roll
    
    @r_roll.setter
    def r_roll(self, double value):
        self._c_object.r_roll = value
    
    @property
    def r_pitch(self):
        return self._c_object.r_pitch
    
    @r_pitch.setter
    def r_pitch(self, double value):
        self._c_object.r_pitch = value
    
    @property
    def r_yaw(self):
        return self._c_object.r_yaw
    
    @r_yaw.setter
    def r_yaw(self, double value):
        self._c_object.r_yaw = value
    
    @property
    def cog(self):
        value = py_Coordinates_3D()
        value._c_object = self._c_object.cog
        return value
    
    @cog.setter
    def cog(self, py_Coordinates_3D value):
        self._c_object.cog = value._c_object


cdef class py_Thruster:
    '''
    Class to define a thruster on the ASV.
    '''

    def __cinit__(self, py_Coordinates_3D position=py_Coordinates_3D(0,0,0)):
        self._c_object = thruster_new(position._c_object)
    
    def __dealloc__(self):
        thruster_delete(self._c_object)
    
    cdef void __check_error_throw_exception(self):
        cdef char* error = thruster_get_error_msg(self._c_object)
        if error != NULL:
            raise ValueError(<bytes>error)

    cdef void set_thrust(self, Coordinates_3D orientation, double magnitude):
        '''
        Set the orientation and magnitude of thrust vector. 
        :param Coordinates_3D orientation: Orientation of the thrust vector, in radians, in body-fixed frame.
        :param float magnitude: Magnitude of the thrust in N.
        '''
        thruster_set_thrust(self._c_object, orientation, magnitude)
        self.__check_error_throw_exception()
    
    def py_set_thrust(self, py_Coordinates_3D orientation, double magnitude):
        '''
        Set the orientation and magnitude of thrust vector. 
        :param Coordinates_3D orientation: Orientation of the thrust vector, in radians, in body-fixed frame.
        :param float magnitude: Magnitude of the thrust in N.
        '''
        self.set_thrust(orientation._c_object, magnitude)

    cdef Coordinates_3D get_position(self):
        '''
        Get the position of the thruster.
        '''
        cdef Coordinates_3D value = thruster_get_position(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_position(self) -> py_Coordinates_3D:
        '''
        Get the position of the thruster.
        '''
        position = py_Coordinates_3D()
        position._c_object = self.get_position()
        return position

cdef class py_Asv:
    '''
    Class to define an ASV. 
    ''' 
    def __cinit__(self, py_Asv_specification specification, py_Sea_surface sea_surface, py_Coordinates_3D position, py_Coordinates_3D attitude):
        '''
        Create and initialise an ASV.
        :param Asv_specification spec: Specification of the ASV.
        :param Sea_surface sea_surface: Sea surface for the ASV.
        :param Coordinate_3D position: Position of ASV on the sea surface.
        :param Coordinate_3D attitude: Floating attitude of the ASV.
        '''
        self._c_object = asv_new(specification._c_object, sea_surface._c_object, position._c_object, attitude._c_object)
        self._sea_surface = sea_surface
        self._thrusters = []
    
    def __dealloc__(self):
        asv_delete(self._c_object)
    
    cdef void __check_error_throw_exception(self):
        cdef char* error = asv_get_error_msg(self._c_object)
        if error != NULL:
            raise ValueError(<bytes>error)

    cdef void set_thrusters(self, Thruster **thrusters, int count_thrusters):
        '''
        Set the thrusters for the asv.
        :param list thrusters: List of thrusters for the asv.
        '''
        asv_set_thrusters(self._c_object, thrusters, count_thrusters)
        self.__check_error_throw_exception()
  
    def py_set_thrusters(self, list py_thrusters):
        '''
        Set the thrusters for the asv.
        :param list thrusters: List of thrusters for the asv.
        '''
        cdef int count_thrusters = len(py_thrusters)
        if count_thrusters > 10:
            raise ValueError("Size of thrusters exceeded max limit of 10.")
        cdef Thruster* c_thrusters[10] 
        cdef int i = 0
        cdef Coordinates_3D c_thruster_position
        cdef py_Coordinates_3D py_thruster_position
        for i in range(count_thrusters):
            py_thruster_position = py_thrusters[i].py_get_position()
            c_thruster_position = py_thruster_position._c_object
            c_thrusters[i] = thruster_new(c_thruster_position)
        self.set_thrusters(c_thrusters, count_thrusters)
        self._thrusters = py_thrusters
    
    cdef Thruster** get_thrusters(self):
        '''
        Get the list of thrusters.
        '''
        cdef Thruster** value = asv_get_thrusters(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_thrusters(self) -> list:
        '''
        Get the list of thrusters.
        '''
        return self._thrusters

    cdef int get_count_thrusters(self):
        cdef int value = asv_get_count_thrusters(self._c_object)
        self.__check_error_throw_exception()
        return value 
    
    def py_get_count_thrusters(self) -> int:
        return self.get_count_thrusters()

    cdef void set_sea_state(self, Sea_surface* sea_surface):
        '''
        Modify the current sea state to a new sea state.
        '''
        asv_set_sea_state(self._c_object, sea_surface)
        self.__check_error_throw_exception()
    
    def py_set_sea_state(self, py_Sea_surface sea_surface):
        '''
        Modify the current sea state to a new sea state.
        '''
        self.set_sea_state(sea_surface._c_object)
        self._sea_surface = sea_surface

    cdef void compute_dynamics(self, double time_step_size):
        '''
        Increment time and compute dynamics for the ASV.
        :param float time_step_size: Step size, in milliseconds, to increment the current time.
        '''
        asv_compute_dynamics(self._c_object, time_step_size)
        self.__check_error_throw_exception()

    def py_compute_dynamics(self, double time_step_size):
        '''
        Increment time and compute dynamics for the ASV.
        :param float time_step_size: Step size, in milliseconds, to increment the current time.
        '''
        self.compute_dynamics(time_step_size)
        self.__check_error_throw_exception()

    cdef void wg_set_thrust_tuning_factor(self, double tuning_factor):
        '''
        Overwrite the default tuning factor of 1 used for computing wave glider thrust.
        :param float tuning_factor: tuning factor.
        '''
        wave_glider_set_thrust_tuning_factor(self._c_object, tuning_factor)
        self.__check_error_throw_exception()
    
    def py_wg_set_thrust_tuning_factor(self, double tuning_factor):
        '''
        Overwrite the default tuning factor of 1 used for computing wave glider thrust.
        :param float tuning_factor: tuning factor.
        '''
        self.wg_set_thrust_tuning_factor(tuning_factor)

    cdef void wg_compute_dynamics(self, double rudder_angle, double time_step_size):
        '''
        Similar to function compute_dynamics but should be used only for a wave glider.
        The function computes the dynamics of the wave glider for the next time step,
        and also computes the wave thrust force generated by the underwater glider.
        :param float rudder_angle: Angle of the rudder with respect to X axis of the ASV. 
        Rudder angle must within (-PI/2, PI/2). Angle is positive when the vehicle has to turn 
        to starboard (ie. aft end of the rudder points to starboard side). 
        :param float time_step_size: Step size, in milliseconds, to increment the current time.
        '''
        wave_glider_compute_dynamics(self._c_object, rudder_angle, time_step_size)
        self.__check_error_throw_exception()
    
    def py_wg_compute_dynamics(self, double rudder_angle, double time_step_size):
        '''
        Similar to function compute_dynamics but should be used only for a wave glider.
        The function computes the dynamics of the wave glider for the next time step,
        and also computes the wave thrust force generated by the underwater glider.
        :param float rudder_angle: Angle of the rudder with respect to X axis of the ASV. 
        Rudder angle must within (-PI/2, PI/2). Angle is positive when the vehicle has to turn 
        to starboard (ie. aft end of the rudder points to starboard side). 
        :param float time_step_size: Step size, in milliseconds, to increment the current time.
        '''
        self.wg_compute_dynamics(rudder_angle, time_step_size)
        self.__check_error_throw_exception()

    cdef void wg_run(self, bint(*callback_precompute)(double*), void(*callback_postcompute)(), double time_step_size):
        '''
        Simulate the wave glider for multiple time steps. 
        :param callback_precompute: callback at the beginning of each time step. Can be used for setting rudder angle
        for the time step, modify the sea_surface for the time step, etc. callback_precompute takes as argument a pointer to the 
        rudder angle that is to be set. The return value of the callback is used to continue
        or exit from the simulation. If the callback returned false, simulation is terminated.
        :param callback_postcompute: callback at the end of each time step. Can be used for fetching/printing the results 
        of the time step.
        '''
        wave_glider_run(self._c_object, callback_precompute, callback_postcompute, time_step_size)
        self.__check_error_throw_exception()

    cdef Sea_surface* get_sea_surface(self):
        '''
        Get the sea surface initialised for the ASV.
        '''
        cdef Sea_surface *value = asv_get_sea_surface(self._c_object)
        self.__check_error_throw_exception()
        return value

    def py_get_sea_surface(self) -> py_Sea_surface:
        '''
        Get the sea surface initialised for the ASV.
        '''
        return self._sea_surface

    cdef Coordinates_3D get_position_cog(self):
        '''
        Get the position of the asv using the COG of the vehicle.
        '''
        cdef Coordinates_3D value = asv_get_position_cog(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_position_cog(self) -> py_Coordinates_3D:
        '''
        Get the position of the asv using the COG of the vehicle.
        '''
        position = py_Coordinates_3D()
        position._c_object = self.get_position_cog()
        return position

    cdef Coordinates_3D get_position_origin(self):
        '''
        Get the position of the asv using the origin of the vehicle.
        '''
        cdef Coordinates_3D value = asv_get_position_origin(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_position_origin(self) -> py_Coordinates_3D:
        '''
        Get the position of the asv using the origin of the vehicle.
        '''
        position = py_Coordinates_3D()
        position._c_object = self.get_position_origin()
        return position

    cdef Coordinates_3D get_attitude(self):
        '''
        Get the floating attitude of the asv.
        '''
        cdef Coordinates_3D value = asv_get_attitude(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_attitude(self) -> py_Coordinates_3D:
        '''
        Get the floating attitude of the asv.
        '''
        attitude = py_Coordinates_3D()
        attitude._c_object = self.get_attitude()
        return attitude

    cdef Rigid_body_DOF get_F(self):
        '''
        Get force.
        '''
        cdef Rigid_body_DOF value = asv_get_F(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_F(self) -> py_Rigid_body_DOF:
        '''
        Get force.
        '''
        F = py_Rigid_body_DOF()
        F._c_object = self.get_F()
        return F
    
    cdef Rigid_body_DOF get_A(self):
        '''
        Get acceleration.
        '''
        cdef Rigid_body_DOF value = asv_get_A(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_A(self) -> py_Rigid_body_DOF:
        '''
        Get acceleration.
        '''
        A = py_Rigid_body_DOF()
        A._c_object = self.get_A()
        return A

    cdef Rigid_body_DOF get_V(self):
        '''
        Get velocity.
        '''
        cdef Rigid_body_DOF value = asv_get_V(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_V(self) -> py_Rigid_body_DOF:
        '''
        Get velocity.
        '''
        V = py_Rigid_body_DOF()
        V._c_object = self.get_V()
        return V

    cdef Asv_specification get_spec(self):
        '''
        Get ASV specification.
        '''
        cdef Asv_specification value = asv_get_spec(self._c_object)
        self.__check_error_throw_exception()
        return value
    
    def py_get_spec(self) -> py_Asv_specification:
        '''
        Get ASV specification.
        '''
        spec = py_Asv_specification()
        spec._c_object = self.get_spec()
        return spec
