from geometry cimport Coordinates_3D, Rigid_body_DOF
from regular_wave cimport Regular_wave
from sea_surface cimport Sea_surface, py_Sea_surface

cdef extern from "asv.h":
    struct Asv_specification:
        double L_wl
        double B_wl
        double D
        double T
        double max_speed
        double disp
        double r_roll
        double r_pitch
        double r_yaw
        Coordinates_3D cog
    
    struct Thruster:
        pass
    
    struct Asv:
        pass
    
    Thruster* thruster_new(Coordinates_3D position)
    void thruster_delete(Thruster* thruster)
    char* thruster_get_error_msg(Thruster* thruster)
    void thruster_set_thrust(Thruster* thruster, Coordinates_3D orientation, double magnitude)
    Coordinates_3D thruster_get_position(Thruster* thruster)
    Asv* asv_new(Asv_specification specification, Sea_surface* sea_surface, Coordinates_3D position, Coordinates_3D attitude)
    void asv_delete(Asv* asv)
    char* asv_get_error_msg(Asv* asv)
    void asv_set_thrusters(Asv* asv, Thruster** thrusters, int count_thrusters)
    Thruster** asv_get_thrusters(Asv* asv)
    int asv_get_count_thrusters(Asv* asv)
    void asv_set_sea_state(Asv* asv, Sea_surface* sea_surface)
    void asv_set_ocean_current(Asv* asv, double zonal_velocity, double meridional_velocity)
    void asv_compute_dynamics(Asv* asv, double time_step_size)
    void wave_glider_set_thrust_tuning_factor(Asv* asv, double tuning_factor)
    void wave_glider_compute_dynamics(Asv* asv, double rudder_angle, double time_step_size)
    void wave_glider_run(Asv* asv, bint(*callback_precompute)(double*), void(*callback_postcompute)(), double time_step_size)
    Sea_surface* asv_get_sea_surface(Asv* asv)
    Coordinates_3D asv_get_position_cog(Asv* asv)
    Coordinates_3D asv_get_position_origin(Asv* asv)
    Coordinates_3D asv_get_attitude(Asv* asv)
    Rigid_body_DOF asv_get_F(Asv* asv)
    Rigid_body_DOF asv_get_A(Asv* asv)
    Rigid_body_DOF asv_get_V(Asv* asv)
    Asv_specification asv_get_spec(Asv* asv)

cdef class py_Asv_specification:
    cdef Asv_specification _c_object

cdef class py_Thruster:
    cdef Thruster* _c_object
    cdef void __check_error_throw_exception(self)
    cdef void set_thrust(self, Coordinates_3D orientation, double magnitude)
    cdef Coordinates_3D get_position(self)

cdef class py_Asv:
    cdef Asv* _c_object
    cdef py_Sea_surface _sea_surface
    cdef list _thrusters
    cdef void __check_error_throw_exception(self)
    cdef void set_thrusters(self, Thruster **thrusters, int count_thrusters)
    cdef Thruster** get_thrusters(self)
    cdef int get_count_thrusters(self)
    cdef void set_sea_state(self, Sea_surface* sea_surface)
    cdef void set_ocean_current(self, double zonal_velocity, double meridional_velocity)
    cdef void compute_dynamics(self, double time_step_size)
    cdef void wg_set_thrust_tuning_factor(self, double tuning_factor)
    cdef void wg_compute_dynamics(self, double rudder_angle, double time_step_size)
    cdef void wg_run(self, bint(*callback_precompute)(double*), void(*callback_postcompute)(), double time_step_size)
    cdef Sea_surface* get_sea_surface(self)
    cdef Coordinates_3D get_position_cog(self)
    cdef Coordinates_3D get_position_origin(self)
    cdef Coordinates_3D get_attitude(self)
    cdef Rigid_body_DOF get_F(self)
    cdef Rigid_body_DOF get_A(self)
    cdef Rigid_body_DOF get_V(self)
    cdef Asv_specification get_spec(self)


# NOTE:
# 1. Go back to all places and check if I am returning Coordinates_3D instead of py_Coordinates_3D
# 2. Check if I have mentioned the return type for all python methods.else
# A good example is 
# def py_get_position(self) -> py_Coordinates_3D:
#     cdef py_Coordinates_3D positon(0,0,0)
#     positon._c_object = self.get_position()
#     return position
# 3. Compare all methods especially the distructors with my implementation in ctypes