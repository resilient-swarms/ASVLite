cdef extern from "geometry.h":
    int COUNT_COORDINATES
    int COUNT_DOF

    struct Coordinate_keys:
        double x
        double y
        double z
    
    union Coordinates_3D:
        Coordinate_keys keys 
        double *array
    
    struct Rigid_body_DOF_keys:
        double surge
        double sway
        double heave
        double roll
        double pitch
        double yaw
    
    union Rigid_body_DOF:
        Rigid_body_DOF_keys keys 
        double *array 
    
    double normalise_angle_PI(double angle)
    double normalise_angle_2PI(double angle)


cdef class py_Coordinates_3D:
    cdef Coordinates_3D _c_object
    cdef int __i # Index for iterator

cdef class py_Rigid_body_DOF:
    cdef Rigid_body_DOF _c_object
    cdef int __i # Index for iterator