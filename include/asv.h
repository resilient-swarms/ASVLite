#ifndef ASV_H
#define ASV_H

#include "geometry.h"

struct Wave;

/**
 * Struct to hold specification of the vehicle. 
 * Coordinate system used is body-centric frame. 
 * The origin of the frame is on the waterplane at the aft-centreline.
 */
struct Asv_specification
{
  double L_wl;              //!< Length at waterline in m.
  double B_wl;              //!< Breadth at waterline in m.
  double D;                 //!< Depth of the ASV in m.
  double T;                 //!< Draught of the ASV in m.
  double max_speed;         //!< Maximum operational speed of the ASV in m/s.
  double disp;              //!< Displacement, in m3, at draught T.
  double r_roll;            //!< Roll radius of gyration in m.
  double r_pitch;           //!< Pitch radius of gyration in m.
  double r_yaw;             //!< Yaw radius of gyration in m.
  union Coordinates_3D cog; //!< Centre of gravity in body-fixed frame. Coordinates in meter.
};

/**
 * An instance of Thruster should only be created by calling the function thruster_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to thruster_new() should be paired with a call to thruster_delete() 
 * to avoid memory leaks. 
 * 
 * All functions operating on an instance of a Thruster have a mechanism to notify of 
 * exceptions. All instances of Thruster have a member variable that holds a pointer 
 * to an error message. When there are no errors, the pointer is set to null. If 
 * an error occurs in a call to a function that takes an instance of Thruster, an error 
 * message is set within the instance. The error message can be fetched using the 
 * function thruster_get_error_msg(). The expected usage is to pair all function calls 
 * that take an instance of Thruster with a call to thruster_get_error_msg() and check for 
 * a null pointer. If a null pointer is returned, there is no error; otherwise, an 
 * error has occurred. Any subsequent calls to other functions that take an instance 
 * of Thruster will reset the last know error message. 
 */
struct Thruster;

/**
 * An instance of Asv should only be created by calling the function asv_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to asv_new() should be paired with a call to asv_delete() 
 * to avoid memory leaks. 
 * 
 * All functions operating on an instance of a Asv have a mechanism to notify of 
 * exceptions. All instances of Asv have a member variable that holds a pointer 
 * to an error message. When there are no errors, the pointer is set to null. If 
 * an error occurs in a call to a function that takes an instance of Asv, an error 
 * message is set within the instance. The error message can be fetched using the 
 * function asv_get_error_msg(). The expected usage is to pair all function calls 
 * that take an instance of Asv with a call to asv_get_error_msg() and check for 
 * a null pointer. If a null pointer is returned, there is no error; otherwise, an 
 * error has occurred. Any subsequent calls to other functions that take an instance 
 * of Asv will reset the last know error message. 
 */
struct Asv;

/**
 * Create and initialise a Thruster.
 * @param position of the thruster in ASV's body-fixed frame.
 * @return pointer to the initialised object if the operation was successful; else, returns a null pointer.
 */
struct Thruster* thruster_new(const union Coordinates_3D position);

/**
 * Free memory allocated for the thruster.
 * @param thruster is a non-null pointer to an instance of Thruster to be deallocated.
 */
void thruster_delete(struct Thruster* thruster);

/**
 * Returns error message related to the last function called for the instance of Thruster.
 * @return pointer to the error msg, if any, else returns a null pointer. 
 */
const char* thruster_get_error_msg(const struct Thruster* thruster);

/**
 * Set the orientation and magnitude of thrust vector. 
 * @param orientation of the thrust vector, in radians, in body-fixed frame.
 * @param magnitude of the thrust in N.
 */
void thruster_set_thrust(struct Thruster* thruster, 
                         const union Coordinates_3D orientation, 
                         double magnitude); 

/**
 * Create and initialise an asv.
 * @param specification of the ASV. 
 * @param wave is the irregular sea surface for the asv. 
 * @param position of the asv on the sea surface. 
 * @param attitude of the asv.
 * @return pointer to the initialised object if the operation was successful; else, returns a null pointer.
 */
struct Asv* asv_new(const struct Asv_specification specification, 
                    const struct Wave* wave, 
                    union Coordinates_3D position, 
                    union Coordinates_3D attitude);

/**
 * Free memory allocated for the asv.
 * @param asv is a non-null pointer to an instance of Asv to be deallocated.
 */
void asv_delete(struct Asv* asv);

/**
 * Returns error message related to the last function called for the instance of Asv.
 * @return pointer to the error msg, if any, else returns a null pointer. 
 */
const char* asv_get_error_msg(const struct Asv* asv);

/**
 * Set the thrusters for the asv.
 * @param thrusters array of thrusters for the asv.
 * @param cout_thrusters is the size of thrusters array.
 */
void asv_set_thrusters(struct Asv* asv, 
                       struct Thruster** thrusters, 
                       int cout_thrusters);

/**
 * Get the array of pointers to the thrusters.
 * @return array of pointers to thrusters.
 */
struct Thruster** asv_get_thrusters(struct Asv* asv);

/**
 * Get the number of thruster for the asv. 
 * @return number of thrusters attached to the asv.
 */
int asv_get_count_thrusters(struct Asv* asv);

/**
 * Function to modify the current sea state to a new sea state. If operation was unsuccessful then the 
 * new wave is rejected and the asv instance retains the pointer to its existing wave. Since the asv
 * may retain the pointer to the old wave if error occurred, check for error by calling asv_get_error_msg() before
 * calling wave_delete() on the replaced instance of Wave.  
 * @param wave is a non-null pointer for the new instance of irregular wave. 
 * If memory is to  be cleaned, call wave_delete() on the pointer to the old irregular wave instance.   
 */
void asv_set_sea_state(struct Asv* asv, const struct Wave* wave);

/**
 * Function to compute dynamics of the ASV by incrementing time.
 * @param time_step_size in milliseconds to increment the current time.
 */
void asv_compute_dynamics(struct Asv* asv, double time_step_size);

/**
 * Similar to function asv_compute_dynamics but should be used only for a wave glider. 
 * The function computes the dynamics of the wave glider for the next time step, 
 * and also computes the wave thrust force generated by the underwater glider.
 * @param rudder_angle is the angle of the rudder with respect to X axis of the ASV. 
 * Rudder angle must within (-PI/2, PI/2). Angle is positive when the vehicle has to turn 
 * to starboard (ie. aft end of the rudder points to starboard side). 
 * @param time_step_size in milliseconds to increment the current time.
 */
void wave_glider_compute_dynamics(struct Asv* asv, 
                                  double rudder_angle, 
                                  double time_step_size);

/**
 * Get the position of the asv using the COG of the vehicle. 
 * @return position of the asv with coordinates in meter.
 */
union Coordinates_3D asv_get_position_cog(struct Asv* asv);

/**
 * Get the position of the asv using the origin of the body reference frame of the vehicle. 
 * @return position of the asv with coordinates in meter.
 */
union Coordinates_3D asv_get_position_origin(struct Asv* asv);

/**
 * Get the floating attitude of the asv. 
 * @return attitude of the asv with angles in radians.
 */
union Coordinates_3D asv_get_attitude(struct Asv* asv);

/**
 * Get force.
 * @return force, in N, along each DOF. 
 */
union Rigid_body_DOF asv_get_F(struct Asv* asv);

/**
 * Get acceleration. 
 * @return acceleration, in m/s2, along each DOF.
 */
union Rigid_body_DOF asv_get_A(struct Asv* asv);

/**
 * Get velocity. 
 * @return velocity, in m/s, along each DOF.
 */
union Rigid_body_DOF asv_get_V(struct Asv* asv);

/**
 * Get asv specification.
 */
struct Asv_specification asv_get_spec(struct Asv* asv);

#endif // ASV_H
