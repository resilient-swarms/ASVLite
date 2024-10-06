#ifndef ASV_H
#define ASV_H

#include "stdbool.h"
#include "geometry.h"

struct Sea_surface;

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
 * @file
 * An instance of Thruster or Asv should only be created by calling the function thruster_new() or Asv_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to Thruster_new() and Asv_new() should be paired with a call to Thruster_delete() and Asv_delete() 
 * to avoid memory leaks. 
 * 
 * All functions operating on an instance of a Thruster or Asv have a mechanism to notify of 
 * exceptions. All instances of Thruster and Asv have a member variable that holds a pointer 
 * to an error message. When there are no errors, the pointer is set to null. If 
 * an error occurs in a call to a function that takes an instance of the struct, an error 
 * message is set within the instance. The error message can be fetched using the 
 * function thruster_get_error_msg() or asv_get_error_msg(). The expected usage is to pair all function calls 
 * that take an instance of Thruster or Asv with a call to the corresponding _get_error_msg() and check for 
 * a null pointer. If a null pointer is returned, there is no error; otherwise, an 
 * error has occurred. Any subsequent calls to other functions that take an instance 
 * of Thruster will reset the last know error message in Thruster. Similarly, any subsequent 
 * calls to other functions that take an instance of Asv will reset the last know error message in the Asv. 
 */
struct Thruster;
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
 * Get the position of the thruster. 
 * @return position of the thruster in body-fixed frame of the ASV.
 */
const union Coordinates_3D thruster_get_position(struct Thruster* thruster);

/**
 * Create and initialise an asv.
 * @param specification of the ASV. 
 * @param sea_surface is the irregular sea surface for the asv. 
 * @param position of the asv on the sea surface. 
 * @param attitude of the asv.
 * @return pointer to the initialised object if the operation was successful; else, returns a null pointer.
 */
struct Asv* asv_new(const struct Asv_specification specification, 
                    const struct Sea_surface* sea_surface, 
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
 * @param count_thrusters is the size of thrusters array.
 */
void asv_set_thrusters(struct Asv* asv, 
                       struct Thruster** thrusters, 
                       int count_thrusters);

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
 * new sea_surface is rejected and the asv instance retains the pointer to its existing sea_surface. Since the asv
 * may retain the pointer to the old sea_surface if error occurred, check for error by calling asv_get_error_msg() before
 * calling sea_surface_delete() on the replaced instance of Sea_surface.  
 * @param sea_surface is a non-null pointer for the new instance of irregular sea_surface. 
 * If memory is to  be cleaned, call sea_surface_delete() on the pointer to the old irregular sea_surface instance.   
 */
void asv_set_sea_state(struct Asv* asv, const struct Sea_surface* sea_surface);

void asv_set_ocean_current(struct Asv* asv, double zonal_velocity, double meridional_velocity);

/**
 * Function to compute dynamics of the ASV by incrementing time.
 * @param time_step_size in milliseconds to increment the current time.
 */
void asv_compute_dynamics(struct Asv* asv, double time_step_size);

/**
 * Set to true to halt surge and sway motions. All the remainig 4 dof are not ignored.
*/
void asv_set_surge_sway_halt(struct Asv* asv, bool status);

/**
 * Overwrite the default factor for scaling the thrust computed. The default
 * factor is 1.
 */
void wave_glider_set_thrust_tuning_factor(struct Asv* asv, double tuning_factor);

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
 * Simulate the wave glider for multiple time steps. 
 * @param callback_precompute callback at the beginning of each time step. Can be used for setting rudder angle
 * for the time step, modify the sea_surface for the time step, etc. callback_precompute takes as argument a pointer to the 
 * rudder angle that is to be set. The return value of the callback is used to continue
 * or exit from the simulation. If the callback returned false, simulation is terminated. 
 * @param callback_postcompute callback at the end of each time step. Can be used for fetching/printing the results 
 * of the time step. 
 */
void wave_glider_run(struct Asv* asv, int(*callback_precompute)(double*), void(*callback_postcompute)(void), double time_step_size);

/**
 * Get the sea state initialised for the asv.
 */
struct Sea_surface* asv_get_sea_surface(struct Asv* asv); 

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
