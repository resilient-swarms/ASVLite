#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "geometry.h"

/**
 * @file
 * An instance of Controller should only be created by calling the function controller_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to controller_new() should be paired with a call to controller_delete() 
 * to avoid memory leaks. 
 * 
 * All functions operating on an instance of a Controller have a mechanism to notify of 
 * exceptions. All instances of Controller have a member variable that holds a pointer 
 * to an error message. When there are no errors, the pointer is set to null. If 
 * an error occurs in a call to a function that takes an instance of Controller, an error 
 * message is set within the instance. The error message can be fetched using the 
 * function controller_get_error_msg(). The expected usage is to pair all function calls 
 * that take an instance of Controller with a call to controller_get_error_msg() and check for 
 * a null pointer. If a null pointer is returned, there is no error; otherwise, an 
 * error has occurred. Any subsequent calls to other functions that take an instance 
 * of Controller will reset the last know error message. 
 */
struct Controller;
struct Asv;

/**
 * Function to initialise the member variables of struct controller.
 * @param controller to be initialised.
 * @return pointer to the initialised object if the operation was successful; else, returns a null pointer.
 */
struct Controller* controller_new(struct Asv* asv);

/**
 * Free memory allocated for the asv.
 * @param controller is a non-null pointer to an instance of Controller to be deallocated.
 */
void controller_delete(struct Controller* controller);

/**
 * Returns error message related to the last function called for the instance of Controller.
 * @return pointer to the error msg, if any, else returns a null pointer. 
 */
const char* controller_get_error_msg(const struct Controller* controller);

/**
 * Tunes the controller and sets optimal values for position and heading gain terms. 
 * Results from tunning iterations are written to a file ./tunning. 
 */
void controller_tune(struct Controller* controller);

/**
 * Function to set the gain terms for position.
 */
void controller_set_gains_position(struct Controller* controller, 
                                      double p, double i, double d);

/**
 * Function to set the gain terms for heading.
 */
void controller_set_gains_heading(struct Controller* controller, 
                                     double p, double i, double d);

/**
 * Function to calculate thruster forces on each of the four thrusters - 
 * fore_ps, fore_sb, aft_ps and aft_sb, and set the thrust on the asv's thrusters. 
 */
void controller_set_thrust(struct Controller* controller, union Coordinates_3D way_point);

/**
 * Get the gain terms for position.  
 * @return gain terms returned as a coordinate, where x = p, y = i, z = d.
 */
union Coordinates_3D controller_get_gains_position(struct Controller* controller);

/**
 * Get the gain terms for heading.  
 * @return gain terms returned as a coordinate, where x = p, y = i, z = d.
 */
union Coordinates_3D controller_get_gains_heading(struct Controller* controller);

#endif
