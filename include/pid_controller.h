#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "geometry.h"

struct PID_controller
{
  // Inputs
  // ------
  union Coordinates_3D asv_position; // Current position of the ASV in the X-Y 
                                     // coordinate space. struct Point is for 3D 
                                     // coordinate space, ignore z and set it to 0.0m.
  union Coordinates_3D asv_attitude; // Current roll, pitch and yaw angles of
                                     // ASV. The angles are measured in radians.
  union Coordinates_3D way_point;    // Desired position.
  double kp_heading;
  double ki_heading;
  double kd_heading;
  double kp_position;
  double ki_position;
  double kd_position;
  
  // Outputs
  // -------
  double thrust_fore_ps; // Thrust to be applied on the fore left propeller.
  double thrust_fore_sb; // Thrust to be applied on the fore right propeller.
  double thrust_aft_ps;  // Thrust to be applied on the aft left propeller.
  double thrust_aft_sb;  // Thrust to be applied on the aft right propeller.
  
  // Intermediate calculation variables
  // ----------------------------------
  double error_heading;
  double error_int_heading;
  double error_diff_heading;
  double error_position;
  double error_int_position;
  double error_diff_position;
};

/**
 * Function to initialise the member variables of struct controller.
 * @param controller to be initialised.
 * @return pointer to the initialised object if the operation was successful; else, returns a null pointer.
 */
struct PID_controller* pid_controller_new();

/**
 * Free memory allocated for the asv.
 * @param controller is a non-null pointer to an instance of PID_controller to be deallocated.
 */
void pid_controller_delete(struct PID_controller* controller);

/**
 * Function to set the gain terms for position.
 */
void pid_controller_set_gains_position(struct PID_controller* controller, 
                                      double p, double i, double d);

/**
 * Function to set the gain terms for heading.
 */
void pid_controller_set_gains_heading(struct PID_controller* controller, 
                                     double p, double i, double d);

/**
 * Function to set the current position and attitude of the ASV.
 * @param controller for which the inputs are to be set.
 * @param position is the current position of the ASV. 
 * @param attitude is the current attitude of the ASV. Angles are in radians.
 */
void pid_controller_set_current_state(struct PID_controller* controller,
                                      union Coordinates_3D position,
                                      union Coordinates_3D attitude);

/**
 * Function to set the destination point for the ASV.
 * @param controller for which the way-point is to be set.
 * @param way_point desired destination point for the ASV.
 */
void pid_controller_set_way_point(struct PID_controller* controller,
                                  union Coordinates_3D way_point);

/**
 * Function to calculate thruster forces on each of the four thrusters - 
 * fore_ps, fore_sb, aft_ps and aft_sb.
 */
void pid_controller_set_thrust(struct PID_controller* controller);

#endif
