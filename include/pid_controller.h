#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "geometry.h"

struct PID_controller
{
  // Inputs
  // ------
  struct Point asv_position; // Current position of the ASV in the X-Y 
                             // coordinate space. struct Point is for 3D 
                             // coordinate space, ignore z and set it to 0.0m.
  struct Attitude asv_attitude; // Current roll, pitch and yaw angles of
                                // ASV. The angles are measured in radians.
  struct Point way_point; // Desired position.
  
  // Outputs
  // -------
  double thrust_fore_ps; // Thrust to be applied on the fore left propeller.
  double thrust_fore_sb; // Thrust to be applied on the fore right propeller.
  double thrust_aft_ps;  // Thrust to be applied on the aft left propeller.
  double thrust_aft_sb;  // Thrust to be applied on the aft right propeller.
  
  // Intermediate calculation variables
  // ----------------------------------
  
};

/**
 * Function to set the current position and attitude of the ASV.
 * @param controller for which the inputs are to be set.
 * @param position is the current position of the ASV. 
 * @param attitude is the current attitude of the ASV. Angles are in radians.
 */
void pid_controller_set_current_state(struct PID_controller* controller,
                                      struct Point position,
                                      struct Attitude attitude);

/**
 * Function to set the destination point for the ASV.
 * @param controller for which the way-point is to be set.
 * @param way_point desired destination point for the ASV.
 */
void pid_controller_set_way_point(struct PID_controller* controller,
                                  struct Point way_point);

/**
 * Function to calculate thruster forces on each of the four thrusters - 
 * fore_ps, fore_sb, aft_ps and aft_sb.
 */
void pid_controller_set_thrust(struct PID_controller* controller);

#endif