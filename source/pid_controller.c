#include <stdio.h>
#include <stdlib.h>
#include "pid_controller.h"
#include "constants.h"
#include "asv.h"

struct PID_controller
{
  // Inputs
  // ------
  struct Asv* asv;
  double kp_heading;
  double ki_heading;
  double kd_heading;
  double kp_position;
  double ki_position;
  double kd_position;
   
  // Intermediate calculation variables
  // ----------------------------------
  double error_heading;
  double error_int_heading;
  double error_diff_heading;
  double error_position;
  double error_int_position;
  double error_diff_position;
};

struct PID_controller* pid_controller_new(struct Asv* asv)
{
  struct PID_controller* controller = (struct PID_controller*)malloc(sizeof(struct PID_controller));
  if(controller)
  {
    controller->asv                 = asv;
    controller->kp_heading          = 0.0;
    controller->ki_heading          = 0.0;
    controller->kd_heading          = 0.0;
    controller->kp_position         = 0.0;
    controller->ki_position         = 0.0;
    controller->kd_position         = 0.0;
    controller->error_heading       = 0.0;
    controller->error_int_heading   = 0.0;
    controller->error_diff_heading  = 0.0;
    controller->error_position      = 0.0;
    controller->error_int_position  = 0.0;
    controller->error_diff_position = 0.0;
  }
  return controller;
}

void pid_controller_delete(struct PID_controller* controller)
{
  free(controller);
  controller = NULL;
}

void pid_controller_set_gains_position(struct PID_controller* controller, 
                                      double p, double i, double d)
{
  controller->kp_position = p;
  controller->ki_position = i;
  controller->kd_position = d;
}

void pid_controller_set_gains_heading(struct PID_controller* controller, 
                                      double p, double i, double d)
{
  controller->kp_heading = p;
  controller->ki_heading = i;
  controller->kd_heading = d;
}

void pid_controller_set_thrust(struct PID_controller* controller, union Coordinates_3D way_point)
{
  // Calculate the heading required in radian.
  union Coordinates_3D p1 = asv_get_position_origin(controller->asv);
  union Coordinates_3D p2 = asv_get_position_cog(controller->asv);
  union Coordinates_3D p3 = way_point;

  const double limit_error_magnitude = PI; 
 
  double error_position = sqrt(pow(p3.keys.x - p1.keys.x, 2.0) + pow(p3.keys.y - p1.keys.y, 2.0));
  error_position = (error_position > limit_error_magnitude)? limit_error_magnitude : error_position; 
                                          // The heading error is always in the range (-PI, PI).
                                          // But the position error has no limits. It could be 
                                          // in the range (-Inf, Inf) depending on the position of
                                          // the w.r.t vehicle. Clamp the position error so that 
                                          // it is in similar magnitude to that of heading error.
 
  // Calculate the integral error for position.
  controller->error_int_position = error_position + 0.9 * controller->error_int_position;
  
  // Calculate the differential error for position.
  controller->error_diff_position = error_position - controller->error_position;
  controller->error_position = error_position;
   
  // Calculate the heading error in radian.
  // Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
  double m1 = (p2.keys.y == p1.keys.y)? __DBL_MAX__ : (p2.keys.x - p1.keys.x)/(p2.keys.y - p1.keys.y);
  double m2 = (p3.keys.y == p1.keys.y)? __DBL_MAX__ : (p3.keys.x - p1.keys.x)/(p3.keys.y - p1.keys.y);
  double error_heading = atan((m2-m1)/(1+ m1*m2));
  // Correction for angles in 3rd and 4th quadrants.
  if(p3.keys.x<p1.keys.x && p3.keys.y<p1.keys.y)
  {
    error_heading = -PI + error_heading;
  }
  if(p3.keys.x>=p1.keys.x && p3.keys.y<p1.keys.y)
  {
    error_heading = PI + error_heading;
  }
  // *****
  // Uncomment below code if using a limit_error_magnitude < PI
  // *****
  // Limit heading errors.
  // if(error_heading > limit_error_magnitude)
  // {
  //   error_heading = limit_error_magnitude;
  // }
  // if(error_heading < limit_error_magnitude)
  // {
  //   error_heading = -limit_error_magnitude;
  // }
  
  // Calculate the integral heading error.
  double past_retention_rate = 0.5; // Should be in the range (0,1).
                                    // Value = 1, implies the past error is never forgotten.
                                    // Value = 0, implies the past error is always ignored. 
                                    // Value between 0 and 1 implies the past errors gradually decreases. 
                                    // Value > 0 implies past errors are magnified.  
  controller->error_int_heading = error_heading + past_retention_rate * controller->error_int_heading;
  
  // Calculate the differential heading error.
  controller->error_diff_heading = error_heading - controller->error_heading;
  controller->error_heading = error_heading; 
 
  // Calculate propeller thrust.
  double max_thrust = 5.0; // SMARTY platform thruster has a maximum capacity of 5N. 

  double heading_thrust = 
    controller->kp_heading * controller->error_heading      + 
    controller->ki_heading * controller->error_int_heading  + 
    controller->kd_heading * controller->error_diff_heading; 
  // Do not use more than 20% of thruster capacity for heading correction.
  // heading_thrust = (fabs(heading_thrust) > 0.2 * max_thrust)? max_thrust * 0.2 : heading_thrust;

  double position_thrust= 
    controller->kp_position * controller->error_position      +
    controller->ki_position * controller->error_int_position  + 
    controller->kd_position * controller->error_diff_position;
  
  double thrust_ps = 0.0; // left side thrust
  double thrust_sb = 0.0; // right side thrust
  if(fabs(error_heading) > PI/2.0)
  {
    // A large turn. Focus on turning instead of moving forward.
    thrust_ps = +heading_thrust;
    thrust_sb = -heading_thrust;
  }
  else
  {
    thrust_ps = position_thrust + heading_thrust; 
    thrust_sb = position_thrust - heading_thrust; 
  }

  double max_value = (fabs(thrust_ps) > fabs(thrust_sb))? fabs(thrust_ps) : fabs(thrust_sb);
  if(max_value > max_thrust)
  {
    double ratio = max_thrust / max_value;
    thrust_ps = thrust_ps * ratio;
    thrust_sb = thrust_sb * ratio;
  } 

  // Set propeller thrust on each of the 4 propellers.
  union Coordinates_3D orientation_fore_thrusters = {0.0, PI, 0.0};
  union Coordinates_3D orientation_aft_thrusters  = {0.0, 0.0, 0.0};
  struct Propellers** propellers = asv_get_propellers(controller->asv);
  // Propeller configuration
  //
  // Thust direction is towards aft
  //  |              |
  //  v              v
  //
  // Fore PS        Fore SB
  // [0] -----------[1] 
  //  +              +
  //  |              |
  //  |              |
  //  +              +
  // [2] -----------[3] 
  // Aft PS         Aft SB
  // 
  //  ^              ^
  //  |              |
  // Thrust direction is towards fore.     
  //    
  if(thrust_ps >= 0.0)
  {
    propeller_set_thrust(propellers[2], orientation_aft_thrusters, thrust_ps);
    propeller_set_thrust(propellers[0], orientation_fore_thrusters, 0.0);
  }
  else
  {
    propeller_set_thrust(propellers[2], orientation_aft_thrusters, 0.0);
    propeller_set_thrust(propellers[0], orientation_fore_thrusters, thrust_ps);
  }
  if(thrust_sb >= 0.0)
  {
    propeller_set_thrust(propellers[3], orientation_aft_thrusters, thrust_sb);
    propeller_set_thrust(propellers[1], orientation_fore_thrusters, 0.0);
  }
  else
  {
    propeller_set_thrust(propellers[3], orientation_aft_thrusters, 0.0);
    propeller_set_thrust(propellers[1], orientation_fore_thrusters, thrust_sb);
  }
}

// TODO:
// Tune the PID controllers. 
// The two controllers should be tuned separately. 
// The heading controller should be tuned with a scenario where is turns on the spot and correct heading. 
// The heading controller should also learn to stop turning after reaching the desired heading. 
// The position controller should be tuned with a scenario where is moves head on to a waypoint and stop on reaching the waypoint. 