#include <stdio.h>
#include <stdlib.h>
#include "pid_controller.h"
#include "simulation.h"
#include "asv.h"
#include "constants.h"
#include "errors.h"

struct Controller
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
  char* error_msg;
};

struct Controller* controller_new(struct Asv* asv)
{
  struct Controller* controller = (struct Controller*)malloc(sizeof(struct Controller));
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

void controller_delete(struct Controller* controller)
{
  if(controller)
  {
    free(controller->error_msg);
    free(controller);
    controller = NULL;
  }
}

const char* controller_get_error_msg(const struct Controller* controller)
{
  if(controller)
  {
    return controller->error_msg;
  }
  return NULL;
}

void controller_set_gains_position(struct Controller* controller, 
                                      double p, double i, double d)
{
  clear_error_msg(controller->error_msg);
  if(controller)
  {
    controller->kp_position = p;
    controller->ki_position = i;
    controller->kd_position = d;
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}

void controller_set_gains_heading(struct Controller* controller, 
                                      double p, double i, double d)
{
  clear_error_msg(controller->error_msg);
  if(controller)
  {
    controller->kp_heading = p;
    controller->ki_heading = i;
    controller->kd_heading = d;
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}

void controller_set_thrust(struct Controller* controller, union Coordinates_3D way_point)
{
  clear_error_msg(controller->error_msg);
  if(controller)
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
                                            // the waypoint w.r.t vehicle. Clamp the position error so that 
                                            // it is in similar magnitude to that of heading error.
  
    // Calculate the integral error for position.
    double gamma_position_error = 0.9; // Should be in the range (0,1).
                                      // Value = 1, implies the past error is never forgotten.
                                      // Value = 0, implies the past error is always ignored. 
                                      // Value between 0 and 1 implies the past errors gradually decreases. 
                                      // Value > 0 implies past errors are magnified. 
    controller->error_int_position = error_position + gamma_position_error * controller->error_int_position;
    
    // Calculate the differential error for position.
    controller->error_diff_position = error_position - controller->error_position;
    controller->error_position = error_position;
    
    // Calculate the heading error in radian.
    // Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
    double m1 = (p2.keys.y == p1.keys.y)? __DBL_MAX__ : (p2.keys.x - p1.keys.x)/(p2.keys.y - p1.keys.y);
    double m2 = (p3.keys.y == p1.keys.y)? __DBL_MAX__ : (p3.keys.x - p1.keys.x)/(p3.keys.y - p1.keys.y);
    double error_heading = atan((m2-m1)/(1+ m1*m2)); // radians
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
    double gamma_heading_error = 0.9; // Should be in the range (0,1).
                                      // Value = 1, implies the past error is never forgotten.
                                      // Value = 0, implies the past error is always ignored. 
                                      // Value between 0 and 1 implies the past errors gradually decreases. 
                                      // Value > 0 implies past errors are magnified.  
    controller->error_int_heading = error_heading + gamma_heading_error * controller->error_int_heading;
    
    // Calculate the differential heading error.
    controller->error_diff_heading = error_heading - controller->error_heading;
    controller->error_heading = error_heading; 
  
    // Calculate thruster thrust.
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
    
    double thrust_ps = position_thrust + heading_thrust; // left side thrust
    double thrust_sb = position_thrust - heading_thrust; // right side thrust

    double max_value = (fabs(thrust_ps) > fabs(thrust_sb))? fabs(thrust_ps) : fabs(thrust_sb);
    if(max_value > max_thrust)
    {
      double ratio = max_thrust / max_value;
      thrust_ps = thrust_ps * ratio;
      thrust_sb = thrust_sb * ratio;
    } 

    // Set thruster thrust on each of the 4 thrusters.
    union Coordinates_3D orientation_fore_thrusters = {0.0, PI, 0.0};
    union Coordinates_3D orientation_aft_thrusters  = {0.0, 0.0, 0.0};
    struct Thruster** thrusters = asv_get_thrusters(controller->asv);
    // Thruster configuration
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
      thruster_set_thrust(thrusters[2], orientation_aft_thrusters, thrust_ps);
      thruster_set_thrust(thrusters[0], orientation_fore_thrusters, 0.0);
    }
    else
    {
      thruster_set_thrust(thrusters[2], orientation_aft_thrusters, 0.0);
      thruster_set_thrust(thrusters[0], orientation_fore_thrusters, thrust_ps);
    }
    if(thrust_sb >= 0.0)
    {
      thruster_set_thrust(thrusters[3], orientation_aft_thrusters, thrust_sb);
      thruster_set_thrust(thrusters[1], orientation_fore_thrusters, 0.0);
    }
    else
    {
      thruster_set_thrust(thrusters[3], orientation_aft_thrusters, 0.0);
      thruster_set_thrust(thrusters[1], orientation_fore_thrusters, thrust_sb);
    }
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}

void controller_tune(struct Controller* controller)
{
  clear_error_msg(controller->error_msg);
  if(controller)
  {
    // Initial values for gain terms
    controller->kp_position = 1.0;
    controller->ki_position = 1.0;
    controller->kd_position = 1.0;
    controller->kp_heading  = 1.0;
    controller->ki_heading  = 1.0;
    controller->kd_heading  = 1.0;
    int max_count_iterations = 20;
    double max_significant_wave_height = 5.0; // m
    for(int i=0; i < max_count_iterations; ++i)
    {
      for(double significant_wave_height = 1.0; significant_wave_height < max_significant_wave_height; significant_wave_height += 1.0)
      {
        for(double asv_heading = 0.0; asv_heading < 2.0*PI; asv_heading += PI/4.0)
        {
          // Create wave

          // Create Asv

          // Append the asv to the array
        }
      }
    }
    // Create simulation using array of asvs

    // Run simulation for a fixed time. 

    // Compute the next gain terms.
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}