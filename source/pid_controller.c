#include "pid_controller.h"
#include "constants.h"

void pid_controller_init(struct PID_controller* controller)
{
  controller->error_heading          = 0.0;
  controller->error_int_heading      = 0.0;
  controller->error_diff_heading     = 0.0;
  controller->error_position         = 0.0;
  controller->error_int_position     = 0.0;
  controller->error_diff_position    = 0.0;
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

void pid_controller_set_current_state(struct PID_controller* controller,
                                      struct Dimensions position,
                                      struct Dimensions attitude)
{
  controller->asv_position.x = position.x;
  controller->asv_position.y = position.y;
  controller->asv_position.z = position.z;
  controller->asv_attitude.x = attitude.x;
  controller->asv_attitude.y = attitude.y;
  controller->asv_attitude.z = attitude.z;  
}

void pid_controller_set_way_point(struct PID_controller* controller,
                                  struct Dimensions way_point)
{
  controller->way_point.x = way_point.x;
  controller->way_point.y = way_point.y;
  controller->way_point.z = way_point.z;
}

void pid_controller_set_thrust(struct PID_controller* controller)
{
  // Calculate the heading required in radian.
  double x1 = controller->asv_position.x;
  double y1 = controller->asv_position.y;
  double x2 = controller->way_point.x;
  double y2 = controller->way_point.y;
 
  // Calculate position error - distance to way-point from current position.
  double max_error_position = 500.0; // Set the max position error to so that
                                     // propeller can operate full throttle 
                                     // if possible.
  double error_position = sqrt(pow(x2-x1, 2.0) + pow(y2-y1, 2.0));

  // Clamp the position error.
  // The error is always going to be positive if measured as distance but still
  // to make the code future proof, the negative magnitude is also considered.
  if(fabs(error_position) > max_error_position)
  {
    error_position = (error_position > 0.0)? 
                      max_error_position : -max_error_position;
  }
  
  // Calculate the integral error for position.
  controller->error_int_position += error_position;
  
  // Clamp the integral error for position.
  if(fabs(controller->error_int_position) > max_error_position)
  {
    controller->error_int_position = (controller->error_int_position > 0.0)?
                                      max_error_position : -max_error_position;
  }
  
  // Calculate the differential error for position.
  controller->error_diff_position = error_position - controller->error_position;
  controller->error_position = error_position;
  
  // Clamp the differential error for position.
  if(fabs(controller->error_diff_position) > max_error_position)
  {
    controller->error_diff_position = (controller->error_diff_position > 0.0)?
                                       max_error_position : -max_error_position;
  }
  
  // Calculate the heading error in radian.
  double heading_required = atan((x2 - x1)/(y2 - y1));
  // atan() gives values in the range of -PI/2 to PI/2.
  // correct the angle if it is in the 3rd of 4th quadrants. 
  if((x2-x1) < 0.0 && (y2-y1) < 0.0)
  {
    // 3rd quadrant
    heading_required += -PI;
  } 
  else if((x2-x1) > 0.0 && (y2-y1) < 0.0)
  {
    // 4th quadrant
    heading_required += PI;
  }
  
  double error_heading = heading_required - controller->asv_attitude.z;
  // Clamp the heading error
  double max_error_heading = PI/6.0; // Set the max heading error.
   if(fabs(error_heading) > max_error_heading)
  {
    error_heading = (error_heading > 0.0)? 
                     max_error_heading : -max_error_heading;
  }
  
  // Calculate the integral heading error.
  controller->error_int_heading += error_heading;
  
  // Clamp the integral heading error.
  if(fabs(controller->error_int_heading) > max_error_heading)
  {
    controller->error_int_heading = (controller->error_int_heading > 0.0)?
                                    max_error_heading : -max_error_heading;
  }
  
  // Calculate the differential heading error.
  controller->error_diff_heading = error_heading - controller->error_heading;
  controller->error_heading = error_heading; 
  
  // Clamp the differential heading error.
  if(fabs(controller->error_diff_heading) > max_error_heading)
  {
    controller->error_diff_heading = (controller->error_diff_heading > 0.0)?
                                      max_error_heading : -max_error_heading;
  }
 
  // Calculate propeller thrust.
  double max_thrust = 1.0; // SMARTY platform thruster has a maximum 
                           // capacity of 5N. 

  double heading_thrust = 
    controller->kp_heading * controller->error_heading      + 
    controller->ki_heading * controller->error_int_heading  + 
    controller->kd_heading * controller->error_diff_heading; 
  // Do not use more than 20% of thruster capacity for heading correction.
  if(fabs(heading_thrust) > max_thrust * 0.2)
  {
    if(heading_thrust > 0.0)
    {
      heading_thrust = max_thrust * 0.2;
    }
    else
    {
      heading_thrust = -max_thrust * 0.2;
    }
  }
  double position_thrust= 
    controller->kp_position * controller->error_position      +
    controller->ki_position * controller->error_int_position  + 
    controller->kd_position * controller->error_diff_position;
  
  double thrust_ps = position_thrust + heading_thrust; // left side thrust
  double thrust_sb = position_thrust - heading_thrust; // right side thrust
  if(fabs(thrust_ps) > max_thrust)
  {
    thrust_ps = (thrust_ps > 0.0)? max_thrust : -max_thrust;
  } 
  if(fabs(thrust_sb) > max_thrust)
  {
    thrust_sb = (thrust_sb > 0.0)? max_thrust : -max_thrust;
  }
  controller->thrust_fore_ps = controller->thrust_aft_ps  = thrust_ps;
  controller->thrust_fore_sb = controller->thrust_aft_sb  = thrust_sb;
}
