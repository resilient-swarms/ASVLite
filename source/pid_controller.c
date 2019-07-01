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

void pid_controller_set_current_state(struct PID_controller* controller,
                                      struct Point position,
                                      struct Attitude attitude)
{
  controller->asv_position.x = position.x;
  controller->asv_position.y = position.y;
  controller->asv_position.z = position.z;
  controller->asv_attitude.heel = attitude.heel;
  controller->asv_attitude.trim = attitude.trim;
  controller->asv_attitude.heading = attitude.heading;  
}

void pid_controller_set_way_point(struct PID_controller* controller,
                                  struct Point way_point)
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
  double max_error_position = 5.0; // Set the max position error to 5m.
  double error_position = sqrt(pow(x2-x1, 2.0) + pow(y2-y1, 2.0));
  // Using distance for error measurement has a drawback - distance is always 
  // positive and does not let provide the information as to whether the 
  // way-point is in front or behind. This next step gives a sign convention
  // to the error. 
  error_position = error_position * (sqrt(x2*x2 + y2*y2) - 
                                     sqrt(x1*x1 + y1*y1));

  // Clamp the position error.
  error_position = (error_position > max_error_position)? 
                    max_error_position: error_position;
  
  // Calculate the integral error for position.
  controller->error_int_position += controller->ki_position * error_position;
  
  // Clamp the integral error for position.
  controller->error_int_position = 
    (controller->error_int_position > 4.0*max_error_position)?
    4.0*max_error_position: controller->error_int_position;
  
  // Calculate the differential error for position.
  controller->error_diff_position = error_position - controller->error_position;
  controller->error_position = error_position;
  
  // Clamp the differential error for position.
  controller->error_diff_position = 
    (controller->error_diff_position > 4.0*max_error_position)?
    4.0*max_error_position: controller->error_diff_position;
  
  // Calculate the heading error in radian.
  double heading_required = atan((x2 - x1)/(y2 - y1));
  // atan gives values in the range of -PI/2 to PI/2.
  // correct the angle if it is in the 3rd of 4th quadrants. 
  if(((x2-x1) < 0.0 && (y2-y1) < 0.0) || 
     ((x2-x1) > 0.0 && (y2-y1) < 0.0))
  {
    heading_required += PI;
  } 
  
  double error_heading = heading_required - controller->asv_attitude.heading;
  // Clamp the heading error
  double max_error_heading = PI/6.0; // Set the max heading error.
  error_heading = (error_heading > max_error_heading)? 
                  max_error_heading: error_heading;
  
  // Calculate the integral heading error.
  controller->error_int_heading += controller->ki_heading * error_heading;
  
  // Clamp the integral heading error.
  controller->error_int_heading = 
    (controller->error_int_heading > 4.0*max_error_heading)?
    4.0*max_error_heading: controller->error_int_heading;
  
  // Calculate the differential heading error.
  controller->error_diff_heading = error_heading - controller->error_heading;
  controller->error_heading = error_heading; 
  
  // Clamp the differential heading error.
  controller->error_diff_heading = 
    (controller->error_diff_heading > 4.0*max_error_heading)?
    4.0*max_error_heading: controller->error_diff_heading;
 
  // Calculate propeller thrust.
  double max_thrust = 5.0; // SMARTY platform thruster has a maximum 
                           // capacity of 5N. 

  double heading_thrust = controller->kp_heading * controller->error_heading + 
                        controller->error_int_heading + 
                        controller->kd_heading * controller->error_diff_heading; 
  // Do not use more than 20% of thruster capacity for heading correction.
  heading_thrust = (fabs(heading_thrust) > max_thrust) ? 
                    max_thrust : heading_thrust;  
  double position_thrust= controller->kp_position* controller->error_position +
                        controller->error_int_position + 
                        controller->kd_position*controller->error_diff_position;
  
  double thrust_ps = position_thrust + heading_thrust; // left side thrust
  double thrust_sb = position_thrust - heading_thrust; // right side thrust
  thrust_ps = (fabs(thrust_ps) > max_thrust)? max_thrust : thrust_ps;
  thrust_sb = (fabs(thrust_sb) > max_thrust)? max_thrust : thrust_sb;
  // Before applying the thrust to the thrusters check if the position is within
  // the error margin, if so then set the thrust to 0N.
  if(sqrt(pow(x2-x1, 2.0) + pow(y2-y1, 2.0)) <= 1.0)
  {
    thrust_ps = thrust_sb = 0.0;
  }
  controller->thrust_fore_ps = controller->thrust_aft_ps  = thrust_ps;
  controller->thrust_fore_sb = controller->thrust_aft_sb  = thrust_sb;
}
