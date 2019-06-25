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
  
  // Calculate the heading required in radian.
  double x1 = controller->asv_position.x;
  double y1 = controller->asv_position.y;
  double x2 = controller->way_point.x;
  double y2 = controller->way_point.y;
  double heading_required = atan((x2 - x1)/(y2 - y1));
  if(heading_required < 0.0)
  {
    heading_required += 2*PI;
  }
  controller->heading_required = heading_required;
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
  // Calculate the heading error in radian.
  double error_heading = controller->heading_required - 
                         controller->asv_attitude.heading;
  
  // Calculate the integral heading error.
  controller->error_int_heading += controller->ki_heading * error_heading;
  
  // Clamp the integral heading error.
  
  // Calculate the differential heading error.
  controller->error_diff_heading = error_heading - controller->error_heading;
  controller->error_heading = error_heading; 
  
  // Clamp the differential heading error.
  
  // Calculate position error - distance to way-point from current position.
  double x1 = controller->asv_position.x;
  double y1 = controller->asv_position.y;
  double x2 = controller->way_point.x;
  double y2 = controller->way_point.y;
  double error_position = sqrt(pow(x2-x1, 2.0) + pow(y2-y1, 2.0));
  
  // Calculate the integral error for position.
  controller->error_int_position += controller->ki_position * error_position;
  
  // Clamp the integral error for position.
  
  // Calculate the differential error for position.
  controller->error_diff_position = error_position - controller->error_position;
  controller->error_position = error_position;
  
  // Clamp the differential error for position.
  
  // Calculate propeller thrust.
  double heading_thrust = controller->kp_heading * controller->error_heading + 
                        controller->error_int_heading + 
                        controller->kd_heading * controller->error_diff_heading; 
  double position_thrust= controller->kp_position* controller->error_position +
                        controller->error_int_position + 
                        controller->kd_position*controller->error_diff_position;
  
  double thrust = 0.0;
  
  thrust = position_thrust - heading_thrust;
  thrust = (thrust > 5.0)? 5.0:thrust;
  controller->thrust_fore_ps = thrust;
  controller->thrust_aft_sb  = thrust;
  
  thrust = position_thrust + heading_thrust;
  thrust = (thrust > 5.0)? 5.0:thrust;
  controller->thrust_fore_sb = thrust;
  controller->thrust_aft_ps  = thrust;
}
