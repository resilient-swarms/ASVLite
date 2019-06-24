#include "pid_controller.h"

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
  
  // Calculate the heading error in radian.
  
  // Calculate the integral heading error.
  
  // Clamp the integral heading error.
  
  // Calculate the differential heading error.
  
  // Clamp the differential heading error.
  
  // Calculate position error - distance to way-point from current position.
  
  // Calculate the integral error for position.
  
  // Clamp the integral error for position.
  
  // Calculate the differential error for position.
  
  // Clamp the differential error for position.
  
  // Calculate propeller thrust.
}
