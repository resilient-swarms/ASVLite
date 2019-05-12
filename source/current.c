#include <math.h>
#include "current.h"

void current_init(struct Current* current, double speed, double direction)
{
  current->speed = speed;
  current->direction = direction;
}

void current_get_velocity(double velocity[2], 
                          struct Current* current, 
                          struct Point* location, 
                          double time)
{
  if(location->z >= 0.0)
  {
    // No water current in air.
    velocity[0] = 0.0;
    velocity[1] = 0.0;
  }
  else
  {
    velocity[0] = current->speed * sin(current->direction); // x direction.
    velocity[1] = current->speed * cos(current->direction); // y direction.
  }
}

