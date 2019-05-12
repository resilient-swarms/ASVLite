#include <math.h>
#include "wind.h"

void wind_init(struct Wind* wind, double speed, double direction)
{
  wind->speed = speed;
  wind->direction = direction;
}

void wind_get_velocity(double velocity[2], 
                       struct Wind* wind, 
                       struct Point* location, 
                       double time)
{
  if(location->z <= 0.0)
  {
    // No wind under water
    velocity[0] = 0.0;
    velocity[1] = 0.0;
  }
  else
  {
    velocity[0] = wind->speed * sin(wind->direction); // x direction.
    velocity[1] = wind->speed * cos(wind->direction); // y direction.
  }
}

