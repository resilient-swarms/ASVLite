#include <stdio.h>
#include "constants.h"
#include "environment.h"

int main()
{
  // Environment condition.
  double wind_speed = 7.5; // m/s
  double wind_direction = PI/6.0; // radians
  double current_speed = 0.4; // m/s
  double current_direction = 11.0*PI/6.0; //radians
  
  // Initialise the environment
  struct Environment environment;
  environment_init(&environment, 
                   wind_speed, wind_direction,
                   current_speed, current_direction);

  // Print the wave stats
  return 0;
}
