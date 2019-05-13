#include "environment.h"

void environment_init(struct Environment* environment,
                      double wind_speed,
                      double wind_direction,
                      double current_speed,
                      double current_direction)
{ 
  wave_init(&(environment->wave), wind_speed, wind_direction);
  wind_init(&(environment->wind), wind_speed, wind_direction);
  current_init(&(environment->current), current_speed, current_direction);
}
