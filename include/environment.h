#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "wave.h"
#include "wind.h"
#include "current.h"

/**
 * Environment is a container for wave, wind and current. 
 */
struct Environment
{
  struct Wave wave;
  struct Wind wind;
  struct Current current;
};

/**
 * Function to initialise the environment. Ensures that the wave, wind and
 * current are correctly initialised and also ensures that they are co-related.
 * @param environment is the pointer to the object to be initialised.
 * @param wind_speed in m/s.
 * @param wind_direction in radians measured with respect to north such that
 * east is at PI/2 radians to the north.
 * @param current_speed in m/s.
 * @param current_direction in radians with respect to the north such that east
 * is at PI/2 radians to the north.
 */
void environment_init(struct Environment* environment,
                      double wind_speed, 
                      double wind_direction,
                      double current_speed,
                      double current_direction);

#endif // ENVIRONMENT_H
