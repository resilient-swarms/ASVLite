#ifndef WIND_H
#define WIND_H

#include "geometry.h"

/**
 * Structure to define the model of wind in the sea. Assume constant wind speed 
 * throughout the field and also assumes the wind is constant for the time of
 * simulation.
 */
struct Wind
{
  double speed; // Wind speed in m/s.
  double direction; // Direction of wind measured with respect to the geographic
                    // north. Angle measured +ve clockwise such that the east is
                    // at PI/2 radians to the north.
};

/**
 * Function to initialise the wind model.
 * @param wind is the pointer to the structure to be initialised.
 * @param speed is the magnitude of the wind speed.
 * @param direction at which wind is blowing. 
 */
void wind_init(struct Wind* wind, double speed, double direction);

/**
 * Function to get the velocity of the wind resolved in the x and y direction.
 * @param velocity is the buffer to contain the return value. The return is an
 * array of size 2. The first element (at index 0) of the array is the velocity 
 * along the x direction and the second element (at index 1) is the velocity 
 * along the y direction.
 * @param wind is the pointer to the model of the wind.
 * @param location at which the wind velocity is to be calculated.
 * @param time for which the wind velocity is to be calculated.
 */
void wind_get_velocity(double velocity[2], 
                       struct Wind* wind, 
                       struct Point* location, 
                       double time);


#endif // WIND_H
