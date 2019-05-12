#ifndef CURRENT_H
#define CURRENT_H

#include "geometry.h"

/**
 * Structure to define the model of current in the sea. Assume constant current
 * velocity throughout the field and also assumes the current is constant for 
 * the time of simulation.
 */
struct Current
{
  double speed; // Current speed in m/s.
  double direction; // Direction of current measured with respect to the 
                    // geographic north. Angle measured +ve clockwise such that 
                    // the east is at PI/2 radians to the north.
};

/**
 * Function to initialise the current model.
 * @param current is the pointer to the structure to be initialised.
 * @param speed is the magnitude of the current speed.
 * @param direction of current. 
 */
void current_init(struct Current* current, double speed, double direction);

/**
 * Function to get the current velocity resolved in the x and y direction.
 * @param velocity is the buffer to contain the return value. The return is an
 * array of size 2. The first element (at index 0) of the array is the velocity 
 * along the x direction and the second element (at index 1) is the velocity 
 * along the y direction.
 * @param current is the pointer to the model of the wind.
 * @param location at which the current velocity is to be calculated.
 * @param time for which the current velocity is to be calculated.
 */
void current_get_velocity(double velocity[2], 
                          struct Current* current, 
                          struct Point* location, 
                          double time);


#endif // CURRENT_H
