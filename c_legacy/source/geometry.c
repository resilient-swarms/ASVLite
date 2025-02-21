#include <math.h>
#include "geometry.h"
#include "constants.h"

double normalise_angle_PI(double angle)
{
    // Reduce the angle if greater than 2PI
    angle = fmod(angle, 2.0*PI);
    // Set to range (-PI, PI]
    if(angle > PI)
    {
        angle -= (2.0*PI);
    }
    if(angle < -PI)
    {
        angle += (2.0*PI);
    }
    return angle;
}

double normalise_angle_2PI(double angle)
{
    // Reduce the angle if greater than 2PI
    angle = fmod(angle, 2.0*PI);
    // Set to range [0, PI)
    angle = fmod(angle + 2.0*PI, 2.0*PI);
    return angle;
}