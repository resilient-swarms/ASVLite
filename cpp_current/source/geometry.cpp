#include <cmath>
#include "ASVLite/geometry.h"


double ASVLite::Geometry::normalise_angle_PI(const double angle) {
    // Reduce the angle if greater than 2PI
    double value = fmod(angle, 2.0*M_PI);
    // Set to range (-PI, PI]
    if(value > M_PI)
    {
        value -= (2.0*M_PI);
    }
    if(value < -M_PI)
    {
        value += (2.0*M_PI);
    }
    return value;
}

double ASVLite::Geometry::normalise_angle_2PI(const double angle) {
    // Reduce the angle if greater than 2PI
    double value = fmod(angle, 2.0*M_PI);
    // Set to range [0, PI)
    value = fmod(angle + 2.0*M_PI, 2.0*M_PI);
    return value;
}