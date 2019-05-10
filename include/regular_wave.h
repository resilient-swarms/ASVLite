#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "geometry.h"

/**
 * Structure to defines a regular sinusoidal wave. 
 */
struct Regular_wave
{
  double amplitude; /* Amplitude of the wave in meter. */
  double frequency; /* Frequency of the wave in Hz. */
  double phase; /* Phase angle of the wave in radian. */
  double direction; /* Direction of propagation of the wave with respect to 
                     * geographic north. Angles are measured positive in 
                     * clockwise direction. 
                     */
}; 

/**
 * Get the time period of the regular wave.
 * @return time period in seconds.
 */
double regular_wave_get_time_period(struct Regular_wave* wave);

/**
 * Get the wave length of a regular wave.
 * @return wave length in meter.
 */
double regular_wave_get_length(struct Regular_wave* wave);

/**
 * Get the wave number of a regular wave.
 * @return wave number.
 */
double regular_wave_get_wave_number(struct Regular_wave* wave);

/**
 * Get wave elevation a given point for a given time.
 * @param point at which the elevation is to be found with all coordinates in m.
 * @param time at which elevation is to be found with time measured as seconds
 * from the start of simulation.
 */
double regular_wave_get_elevation(struct Point* point, double time);

#endif // REGULAR_WAVE_H
