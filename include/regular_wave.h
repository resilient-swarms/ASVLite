#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "geometry.h"

/**
 * Structure to defines a regular sinusoidal wave. 
 */
struct Regular_wave
{
  double amplitude; // Amplitude of the wave in meter.
  double frequency; // Frequency of the wave in Hz.
  double phase;     // Phase angle of the wave in radian.
  double direction; // Direction of propagation of the wave with respect to 
                    // geographic north. Angle are measured positive in 
                    // clockwise direction such that east is at PI/2 radians to
                    // north.
  double time_period; // Time period of the wave in seconds.
  double wave_length; // Wave length in meter.
  double wave_number; // Wave number. Dimension less.
}; 

/**
 * Initialise a regular wave.
 * @param wave is the pointer to the wave to be initialised. The function
 * assumes that wave is not a null pointer and that all the values in the
 * structure are to be overwritten.
 * @param amplitude of the wave in meter. Assumes wave length is non-zero
 * positive value.
 * @param frequency of the wave in Hz. Assumes wave frequency is non-zero
 * positive value.
 * @param phase of the wave in radians.
 * @param direction of propagation of the wave in radians with respect to the
 * geographic north. Angle measured positive in clockwise direction such that 
 * the east is at PI/2 radians to north.
 */
void regular_wave_init(struct Regular_wave* wave, 
                         double amplitude, 
                         double frequency, 
                         double phase, 
                         double direction);
/**
 * Get wave elevation a given point for a given time.
 * @param wave for which the elevation is to be calculated.
 * @param location at which the elevation is to be computed.
 * @param time for which elevation is to be computed. Time is measured in 
 * seconds from the start of simulation.
 * @return wave elevation in meter.
 */
double regular_wave_get_elevation(struct Regular_wave* wave, 
                                  struct Point* location, 
                                  double time);

#endif // REGULAR_WAVE_H
