#ifndef WAVE_H
#define WAVE_H

#include "regular_wave.h"
#include "constants.h"

struct Wave
{
  // List of regular waves in the irregular sea.
  struct Regular_wave spectrum[COUNT_WAVE_SPECTRAL_DIRECTIONS]
                              [COUNT_WAVE_SPECTRAL_FREQUENCIES];
  double min_spectral_frequency;  // Lower limit (0.1%) of spectral energy 
                                  // threshold.
  double max_spectral_frequency;  // Upper limit (99.9%) of spectral energy 
                                  // threshold.
  double peak_spectral_frequency; // Spectral peak frequency in Hz.
  double min_spectral_wave_heading; // Minimum angle, in radians, in spectrum
                                    // for wave heading.
  double max_spectral_wave_heading; // Maximum angle, in radians, in spectrum 
                                    // for wave heading.
  double significant_wave_height;   // Significant wave height in meter.
  double heading; // wave heading in radians
  long random_number_seed; 
};

/**
 * Initialise the irregular wave on the sea using significant wave height as
 * input.
 * @param wave is the pointer to the wave object to be initialised. Assumes 
 * wave is not a null pointer and that all values in the structure are to be
 * overwritten.
 * @param sig_wave_height is the significant wave height to achieve for the
 * irregular sea being initialised. Value should be non-zero positive.
 * @param wave_heading in radians
 * @param rand_seed is the seed for random number generator. 
 */
void wave_init(struct Wave* wave, 
               double sig_wave_height, 
               double wave_heading, 
               long rand_seed);

/**
 * Get sea surface elevation at the given location for the given time. 
 * @param wave is the pointer to the irregular sea surface wave. Assumes wave to
 * be not a null pointer.
 * @param location at which the elevation is to be computed.
 * @param time for which the elevation is to be computed. Time is measured in 
 * seconds from start of simulation.
 * @return wave elevation in meter.
 */
double wave_get_elevation(struct Wave* wave, 
                          struct Point* location, 
                          double time);

#endif // WAVE_H
