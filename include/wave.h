#ifndef WAVE_H
#define WAVE_H

#include "regular_wave.h"

#define COUNT_WAVE_SPECTRAL_FREQUENCIES 20
#define COUNT_WAVE_SPECTRAL_DIRECTIONS  10

/**
 * Structure to define the model of irregular wave on the sea surface. The wave 
 * model can provide the wave elevation at any location at any instance of time.
 *
 * The irregular sea wave is considered as resultant of super-positioning of a 
 * collection of many regular waves. Wave spectrum helps to make the collection 
 * of regular waves such that the irregular wave formed by the super-positioning 
 * has the required statistical properties of the irregular sea simulated.
 *
 * Bretschneider spectrum is used for creating the collection of regular wave.
 * Bretschneider spectrum is a continues spectrum but for the implementation it 
 * has be converted to a discrete spectrum. COUNT_SPECTRAL_FREQUENCIES specifies 
 * the number of discrete frequencies in the spectrum and 
 * COUNT_SPECTRAL_DIRECTIONS specifies the number of discrete number of
 * directions in the spectrum.
 */
struct Wave
{
  struct Regular_wave* spectrum ; // List of regular waves in the irregular sea.
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
};

/**
 * Initialise the irregular wave on the sea using significant wave height as
 * input.
 * @param wave is the pointer to the wave object to be initialised. Assumes wave 
 * is not a null pointer and that all values in the structure are to be
 * overwritten.
 * @param sig_wave_height is the significant wave height to achieve for the
 * irregular sea being initialised. Value should be non-zero positive.
 * @param wave_heading in radians
 */
void wave_init(struct Wave* wave, 
               double sig_wave_height, 
               double wave_heading);

/**
 * Free memory allocated.
 */
void wave_clean(struct Wave* wave);

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
