#ifndef WAVE_H
#define WAVE_H

#include "regular_wave.h"

#define COUNT_SPECTRAL_FREQUENCIES 20
#define COUNT_SPECTRAL_DIRECTIONS  10

/**
 * Structure to define the irregular wave on the sea surface. The irregular sea 
 * wave is considered as resultant of super-positioning  of a collection of many
 * regular waves. Wave spectrum helps to create this collection of regular waves 
 * such that the irregular wave formed by the super-positioning of the regular 
 * waves have the required statistical properties of an irregular sea simulated.
 *
 * The wave spectrum is considered as consisting of discrete number of wave
 * frequencies and wave headings which are set by the constant variables
 * COUNT_SPECTRAL_FREQUENCIES and COUNT_SPECTRAL_DIRECTIONS.
 */
struct Wave
{
  // Input variables
  // ---------------
  double wind_fetch;// The length, in meter, of sea over which the wind blows.
  double wind_speed;// Wind speed in m/s.
  double wind_direction; // Direction of wind measured with respect to 
                         // geographic north. Angle measure +ve clockwise such 
                         // that the east is at PI/2 radians to the north.
  
  // Wave spectrum variables
  // -----------------------
  struct Regular_wave spectrum[COUNT_SPECTRAL_DIRECTIONS]
                              [COUNT_SPECTRAL_FREQUENCIES]; // List of regular
                                                            // waves in the 
                                                            // irregular sea.
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
 * Initialise the irregular wave on the sea surface. 
 * @param wave pointer to the wave to be initialised. Assume wave is not a null
 * pointer and that all values in the structure are to be overwritten.
 * @param wind_speed in m/s. Assumes the wind speed to be a non-zero positive
 * value.
 * @param wind_fetch in m. Assumes the wind_fetch to be a non-zero positive
 * value.
 * @param wind_direction in radians measured with respect to geographic north.
 * Angle measured positive in clockwise direction such that the east is at PI/2
 * radians to the north. Assumes wind_direction to have a value within the range 
 * (0, 2PI).
 */
void wave_init(struct Wave* wave, 
               double wind_speed, 
               double wind_fetch, 
               double wind_direction);

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
