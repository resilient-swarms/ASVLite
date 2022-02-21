#ifndef WAVE_H
#define WAVE_H

#include "geometry.h"

/**
 * An irregular wave.
 */
struct Wave;

/**
 * Create and initialise an irregular wave.
 * @param sig_wave_height is the significant wave height, in meter, of the irregular sea surface. Value should be non-zero positive.
 * @param wave_heading in radians.
 * @param rand_seed is the seed for random number generator. 
 * @param count_wave_spectral_directions is the number of direction bands in the wave spectrum.
 * @param count_wave_spectral_frequencies is the number of frequency bands in the wave spectrum. 
 * @return pointer to the initialised object if operation successful else returns a NULL.
 */
const struct Wave* wave_new(double sig_wave_ht,
                            double wave_heading, 
                            long rand_seed,
                            int count_wave_spectral_directions,
                            int count_wave_spectral_frequencies);
/**
 * Free memory allocated for the wave.
 */
void wave_delete(const struct Wave* wave);

/**
 * Get the error message corresponding to the last function called for a wave object. 
 * If no error message then returns NULL.
 */
const char* wave_get_error_msg(const struct Wave* wave);

/**
 * Get sea surface elevation at the given location for the given time. 
 * @param wave is the pointer to the irregular sea surface wave. Assumes wave to
 * be not a null pointer.
 * @param location at which the elevation is to be computed.
 * @param time for which the elevation is to be computed. Time is measured in 
 * seconds from start of simulation.
 * @return wave elevation in meter. 
 */
double wave_get_elevation(const struct Wave* wave, 
                          union Coordinates_3D location, 
                          double time);

/**
 * Function to get the number of direction bands in the wave spectrum.
 */ 
int wave_get_count_wave_spectral_directions(const struct Wave* wave);

/** 
 * Function to get the number of frequency bands in the wave spectrum.
 */
int wave_get_count_wave_spectral_frequencies(const struct Wave* wave);

/**
 * Function to get the regular wave at spectrum[d][f].
 * @param d is the index for direction and should be in the range [0, count_wave_spectral_directions)
 * @param f is the index for frequency and should be in the range [0, count_wave_spectral_frequencies)
 * @return pointer to the regular wave if found else NULL.
 */
const struct Regular_wave* wave_get_regular_wave_at(const struct Wave* wave, int d, int f); 

#endif // WAVE_H
