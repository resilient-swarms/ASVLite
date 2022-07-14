#ifndef WAVE_H
#define WAVE_H

#include "geometry.h"

/**
 * An irregular wave.
 * An instance of Wave should only be created by calling 
 * the method wave_new(). This function allocates and 
 * initialises a block of memory on the stack, and therefore 
 * all calls to wave_new() should be paired with a call to 
 * wave_delete() to avoid memory leaks. All function calls 
 * may not result in a successful operation due to error. 
 * To find the status of an operation, call the function 
 * wave_get_error_msg(), which returns a null pointer if 
 * the operation was successful; else returns an error message. 
 */
struct Wave;

/**
 * Create and initialise an irregular wave.
 * @param sig_wave_height is the significant wave height, in meter, of the irregular sea surface. Value should be non-negative.
 * @param wave_heading is the predominant wave heading with respect to the
 * geographic north. The angle measured is positive in the clockwise direction such that 
 * the geographic east is at PI/2 radians to the north.
 * @param rand_seed is the seed for random number generator. 
 * @param count_wave_spectral_directions is the number of discrete direction bands in the wave spectrum. Value should be greater than 1.
 * @param count_wave_spectral_frequencies is the number of discrete frequency bands in the wave spectrum. Value should be greater than 1.
 * @return pointer to the initialised object if the operation was successful; else, returns a null pointer.
 */
const struct Wave* wave_new(double sig_wave_ht,
                            double wave_heading, 
                            long rand_seed,
                            int count_wave_spectral_directions,
                            int count_wave_spectral_frequencies);
/**
 * Free memory allocated for the wave.
 * @param wave is a non-null pointer to an instance of Wave to be deallocated.
 */
void wave_delete(const struct Wave* wave);

/**
 * Returns error message related to the last function called for the instance of Wave.
 * @param wave is a non-null pointer to an instance of Wave for which the error message is to be fetched.
 */
const char* wave_get_error_msg(const struct Wave* wave);

/**
 * Get sea surface elevation at the given location for the given time. 
 * @param wave is a non-null pointer to an instance of Wave for which the surface elevation is to be computed.
 * @param location at which the elevation is to be computed.
 * @param time for which the elevation is to be computed. Time is measured in seconds from start of simulation.
 * @return wave elevation in meter. 
 */
double wave_get_elevation(const struct Wave* wave, 
                          union Coordinates_3D location, 
                          double time);

/**
 * Function to get the number of direction bands in the wave spectrum.
 * @param wave is a non-null pointer to an instance of Wave for which the number of 
 * discrete direction bands contained in the wave spectrum is to be fetched.
 */ 
int wave_get_count_wave_spectral_directions(const struct Wave* wave);

/** 
 * Function to get the number of frequency bands in the wave spectrum.
 * @param wave is a non-null pointer to an instance of Wave for which the number of 
 * discrete frequency bands contained in the wave spectrum is to be fetched.
 */
int wave_get_count_wave_spectral_frequencies(const struct Wave* wave);

/**
 * Function to get the regular wave at spectrum[d][f].
 * @param wave is a non-null pointer to an instance of Wave from which the regular wave is to be fetched.
 * @param d is the index for direction and should be in the range [0, count_wave_spectral_directions)
 * @param f is the index for frequency and should be in the range [0, count_wave_spectral_frequencies)
 * @return pointer to the regular wave if found; else returns a null pointer.
 */
const struct Regular_wave* wave_get_regular_wave_at(const struct Wave* wave, int d, int f); 

#endif // WAVE_H
