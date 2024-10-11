#ifndef SEA_SURFACE_H
#define SEA_SURFACE_H

#include "geometry.h"

/**
 * @file
 * An instance of Sea_surface should only be created by calling the function sea_surface_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to sea_surface_new() should be paired with a call to sea_surface_delete() 
 * to avoid memory leaks. 
 * 
 * All functions operating on an instance of a Sea_surface have a mechanism to notify of 
 * exceptions. All instances of Sea_surface have a member variable that holds a pointer 
 * to an error message. When there are no errors, the pointer is set to null. If 
 * an error occurs in a call to a function that takes an instance of Sea_surface, an error 
 * message is set within the instance. The error message can be fetched using the 
 * function sea_surface_get_error_msg(). The expected usage is to pair all function calls 
 * that take an instance of Sea_surface with a call to sea_surface_get_error_msg() and check for 
 * a null pointer. If a null pointer is returned, there is no error; otherwise, an 
 * error has occurred. Any subsequent calls to other functions that take an instance 
 * of Sea_surface will reset the last know error message. 
 */
struct Sea_surface;

/**
 * Create and initialise an irregular sea surface.
 * @param sig_wave_height is the significant wave height, in meter, of the irregular 
 * sea surface. Value should be non-negative.
 * @param wave_heading is the predominant wave heading with respect to the geographic 
 * north. The angle measured is positive in the clockwise direction such that the 
 * geographic east is at PI/2 radians to the north.
 * @param rand_seed is the seed for random number generator. 
 * @param count_component_waves is the number of regular component waves in 
 * the wave spectrum. Value should be an odd number.
 * @return pointer to the initialised object if the operation was successful; 
 * else, returns a null pointer.
 */
struct Sea_surface* sea_surface_new(double sig_wave_ht,
                      double wave_heading, 
                      int rand_seed,
                      int count_component_waves);

/**
 * Similar to function sea_surface_new(), except here we create and initialise an
 * irregular sea surface using wind speed as input instead of significant wave height.
 * @param wind_velocity_u is the eastward component of the 10m wind, indicating air
 * speed towards the east at 10m above the surface, in m/s.
 * @param wind_velocity_v is the northward component of the 10m wind, indicating air
 * speed towards the north at 10m above the surface, in m/s.
 * @param rand_seed is the seed for random number generator.
 * @param count_component_waves is the number of regular component waves in
 * the wave spectrum. Value should be an odd number.
 * @return pointer to the initialised object if the operation was successful;
 * else, returns a null pointer.
 */
struct Sea_surface* sea_surface_new_from_wind(double wind_velocity_u,
                                              double wind_velocity_v,
                                              int rand_seed,
                                              int count_component_waves);

/**
 * Free memory allocated for the sea_surface.
 * @param sea_surface is a non-null pointer to an instance of Sea_surface to be deallocated.
 */
void sea_surface_delete(struct Sea_surface* sea_surface);

/**
 * Returns error message related to the last function called for the instance of Sea_surface.
 * @return pointer to the error msg, if any, else returns a null pointer. 
 */
const char* sea_surface_get_error_msg(const struct Sea_surface* sea_surface);

/**
 * Get sea surface elevation at the given location for the given time. 
 * @param location with coordinates in meter, at which the elevation is to be computed.
 * @param time for which the elevation is to be computed. Time is measured in seconds 
 * from start of simulation.
 * @return wave elevation in meter. 
 */
double sea_surface_get_elevation(const struct Sea_surface* sea_surface, 
                          union Coordinates_3D location, 
                          double time);

/**
 * Function to get the number of regular component waves in the spectrum.
 */ 
int sea_surface_get_count_component_waves(const struct Sea_surface* sea_surface);

/**
 * Function to get the regular wave at spectrum[d][f].
 * @param i is the index in the range [0, count_component_waves)
 * @return pointer to the regular wave if found; else returns a null pointer.
 */
const struct Regular_wave* sea_surface_get_regular_wave_at(const struct Sea_surface* sea_surface, int i); 

/**
 * Function to get the minimum spectral frequency, in Hz, for the wave spectrum.
 */ 
double sea_surface_get_min_spectral_frequency(const struct Sea_surface* sea_surface);

/**
 * Function to get the maximum spectral frequency, in Hz, for the wave spectrum.
 */ 
double sea_surface_get_max_spectral_frequency(const struct Sea_surface* sea_surface);

/**
 * Function to get the peak spectral frequency, in Hz, for the wave spectrum.
 */ 
double sea_surface_get_peak_spectral_frequency(const struct Sea_surface* sea_surface);

/**
 * Function to get the significant wave height, in meter, for the sea state.
 */ 
double sea_surface_get_significant_height(const struct Sea_surface* sea_surface);

/**
 * Function to get the rand seed used for generating the component waves.
 */ 
int sea_surface_get_rand_seed(const struct Sea_surface* sea_surface);

/**
 * Function to get the predominant wave heading, in radians, for the sea state.
 */ 
double sea_surface_get_predominant_heading(const struct Sea_surface* sea_surface);

/**
 * Function to get the mean wavenumber for the sea state.
 */
double sea_surface_get_mean_wavenumber(const struct Sea_surface* sea_surface);


#endif // SEA_SURFACE_H
