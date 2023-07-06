#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "geometry.h"

/**
 * @file
 * An instance of Regular_wave should only be created by calling the 
 * function regular_wave_new(). This function allocates and initialises 
 * a block of memory on the stack, and therefore all calls to 
 * regular_wave_new() should be paired with a call to regular_wave_delete() 
 * to avoid memory leaks. 
 * 
 * All functions operating on an instance of a Regular_wave have a mechanism 
 * to notify of exceptions. All instances of Regular_wave have a member variable 
 * that holds a pointer to an error message. When there are no errors, the pointer 
 * is set to null. If an error occurs in a call to a function that takes an 
 * instance of Regular_wave, an error message is set within the instance. The error 
 * message can be fetched using the function regular_wave_get_error_msg(). The expected 
 * usage is to pair all function calls that take an instance of Regular_wave with a 
 * call to regular_wave_get_error_msg() and check for a null pointer. If a null pointer 
 * is returned, there is no error; otherwise, an error has occurred. Any subsequent calls 
 * to other functions that take an instance of Regular_wave will reset the last know 
 * error message. 
 */
struct Regular_wave;

/**
 * Create and initialise a regular wave.
 * @param amplitude of the wave in meter.
 * @param frequency of the wave in Hz.
 * @param phase of the wave in radians. 
 * @param direction of propagation of the wave in radians with respect to the
 * geographic north. The angle measured is positive in the clockwise direction such that 
 * the geographic east is at PI/2 radians to the north.
 * @return pointer to initialised object if the operation was successful, else, returns a null pointer. 
 */
struct Regular_wave* regular_wave_new(double amplitude, 
                                      double frequency, 
                                      double phase_lag, 
                                      double direction);

/**
 * Free memory allocated for the regular wave. 
 * @param wave is a non-null pointer to an instance of Regular_wave to be deallocated.
 */
void regular_wave_delete(struct Regular_wave* regular_wave);

/**
 * Returns error message related to the last function called for the instance of Regular_wave.
 * @return pointer to the error msg, if any, else returns a null pointer. 
 */
const char* regular_wave_get_error_msg(const struct Regular_wave* regular_wave);

/**
 * Get wave amplitude.
 * @return wave amplitude in meter. 
 */ 
double regular_wave_get_amplitude(const struct Regular_wave* regular_wave);

/**
 * Get wave frequency. 
 * @return wave frequency in Hz.
 */ 
double regular_wave_get_frequency(const struct Regular_wave* regular_wave);

/** 
 * Get wave direction.
 * @return direction of propagation of the wave in radians with respect to the
 * geographic north. The angle measured is positive in the clockwise direction such that 
 * the geographic east is at PI/2 radians to the north.
 */ 
double regular_wave_get_direction(const struct Regular_wave* regular_wave);

/**
 * Get the phase of the wave at a given point for a given time.
 * @param location with all coordinates in meter, at which the phase is to be calculated.
 * @param time for which the phase is to be calculated. Time is measured in seconds from 
 * the start of simulation. Time should be non-negative.
 * @return wave phase in radian.
 */
double regular_wave_get_phase(const struct Regular_wave* regular_wave, 
                              union Coordinates_3D location, 
                              double time);
/**
 * Get elevation of the wave at a given point for a given time.
 * @param location with all coordinates in meter, at which the elevation is to be computed.
 * @param time for which elevation is to be computed. Time is measured in 
 * seconds from the start of simulation. Time should be non-negative.
 * @return wave elevation in meter. 
 */
double regular_wave_get_elevation(const struct Regular_wave* regular_wave, 
                                  union Coordinates_3D location, 
                                  double time);

/**
 * Get wave pressure amplitude at a given depth.
 * @param depth in meter at which the pressure amplitude is to be computed. Depth 
 * should be positive value. 
 * @return pressure amplitude in N/m2.
 */
double regular_wave_get_pressure_amp(const struct Regular_wave* regular_wave, double depth);
#endif // REGULAR_WAVE_H
