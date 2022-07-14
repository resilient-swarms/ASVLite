#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "geometry.h"

/**
 * A regular wave. 
 * An instance of Regular_wave should only be created by calling 
 * the method regular_wave_new(). This function allocates and 
 * initialises a block of memory on the stack, and therefore all 
 * calls to regular_wave_new() should be paired with a call to 
 * regular_wave_delete() to avoid memory leaks. All function calls 
 * may not result in a successful operation due to error. To find 
 * the status of an operation, call the function regular_wave_get_error_msg(), 
 * which returns a null pointer if the operation was successful; 
 * else returns an error message. 
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
const struct Regular_wave* regular_wave_new(double amplitude, 
                                            double frequency, 
                                            double phase_lag, 
                                            double direction);

/**
 * Free memory allocated for the regular wave. 
 * @param wave is a non-null pointer to an instance of Regular_wave to be deallocated.
 */
void regular_wave_delete(const struct Regular_wave* regular_wave);

/**
 * Returns error message related to the last function called for the instance of Regular_wave.
 * @param wave is a non-null pointer to an instance of Regular_wave for which the error message is to be fetched.
 */
const char* regular_wave_get_error_msg(const struct Regular_wave* regular_wave);

/**
 * Get wave amplitude.
 * @param wave is a non-null pointer to an instance of Regular_wave for which the amplitude is to be fetched.
 */ 
double regular_wave_get_amplitude(const struct Regular_wave* regular_wave);

/**
 * Get wave frequency. 
 * @param wave is a non-null pointer to an instance of Regular_wave for which the frequency is to be fetched.
 */ 
double regular_wave_get_frequency(const struct Regular_wave* regular_wave);

/** 
 * Get wave direction.
 * @param wave is a non-null pointer to an instance of Regular_wave for which the wave direction is to be fetched.
 */ 
double regular_wave_get_direction(const struct Regular_wave* regular_wave);

/**
 * Get the phase of the wave at a given point for a given time.
 * @param wave is a non-null pointer to an instance of Regular_wave for which the phase is to be calculated. 
 * @param location at which the phase is to be calculated.
 * @param time for which the phase is to be calculated. Time is measured in 
 * seconds from the start of simulation. Time should be non-negative.
 * @return wave phase in radian.
 */
double regular_wave_get_phase(const struct Regular_wave* regular_wave, 
                              union Coordinates_3D location, 
                              double time);
/**
 * Get elevation of the wave at a given point for a given time.
 * @param wave is a non-null pointer to an instance of Regular_wave for which the elevation is to be calculated.
 * @param location at which the elevation is to be computed.
 * @param time for which elevation is to be computed. Time is measured in 
 * seconds from the start of simulation. Time should be non-negative.
 * @return wave elevation in meter. 
 */
double regular_wave_get_elevation(const struct Regular_wave* regular_wave, 
                                  union Coordinates_3D location, 
                                  double time);

/**
 * Get wave pressure amplitude at a given depth.
 * @param wave is a non-null pointer to an instance of Regular_wave for which the pressure amplitude is to be computed. 
 * @z is the depth at which the pressure amplitude is to be computed. 
 * @return pressure amplitude in N/m2.
 */
double regular_wave_get_pressure_amp(const struct Regular_wave* regular_wave, 
                                     double z);
#endif // REGULAR_WAVE_H
