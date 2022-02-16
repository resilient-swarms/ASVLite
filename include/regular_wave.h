#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "geometry.h"

/**
 * A regular wave. 
 */
struct Regular_wave;

/**
 * Create and initialise a regular wave.
 * @param amplitude of the wave in meter.
 * @param frequency of the wave in Hz.
 * @param phase of the wave in radians. 
 * @param direction of propagation of the wave in radians with respect to the
 * geographic north. Angle measured positive in clockwise direction such that 
 * the east is at PI/2 radians to north.
 * @return pointer to initialised object if operation successful else returns a NULL. 
 */
const struct Regular_wave* regular_wave_new(double amplitude, 
                                            double frequency, 
                                            double phase_lag, 
                                            double direction);

/**
 * Free memory allocated for the regular wave. 
 */
void regular_wave_delete(const struct Regular_wave* regular_wave);

/**
 * Returns error message related to the last function called for a regular wave object.
 */
const char* regular_wave_get_error_msg(const struct Regular_wave* regular_wave);

/**
 * Get the amplitude.
 */ 
double regular_wave_get_amplitude(const struct Regular_wave* regular_wave);

/**
 * Get frequency. 
 */ 
double regular_wave_get_frequency(const struct Regular_wave* regular_wave);

/** 
 * Get direction.
 */ 
double regular_wave_get_direction(const struct Regular_wave* regular_wave);

/**
 * Get the phase of the wave at a given point for a given time.
 * @param wave for which the phase is to be calculated.
 * @param location at which the phase is to be calculated.
 * @param time for which the phase is to be calculated.
 * @return wave phase in radian.
 */
double regular_wave_get_phase(const struct Regular_wave* regular_wave, 
                              struct Cartesian_coordinate_3D location, 
                              double time);
/**
 * Get elevation of the wave at a given point for a given time.
 * @param wave for which the elevation is to be calculated.
 * @param location at which the elevation is to be computed.
 * @param time for which elevation is to be computed. Time is measured in 
 * seconds from the start of simulation.
 * @return wave elevation in meter. 
 */
double regular_wave_get_elevation(const struct Regular_wave* regular_wave, 
                                  struct Cartesian_coordinate_3D location, 
                                  double time);

/**
 * Get wave pressure amplitude at a given depth.
 * @param wave for which the pressure amplitude is to be computed. 
 * @z is the depth at which the pressure amplitude is to be computed. 
 * @return pressure amplitude in N/m2.
 */
double regular_wave_get_pressure_amp(const struct Regular_wave* regular_wave, 
                                     double z);
#endif // REGULAR_WAVE_H
