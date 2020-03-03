#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "geometry.h"

/**
 * Structure to define a regular wave. 
 */
struct Regular_wave
{
  // Input variables
  // ---------------
  double amplitude; //!< Input variable. Amplitude of the wave in meter.
  double frequency; //!< Input variable. Frequency of the wave in Hz.
  double phase_lag; //!< Input variable. Phase lag of the wave in radian.
  double direction; //!< Input variable. Direction of propagation of the wave 
                    //!< with respect to geographic north. Angle measured
                    //!< positive in clockwise direction such that east is at
                    //!< PI/2 radians to !< north.
  
  // Output variables
  // ----------------
  double time_period; //!< Output variable. Time period of the wave in seconds.
  double wave_length; //!< Output variable. Wave length in meter.
  double wave_number; //!< Output variable. Wave number. Dimensionless.
}; 

/**
 * Initialise a regular wave.
 * @param wave is the pointer to the wave to be initialised.
 * @param amplitude of the wave in meter.
 * @param frequency of the wave in Hz.
 * @param phase of the wave in radians. 
 * @param direction of propagation of the wave in radians with respect to the
 * geographic north. Angle measured positive in clockwise direction such that 
 * the east is at PI/2 radians to north.
 * @return 0 if no error encountered. 
 *         1 if wave is a nullptr. 
 *         2 if amplitude is 0 or negative value. 
 *         3 if frequency is 0 or negative value. 
 */
int regular_wave_init(struct Regular_wave* wave, 
                         double amplitude, 
                         double frequency, 
                         double phase_lag, 
                         double direction);

/**
 * Get the wave phase for the given point for the given time.
 * @param wave for which the phase is to be calculated.
 * @param location at which the phase is to be calculated.
 * @param time for which the phase is to be calculated.
 * @return wave phase in radian. Function returns value 0 if wave is nullptr or
 * if time is negative.
 */
double regular_wave_get_phase(struct Regular_wave* wave, 
                              struct Dimensions* location, 
                              double time);
/**
 * Get wave elevation at a given point for a given time.
 * @param wave for which the elevation is to be calculated.
 * @param location at which the elevation is to be computed.
 * @param time for which elevation is to be computed. Time is measured in 
 * seconds from the start of simulation.
 * @return wave elevation in meter. Function returns value 0 if wave is nullptr
 * or if time is negative. 
 */
double regular_wave_get_elevation(struct Regular_wave* wave, 
                                  struct Dimensions* location, 
                                  double time);

/**
 * Get wave pressure amplitude at depth z.
 * @param wave for which the pressure amplitude is to be computed. 
 * @z is the depth at which the pressure amplitude is to be computed. 
 * @return pressure amplitude in N/m2.
 */
double regular_wave_get_pressure_amp(struct Regular_wave* wave, double z);

#endif // REGULAR_WAVE_H
