/*! \mainpage ASV-Swarm
 * This document provide details of the programming interface of ASV-Swarm to 
 * use it as a software library in an application. 
 * \code{.c}
 *  #include <asv.h>
 *
 *  struct Asv asv; 
 *  
 *  // Input ASV specification
 *  asv.spec.L_wl = 0.3; // m
 *  asv.spec.B_wl = 0.3; // m
 *  asv.spec.D    = 0.21;// m
 *  asv.spec.T    = 0.11;// m
 *  asv.spec.max_speed = 2.0;   // m/s 
 *  asv.spec.disp      = 0.007; // m3  
 *  asv.spec.r_roll    = 0.08;  // m 
 *  asv.spec.r_pitch   = 0.08;  // m
 *  asv.spec.r_yaw     = 0.106; // m
 *  asv.spec.cog = (struct Dimensions){0.15, 0.0, -0.2}; 
 *  
 *  // Input propeller configuration
 *  asv.count_propellers = 1;
 *  asv.propellers[0].position = (struct Dimensions){0.0, 0.0, 0.0};
 *  asv.wave_type = irregular_wave; 
 *  
 *  // Initialise the ASV
 *  asv_init(&asv);
 *  
 *  // Simulate
 *  double time_step_size = 0.04; // sec
 *  for(int t = 0; ; ++t)
 *  {
 *    // Calculate the time, in sec, since start of simulation.
 *    double time = t*time_step_size;
 *    // Set the propeller thrust and direction
 *    asv.propellers[0].thrust = 0.25; // N
 *    asv.propellers[0].orientation = (struct Dimensions){0.0, 0.0, 0.0};
 *    // Compute the new position and attitude of the vehicle
 *    asv_compute_dynamics(&asv, time);
 *    
 *    // Get the wave elevation at the location of the vehicle.
 *    double wave_elevation = wave_get_elevation(&asv.wave, 
 *                                               &asv.cog_position,
 *                                               time);
 *    // Get the position of the vehicle
 *    struct Dimensions position = asv.cog_position;
 *    // Get the floating attitude of the vehicle
 *    struct Dimensions attitude = asv.attitude;
 *
 *    // Get forward velocity of the vehicle
 *    double velocity_surge = asv.dynamics.V[surge];
 *    // Get forward acceleration of the vehicle
 *    double acceleration_surge = asv.dynamics.A[surge];
 *
 *    // Check if the simulation is to continue else exit the loop
 *    // if(stop_simulation) break; 
 *  }
 *
 * \endcode
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#define PI M_PI
#define G 9.81 /*!< Acceleration due to gravity in m/s2 */ 
#define SEA_WATER_DENSITY 1025 /*!< Sea water density in Kg/m3 */
#define AIR_DENSITY 1.2 /*!< Kg/m3 */

#define COUNT_WAVE_SPECTRAL_FREQUENCIES 15/*!< Number of frequency bands in the 
                                            wave spectrum. */
#define COUNT_WAVE_SPECTRAL_DIRECTIONS  5 /*!< Number of direction bands in the 
                                            wave spectrum. Ideal if this is an 
                                            odd number.*/

#define COUNT_ASV_SPECTRAL_DIRECTIONS 360 /*!< Number of directions in 
                                            the wave force spectrum. */
#define COUNT_ASV_SPECTRAL_FREQUENCIES 100 /*!< Number of frequencies in the
                                             wave force spectrum. */

#define COUNT_DOF 6 /*!< Number of degrees of freedom for the motion of ASV. */
#define COUNT_PROPELLERS_MAX 4 /*!< Maximum number of propellers an ASV can 
                                 have.*/

#define COUNT_WAYPOINTS_MAX 20 /*!< Maximum number of waypoints through which 
                                 the ASV will navigate. */

#define OUTPUT_BUFFER_SIZE 200000 /*!< Output buffer size. */

#endif // CONSTANTS_H
