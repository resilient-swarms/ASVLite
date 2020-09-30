/*! \mainpage ASV-Swarm
 * This document describes the API of ASV-Swarm. The annotated and relatively
 * simple code below demonstrates:
 * <ul>
 * <li> creating an instance of Asv,
 * <li> setting the physical parameters of the ASV,
 * <li> setting the position of each propeller,
 * <li> setting the sea state,
 * <li> setting the propeller thrust vector for each time step,
 * <li> simulating wave and vehicle dynamics for each time step,
 * <li> getting the simulated wave elevation and the position and attitude of
 * the vehicle for each time step.
 * </ul>
 * For more details, we recommend reading the pages in <i>Files/File
 * List/include</i> in the following order:
 * <ul>
 * <li> asv.h - contains the programming interface for struct Asv and describes
 * the functions that operate on an instance of Asv;
 * <li> wave.h - describes the struct Wave, which represents an irregular sea
 * surface, and the functions that operate on an instance of struct Wave;
 * <li> regular_wave.h - describes the struct Regular_wave, which represents the
 * component waves in the irregular sea (struct Wave), and the functions that
 * operate on an instance of struct Rugular_wave.
 * </ul>
 *
 *
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
 *  // Initialise the sea state
 *  double wave_ht = 1.2; // significant wave height (m) for the simulated sea.
 *  double wave_heading = 20; // predominant wave heading direction measure in
 *                            // deg with respect to North direction.
 *  int rand_seed = 3;
 *  asv.wave_type = irregular_wave;
 *  wave_init(&asv.wave, wave_ht, wave_heading * PI/180.0, rand_seed); 
 *  
 *  // Initialise the ASV
 *  // Note: call asv_init only after setting all inputs and initialising the
 *  // irregular sea surface with wave_init().
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
 *    double x_coordinate = position.x; // extract x coordinate
 *    double y_coordinate = position.y; // extract y coordinate
 *    double z_coordinate = position.z; // extract z coordinate
 *    
 *    // Get the floating attitude of the vehicle
 *    struct Dimensions attitude = asv.attitude;
 *    double trim_angle = attitude.x; // extract trim angle in radians
 *    double heel_angle = attitude.y; // extract heel angle in radians
 *    double yaw_angle = attitude.z;  // extract yaw angle in radians
 *
 *    // Get the vehicle velocity
 *    double velocity_surge = asv.dynamics.V[surge];
 *    double velocity_sway = asv.dynamics.V[sway];
 *    double velocity_heave = asv.dynamics.V[heave];
 *    double velocity_roll = asv.dynamics.V[roll];
 *    double velocity_pitch = asv.dynamics.V[pitch];
 *    double velocity_yaw = asv.dynamics.V[yaw];
 *    
 *    // Get the vehicle acceleration
 *    double acceleration_surge = asv.dynamics.A[surge];
 *    double acceleration_sway = asv.dynamics.A[sway];
 *    double acceleration_heave = asv.dynamics.A[heave];
 *    double acceleration_roll = asv.dynamics.A[roll];
 *    double acceleration_pitch = asv.dynamics.A[pitch];
 *    double acceleration_yaw = asv.dynamics.A[yaw];
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

#define OUTPUT_BUFFER_SIZE 20000 /*!< Output buffer size. */

#endif // CONSTANTS_H
