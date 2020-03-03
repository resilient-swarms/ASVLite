/*! \mainpage ASV-Swarm
 * This document outlines the programming interface of ASV-Swarm to use it as a
 * software library in an application. 
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

#define COUNT_WAYPOINTS_MAX 20 /*!< Maximum number of waypoints through which the 
                                    ASV will navigate. */

#define OUTPUT_BUFFER_SIZE 200000 /*!< Output buffer size. */

#endif // CONSTANTS_H
