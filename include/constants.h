#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

#define PI M_PI
#define G 9.81 /* Acceleration due to gravity in m/s */ 
#define SEA_WATER_DENSITY 1025 /* Sea water density in Kg/m3 */
#define AIR_DENSITY 1.2 /* Kg/m3 */

#define COUNT_ASV_SPECTRAL_DIRECTIONS 360
#define COUNT_ASV_SPECTRAL_FREQUENCIES 100

#define COUNT_DOF 6 /* Number of degrees of freedom for the motion of ASV. */
#define COUNT_PROPELLERS_MAX 4 /* Maximum number of propellers an ASV can have*/

#define COUNT_WAYPOINTS_MAX 20 /* Maximum number of waypoints through which the 
                                  ASV will navigate */


#endif // CONSTANTS_H
