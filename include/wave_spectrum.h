/**
 * This header file contains programming interface for defining a wave spectrum.
 */

#ifndef WAVE_SPECTRUM_H
#define WAVE_SPECTRUM_H

#include "units_and_constants.h"
#include "regular_wave.h"
#include <vector>

/**
 * This class generates a list of waves such that the resultant irregular wave 
 * formed by linear super-positioning of all the regular waves have a specific
 * statistical property in-line with the sea-state simulated. The wind direction
 * is measured from 0 deg to 360 deg. Wind blowing from north is considered as 
 * 0 deg, wind blowing from east is 90 deg and wind blowing from west is 270deg.
 */
class Wave_spectrum
{
  public:
  /**
   * Constructor. 
   * @param fetch length in Boost::units::si::length. Should be greater than 0.
   * @param wind_speed in Boost::units::si::velocity. Should be greater than 0.
   * @param wind_direction is the predominant wind direction measured in radian.
   * Wind direction should be within the range (0, 2PI).
   */
  Wave_spectrum( Quantity<Units::length> fetch, 
                 Quantity<Units::velocity> wind_speed,
                 Quantity<Units::plane_angle> wind_direction);

  /**
   * Method to set the number of discrete frequencies in the spectrum.
   * @param count, the number of frequencies in the spectrum.
   * @return true if value set else returns false.
   */
  bool set_frequency_cont(unsigned int count);

  /**
   * Method to set the number of discrete directions in the spectrum.
   * @param count, the number of directions in the spectrum.
   * @return true if value set else returns false.
   */
  bool set_direction_count(unsigned int count);

  /**
   * Method to initialise the wave spectrum.
   * @return true if spectrum initialised.
   */
  bool set_wave_spectrum();

  private:
  Quantity<Units::length> fetch; /* The length of fetch in meter. */
  Quantity<Units::velocity> wind_speed; /* Wind speed in m/sec. */
  Quantity<Units::plane_angle> wind_direction; /* Predominant wind direction.*/
  std::vector<std::vector<Regular_wave>> spectrum; /* Each row represents
                                                     wave spectrum for a single
                                                     direction.*/
  int f_count; /* Number of discrete frequencies in the spectrum.
                  Default value is 50. */
  int d_count; /* Number of discrete directions in the spectrum 
                  Default value is 180. */ 
  Quantity<Units::frequency> min_freq; /* Minimum frequency in the spectrum.
                                          Default value is 0.3 Hz. */
  Quantity<Units::frequency> max_freq;/* Maximum frequency in the spectrum.
                                         Default value is 6.0 Hz. */
  Quantity<Units::plane_angle> min_angle; /* Minimum angle in spectrum. 
                                             Default value is 
                                             wind_direction - PI/2. */
  Quantity<Units::plane_angle> max_angle; /* Maximum angle in spectrum.
                                             Default value is
                                             wind_direction + PI/2. */


};


#endif // WAVE_SPECTRUM_H
