/**
 * This header file contains programming interface for defining a wave spectrum.
 */

#ifndef WAVE_SPECTRUM_H
#define WAVE_SPECTRUM_H

#include "units_and_constants.h"
#include "regular_wave.h"
#include <vector>

namespace asv_swarm
{
/**
 * This class generates a collection of waves such that the resultant irregular 
 * wave formed by linear super-positioning of all the regular waves have a 
 * required statistical property in-line with the sea-state simulated.
 */
class Wave_spectrum
{
public:
  /**
   * Constructor. Default values set by the constructor: 
   * - number of frequencies in the wave spectrum = 20 
   * - frequency range = (0.3Hz, 6.0Hz)
   * - number of directions in the wave spectrum = 10 
   * - direction range = (wind direction - PI/2, wind direction + PI/2)  
   * @param wind_fetch length in Boost::units::si::length. Should be greater 
   * than 0.
   * @param wind_speed in Boost::units::si::velocity. Should be greater than 0.
   * @param wind_direction is the predominant wind direction measured in radian.
   * Wind direction should be within the range (0, 2PI). 
   */
  Wave_spectrum( Quantity<Units::velocity> wind_speed,
                 Quantity<Units::length> wind_fetch, 
                 Quantity<Units::plane_angle> wind_direction);

  /**
   * Method to override the default number of discrete frequencies in the 
   * wave spectrum.
   * @param count, the number of frequencies in the spectrum.
   */
  void set_frequency_cont(unsigned int count);

  /**
   * Method to override the number of discrete directions in the wave spectrum.
   * @param count, the number of directions in the spectrum.
   */
  void set_direction_count(unsigned int count);

  /**
   * Return a table containing waves. Each row of the table is an array of wave
   * for a given direction.
   * @return table of waves for all directions.
   */
  std::vector<std::vector<Regular_wave>>& get_waves();

  /**
   * Returns a list of directions considered in the spectrum.
   */
  std::vector<Quantity<Units::plane_angle>>& get_directions();

  /**
   * Returns a list of frequencies considered in the spectrum.
   */
  std::vector<Quantity<Units::frequency>>& get_frequencies();

protected:
  /**
   * Method to initialise the wave spectrum.
   * @return true if spectrum initialised.
   */
  void set_wave_spectrum();

  Quantity<Units::length> wind_fetch; /* The length of fetch in meter. */
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
  std::vector<Quantity<Units::plane_angle>> directions_list;
  std::vector<Quantity<Units::frequency>> frequency_list;
}; // class Wave_spectrum
} // namespace asv_swarm


#endif // WAVE_SPECTRUM_H
