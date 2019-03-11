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
namespace Hydrodynamics
{
/**
 * This class generates a collection of waves such that the resultant irregular 
 * wave formed by linear super-positioning of all the regular waves have the 
 * required statistical properties of the sea-state simulated.
 */
class Wave_spectrum
{
public:
  /**
   * Constructor. Default values set by the constructor: 
   * - number of frequency bands in the wave spectrum = 20 
   * - number of directions of wave heading in the wave spectrum = 10 
   * - direction range = (wind direction - PI/2, wind direction + PI/2)  
   * @param wind_fetch length in meter. Should be greater than 0.
   * @param wind_speed in meter/sec. Should be greater than 0.
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
  void set_freq_band_count(unsigned int count);

  /**
   * Method to override the number of discrete directions in the wave spectrum.
   * @param count, the number of directions in the spectrum.
   */
  void set_wave_direction_count(unsigned int count);

  /**
   * Return a table containing waves. Each row of the table is an array of wave
   * for a given direction.
   * @return table of waves for all directions.
   */
  std::vector<std::vector<Regular_wave>>& get_spectrum(){
    return spectrum;}

  /**
   * Returns a list of directions considered in the spectrum.
   */
  std::vector<Quantity<Units::plane_angle>>& get_directions(){
    return wave_direction_list;}

  /**
   * Returns a list of frequencies considered in the spectrum.
   */
  std::vector<Quantity<Units::frequency>>& get_frequencies(){
    return freq_band_list;}

  /**
   * Return the wind speed in m/s.
   */
  Quantity<Units::velocity> get_wind_speed(){
    return wind_speed;}
  
  /**
   * Returns the wind fetch in m.
   */
  Quantity<Units::length> get_wind_fetch(){
    return wind_fetch;}

  /**
   * Return the wind direction in radians.
   */
  Quantity<Units::plane_angle> get_wind_direction(){
    return wind_direction;}

  /**
   * Return the spectral peak frequency.
   */
  Quantity<Units::frequency> get_spectral_peak_frequency(){
    return spectral_peak_freq;}

  /**
   * Return the minimum lower limit (0.1%) of the spectral energy threshold.
   */
  Quantity<Units::frequency> get_min_frequency(){
    return min_freq;}

  /**
   * Return the upper limit (99.9%) of the spectral energy threshold.
   */
  Quantity<Units::frequency> get_max_frequency(){
    return max_freq;}

  /**
   * Return the significant wave height.
   */
  Quantity<Units::length> get_significant_wave_height(){
    return significant_wave_height;}


protected:
  /**
   * Method to generate all the regular waves forming the wave spectrum.
   * @return true if spectrum initialised.
   */
  void set_wave_spectrum();

  Quantity<Units::length> wind_fetch; /* The length of fetch in meter. */
  Quantity<Units::velocity> wind_speed; /* Wind speed in m/sec. */
  Quantity<Units::plane_angle> wind_direction; /* Predominant wind direction.*/
  std::vector<std::vector<Regular_wave>> spectrum; /* Each row represents
                                                     wave spectrum for a single
                                                     direction.*/
  int freq_band_count; /* Number of frequency bands in the spectrum.*/
  int wave_direction_count; /* Number of wave direction bands in the spectrum */
  Quantity<Units::frequency> min_freq; /* Lower limit (0.1%) of spectral energy 
                                          threshold. */
  Quantity<Units::frequency> max_freq;/* Upper limit (99.9%) of the spectral
                                         energy threshold. */
  Quantity<Units::frequency> spectral_peak_freq; /* Spectral peak frequency. */
  Quantity<Units::length> significant_wave_height; /*Significant wave height*/
  Quantity<Units::plane_angle> wave_direction_min; /* Minimum angle in spectrum 
                                                      for wave heading. Default 
                                                      value is 
                                                      wind_direction - PI/2. */
  Quantity<Units::plane_angle> wave_direction_max; /* Maximum angle in spectrum 
                                                      for wave heading. Default 
                                                      value is 
                                                      wind_direction + PI/2. */
  std::vector<Quantity<Units::plane_angle>> wave_direction_list; /* A list of 
                                                      all wave directions 
                                                      considered in the 
                                                      spectrum.*/
  std::vector<Quantity<Units::frequency>> freq_band_list; /* A list of all
                                                      frequency bands considered
                                                      in the spectrum. */
  /* Spectral parameters */
  double A;
  double B;
  double alpha;
  double beta;
  double gamma;
  
}; // class Wave_spectrum

} // namespace Hydrodynamics
} // namespace asv_swarm


#endif // WAVE_SPECTRUM_H
