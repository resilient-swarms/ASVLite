/**
 * This header file contains programming interface for defining regular ocean
 * waves.
 */
#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "units_and_constants.h"

/**
 * This class defines a regular sinusoidal wave. 
 */
class Regular_wave
{

public:
  /**
   * Constructor for Regular_wave.
   * @param amplitude of wave in meters. Data type: boost::units::si::length.
   * @param frequency of wave in radian/sec. Data type: 
   * boost::units::si::frequency.
   * @param direction wave heading in degree with respect to x-axis; angle
   * measured positive anti-clockwise. 
   * @param phase angle of the wave in radian.
   */
  Regular_wave( Quantity<Units::length> amplitude, 
                Quantity<Units::frequency> frequency, 
                Quantity<Units::plane_angle> direction,
                Quantity<Units::plane_angle> phase);

  /**
   * Method to get the wave length.
   * @return wave length in m.
   */
  Quantity<Units::length> get_wave_length();

  /**
   * Method to get wave number.
   * @return wave number.
   */
  Quantity<Units::dimensionless> get_wave_number();

  /**
   * Method to get wave period.
   * @return wave period in sec.
   */
  Quantity<Units::time> get_wave_period();


private:
  Quantity<Units::length> amp; /* Amplitude of the wave. */
  Quantity<Units::frequency> freq; /* Circular frequency of the wave. */
  Quantity<Units::plane_angle> phase; /* Phase angle of the wave in radian. */
  Quantity<Units::plane_angle> direction; /* Direction of propagation of the 
  wave with respect to x-axis. Angle measured positive anti-clockwise. */
  Quantity<Units::length> wave_length; /* Wave length in meter. */
  Quantity<Units::wavenumber> wave_number; /* Wave number */
  Quantity<Units::time> wave_period; /* Wave period in sec. */
};

#endif // REGULAR_WAVE_H
