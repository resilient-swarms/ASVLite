/**
 * This header file contains programming interface for defining regular ocean
 * waves.
 */
#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

#include "units_and_constants.h"

namespace asv_swarm
{
/**
 * This class defines a regular sinusoidal wave. 
 */
class Regular_wave
{
public:
  /**
   * Constructor for Regular_wave.
   * @param amplitude of wave in meters. Value should be greater than 0.0.
   * @param frequency of wave in radian/sec. Value should be greater than 0.0.
   * @param direction of wave heading in degree with respect to global
   * coordinate system's North direction. The direction of angle measurement
   * should be such that the East is at PI/2 radians to North. 
   * @param phase angle of the wave in radian.
   */
  Regular_wave( Quantity<Units::length> amplitude, 
                Quantity<Units::frequency> frequency, 
                Quantity<Units::plane_angle> direction,
                Quantity<Units::plane_angle> phase);

  /**
   * Method to get the wave length.
   * @return wave length as object of type boost::units::length.
   */
  Quantity<Units::length> get_wave_length(){return wave_length;}

  /**
   * Method to get wave number.
   * @return wave number as object of type boost::units::wavenumber.
   */
  Quantity<Units::wavenumber> get_wave_number(){return wave_number;}

  /**
   * Method to get wave period.
   * @return wave period as object of type boost::units::time.
   */
  Quantity<Units::time> get_wave_period(){return wave_period;}

  /**
   * Method to get wave amplitude.
   * @return wave amplitude as object of type boost::units::length.
   */
  Quantity<Units::length> get_wave_amplitude(){return amplitude;}

  /**
   * Method to get wave frequency.
   * @return wave frequency as object of type boost::units::frequency.
   */
  Quantity<Units::frequency> get_wave_frequency(){return frequency;}

  /**
   * Method to get wave phase. The angle is measured positive anti-clockwise.
   * @return wave phase as object of type boost::units::plane_angle.
   */
  Quantity<Units::plane_angle> get_phase(){return phase;}

  /**
   * Method to get wave heading direction.
   * @return wave direction as object of type boost::units::plane_angle.
   */
  Quantity<Units::plane_angle> get_direction(){return direction;}

  /**
   * Method to get wave elevation at a given location, at a given time.
   * @param x coordinate of the location.
   * @param y coordinate of the location.
   * @param t time measured in seconds since start of simulation.
   * @return the wave elevation as object of type boost::units::length.
   */
  Quantity<Units::length> get_wave_elevation(Quantity<Units::length> x,
                                             Quantity<Units::length> y,
                                             Quantity<Units::time> t);

private:
  Quantity<Units::length> amplitude; /* Amplitude of the wave. */
  Quantity<Units::frequency> frequency; /* Circular frequency of the wave. */
  Quantity<Units::plane_angle> phase; /* Phase angle of the wave in radian. */
  Quantity<Units::plane_angle> direction; /* Direction of propagation of the 
  wave with respect to x-axis. Angle measured positive anti-clockwise. */
  Quantity<Units::length> wave_length; /* Wave length in meter. */
  Quantity<Units::wavenumber> wave_number; /* Wave number */
  Quantity<Units::time> wave_period; /* Wave period in sec. */
}; // class Regular_wave
} // namespace asv_swarm

#endif // REGULAR_WAVE_H
