/**
 * This header file contains programming interface for defining regular ocean
 * waves.
 */
#ifndef REGULAR_WAVE_H
#define REGULAR_WAVE_H

/**
 * This class defines a regular sinusoidal wave. 
 */
class Regular_wave
{
public:
  /**
   * Constructor for Regular_wave. 
   * @param amplitude of wave in meter.
   * @param frequency of wave in radian/sec.
   * @param direction wave heading in degree with respect to x-axis; angle
   * measured positive anti-clockwise. 
   * @param phase angle of the wave in radian.
   */
  Regular_wave( double amplitude, 
                double frequency, 
                double direction,
                double phase);
  
  /**
   * Method to get the wave length.
   * @return wave length in m.
   */
  double get_wave_length();

  /**
   * Method to get wave number.
   * @return wave number.
   */
  double get_wave_number();

  /**
   * Method to get wave period.
   * @return wave period in sec.
   */
  double get_wave_period();


private:
  const double G = 9.81; // acceleration due to gravity in m/s2
  double amplitude; // amplitude of the wave in meter
  double frequency; // circular frequency of the wave
  double phase; // phase angle of the wave in rad
  double direction; // direction of propagation of the wave with respect to
                    // x-axis. Angle measured positive anti-clockwise.
  double wave_length; // wave length in meter
  double wave_number; // wave number
  double wave_period; // wave period in sec
};

#endif // REGULAR_WAVE_H
