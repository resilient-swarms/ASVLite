#include "regular_wave.h"
#include <boost/units/cmath.hpp>

using namespace boost::units;

  Regular_wave::Regular_wave( Quantity<Units::length> amplitude, 
                              Quantity<Units::frequency> frequency, 
                              Quantity<Units::plane_angle> direction,
                              Quantity<Units::plane_angle> phase)
  {
    this->amp = amplitude;
    this->freq = frequency;
    this->direction = direction;
    this->phase = phase;
    this->wave_length = (2 * Const::PI * Const::G) / pow<2>(freq);
    this->wave_number = (2 * Const::PI)/wave_length;
    this->wave_period = sqrt((2 * Const::PI * wave_length)/ Const::G);
  }