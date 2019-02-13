#include "regular_wave.h"
#include "exception.h"
#include <boost/units/cmath.hpp>

using namespace asv_swarm;

Regular_wave::Regular_wave( Quantity<Units::length> amplitude, 
                            Quantity<Units::frequency> frequency, 
                            Quantity<Units::plane_angle> direction,
                            Quantity<Units::plane_angle> phase) :
  amplitude {amplitude},
  frequency {frequency},
  direction {direction},
  phase {phase},
  wave_length {(2 * Const::PI * Const::G) / pow<2>(frequency)},
  wave_number {(2 * Const::PI)/wave_length},
  wave_period {(2.0*Const::PI)/frequency}
{
  // Check if inputs are valid. If not throw ValueError.
  if( amplitude.value() <= 0.0 ||
      frequency.value() <= 0.0 )
  {
    throw ValueError("Constructor error. Class: Regular_wave."
                     "Invalid input.");
  }
}

Quantity<Units::length> Regular_wave::get_wave_elevation(
                                                    Quantity<Units::length> x,
                                                    Quantity<Units::length> y,
                                                    Quantity<Units::time> t)
{
  /* The formula for calculating wave elevation is:
   * elevation = amplitude * cos(A - B + phase)
   * where:
   * A = wave_number * (x * cos(direction) + y * sin(direction))
   * B = frequency * t
   */
  
  Quantity<Units::plane_angle> A {(wave_number * (x * cos(direction) +
                                                  y * sin(direction))) *
                                                  Units::radian};
  Quantity<Units::plane_angle> B {frequency * t * Units::radian};
  return amplitude* cos(A - B + phase);
}
