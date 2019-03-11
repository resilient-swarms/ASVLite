#include "regular_wave.h"
#include "exception.h"
#include <boost/units/cmath.hpp>

using namespace asv_swarm;
using namespace Hydrodynamics;

Regular_wave::Regular_wave( Quantity<Units::length> amplitude, 
                            Quantity<Units::frequency> frequency, 
                            Quantity<Units::plane_angle> direction,
                            Quantity<Units::plane_angle> phase) :
  amplitude {amplitude},
  frequency {frequency},
  direction {direction},
  phase {phase}
{
  // Check if inputs are valid. If not throw ValueError.
  if( amplitude.value() <= 0.0 ||
      frequency.value() <= 0.0 )
  {
    throw Exception::ValueError("Constructor error. Class: Regular_wave."
                                "Invalid input.");
  }
  wave_length = (Constant::G/(2.0 * Constant::PI))*pow<2>(1.0/frequency);
  wave_number = (2.0 * Constant::PI)/wave_length;
  wave_period = 1.0/frequency;
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
  Quantity<Units::plane_angle> B {2*Constant::PI*frequency * t * Units::radian};
  return amplitude* cos(A - B + phase);
}
