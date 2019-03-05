#include <gtest/gtest.h>
#include "regular_wave.h"
#include "exception.h"

using namespace asv_swarm;
using namespace asv_swarm::Hydrodynamics;

TEST(regular_wave, check_constructor)
{
  // Check if the constructor sets the inputs passed correctly.
  Quantity<Units::length> amplitude{2.5*Units::meter};
  Quantity<Units::frequency> frequency{0.1*Units::hertz};
  Quantity<Units::plane_angle> direction{0.0*Units::radian};
  Quantity<Units::plane_angle> phase{Constant::PI*Units::radian};
  Regular_wave wave{amplitude, frequency, direction, phase};
  EXPECT_EQ(2.5, wave.get_wave_amplitude().value());
  EXPECT_EQ(0.1, wave.get_wave_frequency().value());
  EXPECT_EQ(0.0, wave.get_direction().value());
  EXPECT_EQ(Constant::PI, wave.get_phase().value());
}

TEST(regular_wave, check_wave_properties)
{
  //Check the properties of the wave that are not inputs.
  Quantity<Units::length> amplitude{2.5*Units::meter};
  Quantity<Units::frequency> frequency{0.1*Units::hertz};
  Quantity<Units::plane_angle> direction{0.0*Units::radian};
  Quantity<Units::plane_angle> phase{Constant::PI*Units::radian};
  Regular_wave wave{amplitude, frequency, direction, phase};
  EXPECT_NEAR(6163.804, wave.get_wave_length().value(), 0.001);
  EXPECT_NEAR(0.00101, wave.get_wave_number().value(), 0.00001);
  EXPECT_NEAR(62.831, wave.get_wave_period().value(), 0.001);
}
 
TEST(regular_wave, check_wave_elevation)
{
  //Check the wave elevation.
  Quantity<Units::length> amplitude{2.5*Units::meter};
  Quantity<Units::frequency> frequency{0.1*Units::hertz};
  Quantity<Units::plane_angle> direction{(Constant::PI/4)*Units::radian};
  Quantity<Units::plane_angle> phase{(Constant::PI/6)*Units::radian};
  Regular_wave wave{amplitude, frequency, direction, phase};
  EXPECT_NEAR(2.4361, 
              wave.get_wave_elevation(1.5 * Units::meter,
                                      2.5 * Units::meter,
                                      3.0*Units::seconds).value(), 
              0.0001); 
}

TEST(regular_wave, check_exception_invalid_amplitude)
{
  //Check the wave elevation.
  Quantity<Units::length> amplitude{0.0*Units::meter};
  Quantity<Units::frequency> frequency{0.1*Units::hertz};
  Quantity<Units::plane_angle> direction{(Constant::PI/4)*Units::radian};
  Quantity<Units::plane_angle> phase{(Constant::PI/6)*Units::radian};
  EXPECT_THROW(Regular_wave(amplitude, frequency, direction, phase), 
               Exception::ValueError);  
}

TEST(regular_wave, check_exception_invalid_frequency)
{
  //Check the wave elevation.
  Quantity<Units::length> amplitude{2.5*Units::meter};
  Quantity<Units::frequency> frequency{0.0*Units::hertz};
  Quantity<Units::plane_angle> direction{(Constant::PI/4)*Units::radian};
  Quantity<Units::plane_angle> phase{(Constant::PI/6)*Units::radian};
  EXPECT_THROW(Regular_wave(amplitude, frequency, direction, phase), 
               Exception::ValueError);  
}


   
