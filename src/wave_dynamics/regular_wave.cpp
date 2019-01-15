#include "regular_wave.h"
#include <cmath>

  Regular_wave::Regular_wave( double amplitude, 
                              double frequency, 
                              double direction,
                              double phase)
  {
    this->amplitude = amplitude;
    this->frequency = frequency;
    this->direction = direction;
    this->phase = phase;
    wave_length = (2 * M_PI * G)/pow(frequency, 2);
    wave_number = (2 * M_PI)/wave_length;
    wave_period = sqrt((2 * M_PI * wave_length)/ G);
  }