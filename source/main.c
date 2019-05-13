#include <stdio.h>
#include "wave.h"
#include "constants.h"

int main()
{
  double wind_speed = 7.5; // m/s
  double wind_fetch = 100.0 * 1000.0; // m
  double wind_direction = PI/6.0; // radians
  
  struct Wave wave;
  wave_init(&wave, wind_speed, wind_direction);

  // Print the wave stats
  return 0;
}
