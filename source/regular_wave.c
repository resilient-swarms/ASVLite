#include "regular_wave.h"
#include "constants.h"

double regular_wave_get_time_period(struct Regular_wave* wave)
{
  return (1.0/wave->frequency); // time in seconds
}

double regular_wave_get_length(struct Regular_wave* wave)
{
  double t = regular_wave_get_time_period(wave);
  return (G* t*t/ (2.0*PI)); // length in meter 
}

double regular_wave_get_wave_number(struct Regular_wave* wave)
{
  double l = regular_wave_get_length(wave);
  return ((2.0*PI)/l);
}
