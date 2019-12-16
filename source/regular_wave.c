#include <math.h>
#include "regular_wave.h"
#include "constants.h"

int regular_wave_init(struct Regular_wave* wave,
                      double amplitude,
                      double frequency,
                      double phase_lag,
                      double direction)
{
  // Check if null pointer passed. If yes then return 1.
  if(!wave)
  {
    return 1;
  }
  // Check if amplitude is non-zero positive value. 
  if(amplitude <= 0.0)
  {
    return 2;
  }
  // Check if frequency is non-zero positive value.
  if(frequency <= 0.0)
  {
    return 3;
  }
  wave->amplitude = amplitude; 
  wave->frequency = frequency;
  wave->phase_lag = phase_lag;
  wave->direction = direction;
  wave->time_period = (1.0/frequency);
  wave->wave_length = (G * wave->time_period * wave->time_period)/(2.0 * PI);
  wave->wave_number = (2.0 * PI)/wave->wave_length;
  return 0; // no error return. 
}

double regular_wave_get_phase(struct Regular_wave* wave, 
                              struct Dimensions* location, 
                              double time)
{
  // Check if wave is nullptr or time is -ve.
  if(!wave || time < 0.0)
  {
    return 0;
  }
  // elevation = amplitude * cos(A - B + phase)
  // where:
  // A = wave_number * (x * cos(direction) + y * sin(direction))
  // B = 2 * PI * frequency * time
  //
  // NOTE:
  // In the coordinate system that we use here, angular measurements are made 
  // with respect to north which is represented by y-axis and not x-axis.
  // Therefore the above formula needs to be modified as:
  // A = wave_number * (x * sin(direction) + y * cos(direction))
  
  double A = wave->wave_number * (location->x * sin(wave->direction) + 
                                  location->y * cos(wave->direction));
  double B = 2.0 * PI * wave->frequency * time;
  return (A - B + wave->phase_lag);

}

double regular_wave_get_elevation(struct Regular_wave* wave,
                                  struct Dimensions* location,
                                  double time)
{
  // Check if wave is nullptr or if time is -ve. 
  if(!wave || time < 0.0)
  {
    return 0;
  }
  double wave_phase = regular_wave_get_phase(wave, location, time); 
  return wave->amplitude * cos(wave_phase);
} 

double regular_wave_get_pressure_amp(struct Regular_wave* wave, double z)
{
  // Check if wave is nullptr.
  if(!wave)
  {
    return 0;
  }
  double P = SEA_WATER_DENSITY* G* wave->amplitude* exp(wave->wave_number* z);
  return P;
}
