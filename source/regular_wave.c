#include <stddef.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include "regular_wave.h"
#include "constants.h"

struct Regular_wave
{
  // Input variables
  // ---------------
  double amplitude; //!< Input variable. Amplitude of the wave in meter.
  double frequency; //!< Input variable. Frequency of the wave in Hz.
  double phase_lag; //!< Input variable. Phase lag of the wave in radian.
  double direction; //!< Input variable. Direction of propagation of the wave 
                    //!< with respect to geographic north. Angle measured
                    //!< positive in clockwise direction such that east is at
                    //!< PI/2 radians to !< north.
  
  // Output variables
  // ----------------
  double time_period; //!< Output variable. Time period of the wave in seconds.
  double wave_length; //!< Output variable. Wave length in meter.
  double wave_number; //!< Output variable. Wave number. Dimensionless.
  char* error_msg;    //!< Output variable. Error message, if any. 
}; 


static void set_error_msg(struct Regular_wave* regular_wave, 
                          const char* msg)
{
  regular_wave->error_msg = (char*)malloc(sizeof(char) * strlen(msg));
  strcpy(regular_wave->error_msg, msg);
}

static void clear_msg(struct Regular_wave* regular_wave)
{
  free(regular_wave->error_msg);
}


const struct Regular_wave* regular_wave_new(const double amplitude, 
                                            const double frequency, 
                                            const double phase_lag, 
                                            const double direction)
{
  struct Regular_wave* regular_wave = NULL;
  
  // Check if both amplitude and frequency are non-zero positive values. 
  if(amplitude > 0.0 && frequency > 0.0)
  {
    regular_wave = (struct Regular_wave*)malloc(sizeof(struct Regular_wave));
    regular_wave->amplitude = amplitude; 
    regular_wave->frequency = frequency;
    regular_wave->phase_lag = phase_lag;
    regular_wave->direction = direction;
    regular_wave->time_period = (1.0/frequency);
    regular_wave->wave_length = (G * regular_wave->time_period * regular_wave->time_period)/(2.0 * PI);
    regular_wave->wave_number = (2.0 * PI)/regular_wave->wave_length;
    regular_wave->error_msg = NULL;
  }
  
  return regular_wave;  
}


void regular_wave_delete(const struct Regular_wave* regular_wave)
{
  free(regular_wave->error_msg);
  free((struct Regular_wave*)regular_wave);
}

const char* regular_wave_get_error_msg(const struct Regular_wave* regular_wave)
{
  return regular_wave->error_msg;
}

double regular_wave_get_phase(const struct Regular_wave* const regular_wave, 
                              const struct Cartesian_coordinate_3D location, 
                              const double time)
{
  clear_msg((struct Regular_wave*)regular_wave);
  double phase = 0.0;

  // Check if regular_wave is nullptr or time is -ve.
  if(regular_wave && time >= 0.0)
  {
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
    double wave_number = regular_wave->wave_number;
    double direction   = regular_wave->direction;
    double frequency   = regular_wave->frequency;
    double phase_lag   = regular_wave->phase_lag;
    double A = wave_number * (location.x * sin(direction) + location.y * cos(direction));
    double B = 2.0 * PI * frequency * time;
    phase = (A - B + phase_lag);
  }
  else
  {
    set_error_msg((struct Regular_wave*)regular_wave, "regular_wave should not be NULL and time >= 0.0.");
  }

  return phase;
}

double regular_wave_get_elevation(const struct Regular_wave* const regular_wave,
                                  const struct Cartesian_coordinate_3D location,
                                  const double time)
{
  clear_msg((struct Regular_wave*)regular_wave);
  double elevation = 0.0;
  
  // Check if wave is nullptr or if time is -ve. 
  if(regular_wave && time >= 0.0)
  {
    double wave_phase = regular_wave_get_phase(regular_wave, location, time); 
    double amplitude = regular_wave->amplitude;
    elevation = amplitude * cos(wave_phase);
  }
  else
  {
    set_error_msg((struct Regular_wave*)regular_wave, "regular_wave should not be NULL.");
  }
  
  return elevation;
} 

double regular_wave_get_pressure_amp(const struct Regular_wave* regular_wave, double z)
{
  clear_msg((struct Regular_wave*)regular_wave);
  double P = 0.0;
  
  // Check if wave is nullptr.
  if(regular_wave)
  {
    double amplitude = regular_wave->amplitude;
    double wave_number = regular_wave->wave_number;
    P = SEA_WATER_DENSITY* G* amplitude* exp(wave_number* z);
  }
  else
  {
    set_error_msg((struct Regular_wave*)regular_wave, "regular_wave should not be NULL.");
  }

  return P;
}
