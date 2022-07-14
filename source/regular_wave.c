#include <stddef.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include "regular_wave.h"
#include "constants.h"

const char* error_msg_null_regular_wave = "Argument regular wave cannot be a null pointer.";
const char* error_msg_negative_time     = "Argument time cannot be negative.";

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


static void set_error_msg(struct Regular_wave* regular_wave, const char* msg)
{
  regular_wave->error_msg = (char*)malloc(sizeof(char) * strlen(msg));
  strcpy(regular_wave->error_msg, msg);
}

static void clear_msg(struct Regular_wave* regular_wave)
{
  if(regular_wave)
  {
    if(regular_wave->error_msg)
    {
      free(regular_wave->error_msg);
      regular_wave->error_msg = NULL;
    }
  }
}


struct Regular_wave* regular_wave_new(const double amplitude, 
                                      const double frequency, 
                                      const double phase_lag, 
                                      const double direction)
{  
  // Check if both amplitude and frequency are non-zero positive values. 
  if(amplitude > 0.0 && frequency > 0.0)
  {
    struct Regular_wave* regular_wave = NULL;
    // Initialise the pointers ...
    if(regular_wave = (struct Regular_wave*)malloc(sizeof(struct Regular_wave)));
    {
      regular_wave->error_msg = NULL;
      // ... and then the other member variables.
      regular_wave->amplitude = amplitude; 
      regular_wave->frequency = frequency;
      regular_wave->phase_lag = phase_lag;
      regular_wave->direction = direction;
      regular_wave->time_period = (1.0/frequency);
      regular_wave->wave_length = (G * regular_wave->time_period * regular_wave->time_period)/(2.0 * PI);
      regular_wave->wave_number = (2.0 * PI)/regular_wave->wave_length;
      return regular_wave;
    }
  }
  
  return NULL;  
}


void regular_wave_delete(struct Regular_wave* regular_wave)
{
  if(regular_wave)
  {
    free(regular_wave->error_msg);
    free(regular_wave);
    regular_wave = NULL;
  }
}


const char* regular_wave_get_error_msg(const struct Regular_wave* regular_wave)
{
  if(regular_wave)
  {
    return regular_wave->error_msg;
  }
  return NULL;
}


double regular_wave_get_amplitude(const struct Regular_wave* regular_wave)
{
  if(regular_wave)
  {
    clear_msg((struct Regular_wave*)regular_wave);
    return regular_wave->amplitude;
  }
  set_error_msg((struct Regular_wave*)regular_wave, error_msg_null_regular_wave);
  return 0.0;
}


double regular_wave_get_frequency(const struct Regular_wave* regular_wave)
{
  if(regular_wave)
  {
    clear_msg((struct Regular_wave*)regular_wave);
    return regular_wave->frequency;
  }
  set_error_msg((struct Regular_wave*)regular_wave, error_msg_null_regular_wave);
  return 0.0;
}


double regular_wave_get_direction(const struct Regular_wave* regular_wave)
{
  if(regular_wave)
  {
    clear_msg((struct Regular_wave*)regular_wave);
    return regular_wave->direction;
  }
  set_error_msg((struct Regular_wave*)regular_wave, error_msg_null_regular_wave);
  return 0.0;
}


double regular_wave_get_phase(const struct Regular_wave* const regular_wave, 
                              const union Coordinates_3D location, 
                              const double time)
{
  if(regular_wave)
  {
    clear_msg((struct Regular_wave*)regular_wave);

    // Check if time is -ve.
    if(time >= 0.0)
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
      double A = wave_number * (location.keys.x * sin(direction) + location.keys.y * cos(direction));
      double B = 2.0 * PI * frequency * time;
      return (A - B + phase_lag);
    }
    else
    {
      set_error_msg((struct Regular_wave*)regular_wave, error_msg_negative_time);
      return 0.0;
    }
  }
  set_error_msg((struct Regular_wave*)regular_wave, error_msg_null_regular_wave);
  return 0.0;
}


double regular_wave_get_elevation(const struct Regular_wave* const regular_wave,
                                  const union Coordinates_3D location,
                                  const double time)
{
  if(regular_wave)
  {
    clear_msg((struct Regular_wave*)regular_wave);
    
    // Check if time is -ve. 
    if(time >= 0.0)
    {
      double wave_phase = regular_wave_get_phase(regular_wave, location, time); 
      const char* error_msg = regular_wave_get_error_msg(regular_wave);
      if(error_msg)
      {
        // Something went wrong when getting wave phase.
        set_error_msg((struct Regular_wave*)regular_wave, error_msg);
        return 0.0;
      }
      else
      {
        double amplitude = regular_wave->amplitude;
        return (amplitude * cos(wave_phase));
      }
    }
    else
    {
      set_error_msg((struct Regular_wave*)regular_wave, error_msg_negative_time);
      return 0.0;
    }
  }
  set_error_msg((struct Regular_wave*)regular_wave, error_msg_null_regular_wave);
  return 0.0;
} 


double regular_wave_get_pressure_amp(const struct Regular_wave* regular_wave, double z)
{
  if(regular_wave)
  {
    clear_msg((struct Regular_wave*)regular_wave);
    
    double amplitude = regular_wave->amplitude;
    double wave_number = regular_wave->wave_number;
    return (SEA_WATER_DENSITY* G* amplitude* exp(wave_number* z));
  }
  set_error_msg((struct Regular_wave*)regular_wave, error_msg_null_regular_wave);
  return 0.0;
}
