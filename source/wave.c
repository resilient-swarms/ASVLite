#include <stddef.h>
#include <malloc.h>
#include <string.h>
#include <stdlib.h> 
#include <stdbool.h>
#include <math.h>
#include "constants.h"
#include "geometry.h"
#include "regular_wave.h"
#include "wave.h"

struct Wave
{
  // Input variables
  // ---------------
  double significant_wave_height;      //!< Input variable. Significant wave height in meter.
  double heading;                      //!< Input variable. Wave heading in radians.
  long random_number_seed;             //!< Input variable. Random number generator seed. 
  int count_wave_spectral_directions;  //!< Input variable. Number of direction bands in the wave spectrum. 
  int count_wave_spectral_frequencies; //!< Input variable. Number of frequency bands in the wave spectrum.

  // Output variables
  // ----------------
  const struct Regular_wave** spectrum;    //!< Output variable. Table of regular waves in the irregular sea.
  double min_spectral_frequency;    //!< Output variable. Lower limit (0.1%) of spectral energy threshold.
  double max_spectral_frequency;    //!< Output variable. Upper limit (99.9%) of spectral energy threshold.
  double peak_spectral_frequency;   //!< Output variable. Spectral peak frequency in Hz.
  double min_spectral_wave_heading; //!< Output variable. Minimum angle, in radians, in spectrum for wave heading.
  double max_spectral_wave_heading; //!< Output variable. Maximum angle, in radians, in spectrum for wave heading.
  char* error_msg;                  //!< Output variable. Error message, if any.
};

static void set_error_msg(struct Wave* wave, 
                          const char* msg)
{
  wave->error_msg = (char*)malloc(sizeof(char) * strlen(msg));
  strcpy(wave->error_msg, msg);
}

static void clear_msg(struct Wave* wave)
{
  free(wave->error_msg);
}

const struct Wave* wave_new(const double sig_wave_ht,
                            const double wave_heading, 
                            const long rand_seed,
                            const int count_wave_spectral_directions,
                            const int count_wave_spectral_frequencies)
{
  struct Wave* wave = NULL;

  if(sig_wave_ht > 0.0 && 
     count_wave_spectral_directions > 1 && 
     count_wave_spectral_frequencies > 1)
  {
    // Initialise the pointers...
    wave = (struct Wave*)malloc(sizeof(struct Wave));
    wave->spectrum = (const struct Regular_wave**)malloc(sizeof(struct Regular_wave*) * count_wave_spectral_directions * count_wave_spectral_frequencies); 
    wave->error_msg = NULL;
    // ... and then the other member variables.
    wave->heading = wave_heading;
    wave->min_spectral_wave_heading = wave->heading - PI/2.0;
    wave->max_spectral_wave_heading = wave->heading + PI/2.0;
  
    // wave directions should be in the range (0, 2PI)
    wave->min_spectral_wave_heading = fmod(wave->min_spectral_wave_heading,  2.0*PI); 
    wave->max_spectral_wave_heading = fmod(wave->max_spectral_wave_heading , 2.0*PI);
    
    // Bretschneider spectrum
    // Ref: Proceedings of the 23rd ITTC - Vol II, Table A.2, A.3.
    // S(f) = (A/f^5) exp(-B/f^4)
    // A = alpha g^2 (2 PI)^-4
    // B = beta (2PI U/g)^-4
    // alpha = 0.0081
    // beta = 0.74
    // f_p = 0.946 B^(1/4)
    // U = wind speed in m/s
    double alpha = 0.0081;
    double beta = 0.74;
    double A = alpha * G*G * pow(2.0*PI, -4.0);
    double H_s = sig_wave_ht;
    double B = 4.0*alpha*G*G / (pow(2.0*PI, 4.0)* H_s*H_s);
    double f_p = 0.946 * pow(B, 0.25);

    wave->significant_wave_height = H_s;
    wave->peak_spectral_frequency = f_p;
    wave->min_spectral_frequency = 0.652 * f_p;
    wave->max_spectral_frequency = 5.946 * f_p;

    // Create regular waves
    bool has_NULL_in_spectrum = false; // A flag that will be set to true if any of the created component wave turns out to be NULL.
    // For each heading angle
    double wave_heading_step_size = PI/(count_wave_spectral_frequencies - 1);
    double mu = -PI/2.0;
    for(int i = 0; i < count_wave_spectral_directions; ++i)
    {
      mu += wave_heading_step_size;
      double frequency_step_size = (wave->max_spectral_frequency - 
                                    wave->min_spectral_frequency) /
                                    (count_wave_spectral_frequencies - 1);
      for(int j = 0; j < count_wave_spectral_frequencies; ++j)
      {
        double f = wave->min_spectral_frequency + j * frequency_step_size;
        double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * frequency_step_size;
      
        // Direction function G = (2/PI) * cos(mu)*cos(mu) * delta_mu
        // delta_mu = wave_heading_step_size
        double G_spectrum = (2.0/PI) * cos(mu)*cos(mu) * wave_heading_step_size;
      
        // Create a wave
        double amplitude = sqrt(2.0 * S * G_spectrum); 
        wave->random_number_seed = rand_seed;
        srand(wave->random_number_seed);
        double phase = rand(); 
        double wave_heading = mu + wave->heading;
        // wave directions should be in the range (0, 2PI)
        wave_heading = fmod(wave_heading, 2.0 * PI);
        const struct Regular_wave* regular_wave = regular_wave_new(amplitude, f, phase, wave_heading);
        if(regular_wave)
        {
          wave->spectrum[i*count_wave_spectral_directions + j] = regular_wave;
        }
        else 
        {
          // Error encountered when creating a regular waves for the wave spectrum.
          has_NULL_in_spectrum = true;
          break;
        }
      }
      if(has_NULL_in_spectrum)
      {
        break;
      }
    }
    // Check if spectrum is valid
    if(has_NULL_in_spectrum)
    {
      // One or more of the regular wave is NULL. 
      // This is an invalid spectrum.
      wave_delete(wave);
      wave = NULL;
    }
    else
    {
      // Spectrum is OK. Proceed. Nothing to do here.
    }
  }
  
  return wave;
}

void wave_delete(const struct Wave* wave)
{
  for(int i = 0; i < (wave->count_wave_spectral_directions * wave->count_wave_spectral_frequencies); ++i)
  {
    const struct Regular_wave* regular_wave = wave->spectrum[i];
    if(regular_wave)
    {
      regular_wave_delete(regular_wave);
    }
  }
  
  free(wave->spectrum);
  free(wave->error_msg);
  free((struct Wave*)wave);
}

double wave_get_elevation(const struct Wave* wave, 
                          struct Cartesian_coordinate_3D location,
                          double time)
{
  clear_msg((struct Wave*)wave);
  double elevation = 0.0;

  // check if wave is nullptr or time is negative
  if(wave && time >= 0.0)
  {
    for(int i = 0; i < wave->count_wave_spectral_directions; ++i)
    {
      for(int j = 0; j < wave->count_wave_spectral_frequencies; ++j)
      {
        const struct Regular_wave* regular_wave = wave->spectrum[i*wave->count_wave_spectral_directions + j];
        // We do not expect a NULL regular_wave, but still check.
        if(!regular_wave)
        {
          // Something really wrong happened. 
          set_error_msg((struct Wave*)wave, "NULL encountered in spectrum.");
          return 0.0;
        }
        double regular_wave_elevation = regular_wave_get_elevation(regular_wave, location, time);
        const char* error_msg = regular_wave_get_error_msg(regular_wave);
        if(!error_msg)
        {
          elevation += regular_wave_elevation;
        }
        else
        {
          // Something went wrong when getting wave elevation for the regular_wave.
          set_error_msg((struct Wave*)wave, error_msg);
          return 0.0;
        }
      }
    }
  }
  else
  {
    set_error_msg((struct Wave*)wave, "wave should not be NULL and time >= 0.0.");
  }
  return elevation;
}

int wave_get_count_wave_spectral_directions(const struct Wave* wave)
{
  clear_msg((struct Wave*)wave);
  return wave->count_wave_spectral_directions;
}

int wave_get_count_wave_spectral_frequencies(const struct Wave* wave)
{
  clear_msg((struct Wave*)wave);
  return wave->count_wave_spectral_frequencies;
}

const struct Regular_wave* wave_get_regular_wave_at(const struct Wave* wave, int d, int f)
{
  clear_msg((struct Wave*)wave);
  const struct Regular_wave* regular_wave = NULL;
  if(d >= 0 && d < wave->count_wave_spectral_directions &&
     f >= 0 && f < wave->count_wave_spectral_frequencies)
  {
    regular_wave = wave->spectrum[d*wave->count_wave_spectral_directions + f];
  }
  else
  {
    // Incorrect index
    set_error_msg((struct Wave*)wave, "Index invalid for the wave spectrum.");
  }

  return regular_wave;
}
