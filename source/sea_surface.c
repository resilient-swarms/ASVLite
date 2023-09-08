#include <stddef.h>
#include <malloc.h>
#include <string.h>
#include <stdlib.h> 
#include <stdbool.h>
#include <math.h>
#include "constants.h"
#include "geometry.h"
#include "regular_wave.h"
#include "sea_surface.h"
#include "errors.h"

struct Sea_surface
{
  // Input variables
  // ---------------
  double significant_wave_height; //!< Input variable. Significant wave height in meter.
  double heading;                 //!< Input variable. Wave heading in radians.
  long random_number_seed;        //!< Input variable. Random number generator seed. 
  int count_component_waves;      //!< Input variable. Number of component waves with unique freq and heading.  

  // Output variables
  // ----------------
  struct Regular_wave** spectrum;   //!< Output variable. Table of regular waves in the irregular sea. Size is count_component_waves.
  double min_spectral_frequency;    //!< Output variable. Lower limit (0.1%) of spectral energy threshold.
  double max_spectral_frequency;    //!< Output variable. Upper limit (99.9%) of spectral energy threshold.
  double peak_spectral_frequency;   //!< Output variable. Spectral peak frequency in Hz.
  double min_spectral_wave_heading; //!< Output variable. Minimum angle, in radians, in spectrum for wave heading.
  double max_spectral_wave_heading; //!< Output variable. Maximum angle, in radians, in spectrum for wave heading.
  char* error_msg;                  //!< Output variable. Error message, if any.
};

const char* sea_surface_get_error_msg(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    return sea_surface->error_msg;
  }
  return NULL;
}


struct Sea_surface* sea_surface_new(const double sig_wave_ht,
                      const double wave_heading, 
                      const int rand_seed,
                      const int count_component_waves)
{
  struct Sea_surface* sea_surface = NULL;

  if(sig_wave_ht > 0.0 && count_component_waves % 2 != 0) // We always want an odd nunber of component waves
  {
    // Initialise the pointers...
    if(sea_surface = (struct Sea_surface*)malloc(sizeof(struct Sea_surface)))
    {
      sea_surface->spectrum = NULL;
      double* wave_frequencies = (double*)malloc(sizeof(double) * count_component_waves); // List of frequencies for the component waves
      double* wave_freq_step_sizes = (double*)malloc(sizeof(double) * count_component_waves); // Corresponding list of the freq step size
      double* wave_headings = (double*)malloc(sizeof(double) * count_component_waves);  // Corresponding list of headings for the component waves
      sea_surface->spectrum = (struct Regular_wave**)malloc(sizeof(struct Regular_wave*) * count_component_waves);
      if(sea_surface->spectrum && 
         wave_frequencies      && 
         wave_freq_step_sizes  &&
         wave_headings) // Check if memory has been allocated in all cases. 
      {
        sea_surface->error_msg = NULL;
        // ... and then the other member variables.
        sea_surface->random_number_seed = rand_seed;
        srand(sea_surface->random_number_seed);
        sea_surface->heading = normalise_angle_2PI(wave_heading);
        sea_surface->count_component_waves = count_component_waves;
        sea_surface->min_spectral_wave_heading = normalise_angle_2PI(sea_surface->heading - PI/2.0);
        sea_surface->max_spectral_wave_heading = normalise_angle_2PI(sea_surface->heading + PI/2.0);
        
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

        sea_surface->significant_wave_height = H_s;
        sea_surface->peak_spectral_frequency = f_p;
        sea_surface->min_spectral_frequency = 0.652 * f_p;
        sea_surface->max_spectral_frequency = 5.946 * f_p;

        // Create regular waves
        bool has_NULL_in_spectrum = false; // A flag that will be set to true if any of the created component wave turns out to be NULL.
        // compute step size for frequency and heading
        double frequency_step_size = (sea_surface->max_spectral_frequency - sea_surface->min_spectral_frequency) /(count_component_waves - 1);
        double wave_heading_step_size = PI/(count_component_waves - 1);
        // Create a list of frequencies and headings
        // The first in the list is the predominant wave with the freq = peak frequency and heading = wave_heading.
        wave_frequencies[0] = f_p;
        wave_freq_step_sizes[0] = frequency_step_size;
        wave_headings[0] = wave_heading;
        // Now create the rest of the waves
        if(count_component_waves-1 > 0)
        {
          double mu = -PI/2.0;
          int half_count = (count_component_waves-1)/2; // Half of the component wave count
          // Create frequencies between min_spectral_frequency and f_p
          frequency_step_size = (f_p - sea_surface->min_spectral_frequency) / half_count;
          for(int i = 0; i < half_count; ++i)
          {
            double f = sea_surface->min_spectral_frequency + (i * frequency_step_size);
            mu += wave_heading_step_size;
            double wave_heading = normalise_angle_2PI(mu + sea_surface->heading);
            wave_frequencies[i+1] = f;
            wave_freq_step_sizes[i+1] = frequency_step_size;
            wave_headings[i+1] = wave_heading;
          }
          // Create frequencies between f_p and max_spectral_frequency
          mu = PI/2.0;
          frequency_step_size = (sea_surface->max_spectral_frequency - f_p) / half_count;
          for(int i = 0; i < half_count; ++i)
          {
            double f = sea_surface->max_spectral_frequency - (i * frequency_step_size);
            mu -= wave_heading_step_size;
            double wave_heading = normalise_angle_2PI(mu + sea_surface->heading);
            wave_frequencies[half_count+1+i] = f;
            wave_freq_step_sizes[half_count+1+i] = frequency_step_size;
            wave_headings[half_count+1+i] = wave_heading;
          }
        }        
        for(int i = 0; i < count_component_waves; ++i)
        {
          double f = wave_frequencies[i];
          double wave_heading = wave_headings[i];
          double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * frequency_step_size;
        
          // Create a wave
          double amplitude = sqrt(2.0 * S); 
          double phase = rand(); 
          struct Regular_wave* regular_wave = regular_wave_new(amplitude, f, phase, wave_heading);
          if(regular_wave)
          {
            sea_surface->spectrum[i] = regular_wave;
          }
          else 
          {
            // Error encountered when creating a regular waves for the wave spectrum.
            has_NULL_in_spectrum = true;
            break;
          }
          if(has_NULL_in_spectrum)
          {
            break;
          }
        }
        // Clean memory
        if(wave_frequencies)
        {
          free(wave_frequencies);
          wave_frequencies = NULL;
        }
        if(wave_freq_step_sizes)
        {
          free(wave_freq_step_sizes);
          wave_freq_step_sizes = NULL;
        }
        if(wave_headings)
        {
          free(wave_headings);
          wave_headings = NULL;
        }
        // Check if spectrum is valid
        if(has_NULL_in_spectrum)
        {
          // One or more of the regular wave is NULL. 
          // This is an invalid spectrum.
          sea_surface_delete(sea_surface);
          sea_surface = NULL;
        }
      }
      else
      {
        if(sea_surface->spectrum)
        {
          free(sea_surface->spectrum);
          sea_surface->spectrum = NULL;
        }
        if(wave_frequencies)
        {
          free(wave_frequencies);
          wave_frequencies = NULL;
        }
        if(wave_freq_step_sizes)
        {
          free(wave_freq_step_sizes);
          wave_freq_step_sizes = NULL;
        }
        if(wave_headings)
        {
          free(wave_headings);
          wave_headings = NULL;
        }
        free(sea_surface);
        sea_surface = NULL;
      }
    }
  }
  
  return sea_surface;
}

void sea_surface_delete(struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    for(int i = 0; i < sea_surface->count_component_waves; ++i)
    {
      if(sea_surface->spectrum[i])
      {
        regular_wave_delete(sea_surface->spectrum[i]);
      }
    }
    
    free(sea_surface->spectrum);
    free(sea_surface->error_msg);
    free(sea_surface);
    sea_surface = NULL;
  }
}

double sea_surface_get_elevation(const struct Sea_surface* sea_surface, 
                          const union Coordinates_3D location,
                          double time)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    // check if time is negative
    if(time >= 0.0)
    {
      double elevation = 0.0;
      for(int i = 0; i < sea_surface->count_component_waves; ++i)
      {
        const struct Regular_wave* regular_wave = sea_surface_get_regular_wave_at(sea_surface, i);
        if(!regular_wave)
        {
          // Something really wrong happened.
          // Error should already be set by sea_surface_get_regular_wave_at().
          return 0.0;
        }
        double regular_wave_elevation = regular_wave_get_elevation(regular_wave, location, time);
        const char* error_msg = regular_wave_get_error_msg(regular_wave);
        if(error_msg)
        {
          // Something went wrong when getting wave elevation for the regular_wave.
          set_error_msg(&sea_surface->error_msg, error_msg);
          return 0.0;
        }
        else
        {
          elevation += regular_wave_elevation;
        }
      }
      return elevation;
    }
    else
    {
      set_error_msg(&sea_surface->error_msg, error_negative_time);
      return 0.0;
    }
  }
  else
  {
    return 0.0;
  }
}

const struct Regular_wave* sea_surface_get_regular_wave_at(const struct Sea_surface* sea_surface, int i)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    if(i >= 0 && i < sea_surface->count_component_waves)
    {
      return sea_surface->spectrum[i];
    }
    else
    {
      // Incorrect index
      set_error_msg(&sea_surface->error_msg, error_invalid_index);
      return NULL;
    }
  }
  else
  {
    return NULL;
  }
}


int sea_surface_get_count_component_waves(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->count_component_waves;
  }
  else
  {
    return 0.0;
  }
}

double sea_surface_get_min_spectral_frequency(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->min_spectral_frequency;
  }
  else
  {
    return 0.0;
  }
}

double sea_surface_get_max_spectral_frequency(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->max_spectral_frequency;
  }
  else
  {
    return 0.0;
  }
}

double sea_surface_get_peak_spectral_frequency(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->peak_spectral_frequency;
  }
  else
  {
    return 0.0;
  }
}

double sea_surface_get_significant_height(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->significant_wave_height;
  }
  else
  {
    return 0.0;
  }
}

double sea_surface_get_predominant_heading(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->heading;
  }
  else
  {
    return 0.0;
  }
}