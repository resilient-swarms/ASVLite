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
  double significant_wave_height;      //!< Input variable. Significant wave height in meter.
  double heading;                      //!< Input variable. Wave heading in radians.
  long random_number_seed;             //!< Input variable. Random number generator seed. 
  int count_wave_spectral_directions;  //!< Input variable. Number of direction bands in the wave spectrum. 
  int count_wave_spectral_frequencies; //!< Input variable. Number of frequency bands in the wave spectrum.

  // Output variables
  // ----------------
  struct Regular_wave** spectrum;   //!< Output variable. Table of regular waves in the irregular sea.
                                           //!< Size is count_wave_spectral_directions * count_wave_spectral_frequencies.
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
                      const int count_wave_spectral_directions,
                      const int count_wave_spectral_frequencies)
{
  struct Sea_surface* sea_surface = NULL;

  if(sig_wave_ht > 0.0 && 
     count_wave_spectral_directions > 1 && 
     count_wave_spectral_frequencies > 1)
  {
    // Initialise the pointers...
    if(sea_surface = (struct Sea_surface*)malloc(sizeof(struct Sea_surface)))
    {
      if(sea_surface->spectrum = (struct Regular_wave**)malloc(sizeof(struct Regular_wave*) * count_wave_spectral_directions * count_wave_spectral_frequencies)) 
      {
        sea_surface->error_msg = NULL;
        // ... and then the other member variables.
        sea_surface->random_number_seed = rand_seed;
        srand(sea_surface->random_number_seed);
        sea_surface->heading = normalise_angle_2PI(wave_heading);
        sea_surface->count_wave_spectral_directions = count_wave_spectral_directions;
        sea_surface->count_wave_spectral_frequencies = count_wave_spectral_frequencies;
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
        // For each heading angle
        double wave_heading_step_size = PI/(count_wave_spectral_frequencies - 1);
        double mu = -PI/2.0;
        for(int i = 0; i < count_wave_spectral_directions; ++i)
        {
          mu += wave_heading_step_size;
          double frequency_step_size = (sea_surface->max_spectral_frequency - 
                                        sea_surface->min_spectral_frequency) /
                                        (count_wave_spectral_frequencies - 1);
          for(int j = 0; j < count_wave_spectral_frequencies; ++j)
          {
            double f = sea_surface->min_spectral_frequency + j * frequency_step_size;
            double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * frequency_step_size;
          
            // Direction function G = (2/PI) * cos(mu)*cos(mu) * delta_mu
            // delta_mu = wave_heading_step_size
            double G_spectrum = (2.0/PI) * cos(mu)*cos(mu) * wave_heading_step_size;
          
            // Create a wave
            double amplitude = sqrt(2.0 * S * G_spectrum); 
            double phase = rand(); 
            double wave_heading = normalise_angle_2PI(mu + sea_surface->heading);
            struct Regular_wave* regular_wave = regular_wave_new(amplitude, f, phase, wave_heading);
            if(regular_wave)
            {
              sea_surface->spectrum[i*count_wave_spectral_frequencies + j] = regular_wave;
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
          sea_surface_delete(sea_surface);
          sea_surface = NULL;
        }
      }
      else
      {
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
    for(int i = 0; i < (sea_surface->count_wave_spectral_directions * sea_surface->count_wave_spectral_frequencies); ++i)
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
      for(int i = 0; i < sea_surface->count_wave_spectral_directions; ++i)
      {
        for(int j = 0; j < sea_surface->count_wave_spectral_frequencies; ++j)
        {
          const struct Regular_wave* regular_wave = sea_surface_get_regular_wave_at(sea_surface, i, j);
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

int sea_surface_get_count_wave_spectral_directions(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->count_wave_spectral_directions;
  }
  else
  {
    return 0.0;
  }
}

int sea_surface_get_count_wave_spectral_frequencies(const struct Sea_surface* sea_surface)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    return sea_surface->count_wave_spectral_frequencies;
  }
  else
  {
    return 0.0;
  }
}

const struct Regular_wave* sea_surface_get_regular_wave_at(const struct Sea_surface* sea_surface, int d, int f)
{
  if(sea_surface)
  {
    clear_error_msg(&sea_surface->error_msg);
    if(d >= 0 && d < sea_surface->count_wave_spectral_directions &&
      f >= 0 && f < sea_surface->count_wave_spectral_frequencies)
    {
      return sea_surface->spectrum[d*sea_surface->count_wave_spectral_frequencies + f];
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