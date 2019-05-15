#include <stdlib.h> // for generating random numbers.
#include <math.h>
#include "wave.h"
#include "wind.h"
#include "constants.h"

static void wave_init(struct Wave* wave, 
                      struct Wind* wind, 
                      double significant_wave_height, 
                      double peak_spectral_frequency)
{
  double major_wave_direction = (wind == NULL)? 0.0 : wind->direction;
  wave->min_spectral_wave_heading = major_wave_direction - PI/2.0;
  wave->max_spectral_wave_heading = major_wave_direction + PI/2.0;
  // wave directions should be in the range (0, 2PI)
  if(wave->min_spectral_wave_heading < 0.0)
  {
    wave->min_spectral_wave_heading += 2.0*PI; 
  }
  if(wave->max_spectral_frequency > 2.0*PI)
  {
    wave->max_spectral_frequency -= 2.0*PI;
  }
  
  
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
  double B = 0.0;
  double H_s = 0.0;
  double f_p = 0.0;
  if(wind != NULL)
  { 
    double U = wind->speed; 
    B = beta * pow(2.0*PI*U/G, -4.0);
    H_s = 2.0* sqrt(A/B);
    f_p = 0.946 * pow(B, 0.25);
  }
  else if (significant_wave_height != 0.0)
  {
    H_s = significant_wave_height;
    B = 4.0*alpha*G*G / (pow(2.0*PI, 4.0)* H_s*H_s);
    f_p = 0.946 * pow(B, 0.25);
  }
  else if (peak_spectral_frequency != 0.0)
  {
    f_p = peak_spectral_frequency;
    B = (5.0/4.0)*pow(f_p, 4.0);
    H_s = 2.0* sqrt(A/B);
  }
  wave->significant_wave_height = H_s;
  wave->peak_spectral_frequency = f_p;
  wave->min_spectral_frequency = 0.652 * f_p;
  wave->max_spectral_frequency = 5.946 * f_p;

  // Create regular waves
  // For each heading angle
  double wave_heading_step_size = PI / (COUNT_SPECTRAL_DIRECTIONS - 1);
  for(int i = 0; i < COUNT_SPECTRAL_DIRECTIONS; ++i)
  {
    double mu = wave->min_spectral_wave_heading + i * wave_heading_step_size;
    // wave heading is expected to be in the range (0, 2PI). Correct the wave
    // heading if value our of the range.
    if(mu > 2.0*PI)
    {
      mu -= 2.0*PI;
    }

    double frequency_step_size = (wave->max_spectral_frequency - 
                                  wave->min_spectral_frequency) /
                                  (COUNT_SPECTRAL_FREQUENCIES - 1);
    for(int j = 0; j < COUNT_SPECTRAL_FREQUENCIES; ++j)
    {
      double f = wave->min_spectral_frequency + j * frequency_step_size;
      double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * frequency_step_size;
      
      // Direction function G = (2/PI) * cos(mu)*cos(mu) * delta_mu
      // delta_mu = wave_heading_step_size
      double G_spectrum = (2.0/PI) * cos(mu)*cos(mu) * wave_heading_step_size;
      
      // Create a wave
      double amplitude = sqrt(2.0 * S * G_spectrum); 
      double phase = rand(); 
      regular_wave_init(wave->spectrum[i]+j, amplitude, f, phase, mu);
    }
  }
}

void wave_init_with_wind(struct Wave* wave, struct Wind* wind)
{
  wave_init(wave, wind, 0.0, 0.0);
}

void wave_init_with_sig_wave_ht(struct Wave* wave, double sig_wave_ht)
{
  wave_init(wave, NULL, sig_wave_ht, 0.0);
}

void wave_init_with_peak_freq(struct Wave* wave, double peak_spectral_freq)
{
  wave_init(wave, NULL, 0.0, peak_spectral_freq);
}

double wave_get_elevation(struct Wave* wave, 
                          struct Point* location,
                          double time)
{
  double elevation = 0.0;
  for(int i = 0; i < COUNT_SPECTRAL_DIRECTIONS; ++i)
  {
    for(int j = 0; j < COUNT_SPECTRAL_FREQUENCIES; ++j)
    {
      elevation += regular_wave_get_elevation(wave->spectrum[i]+j, 
                                              location, 
                                              time);
    }
  }
  return elevation;
}

