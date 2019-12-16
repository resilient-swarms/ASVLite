#include <stdlib.h> // for generating random numbers.
#include <math.h>
#include "wave.h"
#include "constants.h"

int wave_init(struct Wave* wave, 
              double sig_wave_ht,
              double wave_heading, 
              long rand_seed)
{
  // Check if wave is nullptr
  if(!wave)
  {
    return 1;
  }
  // Check if sig_wave_ht is zero or -ve.
  if(sig_wave_ht <= 0.0)
  {
    return 2;
  }
  // Allocate space for spectrum
  wave->heading = wave_heading;
  if(COUNT_WAVE_SPECTRAL_DIRECTIONS > 1)
  {
    wave->min_spectral_wave_heading = wave->heading - PI/2.0;
    wave->max_spectral_wave_heading = wave->heading + PI/2.0;
  }
  else
  {
    wave->min_spectral_wave_heading = wave->heading;
    wave->max_spectral_wave_heading = wave->heading;
  }
  
  // wave directions should be in the range (0, 2PI)
  if(wave->min_spectral_wave_heading < 0.0)
  {
    wave->min_spectral_wave_heading += 2.0*PI; 
  }
  if(wave->max_spectral_wave_heading >= 2.0*PI)
  {
    wave->max_spectral_wave_heading -= 2.0*PI;
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
  double H_s = sig_wave_ht;
  double B = 4.0*alpha*G*G / (pow(2.0*PI, 4.0)* H_s*H_s);
  double f_p = 0.946 * pow(B, 0.25);

  wave->significant_wave_height = H_s;
  wave->peak_spectral_frequency = f_p;
  wave->min_spectral_frequency = 0.652 * f_p;
  wave->max_spectral_frequency = 5.946 * f_p;

  // Create regular waves
  // For each heading angle
  double wave_heading_step_size = (COUNT_WAVE_SPECTRAL_DIRECTIONS > 1) ?
                                  PI/(COUNT_WAVE_SPECTRAL_DIRECTIONS - 1) : 0.0;
  double mu = -PI/2.0;
  for(int i = 0; i < COUNT_WAVE_SPECTRAL_DIRECTIONS; ++i)
  {
    mu += wave_heading_step_size;

    double frequency_step_size = (wave->max_spectral_frequency - 
                                  wave->min_spectral_frequency) /
                                  (COUNT_WAVE_SPECTRAL_FREQUENCIES - 1);
    for(int j = 0; j < COUNT_WAVE_SPECTRAL_FREQUENCIES; ++j)
    {
      double f = wave->min_spectral_frequency + j * frequency_step_size;
      double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * frequency_step_size;
      
      // Direction function G = (2/PI) * cos(mu)*cos(mu) * delta_mu
      // delta_mu = wave_heading_step_size
      double G_spectrum = 1.0;
      if(COUNT_WAVE_SPECTRAL_DIRECTIONS > 1)
      {
        G_spectrum = (2.0/PI) * cos(mu)*cos(mu) * wave_heading_step_size;
      }
      
      // Create a wave
      double amplitude = sqrt(2.0 * S * G_spectrum); 
      wave->random_number_seed = rand_seed;
      srand(wave->random_number_seed);
      double phase = rand(); 
      double wave_heading = (COUNT_WAVE_SPECTRAL_DIRECTIONS > 1) ? 
                            mu + wave->heading : wave->heading;
      // wave directions should be in the range (0, 2PI)
      if(wave_heading < 0.0)
      {
        wave_heading += 2.0*PI; 
      }
      if(wave_heading >= 2.0*PI)
      {
        wave_heading -= 2.0*PI;
      }
      regular_wave_init(&(wave->spectrum[i][j]), 
                        amplitude, 
                        f, 
                        phase, 
                        wave_heading);
    }
  }
  return 0; // no error return.
}

double wave_get_elevation(struct Wave* wave, 
                          struct Dimensions* location,
                          double time)
{
  double elevation = 0.0;
  for(int i = 0; i < COUNT_WAVE_SPECTRAL_DIRECTIONS; ++i)
  {
    for(int j = 0; j < COUNT_WAVE_SPECTRAL_FREQUENCIES; ++j)
    {
      elevation += regular_wave_get_elevation(&(wave->spectrum[i][j]), 
                                              location, 
                                              time);
    }
  }
  return elevation;
}

