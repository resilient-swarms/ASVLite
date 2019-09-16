#include <stdlib.h> // for generating random numbers.
#include <math.h>
#include "wave.h"
#include "constants.h"

void wave_init(struct Wave* wave, 
               double sig_wave_ht,
               double wave_heading)
{
  // Allocate space for spectrum
  wave->min_spectral_wave_heading = wave_heading - PI/2.0;
  wave->max_spectral_wave_heading = wave_heading + PI/2.0;
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
  double H_s = sig_wave_ht;
  double B = 4.0*alpha*G*G / (pow(2.0*PI, 4.0)* H_s*H_s);
  double f_p = 0.946 * pow(B, 0.25);

  wave->significant_wave_height = H_s;
  wave->peak_spectral_frequency = f_p;
  wave->min_spectral_frequency = 0.652 * f_p;
  wave->max_spectral_frequency = 5.946 * f_p;

  // Create regular waves
  // For each heading angle
  double wave_heading_step_size = PI / (COUNT_WAVE_SPECTRAL_DIRECTIONS - 1);
  for(int i = 0; i < COUNT_WAVE_SPECTRAL_DIRECTIONS; ++i)
  {
    double mu = wave->min_spectral_wave_heading + i * wave_heading_step_size;
    // wave heading is expected to be in the range (0, 2PI). Correct the wave
    // heading if value our of the range.
    mu = (mu > 2.0*PI)? mu - 2.0*PI: mu;

    double frequency_step_size = (wave->max_spectral_frequency - 
                                  wave->min_spectral_frequency) /
                                  (COUNT_WAVE_SPECTRAL_FREQUENCIES - 1);
    for(int j = 0; j < COUNT_WAVE_SPECTRAL_FREQUENCIES; ++j)
    {
      double f = wave->min_spectral_frequency + j * frequency_step_size;
      double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * frequency_step_size;
      
      // Direction function G = (2/PI) * cos(mu)*cos(mu) * delta_mu
      // delta_mu = wave_heading_step_size
      double G_spectrum = (2.0/PI) * cos(mu)*cos(mu) * wave_heading_step_size;
      
      // Create a wave
      double amplitude = sqrt(2.0 * S * G_spectrum); 
      double phase = rand()%(COUNT_WAVE_SPECTRAL_DIRECTIONS * 
                             COUNT_WAVE_SPECTRAL_FREQUENCIES); 
      regular_wave_init(&(wave->spectrum[i][j]), amplitude, f, phase, mu);
    }
  }
}

double wave_get_elevation(struct Wave* wave, 
                          struct Point* location,
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

