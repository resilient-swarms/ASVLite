#include <stdlib.h> // for generating random numbers.
#include <math.h>
#include "wave.h"
#include "constants.h"

void wave_init(struct Wave* wave, 
               double wind_speed, 
               double wind_fetch,
               double wind_direction)
{
  wave->wind_speed = wind_speed;
  wave->wind_fetch = wind_fetch;
  wave->wind_direction = wind_direction;
  wave->min_spectral_wave_heading = wind_direction - PI/2.0;
  wave->max_spectral_wave_heading = wind_direction + PI/2.0;
  // wave directions should be in the range (0, 2PI)
  if(wave->min_spectral_wave_heading < 0.0)
  {
    wave->max_spectral_wave_heading += 2.0*PI; 
  }
  if(wave->max_spectral_frequency > 2.0*PI)
  {
    wave->max_spectral_frequency -= 2.0*PI;
  }

  // JONSWAP spectrum
  // Ref: Proceedings of the 23rd ITTC - Vol II, Table A.4
  // S(f) = (A/f^5) exp(-B/f^4) gamma^exp(-C)
  // A = alpha g^2 (2 PI)^-4
  // B = (5/4) f_p^4
  // C = (f - f_P)^2 / (2 tau^2 f_p^2)
  // alpha = 0.0081
  // gamma = 3.3
  // f_p = (g/U)F_hat^(-1/3)
  // U = wind speed in m/s
  // F_hat = gF/U^2
  // F = wind fetch in m
  double F_hat = (G * wind_fetch)/(wind_speed * wind_speed);
  double f_p = (G/wind_speed) * pow(F_hat, -1.0/3.0);
  wave->peak_spectral_frequency = f_p;
  double alpha = 0.0081;
  double gamma = 3.3;
  double A = alpha * G*G / pow(2.0*PI, 4.0);
  double B = (5.0/4.0) * pow(f_p, 4.0);
  wave->significant_wave_height =(1.555 + 
                                  0.2596*gamma - 
                                  0.02231*gamma*gamma + 
                                  0.001142*gamma*gamma*gamma) *
                                  G * sqrt(alpha) / 
                                  (2.0*PI*f_p * 2.0*PI*f_p);
  wave->min_spectral_frequency = (0.6477 + 
                                  0.005357 * gamma - 
                                  0.0002625 * gamma * gamma) * f_p;
  wave->max_spectral_frequency = (6.3204 - 
                                  0.4377 * gamma + 
                                  0.05261 * gamma * gamma - 
                                  0.002839 * gamma * gamma * gamma) * f_p;

  // Create regular waves
  double wave_heading_step_size = (wave->max_spectral_wave_heading - 
                                   wave->min_spectral_wave_heading) / 
                                   (COUNT_SPECTRAL_DIRECTIONS - 1);
  int i = 0; // counters for angle
  int j = 0; // counter for freq
  for(double mu = wave->min_spectral_wave_heading; 
      mu <= wave->max_spectral_wave_heading; 
      mu += wave_heading_step_size, ++i)
  {
    // wave heading is expected to be in the range (0, 2PI). Correct the wave
    // heading if value our of the range.
    if(mu > 2.0*PI)
    {
      mu -= 2.0*PI;
    }

    double frequency_step_size = (wave->max_spectral_frequency - 
                                  wave->min_spectral_frequency) /
                                  (COUNT_SPECTRAL_FREQUENCIES - 1);
    for(double f = wave->min_spectral_frequency;
        f <= wave->max_spectral_frequency;
        f += frequency_step_size, ++j)
    {
      double tau = (f <= f_p)? 0.07 : 0.09;
      double C = ((f - f_p)*(f - f_p)) / (2.0 * tau*tau * f_p*f_p);
      double S = (A/pow(f,5.0)) * 
                 exp(-B/pow(f,4.0)) * 
                 pow(gamma, exp(-C)) * 
                 frequency_step_size;
      
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

double wave_get_elevation(struct Wave* wave, 
                          struct Point* location,
                          double time)
{
  double elevation = 0.0;
  for(int i = 0; i <= COUNT_SPECTRAL_DIRECTIONS; ++i)
  {
    for(int j = 0; j <= COUNT_SPECTRAL_FREQUENCIES; ++j)
    {
      elevation += regular_wave_get_elevation(wave->spectrum[i]+j, 
                                              location, 
                                              time);
    }
  }
  return elevation;
}

