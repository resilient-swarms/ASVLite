#include "wave_spectrum.h"
#include "exception.h"
#include <boost/units/systems/si/area.hpp>
#include <random>

using namespace asv_swarm;
using namespace asv_swarm::Hydrodynamics;
using namespace asv_swarm::Constant;

Wave_spectrum::Wave_spectrum( Quantity<Units::velocity> wind_speed,
                              Quantity<Units::length> wind_fetch,
                              Quantity<Units::plane_angle> wind_direction) : 
  freq_band_count {20},
  wave_direction_count {10}
{
  // Check if inputs are correct.
  if( wind_fetch.value() <= 0.0 || 
      wind_speed.value() < 0.0 ||
      wind_direction.value() < 0.0 ||
      wind_direction.value() > 2*PI)
  {
    throw Exception::ValueError("Wave_spectrum::Wave_spectrum()."
                     "Invalid input.");
  }

  this->wind_fetch = wind_fetch;
  this->wind_speed = wind_speed;
  this->wind_direction = wind_direction;

  double g = Constant::G.value();
  double PI = Constant::PI.value();
  wave_direction_min = wind_direction - (PI/2 * Units::radians);
  wave_direction_min = (wave_direction_min.value() < 0.0) ?
                        wave_direction_min + 2*Constant::PI*Units::radian : 
                        wave_direction_min;
  wave_direction_max = wind_direction + (PI/2 * Units::radians);
  wave_direction_max = (wave_direction_max.value() > 2*PI) ?
                        wave_direction_max - 2*Constant::PI*Units::radian : 
                        wave_direction_max;

  /* Set the spectral parameters for JONSWAP spectrum */
  double U = wind_speed.value();
  double F = wind_fetch.value();
  double F_hat = g*F/(U*U);
  double f_p = (g/U)*pow(F_hat, -1.0/3.0);
  spectral_peak_freq = f_p * Units::hertz;

  alpha = 0.0081;
  beta = 0.0; // not required for JONSWAP
  gamma = 3.3;
  A = alpha * g*g / pow(2*PI, 4.0);
  B = (5/4)*pow(f_p, 4.0);

  double H_s = (1.555 + 
                (0.2596*gamma) - 
                (0.02231*gamma*gamma) + 
                (0.001142*gamma*gamma*gamma))*
                (9.81*sqrt(alpha)/pow(2*PI*f_p,2.0));
  significant_wave_height = H_s * Units::meter;

  /* Energy thresholds */
  min_freq = (0.6477 + 
              0.005357*gamma - 
              0.0002625*gamma*gamma)*f_p * Units::hertz;
  max_freq = (6.3204 - 
              0.4377*gamma + 
              0.05261*gamma*gamma - 
              0.002839*gamma*gamma*gamma)*f_p * Units::hertz;

  set_wave_spectrum();
}

void Wave_spectrum::set_freq_band_count(unsigned int count)
{
  if(count <= 0)
  {
    throw Exception::ValueError("Wave_spectrum::set_frequency_cont()."
                     "Count should be > 0.");
  }
  freq_band_count = count;
  set_wave_spectrum();
}

void Wave_spectrum::set_wave_direction_count(unsigned int count)
{
  if(count <= 0)
  {
    throw Exception::ValueError("Wave_spectrum::set_direction_count()."
                     "Count should be > 0.");
  }
  wave_direction_count = count;
  set_wave_spectrum();
}

void Wave_spectrum::set_wave_spectrum()
{
  if(spectrum.size() != 0)
  {
    spectrum.erase(spectrum.begin(), spectrum.end());
  }

  if(wave_direction_list.size() != 0)
  {
    wave_direction_list.erase(wave_direction_list.begin(), 
                              wave_direction_list.end());
  }

  if(freq_band_list.size() !=0)
  {
    freq_band_list.erase(freq_band_list.begin(), freq_band_list.end());
  }

  // Create point spectrum for each direction
  Quantity<Units::plane_angle> wave_angle_band_size = 
    (Constant::PI * Units::radian) /
    (wave_direction_count * Units::si_dimensionless);
  Quantity<Units::frequency> freq_band_size = 
    (max_freq - min_freq)/ (freq_band_count * Units::si_dimensionless);
  // Initialise random number generator for creating random phase values for
  // waves.
  std::default_random_engine generator;
  std::uniform_real_distribution<double> rand_num_distribution(0.0, 2*M_PI);
  
  // Create the vectors for direction_list and frequency_list
  for(unsigned int i{0u}; i<wave_direction_count; ++i)
  {
    Quantity<Units::plane_angle> angle = 
      wave_direction_min + i*Units::si_dimensionless*wave_angle_band_size;
    if(angle.value() > 2*Constant::PI.value())
    {
      wave_direction_list.push_back(angle - (2*Constant::PI*Units::radian));
    }
    else
    {
      wave_direction_list.push_back(angle);
    }
  }
  for(Quantity<Units::frequency> freq = min_freq;
        freq < max_freq;
        freq += freq_band_size)
  {
    freq_band_list.push_back(freq);
  }
  
  // Create the JONSWAP spectrum for each direction and frequency.
  for(auto angle : wave_direction_list)
  {
    std::vector<Regular_wave> directional_spectrum;
    for(auto freq : freq_band_list)
    {
      // JONSWAP SPECTRUM
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
      double PI = Constant::PI.value();
      double delta_f = freq_band_size.value();
      double f = freq.value() + delta_f/2;
      double f_p = spectral_peak_freq.value();
      double tau = (f <= f_p)? 0.07 : 0.09;
      double C = ((f - f_p)*(f - f_p))/(2 * tau*tau * f_p*f_p);
      double S = (A/pow(f,5.0)) * exp(-B/pow(f,4.0)) * pow(gamma, exp(-C));
      
      double delta_mu = wave_angle_band_size.value();
      double mu = angle.value() - wind_direction.value() + delta_mu/2;
      // Direction function G = (2/PI) * cos(mu)*cos(mu);
      // Integrate G over the interval mu_1 to mu_2, where:
      // mu_1 = angle
      // mu_2 = angle + delta_mu
      // integral(G) = (1/PI)(sin(2 mu_2)/2 + mu_2 - sin(2 mu_1)/2 - mu_1)
      double mu_1 = angle.value();
      double mu_2 = angle.value() + delta_mu;
      double G_integral = (1/PI)*(sin(2*mu_2)/2 + mu_2 - sin(2*mu_1)/2 - mu_1);
      
      double amp = sqrt(2 * S*delta_f * G_integral);
      Quantity<Units::length> amplitude{amp* Units::meter};
      
      // Generate a random value for phase
      std::uniform_real_distribution<double> distribution(0.0, 2*PI);
      Quantity<Units::plane_angle> phase{
        rand_num_distribution(generator) * Units::radians}; /* generate a random 
                                                               value for phase*/ 
      // Generate a regular wave.
      if(amplitude.value() > 0.0)
      {
        Regular_wave wave{amplitude, freq, angle, phase};
        directional_spectrum.push_back(wave);
      }
    }
    
    // Insert spectrum for one direction into the full spectrum.
    spectrum.push_back(directional_spectrum);
  }
}

