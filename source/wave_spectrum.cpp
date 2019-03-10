#include "wave_spectrum.h"
#include "exception.h"
#include <boost/units/systems/si/area.hpp>
#include <random>
#include <iostream>

using namespace asv_swarm;
using namespace asv_swarm::Hydrodynamics;

Wave_spectrum::Wave_spectrum( Quantity<Units::velocity> wind_speed,
                              Quantity<Units::length> wind_fetch,
                              Quantity<Units::plane_angle> wind_direction) : 
  wind_fetch{wind_fetch},
  wind_speed{wind_speed},
  wind_direction{wind_direction},
  freq_band_count {20},
  wave_angle_count {10},
  wave_angle_min {wind_direction - (Constant::PI/2 * Units::radians)},
  wave_angle_max {wind_direction + (Constant::PI/2 * Units::radians)}
{
  // Check if inputs are correct.
  if( wind_fetch.value() <= 0.0 || 
      wind_speed.value() < 0.0 ||
      wind_direction.value() < 0.0 ||
      wind_direction.value() > 2*Constant::PI)
  {
    throw Exception::ValueError("Wave_spectrum::Wave_spectrum()."
                     "Invalid input.");
  }
  /* Set the spectral parameters for JONSWAP spectrum */
  double g = Constant::G.value();
  double PI = Constant::PI.value();
  double U = wind_speed.value();
  double F = wind_fetch.value();
  double F_hat = g*F/(U*U);
  f_p = (g/U)*pow(F_hat, -1/3);
  alpha = 0.0081;
  beta = 0.0; // not required for JONSWAP
  gamma = 3.3;
  A = alpha * g*g * pow(2*PI, -4);
  B = (5/4)*pow(f_p, 4);

  /* Energy thresholds */
  min_freq = (0.6477 + 
              0.005357*gamma - 
              0.0002625*gamma*gamma)*f_p * Units::hertz;
  max_freq = (6.3204 - 
              0.4377*gamma + 
              0.05261*gamma*gamma - 
              0.002839*gamma*gamma*gamma)*f_p * Units::hertz;

  std::cout<<"min freq = "<<min_freq.value()<<std::endl;
  std::cout<<"max freq = "<<max_freq.value()<<std::endl;
  
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

void Wave_spectrum::set_wave_angle_count(unsigned int count)
{
  if(count <= 0)
  {
    throw Exception::ValueError("Wave_spectrum::set_direction_count()."
                     "Count should be > 0.");
  }
  wave_angle_count = count;
  set_wave_spectrum();
}

void Wave_spectrum::set_wave_spectrum()
{
  if(spectrum.size() != 0)
  {
    spectrum.erase(spectrum.begin(), spectrum.end());
  }

  if(wave_angle_list.size() != 0)
  {
    wave_angle_list.erase(wave_angle_list.begin(), wave_angle_list.end());
  }

  if(freq_band_list.size() !=0)
  {
    freq_band_list.erase(freq_band_list.begin(), freq_band_list.end());
  }

  // Create point spectrum for each direction
  Quantity<Units::plane_angle> wave_angle_band_size = 
    (wave_angle_max - wave_angle_min) / 
    (wave_angle_count * Units::si_dimensionless);
  Quantity<Units::frequency> freq_band_size = 
    (max_freq - min_freq)/
    (freq_band_count * Units::si_dimensionless);
  // Initialise random number generator for creating random phase values for
  // waves.
  std::default_random_engine generator;
  std::uniform_real_distribution<double> rand_num_distribution(0.0, 2*M_PI);
  
  // Create the vectors for direction_list and frequency_list
  for(Quantity<Units::plane_angle> angle = wave_angle_min; 
      angle < wave_angle_max;
      angle += wave_angle_band_size)
  {
    if(angle.value() <0)
    {
      wave_angle_list.push_back(angle + (2*Constant::PI*Units::radian));
    }
    else
    {
      wave_angle_list.push_back(angle);
    }
  }
  for(Quantity<Units::frequency> freq = min_freq;
        freq < max_freq;
        freq += freq_band_size)
  {
    freq_band_list.push_back(freq);
  }
  
  // Create the JONSWAP spectrum for each direction and frequency.
  for(auto angle : wave_angle_list)
  {
    std::vector<Regular_wave> directional_spectrum;
    for(auto freq : freq_band_list)
    {
      // JONSWAP SPECTRUM
      // Ref: Proceedings of the 23rd ITTC - Vol II, Table A.4
      // S(f) = (A/f^5) exp(-B/f^4) gamma^exp(C)
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
      double tau = (f <= f_p)? 0.07 : 0.09;
      double C = ((f - f_p)*(f - f_p))/(2 * tau*tau * f_p*f_p);
      double S = (A/pow(f,5)) * exp(-B/pow(f,4)) * pow(gamma, exp(-C));
      
      
      double delta_mu = wave_angle_band_size.value();
      double mu = wind_direction.value() - angle.value() + delta_mu/2;
      double G = (2/PI) * pow(mu, 2);
      
      double amp = sqrt(2 * S*delta_f * G*delta_mu);
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

