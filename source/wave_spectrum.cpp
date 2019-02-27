#include "wave_spectrum.h"
#include "exception.h"
#include <boost/units/systems/si/area.hpp>
#include <random>
#include <iostream>

using namespace asv_swarm;

Wave_spectrum::Wave_spectrum( Quantity<Units::velocity> wind_speed,
                              Quantity<Units::length> wind_fetch,
                              Quantity<Units::plane_angle> wind_direction) : 
  wind_fetch{wind_fetch},
  wind_speed{wind_speed},
  wind_direction{wind_direction},
  f_count {20},
  d_count {10},
  min_freq {0.3 * Units::hertz},
  max_freq {6.0 * Units::hertz},
  min_angle {wind_direction - (Const::PI/2 * Units::radians)},
  max_angle {wind_direction + (Const::PI/2 * Units::radians)}
{
  // Check if inputs are correct.
  if( wind_fetch.value() <= 0.0 || 
      wind_speed.value() < 0.0 ||
      wind_direction.value() < 0.0 ||
      wind_direction.value() > 2*Const::PI)
  {
    throw ValueError("Wave_spectrum::Wave_spectrum()."
                     "Invalid input.");
  }
  set_wave_spectrum();
}

void Wave_spectrum::set_frequency_cont(unsigned int count)
{
  if(count <= 0)
  {
    throw ValueError("Wave_spectrum::set_frequency_cont()."
                     "Count should be > 0.");
  }
  f_count = count;
  set_wave_spectrum();
}

void Wave_spectrum::set_direction_count(unsigned int count)
{
  if(count <= 0)
  {
    throw ValueError("Wave_spectrum::set_direction_count()."
                     "Count should be > 0.");
  }
  d_count = count;
  set_wave_spectrum();
}

void Wave_spectrum::set_wave_spectrum()
{
  if(spectrum.size() != 0)
  {
    spectrum.erase(spectrum.begin(), spectrum.end());
  }

  if(directions_list.size() != 0)
  {
    directions_list.erase(directions_list.begin(), directions_list.end());
  }

  if(frequency_list.size() !=0)
  {
    frequency_list.erase(frequency_list.begin(), frequency_list.end());
  }

  // Create point spectrum for each direction
  Quantity<Units::plane_angle> d_step = (max_angle - min_angle)/ 
                                        (d_count* Units::si_dimensionless) ;
  Quantity<Units::frequency> f_step = (max_freq - min_freq) / 
                                      (f_count * Units::si_dimensionless);
  // Initialise random number generator for creating random phase values for
  // waves.
  std::default_random_engine generator;
  std::uniform_real_distribution<double> rand_num_distribution(0.0, 2*M_PI);
  
  // Create the vectors for direction_list and frequency_list
  for(Quantity<Units::plane_angle> angle = min_angle; 
      angle <= max_angle;
      angle += d_step)
  {
    if(angle.value() <0)
    {
      directions_list.push_back(angle + (2*Const::PI*Units::radian));
    }
    else
    {
      directions_list.push_back(angle);
    }
  }
  for(Quantity<Units::frequency> freq = min_freq;
        freq <= max_freq;
        freq += f_step)
  {
    frequency_list.push_back(freq);
  }
  
  // Create the JONSWAP spectrum for each direction and frequency.
  for(auto angle = directions_list.begin();
      angle != directions_list.end();
      ++angle)
  {
    std::vector<Regular_wave> directional_spectrum;
    for(auto freq = frequency_list.begin();
        freq != frequency_list.end();
        ++freq)
    {
      // JONSWAP SPECTRUM
      // Ref: Proceedings of the 23rd ITTC - Vol II, Table A.4
      // S(f) = a g^2 (2PI)^-4 f^-5 exp(A) gamma^exp(B)
      // A = (-5/4)(f/f_p)^-4
      // B = (-(f - f_p)^2)/(2 tau^2 f_p^2)
      // a = 0.076 F_hat^-0.22
      // F_hat = gF/U^2
      // U = wind speed
      // g = 9.81
      // F = fetch
      // f_p = (g/U)F_hat^(-1/3)
      // tau = (f <= f_p)? 0.07 : 0.09
      // gamma = 3.3
      double g = 9.81; 
      double PI = M_PI;
      double U = wind_speed.value();
      double F = wind_fetch.value();
      double gamma = 3.3;
      double F_hat = g*F/(U*U);
      double f_p = (g/U)*pow(F_hat, -1/3);
      double f = freq->value();
      double tau = (f <= f_p)? 0.07 : 0.09;
      double a = 0.076 * pow(F_hat, -0.22);
      double A = (-5/4) * pow(f/f_p, -4);
      double B = -((f - f_p)*(f - f_p))/(2 * tau*tau * f_p*f_p);
      double S = a * g*g * pow(2*PI,-4) * pow(f,-5) * 
                 exp(A) * 
                 pow(gamma, exp(B));  
      double G = (2/PI) * pow(cos(angle->value()), 2);
      double amp = sqrt(2 * S * G * f_step.value() * d_step.value());
      Quantity<Units::length> amplitude{amp* Units::meter};
      
      // Generate a random value for phase
      std::uniform_real_distribution<double> distribution(0.0, 2*PI);
      Quantity<Units::plane_angle> phase{
        rand_num_distribution(generator) * Units::radians}; /* generate a random 
                                                               value for phase*/ 
      // Generate a regular wave.
      if(amplitude.value() > 0.0)
      {
        Regular_wave wave{amplitude, *freq, *angle, phase};
        directional_spectrum.push_back(wave);
      }
    }
    
    // Insert spectrum for one direction into the full spectrum.
    spectrum.push_back(directional_spectrum);
  }
}

std::vector<std::vector<Regular_wave>>& Wave_spectrum::get_waves()
{
  return spectrum;
}

std::vector<Quantity<Units::plane_angle>>& Wave_spectrum::get_directions()
{
  return directions_list; 
}

std::vector<Quantity<Units::frequency>>& Wave_spectrum::get_frequencies()
{
  return frequency_list;
}
