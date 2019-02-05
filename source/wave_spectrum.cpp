#include "wave_spectrum.h"
#include "exception.h"
#include <boost/units/systems/si/area.hpp>
#include <random>

using namespace boost::units;

Wave_spectrum::Wave_spectrum( Quantity<Units::length> fetch,
                              Quantity<Units::velocity> wind_speed,
                              Quantity<Units::plane_angle> wind_direction) : 
  fetch{fetch},
  wind_speed{wind_speed},
  wind_direction{wind_direction},
  f_count {50},
  d_count {180},
  min_freq {0.3 * Units::hertz},
  max_freq {6.0 * Units::hertz},
  min_angle {wind_direction - Const::PI/2 * Units::radians},
  max_angle {wind_direction + Const::PI/2 * Units::radians}
{
  // Check if inputs are correct.
  if( fetch.value() <= 0.0 || 
      wind_speed.value() <= 0.0 ||
      wind_direction.value() < 0.0 ||
      wind_direction.value() > 2*Const::PI)
  {
    throw ValueError("Wave_spectrum::Wave_spectrum()."
                     "Invalid input.");
  }
}

bool Wave_spectrum::set_frequency_cont(unsigned int count)
{
  if(count <= 0)
  {
    throw ValueError("Wave_spectrum::set_frequency_cont()."
                     "Invalid input.");
  }
  f_count = count;
  return true;
}

bool Wave_spectrum::set_direction_count(unsigned int count)
{
  if(count <= 0)
  {
    throw ValueError("Wave_spectrum::set_direction_count()."
                     "Invalid input.");
  }
  d_count = count;
  return true;
}

bool Wave_spectrum::set_wave_spectrum()
{
  if(spectrum.size() != 0)
  {
    spectrum.erase(spectrum.begin(), spectrum.end());
  }
  
  // Create point spectrum for each direction
  Quantity<Units::plane_angle> d_step = (max_angle.value() - min_angle.value())/ 
                                        d_count * Units::radians;
  Quantity<Units::frequency> f_step = (max_freq.value() - min_freq.value()) / 
                                      f_count * Units::hertz;
  // Initialise random number generator for creating random phase values for
  // waves.
  std::default_random_engine generator;
  std::uniform_real_distribution<double> rand_num_distribution(0.0, 2*M_PI);
 
  for(Quantity<Units::plane_angle> angle = min_angle; 
      angle <= max_angle;
      angle += d_step)
  {
    std::vector<Regular_wave> directional_spectrum;
    for(Quantity<Units::frequency> freq = min_freq;
        freq <= max_freq;
        freq += f_step)
    {
      double g = Const::G.value();
      double PI = M_PI;
      double omega = freq.value();
      double gamma = 3.3;
      double x = fetch.value();
      double V = wind_speed.value();
      double x_tilde = (g*x)/(V*V);
      double f_m = 3.5 * pow(x_tilde, -0.33);
      double omega_m = (2 * PI * f_m * g)/V;
      double sigma = (omega <= omega_m) ? 0.07 : 0.09;
      double alpha = 0.076 * pow(x_tilde, -0.22);

      double S = alpha * g * pow(omega, -5) * 
                 exp(pow((-5/4)*(omega/omega_m),-4)) * 
                 pow(gamma, 
                     exp(- pow(omega - omega_m,2)/
                          (2 * pow(sigma,2) * pow(omega_m,2))));
      double G = (2/PI) * pow(cos(angle.value()), 2);

      Quantity<Units::length> amplitude{sqrt(2 * S * G)* Units::meter};
      // Generate a random value for phase
      std::default_random_engine random_generator;
      std::uniform_real_distribution<double> distribution(0.0, 2*PI);
      Quantity<Units::plane_angle> phase{
        rand_num_distribution(generator) * Units::radians}; /* generate a random 
                                                               value for phase*/ 
      Regular_wave wave{amplitude, freq, angle, phase};
      directional_spectrum.push_back(wave);
    }
    
    // Insert spectrum for one direction into the full spectrum.
    spectrum.push_back(directional_spectrum);
  }
  
  return true;
}
