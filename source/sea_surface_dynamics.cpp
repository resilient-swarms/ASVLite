#include"sea_surface_dynamics.h"
#include"exception.h"

using namespace asv_swarm;

Sea_surface_dynamics::Sea_surface_dynamics
  (Quantity<Units::length> wind_fetch,
   Quantity<Units::velocity> wind_speed,
   Quantity<Units::plane_angle> wind_direction):
    wind_fetch {wind_fetch},
    field_length {100*Units::meter},
    wind_speed{wind_speed},
    wind_direction{wind_direction},
    wave_spectrum{wind_fetch, wind_speed, wind_direction},
    control_points_count {50},
    continue_simulation{true}
{
  set_control_points();
}

void Sea_surface_dynamics::set_wind_speed(Quantity<Units::velocity> wind_speed)
{
  if(wind_speed.value()< 0.0)
  {
    throw ValueError("Sea_surface_dynamics::set_wind_speed."
                     "Wind speed should be greater than or equal to 0.0m/s.");
  }
  this->wind_speed = wind_speed;
  wave_spectrum = Wave_spectrum(wind_fetch, this->wind_speed, wind_direction);
}

void Sea_surface_dynamics::set_wind_direction
  (Quantity<Units::plane_angle> wind_direction)
{
  this->wind_direction = wind_direction;
  wave_spectrum = Wave_spectrum(wind_fetch, wind_speed, this->wind_direction);
}

void Sea_surface_dynamics::set_fetch(Quantity<Units::length> wind_fetch)
{
  if(wind_fetch.value() <= 0.0)
  {
    throw ValueError("Sea_surface_dynamics::set_fetch."
                     "Fetch should be >= 0.0m.");
  } 
  this->wind_fetch = wind_fetch;
  if(field_length > wind_fetch)
  {
    field_length = wind_fetch;
    set_control_points();
  }
  wave_spectrum = Wave_spectrum(this->wind_fetch, wind_speed, wind_direction);
}

void Sea_surface_dynamics::set_field_length
  (Quantity<Units::length> field_length)
{
  if(field_length > wind_fetch || field_length.value() <= 0.0)
  {
    throw ValueError("Sea_surface_dynamics::set_field_length."
                     "Field length should be positive and <= fetch.");
  }
  this->field_length = field_length;
  set_control_points();
}

void Sea_surface_dynamics::set_control_points_count(unsigned int count)
{
  if(count <= 0)
  {
    throw ValueError("Sea_surface_dynamics::set_control_points."
                     "Count should be > 0");
  }
  control_points_count = count;
  set_control_points();
}


void Sea_surface_dynamics::set_control_points()
{
  double patch_length = field_length.value() / control_points_count;
  // Create a 2D array of control points.
  for(unsigned int i=0; i<control_points_count; ++i)
  {
    std::vector<Point> control_points_row;
    for(unsigned int j=0; j<control_points_count; ++j)
    {
      Quantity<Units::length> x{patch_length*j*Units::meter};
      Quantity<Units::length> y{patch_length*i*Units::meter};
      Quantity<Units::length> z{0.0*Units::meter};
      control_points_row.push_back(Point(x,y,z));
    }
    control_points.push_back(control_points_row);
  }
}

Wave_spectrum& Sea_surface_dynamics::get_wave_spectrum()
{
  return wave_spectrum;
}

void Sea_surface_dynamics::set_sea_surface_profile(
    Quantity<Units::time> current_time)
{
  std::vector<std::vector<Regular_wave>> waves = wave_spectrum.get_waves();
  // For each control point
  for(int i = 0; i<control_points.size(); ++i)
  {
    for(int j = 0; j<control_points[i].size(); ++j)
    {
      Quantity<Units::length> elevation{0*Units::meter};
      // For each direction in the wave spectrum
      for(int u = 0; u<waves.size(); ++u)
      {
        // For each frequency in the wave spectrum
        for(int v = 0; v<waves[u].size(); ++v)
        {
          elevation += waves[u][v].get_wave_elevation(control_points[i][j].x,
                                                      control_points[i][j].y,
                                                      current_time);
        }
      }
      control_points[i][j].z = elevation;
    }
  }
}

