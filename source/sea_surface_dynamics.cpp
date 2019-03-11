#include"sea_surface_dynamics.h"
#include"exception.h"

using namespace asv_swarm;
using namespace Hydrodynamics;

Sea_surface_dynamics::Sea_surface_dynamics(Wave_spectrum* wave_spectrum):
     field_length {100*Units::meter},
     control_points_count {50}
{
  if( !wave_spectrum )
  {
    throw Exception::ValueError("Sea_surface_dynamics::Sea_surface_dynamics."
                              "Parameter wave_spectrum should not be nullptr.");
  }
  this->wave_spectrum = wave_spectrum;
  set_control_points();
}

void Sea_surface_dynamics::set_field_length(
    Quantity<Units::length> field_length)
{
  if( field_length > wave_spectrum->get_wind_fetch() || 
      field_length.value() <= 0.0)
  {
    throw Exception::ValueError("Sea_surface_dynamics::set_field_length."
                      "Field length should be non-zero positive and <= fetch.");
  }
  this->field_length = field_length;
  set_control_points();
}

void Sea_surface_dynamics::set_control_points_count(unsigned int count)
{
  if(count <= 0)
  {
    throw Exception::ValueError("Sea_surface_dynamics::set_control_points."
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
    std::vector<Geometry::Point> control_points_row;
    for(unsigned int j=0; j<control_points_count; ++j)
    {
      Quantity<Units::length> x{patch_length*j*Units::meter};
      Quantity<Units::length> y{patch_length*i*Units::meter};
      Quantity<Units::length> z{0.0*Units::meter};
      control_points_row.push_back(Geometry::Point(x,y,z));
    }
    control_points.push_back(control_points_row);
  }
}

void Sea_surface_dynamics::set_sea_surface_elevations(
    Quantity<Units::time> current_time)
{
  // For each control point
  for(int i = 0; i<control_points.size(); ++i)
  {
    for(int j = 0; j<control_points[i].size(); ++j)
    {
      Quantity<Units::length> elevation{0*Units::meter};
      // Get all the waves in the wave spectrum
      std::vector<std::vector<Regular_wave>>& spectrum = 
        wave_spectrum->get_spectrum();
      // For each direction in the wave spectrum
      for(int u = 0; u<spectrum.size(); ++u)
      {
        // For each frequency in the wave spectrum
        for(int v = 0; v<spectrum[u].size(); ++v)
        {
          elevation += spectrum[u][v].get_wave_elevation(control_points[i][j].x,
                                                      control_points[i][j].y,
                                                      current_time);
        }
      }
      control_points[i][j].z = elevation;
    }
  }
}

