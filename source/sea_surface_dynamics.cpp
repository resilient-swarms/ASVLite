#include"sea_surface_dynamics.h"
#include"exception.h"
#include <iostream>
#include <algorithm>

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

  // current time = 0
  current_time = 0.0*Units::second;
 
  // Initialise all control points in the field.
  set_control_points();
 
  // Initialise recording of wave statistics.
  // Stat point set to the middle of the field.
  stat_point = control_points[control_points_count/2][control_points_count/2];
  stat_point_previous_record = stat_point;
  // All values are initialised to 0.
  zero_crossed = false;
  min_neg = 0.0 * Units::meter;
  max_pos = 0.0 * Units::meter;
  average_wave_height = 0.0 * Units::meter;
  significant_wave_height = 0.0 * Units::meter;
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
  // Clear the control points if not empty
  if(!control_points.empty())
  {
    control_points.erase(control_points.begin(), control_points.end());
  }
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
      Geometry::Point point{x,y,z};
      control_points_row.push_back(point);
    }
    control_points.push_back(control_points_row);
  }
}

Wave_spectrum* Sea_surface_dynamics::get_wave_spectrum()
{
  return wave_spectrum;
}

void Sea_surface_dynamics::set_sea_surface_elevations(
    Quantity<Units::time> current_time)
{
  this->current_time = current_time;
  // Calculate elevation for each control point
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
  
  // Print the wave statistics if required.
#ifdef PRINT_WAVE_STATS
  print_wave_statistics();
#endif
}

void Sea_surface_dynamics::set_wave_statistics()
{
  // Get current reading of stat point.
  stat_point = control_points[control_points_count/2][control_points_count/2];
  // Set min and max 
  min_neg = (stat_point.z < min_neg)? stat_point.z : min_neg;
  max_pos = (stat_point.z > max_pos)? stat_point.z : max_pos;
      
  // Check if zero line crossed.
  if(stat_point_previous_record.z.value() * stat_point.z.value() < 0.0)  
  {
    // zero line crossed first time.
    if(!zero_crossed)
    {
      zero_crossed = true;
    }
    else
    {
      // zero line crossed second time.
      // calculate wave height
      wave_height.push_back(max_pos - min_neg);
      // Sort wave heights in descending order
      std::sort(wave_height.rbegin(), wave_height.rend());
      
      // Calculate average wave height and significant wave height.
      Quantity<Units::length> sum = 0.0 * Units::meter;
      for(unsigned int i {0u}; i<wave_height.size(); )
      {
        sum += wave_height[i++];
        if( i == wave_height.size()/3)
        {
         significant_wave_height = sum/(i*Units::si_dimensionless); 
        }
      }
      average_wave_height = sum/(wave_height.size() * Units::si_dimensionless);
      
      // reset data records for the next wave cycle.
      max_pos = 0.0 * Units::meter;
      min_neg = 0.0 * Units::meter;
      zero_crossed = false;
    }
  }
  stat_point_previous_record = stat_point;
}

void Sea_surface_dynamics::print_wave_statistics()
{
  /* Calculate the wave statistics */ 
  set_wave_statistics();

  /* Print wave stats on std::out */
  std::cout                     <<
    "Min freq(Hz):"             <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(7)                <<
    std::setprecision(2)        <<
    wave_spectrum->get_min_frequency().value();

  std::cout                     <<
    "Peak freq(Hz):"            <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(7)                <<
    std::setprecision(2)        <<
    wave_spectrum->get_spectral_peak_frequency().value();
   
  std::cout                     <<
    "Max freq(Hz):"             <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(7)                <<
    std::setprecision(2)        <<
    wave_spectrum->get_max_frequency().value();
     
  std::cout                     <<
    "Expected sig wave ht(m):"  <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(7)                <<
    std::setprecision(3)        <<
    wave_spectrum->get_significant_wave_height().value();
     
  std::cout                     <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(6)                <<
    "|*|";
  
  std::cout                     <<
    "Time(sec):"                <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(12)               <<
    std::setprecision(3)        <<
    current_time.value();

  std::cout                     <<
    "Elevation(m):"             <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(9)                <<
    std::setprecision(3)        <<
    stat_point.z.value();

  std::cout                     <<
    "Wave cycles count:"        <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(5)                <<
    wave_height.size();


  std::cout                     <<
    "Min_wave_ht(m):"           <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)                <<
    std::setprecision(3)        <<
    (wave_height.empty()? 0.0 : wave_height.back().value());

  std::cout                     <<
    "Max_wave_ht(m):"           <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)                <<
    std::setprecision(3)        <<
    (wave_height.empty()? 0.0 : wave_height.front().value());

  std::cout                     <<
    "Avg_wave_ht(m):"           <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)                <<
    std::setprecision(3)        <<
    average_wave_height.value();

  std::cout                     <<
    "Sig_wave_ht(m):"           <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)                <<
    std::setprecision(3)        <<
    significant_wave_height.value();

  std::cout<<std::endl;

}
