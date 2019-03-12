#include"sea_surface_dynamics.h"
#include"exception.h"
#include <iostream>
#include <algorithm>

using namespace asv_swarm;
using namespace Hydrodynamics;

Sea_surface_dynamics::Sea_surface_dynamics(Wave_spectrum* wave_spectrum):
     field_length {1000*Units::meter},
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
  
  // Initialise the table for recording the wave statistics.
  // All values are initialised to 0.
  for(int i = 0; i<control_points_count; ++i)
  {
    std::vector<double> row; 
    std::vector<bool> bool_row;
    std::vector<std::vector<double>> table;
    for(int j = 0; j<control_points_count; ++j)
    {
      bool_row.push_back(false);
      row.push_back(0.0);
      std::vector<double> wave_ht;
      table.push_back(wave_ht);      
    }
    ctrl_point_first_zero_cross.push_back(bool_row);
    ctrl_point_min_neg.push_back(row);
    ctrl_point_max_pos.push_back(row);
    ctrl_point_wave_height.push_back(table);
  }
  average_wave_height = 0.0*Units::meter;
  significant_wave_height = 0.0*Units::meter;

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
  set_wave_statistics();
  print_wave_statistics();
}

void Sea_surface_dynamics::set_wave_statistics()
{
  
  static std::vector<std::vector<Geometry::Point>> previous_vals;
  if(previous_vals.empty())
  {
    previous_vals = control_points;
    return;
  }
  
  // For each control point
  for(int i = 0; i<control_points.size(); ++i)
  {
    for(int j = 0; j<control_points[i].size(); ++j)
    {
      // Record the wave statistics for each control point
      if(control_points[i][j].z.value() < ctrl_point_min_neg[i][j])
      {
        ctrl_point_min_neg[i][j] = control_points[i][j].z.value();
      }
      if(control_points[i][j].z.value() > ctrl_point_max_pos[i][j])
      {
        ctrl_point_max_pos[i][j] = control_points[i][j].z.value();
      }
      
      // Check if zero line crossed.
      if(previous_vals[i][j].z.value() * control_points[i][j].z.value() < 0.0)  
      {
        // zero line crossed first time.
        if(!ctrl_point_first_zero_cross[i][j])
        {
          ctrl_point_first_zero_cross[i][j] = true;
        }
        else
        {
          // zero line crossed second time.
          ctrl_point_first_zero_cross[i][j] = false;
          // calculate wave height
          ctrl_point_wave_height[i][j].push_back( 
              ctrl_point_max_pos[i][j] - ctrl_point_min_neg[i][j]);
          // reset ctrl point min max recorders
          ctrl_point_max_pos[i][j] = 0.0;
          ctrl_point_min_neg[i][j] = 0.0;
        }
      }
    }
  }

  // Record wave statistics for the entire field.
  double sum = 0.0;
  int sum_count = 0;
  std::vector<double> sorted_wave_heights; 
  
  for(unsigned int i = 0u; i < control_points_count; ++i)
  {
    for(unsigned int j = 0u; j < control_points_count; ++j)
    {
      for(unsigned int k = 0u; k < ctrl_point_wave_height[i][j].size(); ++k)
      {
        sum += ctrl_point_wave_height[i][j][k];
        ++sum_count;
        sorted_wave_heights.push_back(ctrl_point_wave_height[i][j][k]);
      }
    } 
  }
  // sort the wave heights in descending order.
  std::sort(sorted_wave_heights.rbegin(), sorted_wave_heights.rend());
  
  // significant wave height = mean of top 1/3 wave heights.
  double significant_sum = 0.0;
  int significant_count = sorted_wave_heights.size()/3.0;
  for(int i=0; i < significant_count; ++i)
  {
    significant_sum += sorted_wave_heights[i];
  }
  average_wave_height = (sum/sum_count) * Units::meter;
  significant_wave_height = (significant_sum/significant_count)*Units::meter;
}

void Sea_surface_dynamics::print_wave_statistics()
{

  /* Print wave stats on std::out */
  std::cout                     <<
    "Peak freq(Hz):"            <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(7)                <<
    std::setprecision(2)        <<
    wave_spectrum->get_spectral_peak_frequency().value();
   
  std::cout                     <<
    "Min freq(Hz):"             <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(7)                <<
    std::setprecision(2)        <<
    wave_spectrum->get_min_frequency().value();
    
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

  std::cout <<"  |*|  ";
  
  std::cout                     <<
    "Time(sec):"                <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)               <<
    std::setprecision(3)        <<
    current_time.value();

  std::cout                     <<
    "Avg_wave_ht(m):"           <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)               <<
    std::setprecision(3)        <<
    average_wave_height.value();

  std::cout                     <<
    "Sig_wave_ht(m):"           <<
    std::left                   <<
    std::setfill(' ')           <<
    std::setw(8)               <<
    std::setprecision(3)        <<
    significant_wave_height.value();

  std::cout<<std::endl;

}
