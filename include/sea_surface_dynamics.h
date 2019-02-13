#ifndef SEA_SURFACE_DYNAMICS_H
#define SEA_SURFACE_DYNAMICS_H

#include"wave_spectrum.h"
#include"units_and_constants.h"
#include"geometry.h"
#include<vector>

namespace asv_swarm
{
/**
 * Class to represent the sea surface. The sea surface, which could be a mesh,
 * is represented using an array of points, called control points. The control
 * points move up or down for each time step and this represents the wave
 * motion.
 */
class Sea_surface_dynamics
{
public:
  /**
   * Constructor. 
   * @param surface_edge_length is the size of the edge of the square patch of
   * sea surface being simulated. The size of the path should be less than or
   * equal to the fetch length.
   */
  Sea_surface_dynamics(Quantity<Units::length> fetch);

  /**
   * Method to set the wind speed. 
   * @param wind_speed is the wind speed in m/s and should be >= 0.0m/s.
   */
  void set_wind_speed(Quantity<Units::velocity> wind_speed);
    
  /**
   * Method to set the wind direction.
   * @param wind_direction is the direction in which the wind is blowing
   * measured in radians with respect to North as 0 radian and East as PI/2
   * radians.
   */
  void set_wind_direction(Quantity<Units::plane_angle> wind_direction);
    
  /**
   * Method to set the fetch length. If the field length contained is greater
   * than the fetch length provided then this method will also set the field
   * length equal to fetch and reset the control points.
   * @param fetch is the fetch length in meters. The value should be greater
   * than 0.
   */
  void set_fetch(Quantity<Units::length> fetch);

  /**
   * Method to set the length of the edge of the square sea surface being
   * simulated. By default the field length is set same as fetch but this
   * method can be used to overwrite the default value. However the field
   * length should be not be greater than fetch. This method also resets the
   * control points on the surface after changing the field length.
   * @param field_length is the length of the edge of the square sea surface
   * being simulated and should be a positive value that is less than or equal 
   * to fetch.
   */
  void set_field_length(Quantity<Units::length> field_length);

  /**
   * Method to set he number of control points along both x and y directions
   * of the square field. The default value for number of control points is
   * 100. A higher number for the count will result in more dense control
   * points along the surface representing the sea surface.
   * @param count the number of control points along one edge of the sea
   * surface. The value of count should be greater than 0. The method also
   * resets the control points on the sea surface.
   */
  void set_control_points_count(unsigned int count);

  /**
   * Method to set control points along the surface of the sea.
   */
  void set_control_points();

  /**
   * Method to get the wave spectrum.
   * @return reference to the wave spectrum.
   */
  Wave_spectrum& get_wave_spectrum();

  /**
   * Method to simulate the sea surface. This causes the control points to
   * move up or down based on the waves in the field.
   * @param time_step is the simulation time step.
   */
  void simulate_wave_dynamics(Quantity<Units::time> time_step);

private:
  std::vector<std::vector<Point>> control_points;
  unsigned int control_points_count;
  Wave_spectrum wave_spectrum;
  Quantity<Units::velocity> wind_speed;
  Quantity<Units::plane_angle> wind_direction;
  Quantity<Units::length> fetch;
  Quantity<Units::length> field_length;
  bool continue_simulation;
}; // class Sea_surface_dynamics
} // namespace asv_swarm

#endif
