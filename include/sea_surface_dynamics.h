#ifndef SEA_SURFACE_DYNAMICS_H
#define SEA_SURFACE_DYNAMICS_H

#include"wave_spectrum.h"
#include"units_and_constants.h"
#include"geometry.h"
#include<vector>

namespace asv_swarm
{
/**
 * Class to represent the sea surface. The sea surface is represented using an 
 * array of points, called control points. The control points move up or down 
 * for each time step representing the wave motion.
 */
class Sea_surface_dynamics
{
public:
  /**
   * Constructor. Default values set by the constructor are:
   * - field length = 100m
   * - number of control points = 50 x 50
   * @param wind_fetch is the length over which the wind is blowing. The value 
   * should be greater than 0.
   * @param wind_speed is the velocity of wind in meter_per_seconds. The value
   * should be >= 0.
   * @param wind_direction is the angle at which the wind is blowing measured in
   * radians. The value should be between 0 and 2PI and is measured with respect
   * to north direction. 
   */
  Sea_surface_dynamics(Quantity<Units::length> wind_fetch,
                       Quantity<Units::velocity> wind_speed,
                       Quantity<Units::plane_angle> wind_direction);

  /**
   * Method to set the wind speed. 
   * @param wind_speed is the wind speed in m/s and should be greater than or
   * equal to 0.0m/s.
   */
  void set_wind_speed(Quantity<Units::velocity> wind_speed);
    
  /**
   * Method to set the wind direction.
   * @param wind_direction is the direction in which the wind blows on the sea
   * surface measured in radians with respect to North and the direction of 
   * measurement should be such that East is at PI/2 radians to North.
   */
  void set_wind_direction(Quantity<Units::plane_angle> wind_direction);
    
  /**
   * Method to set the fetch length. If the field length contained is greater
   * than the fetch length provided then this method will also set the field
   * length equal to fetch and reset the control points.
   * @param wind_fetch is the length of sea, in m, over which the wind blows.
   *  The value should be greater than 0.
   */
  void set_fetch(Quantity<Units::length> wind_fetch);
  
  /**
   * Override the default edge length of the square sea surface displayed. Also
   * resets the control points on the surface.
   * @param field_length is the edge length in meter. Value of length should be 
   * a non-zero positive value less than or equal to wind fetch.
   */
  void set_field_length(Quantity<Units::length> field_length);

  /**
   * Method to set he number of control points along both x and y directions
   * of the square field. The default value for number of control points is
   * provided by constructor Sea_surface_dynamics::Sea_surface_dynamics. A 
   * higher number for the count will result in more dense mesh representing the 
   * sea surface. After updating the count the method resets all the control
   * points as per the new count value.
   * @param count the number of control points along one edge of the sea
   * surface. The value of count should be greater than 0.
   */
  void set_control_points_count(unsigned int count);

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
  void set_sea_surface_profile(Quantity<Units::time> current_time);

protected:  
  /**
   * Method to set control points along the surface of the sea.
   */
  void set_control_points();

  std::vector<std::vector<Point>> control_points;
  unsigned int control_points_count;
  Wave_spectrum wave_spectrum;
  Quantity<Units::velocity> wind_speed;
  Quantity<Units::plane_angle> wind_direction;
  Quantity<Units::length> wind_fetch;
  Quantity<Units::length> field_length;
  bool continue_simulation;
}; // class Sea_surface_dynamics
} // namespace asv_swarm

#endif
