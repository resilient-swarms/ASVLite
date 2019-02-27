#ifndef SEA_SURFACE_DYNAMICS_H
#define SEA_SURFACE_DYNAMICS_H

#include"wave_spectrum.h"
#include"units_and_constants.h"
#include"geometry.h"
#include<vector>

namespace asv_swarm
{
namespace Hydrodynamics
{
/**
 * Class implementing dynamics of sea surface. The sea surface is modelled as an 
 * array of points, called control points. Each control points move up or down 
 * for each time step emulating waves on sea surface.
 */
class Sea_surface_dynamics : public Wave_spectrum
{
public:
  /**
   * Constructor. Default values set by the constructor are:
   * - field length = 100m
   * - number of control points = 50 x 50
   * The constructor also provides default values for wave spectrum. The default
   * values used for wave spectrum can be found in in constructor 
   * Hydrodynamics::Wave_spectrum.
   * @param wind_speed is the velocity of wind in meter_per_seconds. The value
   * should be >= 0.
   * @param wind_fetch is the length over which the wind is blowing. The value 
   * should be greater than 0.
   * @param wind_direction is the angle at which the wind is blowing measured in
   * radians. The value should be between 0 and 2PI and is measured with respect
   * to north direction. 
   */
  Sea_surface_dynamics(Quantity<Units::velocity> wind_speed,
                       Quantity<Units::length> wind_fetch,
                       Quantity<Units::plane_angle> wind_direction);

  /**
   * Override the default edge length of the square sea surface. Also resets the 
   * control points on the surface.
   * @param field_length is the edge length in meter. Value of length should be 
   * a non-zero positive value less than or equal to wind fetch.
   */
  void set_field_length(Quantity<Units::length> field_length);

  /**
   * Method to set he number of control points along both x and y directions
   * of the square field. The default value for the number of control points is
   * provided by the constructor 
   * Hydrodynamics::Sea_surface_dynamics::Sea_surface_dynamics. A higher number 
   * for the count will result in a more dense mesh representing the sea 
   * surface. After updating the count the method resets all the control points 
   * as per the new count value.
   * @param count the number of control points along one edge of the sea
   * surface. The value of count should be greater than 0.
   */
  void set_control_points_count(unsigned int count);

  /**
   * Method to set the sea surface elevations for all control points for the 
   * current time step.
   * @param time_step is the simulation time step.
   */
  void set_sea_surface_elevations(Quantity<Units::time> current_time);

protected:  
  /**
   * Method to set control points along the surface of the sea.
   */
  void set_control_points();

  /* member variables */
  std::vector<std::vector<Geometry::Point>> control_points;
  unsigned int control_points_count;
  Quantity<Units::velocity> wind_speed;
  Quantity<Units::plane_angle> wind_direction;
  Quantity<Units::length> wind_fetch;
  Quantity<Units::length> field_length;
}; // class Sea_surface_dynamics

} // namespace Hydrodynamics
} // namespace asv_swarm

#endif
