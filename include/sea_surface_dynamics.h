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
class Sea_surface_dynamics
{
public:
  /**
   * Constructor. Default values set by the constructor are:
   * - field length = 100m
   * - number of control points = 50 x 50
   * @param wave_spectrum pointer to the wave spectrum defining the sea state. 
   */
  Sea_surface_dynamics(Wave_spectrum* wave_spectrum);

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
  Wave_spectrum* wave_spectrum; 
  Quantity<Units::length> field_length;
  unsigned int control_points_count;
  std::vector<std::vector<Geometry::Point>> control_points;
}; // class Sea_surface_dynamics

} // namespace Hydrodynamics
} // namespace asv_swarm

#endif
