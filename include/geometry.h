#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "units_and_constants.h"

namespace asv_swarm
{
namespace Geometry
{
class Point
{
public:
  /**
   * Default constructor.
   */
  Point();

  /**
   * Construct with coordinates provided.
   * @param x is the distance along the x axis in meter.
   * @param y is the distance along the y axis in meter.
   * @param z is teh distance along the z axis in meter.
   */
  Point(Quantity<Units::length> x,
        Quantity<Units::length> y,
        Quantity<Units::length> z);

  Quantity<Units::length> x;
  Quantity<Units::length> y;
  Quantity<Units::length> z;
}; // class Point

/**
 * Class to represent the orientation of the vessel with respect to x, y, z
 * coordinates.
 */
class Attitude
{
public:
  /**
   * Default consturctor.
   */
  Attitude();

  /**
   * Constructor with angles to coordinate axis provided.
   * @param x is the angle in radians with respect to x axis.
   * @param y is the angle in radians with respect to y axis.
   * @param z is the angle in radians with respect to z axis.
   */
  Attitude(Quantity<Units::plane_angle> x,
           Quantity<Units::plane_angle> y,
           Quantity<Units::plane_angle> z);

  Quantity<Units::plane_angle> x;
  Quantity<Units::plane_angle> y;
  Quantity<Units::plane_angle> z;
}; // class Attitude

} // namespace Geometry
} // namespace asv_swarm

#endif
