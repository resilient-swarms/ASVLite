#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "units_and_constants.h"

namespace asv_swarm
{
class Point
{
public:
  Point(Quantity<Units::length> x,
        Quantity<Units::length> y,
        Quantity<Units::length> z);

  Quantity<Units::length> x;
  Quantity<Units::length> y;
  Quantity<Units::length> z;
}; // class Point
} // namespace asv_swarm

#endif