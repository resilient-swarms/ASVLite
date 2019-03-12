#include "geometry.h"

using namespace asv_swarm;
using namespace asv_swarm::Geometry;

Point::Point():
  x{0.0*Units::meter},
  y{0.0*Units::meter},
  z{0.0*Units::meter}
{}

Point::Point(Quantity<Units::length> x,
             Quantity<Units::length> y,
             Quantity<Units::length> z):
  x{x},
  y{y},
  z{z}
{}


