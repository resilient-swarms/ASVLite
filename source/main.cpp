#include "visualization.h"

using namespace asv_swarm;

int main()
{
  /* Define sea condition */
  Quantity<Units::length> wind_fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  /* wind direction is 30deg east of north*/
  Quantity<Units::plane_angle> wind_direction {Const::PI/6 * Units::radian};

  /* Add observer to timer event */
  Visualization visualization;
  visualization.set_sea_condition(wind_speed, wind_fetch, wind_direction);
  visualization.start();

  return EXIT_SUCCESS;
}
