#include"sea_surface_visualization.h"

using namespace asv_swarm;

int main()
{
  /* Define sea condition */
  Quantity<Units::length> fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  /* wind direction is 30deg east of north*/
  Quantity<Units::plane_angle> wind_direction {Const::PI/6 * Units::radian};

  /* Visualize the sea condition */
  Sea_surface_visualization visualization {fetch, wind_speed, wind_direction};
  visualization.start_visualization();
}
