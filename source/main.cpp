#include "scene.h"

using namespace asv_swarm;

int main()
{
  /* Create object to coordinate visualization */
  Visualisation::Scene scene;

  /* Set the sea condition */
  Quantity<Units::length> wind_fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  Quantity<Units::plane_angle> wind_direction {Constant::PI * Units::radian};

  scene.initialise_sea_surface_actor(wind_speed, wind_fetch, wind_direction);

  /* Start visualization */
  scene.start();

  return EXIT_SUCCESS;
}
