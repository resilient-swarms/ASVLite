#include "scene.h"
#include <iostream>

using namespace asv_swarm;

int main()
{
  /* Create object to coordinate visualization */
  Visualisation::Scene scene;

  /* Create variables to define the sea condition. */
  Quantity<Units::length> wind_fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  Quantity<Units::plane_angle> wind_direction {Constant::PI * Units::radian};
  
  /* Create the wave spectrum for the sea condition. */
  Hydrodynamics::Wave_spectrum wave_spectrum( wind_speed, 
                                              wind_fetch, 
                                              wind_direction);
  
  /* Create actor for sea surface */
  Visualisation::Sea_surface_actor sea_surface_actor(&wave_spectrum);

  /* Create actor for ASV */
  /*Visualisation::ASV_actor asv_actor();*/
  
  /* Add all actors to the scene */
  scene.add_actor(&sea_surface_actor);
  /*scene.add_actor(&asv_actor);*/

  /* Start visualization */
  scene.start();

  return EXIT_SUCCESS;
}
