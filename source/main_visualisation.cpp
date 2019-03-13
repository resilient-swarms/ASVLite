#include "scene.h"
#include <iostream>

using namespace asv_swarm;

int main()
{
  /* Create object to coordinate visualization */
  Visualisation::Scene scene;

  /* Create variables to define the sea condition. */
  Quantity<Units::length> wind_fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {7.5*Units::meter_per_second};
  Quantity<Units::plane_angle> wind_direction {Constant::PI * Units::radian};
  
  /* Create the wave spectrum for the sea condition. */
  Hydrodynamics::Wave_spectrum wave_spectrum( wind_speed, 
                                              wind_fetch, 
                                              wind_direction);
  /* Override the default number of bands for frequency and directions */
  //wave_spectrum.set_freq_band_count(100);
  //wave_spectrum.set_wave_direction_count(10);

  /* Create actor for sea surface */
  Visualisation::Sea_surface_actor sea_surface_actor(&wave_spectrum);
  /* Override the default field dimension */
  sea_surface_actor.set_field_length(1000 * Units::meter);
  /* Override the default number of control points on the sea surface. */
  sea_surface_actor.set_control_points_count(20);

  /* Create actor for ASV */
  /*Visualisation::ASV_actor asv_actor();*/
  
  /* Add all actors to the scene */
  scene.add_actor(&sea_surface_actor);
  /*scene.add_actor(&asv_actor);*/

  /* Start visualization */
  /* Override default frame rate for animation either by overriding frame rate
   * value or by specifying the timer step size*/
  //scene.set_frame_rate(20);
  //scene.set_timer_step_size(100);
  scene.start();

  return EXIT_SUCCESS;
}
