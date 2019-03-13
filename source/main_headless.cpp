#include <sea_surface_dynamics.h>

using namespace asv_swarm;
using namespace asv_swarm::Hydrodynamics;

int main()
{
  /* Create variables to define the sea condition. */
  Quantity<Units::length> wind_fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {7.5*Units::meter_per_second};
  Quantity<Units::plane_angle> wind_direction {Constant::PI * Units::radian};
  
  /* Create the wave spectrum for the sea condition. */
  Wave_spectrum wave_spectrum( wind_speed, wind_fetch, wind_direction);
  /* Override the default number of bands for frequency and directions */
  //wave_spectrum.set_freq_band_count(100);
  //wave_spectrum.set_wave_direction_count(10);

  /* Create object for sea surface simulation */
  Sea_surface_dynamics sea_surface(&wave_spectrum);
  /* Override the default field dimension */
  sea_surface.set_field_length(1000 * Units::meter);
  /* Override the default number of control points on the sea surface. */
  sea_surface.set_control_points_count(20);

  /* Create objects for ASV simulation */
  /*ASV_dynamics asv();*/
  
  /* Start simulation */
  unsigned int timer_step_size = 40; /* time step size in miiliseconds.
                                        40 millisec = 25fps*/                                    
  for(int timer_count = 0; ; ++timer_count) 
  {
    Quantity<Units::time> time { timer_count * timer_step_size * 
                                 Units::milli *  Units::seconds};

    sea_surface.set_sea_surface_elevations(time); 
  }

  return 0;
}
