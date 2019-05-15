#include <stdio.h>
#include "constants.h"
#include "asv.h"
#include "wave.h"
#include "wind.h"
#include "current.h"

int main()
{
  // Environment specifications.
  double wind_speed = 7.5; // m/s
  double wind_direction = PI/6.0; // radians
  double current_speed = 0.4; // m/s
  double current_direction = 11.0*PI/6.0; //radians

  // Initialise the environment models
  struct Wind wind; // The wind model.
  wind_init(&wind, wind_speed, wind_direction); // Initialise the wind model.
  struct Wave wave; // Wave model.
  wave_init_with_wind(&wave, &wind); // Initialise the wave model with wind data
  struct Current current; // Current model.
  current_init(&current, current_speed, current_direction); // Initialise the 
                                                            // current model.
  
  // ASV specifications
  // TODO: Read all specification and optional specifications from a config
  // file.
  struct Asv_specification spec;
  spec.L_wl = 5.2; // m
  spec.B_wl = 1.4; // m
  spec.D = 1.8;  // m
  spec.T = 0.85; // m
  spec.max_speed = 5.14; // m/s
  spec.disp = PI * (spec.L_wl/2.0) * (spec.B_wl/2.0) * spec.T; // m3
  // COG assumed to be at the volumetric centre of the vessel.
  spec.cog.x = spec.L_wl/2.0; // Distance of COG from aft end.
  spec.cog.y = 0.0; // COG is assumed to be on the transverse centre.
  spec.cog.z = spec.D/2.0 - spec.T; // Distance of COG from the waterline.
  // Radius of gyration
  // Ref: Specialist Committee of 24th ITTC: Stability in waves
  // Recommends a value of 0.25L for pitch and yaw radius of gyration and 0.4B
  // for roll radius of gyration.
  double L = 5.5;  // length overall of the ASV in m
  double B = 1.7;  // moulded beam of the ASV in m
  spec.r_roll = 0.4 * B;
  spec.r_pitch= 0.25 * L; 
  spec.r_yaw = 0.25 * L; 

  // Initialise the ASV model.
  struct Asv asv;
  asv_init(&asv, &spec, &wave, &wind, &current);
  
  // Print the wave stats
  return 0;
}
