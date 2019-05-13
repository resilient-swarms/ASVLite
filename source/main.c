#include <stdio.h>
#include "constants.h"
#include "environment.h"
#include "asv.h"

int main()
{
  // Environment specifications.
  double wind_speed = 7.5; // m/s
  double wind_direction = PI/6.0; // radians
  double current_speed = 0.4; // m/s
  double current_direction = 11.0*PI/6.0; //radians
  
  // Initialise the environment model.
  struct Environment environment;
  environment_init(&environment, 
                   wind_speed, wind_direction,
                   current_speed, current_direction);

  // ASV specifications
  // TODO: Read all specification and optional specifications from a config
  // file.
  struct Asv_specification spec;
  spec.L = 5.5;  // m
  spec.L_wl = 5.2; // m
  spec.B = 1.7;  // m
  spec.B_wl = 1.4; // m
  spec.D = 1.8;  // m
  spec.T = 0.85; // m
  spec.max_speed = 5.14; // m/s
  spec.disp = PI * (spec.L/2.0) * (spec.B/2.0) * spec.T; // m3
  // COG assumed to be at the volumetric centre of the vessel.
  spec.cog.x = spec.L/2.0; // Distance of COG from aft end.
  spec.cog.y = 0.0; // COG is assumed to be on the transverse centre.
  spec.cog.z = spec.D/2.0 - spec.T; // Distance of COG from the waterline.

  // Initialise the ASV model.
  struct Asv asv;
  asv_init(&asv, &spec);
  
  // Print the wave stats
  return 0;
}
