extern "C" {
#include "io.h"
#include "asv.h"
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "scene.h"
#include "constants.h"
#include <iostream>

using namespace asv_swarm;

extern "C" int main(int argc, char** argv)
{
  if(argc != 6)
  {
    fprintf(stderr, 
      "Error. " 
      "Usage: %s in_file out_file sig_wave_ht(m) wave_heading(deg) rand_seed.\n", 
      argv[0]);
    return 1;
  }
  double wave_ht, wave_heading;
  long rand_seed;
  char* in_file = argv[1];
  char* out_file = argv[2];
  sscanf(argv[3], "%lf", &wave_ht);
  sscanf(argv[4], "%lf", &wave_heading);
  sscanf(argv[5], "%ld", &rand_seed);

  // Init vehicle and waypoints
  struct Asv asv;
  struct Waypoints waypoints;
  // set ASV inputs from input file.
  set_input(in_file, &asv, &waypoints);
  // set ASV inputs that were passed in command line
  if(wave_ht != 0.0)
  {
    asv.wave_type = irregular_wave;
    wave_init(&asv.wave, wave_ht, wave_heading * PI/180.0, rand_seed);
  }
  // init the asv after setting all inputs.
  asv_init(&asv);

  // Create object to coordinate visualization
  Visualisation::Scene scene;

  // Create actor for sea surface
  Visualisation::Sea_surface_actor sea_surface_actor(&asv.wave);
  // Override the default field dimension. Field length in m.
  sea_surface_actor.set_field_length(1000.0); 
  // Override the default number of control points on the sea surface. 
  sea_surface_actor.set_sea_surface_grid_size(20);

  // Create actor for ASV 
  //Visualisation::ASV_actor asv_actor();

  // Add all actors to the scene 
  scene.add_actor(&sea_surface_actor);
  //scene.add_actor(&asv_actor);

  // Start visualization 
  // Override default frame rate for animation either by overriding frame rate
  // value or by specifying the timer step size
  //scene.set_frame_rate(20);
  //scene.set_timer_step_size(100);
  scene.start();

  return EXIT_SUCCESS;
}
