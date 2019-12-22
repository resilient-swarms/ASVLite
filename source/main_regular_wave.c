#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <io.h>
#include "asv.h"

int main(int argc, char** argv)
{
  if(argc != 7)
  {
    fprintf(stderr, 
      "Error. " 
      "Usage: %s in_file out_file wave_ht(m) wave_heading(deg) frequency(Hz) "
      "phase_lag(deg).\n", 
      argv[0]);
    return 1;
  }
  double wave_ht, wave_heading, phase_lag, frequency;
  int rand_seed = 1;
  char* in_file = argv[1];
  char* out_file = argv[2];
  sscanf(argv[3], "%lf", &wave_ht);
  sscanf(argv[4], "%lf", &wave_heading);
  sscanf(argv[5], "%lf", &frequency);
  sscanf(argv[6], "%lf", &phase_lag);

  // Init vehicle and waypoints
  struct Asv asv;
  struct Waypoints waypoints;
  // set ASV inputs from input file.
  set_input(in_file, &asv, &waypoints);
  // set ASV inputs that were passed in command line
  if(wave_ht != 0.0)
  {
    asv.wave_type = regular_wave;
    regular_wave_init(&asv.regular_wave, 
                      wave_ht, 
                      frequency, 
                      phase_lag * PI/180.0, 
                      wave_heading * PI/180.0);
  }
  // init the asv after setting all inputs.
  asv_init(&asv);

  // Simulate
  int t = 0; // counter for time
  double time = 0.0;
  double simulation_time = 0.0;
  clock_t start, end;
  start = clock();
  for(int i = 0; i < waypoints.count; ++i)
  {
    for(;;++t) // break when reached the waypoint
    {
      // check buffer limit reached
      if(t >= OUTPUT_BUFFER_SIZE)
      {
        fprintf(stderr, "ERROR: output buffer exceeded.\n");
        // write output to file
        simulation_time = ((double)(end - start)) / CLOCKS_PER_SEC;
        write_output(out_file, 
                     t, 
                     wave_ht, 
                     wave_heading, 
                     rand_seed, 
                     time, 
                     simulation_time);
	      exit(1);
      }

      // check if reached destination
      double proximity_margin = 10.0; // target proximity to waypoint
      double x = asv.cog_position.x - waypoints.points[i].x;
      double y = asv.cog_position.y - waypoints.points[i].y;
      double distance = sqrt(x*x + y*y);
      if(distance <= proximity_margin)
      {
        break;
      }
      else
      {
        // compute time
        time = t * asv.dynamics.time_step_size; //sec

        // set propeller thrust and direction
        for(int p = 0; p < asv.count_propellers; ++p)
        {
          asv.propellers[p].thrust = 0.0; //N
          asv.propellers[p].orientation = (struct Dimensions){0.0, 0.0, 0.0};
        } 

        // compute new position and attitude.
        asv_compute_dynamics(&asv, time);

        // Also compute the wave elevation. 
        double wave_elevation = 0.0;
        if(asv.wave_type == regular_wave)
        {
          struct Dimensions point = {100, 0, 0};
          wave_elevation = regular_wave_get_elevation(&asv.regular_wave, 
                                              &asv.cog_position, time);
        }

        // save simulated data to buffer. 
        buffer[t].sig_wave_ht = asv.wave.significant_wave_height;
        buffer[t].wave_heading = asv.wave.heading * 180.0/PI;
        buffer[t].random_number_seed = rand_seed;
        buffer[t].time = time;
        buffer[t].wave_elevation = wave_elevation;
        buffer[t].cog_x = asv.cog_position.x;
        buffer[t].cog_y = asv.cog_position.y;
        buffer[t].cog_z = asv.cog_position.z - (asv.spec.cog.z - asv.spec.T);
        buffer[t].heel = asv.attitude.x * 180.0/PI;
        buffer[t].trim = asv.attitude.y * 180.0/PI;
        buffer[t].heading = asv.attitude.z * 180.0/PI;
        buffer[t].surge_velocity = asv.dynamics.V[surge];
        buffer[t].surge_acceleration = asv.dynamics.A[surge];
      }
    }
  }
  end = clock();
  simulation_time = ((double)(end - start)) / CLOCKS_PER_SEC;

  // write output to file
  write_output(out_file, 
               t, 
               wave_ht, 
               wave_heading, 
               rand_seed, 
               time, 
               simulation_time);

  return 0;
}
