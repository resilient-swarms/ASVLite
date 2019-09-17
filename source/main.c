#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "io.h"
#include "asv.h"

int main(int argc, char** argv)
{
  if(argc != 4)
  {
    fprintf(stderr, 
      "Error. Usage: %s input_file sig_wave_ht(m) wave_heading(deg).\n", 
      argv[0]);
    return 1;
  }
  double wave_ht, wave_heading;
  sscanf(argv[2], "%lf", &wave_ht);
  sscanf(argv[3], "%lf", &wave_heading);

  // Open output file 
  char out_file[120];
  sprintf(out_file, "%s_%.2f_%.1f", argv[1], wave_ht, wave_heading);

  // Init vehicle
  struct Asv asv;
  asv_init(&asv);

  // Waypoints
  struct Waypoints waypoints;

  // Set input data
  set_input(argv[1], &asv, &waypoints);

  // Init waves
  asv.using_waves = (wave_ht != 0.0); 
  if(asv.using_waves)
  {
    wave_init(&asv.wave, wave_ht, wave_heading * PI/180.0);
  }

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
        write_output(out_file, t, wave_ht, wave_heading, time, simulation_time);
	      exit(1);
      }

      // check if reached destination
      double proximity_margin = 1.0; // target proximity to waypoint
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
        time = t * asv.dynamics.time_step_size /1000.0; //sec

        // set propeller thrust and direction
        for(int p = 0; p < asv.count_propellers; ++p)
        {
          asv.propellers[p].thrust = 5.0;
          asv.propellers[p].orientation = (struct Attitude){0.0, 0.0, 0.0};
        } 

        // compute new position and attitude.
        asv_compute_dynamics(&asv, time);

        // Also compute the wave elevation. 
        double wave_elevation = 0.0;
        if(asv.using_waves)
        {
          wave_elevation = wave_get_elevation(&asv.wave, 
                                              &asv.cog_position, time);
        }

        // save simulated data to buffer. 
        buffer[t].time = time;
        buffer[t].wave_elevation = wave_elevation;
        buffer[t].cog_x = asv.cog_position.x;
        buffer[t].cog_y = asv.cog_position.y;
        buffer[t].cog_z = asv.cog_position.z - (asv.spec.cog.z - asv.spec.T);
        buffer[t].heel = asv.attitude.heel * 180.0/PI;
        buffer[t].trim = asv.attitude.trim * 180.0/PI;
        buffer[t].heel = asv.attitude.heading * 180.0/PI;
      }
    }
  }
  start = clock();
  simulation_time = ((double)(end - start)) / CLOCKS_PER_SEC;

  // write output to file
  write_output(out_file, t, wave_ht, wave_heading, time, simulation_time);

  return 0;
}
