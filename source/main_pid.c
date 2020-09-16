#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "io.h"
#include "asv.h"
#include "pid_controller.h"

int main(int argc, char** argv)
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

  // Initialise the PID controller
  struct PID_controller controller;
  pid_controller_init(&controller);

  // PID controller set gain terms
  double p_position = 1.0 * asv.dynamics.time_step_size;
  double i_position = 0.1 * asv.dynamics.time_step_size;
  double d_position = -10.0 * asv.dynamics.time_step_size;
  pid_controller_set_gains_position(&controller, 
                                    p_position, i_position, d_position);
  double p_heading = 1.0 * asv.dynamics.time_step_size;
  double i_heading = 0.1 * asv.dynamics.time_step_size;
  double d_heading = -10.0 * asv.dynamics.time_step_size;
  pid_controller_set_gains_heading(&controller, 
                                   p_heading, i_heading, d_heading);
  
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
      double proximity_margin = 2.0; // target proximity to waypoint
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

        // The propeller orientation is fixed. Steering is done by differential thrust.
        for(int p = 0; p < asv.count_propellers; ++p)
        {
          // Set a fixed orientation on each propeller
          asv.propellers[p].orientation = (struct Dimensions){0.0, 0.0, 0.0};
        }

        // Set differential thrust on each propeller.
        // ------------------------------------------
        // In controller set the way point for the current time step
        pid_controller_set_way_point(&controller, waypoints.points[i]);
        // Inform PID controller of the current state.
        pid_controller_set_current_state(&controller, asv.cog_position, asv.attitude);
        // PID controller estimate thrust to be applied on each propeller.
        pid_controller_set_thrust(&controller);
        // Set propeller thrust on each of the 4 propellers
        asv.propellers[0].thrust = controller.thrust_fore_ps; //N
        asv.propellers[1].thrust = controller.thrust_fore_sb; //N
        asv.propellers[2].thrust = controller.thrust_aft_ps;  //N
        asv.propellers[3].thrust = controller.thrust_aft_sb;  //N        

        // compute new position and attitude.
        asv_compute_dynamics(&asv, time);

        // Also compute the wave elevation. 
        double wave_elevation = 0.0;
        if(asv.wave_type == irregular_wave)
        {
          wave_elevation = wave_get_elevation(&asv.wave, 
                                              &asv.cog_position, time);
        }

        // save simulated data to buffer. 
        buffer[t].sig_wave_ht = asv.wave.significant_wave_height;
        buffer[t].wave_heading = asv.wave.heading * 180.0/PI;
        buffer[t].random_number_seed = asv.wave.random_number_seed;
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
