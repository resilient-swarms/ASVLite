#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <io.h>
#include <math.h>
#include "asv.h"

#define MAX_LINE_LENGTH 1000

// Function to read field from a csv file
void get_thrust_vec(char* line, double* thrust_vec)
{
  int i = 0; 
  char* column = strtok(line, ",");
  for(int i = 0; column != NULL && i < 4; ++i)
  {
    thrust_vec[i] = atof(column);
    column = strtok(NULL, ",");
  }
}

int main(int argc, char** argv)
{
  if(argc != 8)
  {
    fprintf(stderr, 
      "Error. " 
      "Usage: %s in_file out_file thrust_file wave_ht(m) wave_heading(deg) "
      "frequency(Hz) phase_lag(deg).\n", 
      argv[0]);
    return 1;
  }
  double wave_ht, wave_heading, phase_lag, frequency;
  int rand_seed = 1;
  char* in_file = argv[1];
  char* out_file = argv[2];
  char* thrust_file_name = argv[3];
  sscanf(argv[4], "%lf", &wave_ht);
  sscanf(argv[5], "%lf", &wave_heading);
  sscanf(argv[6], "%lf", &frequency);
  sscanf(argv[7], "%lf", &phase_lag);

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
                      wave_ht/2.0, 
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
  // Read thrust input file
  FILE* thrust_file = fopen(thrust_file_name, "r");
  if(thrust_file == NULL)
  {
    fprintf(stderr, "ERROR: could not open thrust file.");
    exit(1);
  }
  char thrust_file_line[MAX_LINE_LENGTH];
  while(fgets(thrust_file_line, MAX_LINE_LENGTH, thrust_file) != NULL)
  {
    // set propeller thrust and direction
    // Set propeller force.
    double thrust_vec[4];
    get_thrust_vec(thrust_file_line, thrust_vec); 
    asv.propellers[0].thrust = sqrt(thrust_vec[2]*thrust_vec[2] + 
                                    thrust_vec[3]*thrust_vec[3]);
    // time when the propeller should be applied.
    double t1 = thrust_vec[1];
    double thrust_angle = atan(thrust_vec[3]/thrust_vec[2]);
    asv.propellers[0].orientation = (struct Dimensions){0.0, 0.0, thrust_angle};
    
    for(;t<=t1;++t) 
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

      // compute time
      time = t * asv.dynamics.time_step_size; //sec

      // compute new position and attitude.
      asv_compute_dynamics(&asv, time);

      // Also compute the wave elevation. 
      double wave_elevation = 0.0;
      if(asv.wave_type == regular_wave)
      {
        struct Dimensions wave_probe = {0, 0, 0};
        wave_elevation = regular_wave_get_elevation(&asv.regular_wave, 
                                                    &wave_probe, time);
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
      buffer[t].F_surge = thrust_vec[2];
      buffer[t].F_sway = thrust_vec[3];
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
