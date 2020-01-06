#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <io.h>
#include <math.h>
#include <limits.h>
#include "asv.h"

#define MAX_LINE_LENGTH 1000

// Function to read field from a csv file
void get_thrust_vec(char* line, double* thrust_vec)
{
  int i = 0; 
  char* column = strtok(line, ",");
  for(int i = 0; column != NULL && i < 3; ++i)
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
  unsigned long t0 = ULONG_MAX; // counter for time
  unsigned long t = ULONG_MAX; // counter for time
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
    // Set propeller force.
    double thrust_vec[3];
    get_thrust_vec(thrust_file_line, thrust_vec); 
    // time when the propeller should be applied.
    unsigned long epoch = (unsigned long)(thrust_vec[0]*1000.0); // milli-sec
    // Set the simulation start time as equal to start epoch time of thrust file
    if(t > epoch)
    {
      t0 = epoch;
      t = epoch; 
    }
    // set propeller thrust and direction assuming there are 4 propellers in the 
    // order - (aft,ps), (fore, ps), (fore, sb), (aft, sb)
    // Forward thrust - assumptions: uses propeller 0 and 2
    asv.propellers[0].thrust = thrust_vec[1]/2.0;
    asv.propellers[2].thrust = thrust_vec[1]/2.0;
    asv.propellers[0].orientation = (struct Dimensions){0.0, 0.0, 0.0};
    asv.propellers[2].orientation = (struct Dimensions){0.0, 0.0, 0.0};
    // Sway thrust - assumption: uses propellers 1 and 3
    asv.propellers[1].thrust = thrust_vec[2]/2.0;
    asv.propellers[3].thrust = thrust_vec[2]/2.0;
    asv.propellers[1].orientation = (struct Dimensions){0.0, 0.0, 3.0*PI/2.0};
    asv.propellers[3].orientation = (struct Dimensions){0.0, 0.0, 3.0*PI/2.0};
    
    for(;t<=epoch;++t) 
    {
      // check buffer limit reached
      if((t-t0) >= OUTPUT_BUFFER_SIZE)
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
      time = (t - t0) * asv.dynamics.time_step_size; //sec

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
      int i = t - t0;
      buffer[i].sig_wave_ht = asv.wave.significant_wave_height;
      buffer[i].wave_heading = asv.wave.heading * 180.0/PI;
      buffer[i].random_number_seed = rand_seed;
      buffer[i].time = (t0/1000.0) + time;
      buffer[i].wave_elevation = wave_elevation;
      buffer[i].cog_x = asv.cog_position.x;
      buffer[i].cog_y = asv.cog_position.y;
      buffer[i].cog_z = asv.cog_position.z - (asv.spec.cog.z - asv.spec.T);
      buffer[i].heel = asv.attitude.x * 180.0/PI;
      buffer[i].trim = asv.attitude.y * 180.0/PI;
      buffer[i].heading = asv.attitude.z * 180.0/PI;
      buffer[i].surge_velocity = asv.dynamics.V[surge];
      buffer[i].surge_acceleration = asv.dynamics.A[surge];
      buffer[i].F_surge = thrust_vec[1];
      buffer[i].F_sway = thrust_vec[2];
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
