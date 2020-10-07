#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "simulation.h"

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

  char* p_end; 
  char* in_file = argv[1];
  char* out_file = argv[2]; 
  double wave_height = strtod(argv[3], &p_end);
  double wave_heading = strtod(argv[4], &p_end);
  long rand_seed = strtol(argv[5], &p_end, 10);  

  // Set simulation inputs
  struct Simulation* simulation = simulation_new();
  simulation_set_input(simulation,
                       in_file, 
                       wave_height,
                       wave_heading,
                       rand_seed);

  // Simulate and record the time taken for the simulation.
  struct timespec start, finish;
  double elapsed;
  // Ref: https://stackoverflow.com/questions/2962785/c-using-clock-to-measure-time-in-multi-threaded-programs
  // time() provides a resolution of only 1 sec so its not good if simulation is really short. 
  // In a unix the better option is to use clock_gettime() along with CLOCK_MONOTONIC. 
  clock_gettime(CLOCK_MONOTONIC, &start);
  #ifdef ENABLE_TIME_SYNC
  simulation_run_with_time_sync(simulation);
  #else
  simulation_run_without_time_sync(simulation);
  #endif
  clock_gettime(CLOCK_MONOTONIC, &finish);
  elapsed = (finish.tv_sec - start.tv_sec);
  elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

  // write output to file
  simulation_write_output(simulation, out_file, elapsed);

  // Clean the memory
  simulation_clean(simulation);

  return 0;
}
