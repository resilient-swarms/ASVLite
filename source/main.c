#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "io.h"
#include "asv.h"

void compute_dynamics_per_thread(void* first_node)
{
  struct Simulation_data* node = (struct Simulation_data*)first_node;
  // Current time
  double current_time = node->current_time_index * node->asv->dynamics.time_step_size; //sec

  // set propeller thrust and direction
  for(int p = 0; p < node->asv->count_propellers; ++p)
  {
    node->asv->propellers[p].thrust = 0.25; //N
    node->asv->propellers[p].orientation = (struct Dimensions){0.0, 0.0, 0.0};
  }

  // Compute the dynamics of asv for the current time step
  asv_compute_dynamics(node->asv, current_time);

  // Also compute the wave elevation. 
  double wave_elevation = 0.0;
  if(node->asv->wave_type == irregular_wave)
  {
    wave_elevation = wave_get_elevation(&(node->asv->wave), 
                                        &(node->asv->cog_position), 
                                        current_time);
  }

  // save simulated data to buffer. 
  node->buffer[node->current_time_index].sig_wave_ht        = node->asv->wave.significant_wave_height;
  node->buffer[node->current_time_index].wave_heading       = node->asv->wave.heading * 180.0/PI;
  node->buffer[node->current_time_index].random_number_seed = node->asv->wave.random_number_seed;
  node->buffer[node->current_time_index].time               = current_time;
  node->buffer[node->current_time_index].wave_elevation     = wave_elevation;
  node->buffer[node->current_time_index].cog_x              = node->asv->cog_position.x;
  node->buffer[node->current_time_index].cog_y              = node->asv->cog_position.y;
  node->buffer[node->current_time_index].cog_z              = node->asv->cog_position.z - (node->asv->spec.cog.z - node->asv->spec.T);
  node->buffer[node->current_time_index].heel               = node->asv->attitude.x * 180.0/PI;
  node->buffer[node->current_time_index].trim               = node->asv->attitude.y * 180.0/PI;
  node->buffer[node->current_time_index].heading            = node->asv->attitude.z * 180.0/PI;
  node->buffer[node->current_time_index].surge_velocity     = node->asv->dynamics.V[surge];
  node->buffer[node->current_time_index].surge_acceleration = node->asv->dynamics.A[surge];

  // Check if reached the waypoint
  double proximity_margin = 10.0; // target proximity to waypoint
  int i = node->current_waypoint_index;
  double x = node->asv->cog_position.x - node->waypoints->points[i].x;
  double y = node->asv->cog_position.y - node->waypoints->points[i].y;
  double distance = sqrt(x*x + y*y);
  if(distance <= proximity_margin)
  {
    // Reached the current waypoint, so increament the index to the next waypoint.
    // if the current_waypoint_index == waypoint.count ==> reached final waypoint.
    ++(node->current_waypoint_index);
  }
}

void simulate(struct Simulation_data* first_node)
{
  bool buffer_exceeded = false;
  for(long t = 0; ; ++t)
  {
    // Variable to check if all reached the destination.
    bool has_all_reached_final_waypoint = true;

    // Create threads
    // int limit_threads = get_nprocs();
    // spawn threads
    for(struct Simulation_data* node = first_node; node != NULL; node = node->next)
    {
      // Check if asv reached the final waypoint.
      if(node->current_waypoint_index < node->waypoints->count)
      {
        // Not yet reached the final waypoint, but check if buffer limit reached before further computation.
        // Check if buffer exceeded
        if(node->current_time_index >= OUTPUT_BUFFER_SIZE)
        {
          // buffer exceeded
          buffer_exceeded = true;
          fprintf(stderr, "ERROR: output buffer exceeded for asv with id '%s'.\n", node->id);
          break;        
        }
        has_all_reached_final_waypoint = false;
        node->current_time_index = t;
        pthread_create(&(node->thread), NULL, &compute_dynamics_per_thread, (void*)node);
      }
    }
    // join threads
    for(struct Simulation_data* node = first_node; node != NULL; node = node->next)
    {
      if(node->current_waypoint_index < node->waypoints->count)
      {
        pthread_join(node->thread, NULL);
      }
    }

    // stop if all reached the destination or if buffer exceeded.
    if(has_all_reached_final_waypoint || buffer_exceeded)
    {
      break;
    }
  }
}

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
  double wave_height = strtod(argv[3], p_end);
  double wave_heading = strtod(argv[4], p_end);
  long rand_seed = strtol(argv[5], p_end, 10);  

  // Set simulation inputs
  struct Simulation_data* simulation_data = simulation_data_new_node();
  simulation_data_set_input(simulation_data,
                            in_file, 
                            wave_height,
                            wave_heading,
                            rand_seed);

  // Simulate
  struct timespec start, finish;
  double elapsed;
  // Ref: https://stackoverflow.com/questions/2962785/c-using-clock-to-measure-time-in-multi-threaded-programs
  // time() provides a resolution of only 1 sec so its not good if simulation is really short. 
  // In a unix the better option is to use clock_gettime() along with CLOCK_MONOTONIC. 
  clock_gettime(CLOCK_MONOTONIC, &start);
  simulate(simulation_data);
  clock_gettime(CLOCK_MONOTONIC, &finish);
  elapsed = (finish.tv_sec - start.tv_sec);
  elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

  // write output to file
  simulation_data_write_output(simulation_data, out_file, elapsed);

  // Clean the memory
  simulation_data_clean(simulation_data);

  return 0;
}
