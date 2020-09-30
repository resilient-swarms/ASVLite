#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "io.h"
#include "asv.h"

void compute_dynamics_per_thread(void* simulation_data)
{
  struct Simulation_data* data = (struct Simulation_data*)simulation_data;
  // Current time
  double current_time = data->current_time_index * data->asv.dynamics.time_step_size; //sec

  // set propeller thrust and direction
  for(int p = 0; p < data->asv.count_propellers; ++p)
  {
    data->asv.propellers[p].thrust = 0.25; //N
    data->asv.propellers[p].orientation = (struct Dimensions){0.0, 0.0, 0.0};
  }

  // Compute the dynamics of asv for the current time step
  asv_compute_dynamics(&(data->asv), current_time);

  // Also compute the wave elevation. 
  double wave_elevation = 0.0;
  if(data->asv.wave_type == irregular_wave)
  {
    wave_elevation = wave_get_elevation(&(data->asv.wave), 
                                        &(data->asv.cog_position), 
                                        current_time);
  }

  // save simulated data to buffer. 
  data->buffer[data->current_time_index].sig_wave_ht        = data->asv.wave.significant_wave_height;
  data->buffer[data->current_time_index].wave_heading       = data->asv.wave.heading * 180.0/PI;
  data->buffer[data->current_time_index].random_number_seed = data->asv.wave.random_number_seed;
  data->buffer[data->current_time_index].time               = current_time;
  data->buffer[data->current_time_index].wave_elevation     = wave_elevation;
  data->buffer[data->current_time_index].cog_x              = data->asv.cog_position.x;
  data->buffer[data->current_time_index].cog_y              = data->asv.cog_position.y;
  data->buffer[data->current_time_index].cog_z              = data->asv.cog_position.z - (data->asv.spec.cog.z - data->asv.spec.T);
  data->buffer[data->current_time_index].heel               = data->asv.attitude.x * 180.0/PI;
  data->buffer[data->current_time_index].trim               = data->asv.attitude.y * 180.0/PI;
  data->buffer[data->current_time_index].heading            = data->asv.attitude.z * 180.0/PI;
  data->buffer[data->current_time_index].surge_velocity     = data->asv.dynamics.V[surge];
  data->buffer[data->current_time_index].surge_acceleration = data->asv.dynamics.A[surge];

  // Check if reached the waypoint
  double proximity_margin = 10.0; // target proximity to waypoint
  int i = data->current_waypoint_index;
  double x = data->asv.cog_position.x - data->waypoints.points[i].x;
  double y = data->asv.cog_position.y - data->waypoints.points[i].y;
  double distance = sqrt(x*x + y*y);
  if(distance <= proximity_margin)
  {
    // Reached the current waypoint, so increament the index to the next waypoint.
    // if the current_waypoint_index == waypoint.count ==> reached final waypoint.
    ++data->current_waypoint_index;
  }
}

void simulate(struct Simulation_data* simulation_data)
{
  bool buffer_exceeded = false;
  for(long t = 0; ; ++t)
  {
    // Variable to check if all reached the destination.
    bool has_all_reached_final_waypoint = true;

    // Create threads
    // int limit_threads = get_nprocs();
    // spawn threads
    for(struct Simulation_data* data = simulation_data; data != NULL; data = data->next)
    {
      // Check if buffer exceeded
      if(simulation_data->current_time_index >= OUTPUT_BUFFER_SIZE)
      {
        // buffer exceeded
        buffer_exceeded = true;
        fprintf(stderr, "ERROR: output buffer exceeded for asv with id %s.\n", data->id);
        break;        
      }

      // Check if asv reached the final waypoint.
      if(data->current_waypoint_index < data->waypoints.count)
      {
        // Not yet reached the final waypoint
        has_all_reached_final_waypoint = false;
        data->current_time_index = t;
        pthread_create(&(data->thread), NULL, &compute_dynamics_per_thread, (void*)data);
      }
    }
    // join threads
    //for(struct Simulation_data* data = simulation_data; data != NULL; data = data->next)
    //{
    //  pthread_join(data->thread, NULL);
    //}

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
  struct Simulation_data simulation_data;
  simulation_data_set_input(&simulation_data,
                            in_file, 
                            wave_height,
                            wave_heading,
                            rand_seed);

  // Simulate
  clock_t start, end;
  start = clock();
  simulate(&simulation_data);
  end = clock();
  double simulation_time = ((double)(end - start)) / CLOCKS_PER_SEC;

  // write output to file
  simulation_data_write_output(&simulation_data, out_file, simulation_time);

  return 0;
}
