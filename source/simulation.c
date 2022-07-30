#include <stdlib.h>
#include <sys/stat.h> // for creating directory
#include <pthread.h>
#include <string.h>
#include "toml.h"
#include "simulation.h"
#include "pid_controller.h"
#include "asv.h"
#include "wave.h"
#include "geometry.h"
#include "errors.h"
#include "constants.h"

#define OUTPUT_BUFFER_SIZE 200000 /*!< Output buffer size. */

struct Thread_args
{
  struct Simulation* node;
  char* out_dir;
};

/**
 * Structure to record the sea state and vehicle dynamics for a time step of the simulation.
 */
struct Buffer
{ 
  double time; // sec.
  // Sea state
  double sig_wave_ht;  // m
  double wave_heading; // deg
  double wave_elevation; // Wave elevation at the position of the vehicle, m.
  // Vehicle dynamics
  double F_surge; // N
  double surge_acceleration; // m/s2.
  double surge_velocity; // m/s.
  double cog_x;   // m.
  double cog_y;   // m.
  double cog_z;   // m.
  double heel;    // deg.
  double trim;    // deg. 
  double heading; // deg. 
};

/**
 * Simulation is a node in a linked list and it stores simulation data related to a vehicle in simulation.
 */
struct Simulation
{
  // Each simulation runs on its own thread
  pthread_t thread;
  // Inputs and outputs
  char id[32];
  struct Wave* wave;
  struct Asv* asv; 
  struct Controller* controller;
  union Coordinates_3D* waypoints;
  int count_waypoints;
  struct Buffer* buffer;
  // Data related to current time step
  double time_step_size; // milliseconds
  long current_time_index;
  int buffer_index;
  double max_time; // seconds
  int current_waypoint_index;

  // Link pointers
  struct Simulation* previous; // previous in the linked list.
  struct Simulation* next; // next in the linked list.
  void (*simulation_run)(struct Simulation*, char*); // Pointer to function for executing the simulation. 
  char* error_msg;
};

static void simulation_write_output(struct Simulation* node, char* out_dir)
{
  // Check if the directory exist and create it if it does not.
  struct stat st = {0};
  if (stat(out_dir, &st) == -1) 
  {
    mkdir(out_dir, 0700);
  }

  FILE *fp;
  // Create file name as out_dir/node_id
  char file[128];
  strcpy(file, out_dir);
  strcat(file, "/");
  strcat(file, node->id);
  // Open the file
  if (!(fp = fopen(file, "a")))
  {
    fprintf(stderr, "ERROR: Cannot open output file %s.\n", file);
    exit(1);
  }
  // Check if the file is empty and add header only for empty file.
  fseek(fp, 0, SEEK_END);
  long size = ftell(fp);
  if (size == 0)
  {
    // file is empty, add header.
    fprintf(fp,
            "time(sec) "
            "sig_wave_ht(m) "
            "wave_heading(deg) "
            "wave_elevation(m) "
            "F_surge(N) "
            "surge_acc(m/s2) "
            "surge_vel(m/s) "
            "cog_x(m) "
            "cog_y(m) "
            "cog_z(m) "
            "heel(deg) "
            "trim(deg) "
            "heading(deg) ");
  }
  // write buffer to file and close the file.
  for (int i = 0; i < node->buffer_index; ++i)
  {
    fprintf(fp, "\n%f %f %f %f %f %f %f %f %f %f %f %f %f",
            node->buffer[i].time,
            node->buffer[i].sig_wave_ht,
            node->buffer[i].wave_heading,
            node->buffer[i].wave_elevation,
            node->buffer[i].F_surge,
            node->buffer[i].surge_acceleration,
            node->buffer[i].surge_velocity,
            node->buffer[i].cog_x,
            node->buffer[i].cog_y,
            node->buffer[i].cog_z,
            node->buffer[i].heel,
            node->buffer[i].trim,
            node->buffer[i].heading);
  }
  fclose(fp);
}

// Computes dynamics for current node for the current time step.
static void* simulation_run_per_node_per_time_step(void* current_node)
{
  struct Simulation* node = (struct Simulation*)current_node;
  // Current time
  double current_time = (node->current_time_index+1) * node->time_step_size/1000.0; //sec

  // Set differential thrust on each thruster.
  // ------------------------------------------
  // PID controller estimate thrust to be applied on each thruster.
  controller_set_thrust(node->controller, node->waypoints[node->current_waypoint_index]);

  // Compute the dynamics of asv for the current time step
  asv_compute_dynamics(node->asv, node->time_step_size);
  const char* error_msg = asv_get_error_msg(node->asv);
  if(error_msg)
  {
    set_error_msg(node->error_msg, error_msg);
    return NULL;
  }
  struct Asv_specification spec = asv_get_spec(node->asv);
  union Coordinates_3D cog_position = asv_get_position_cog(node->asv);
  union Coordinates_3D attitude = asv_get_attitude(node->asv);

  // save simulated data to buffer. 
  node->buffer[node->buffer_index].time               = current_time;
  node->buffer[node->buffer_index].sig_wave_ht        = wave_get_significant_height(node->wave);
  node->buffer[node->buffer_index].wave_heading       = wave_get_predominant_heading(node->wave) * 180.0/PI;
  node->buffer[node->buffer_index].wave_elevation     = wave_get_elevation(node->wave, cog_position, current_time);
  node->buffer[node->buffer_index].F_surge            = asv_get_F(node->asv).keys.pitch;
  node->buffer[node->buffer_index].surge_acceleration = asv_get_A(node->asv).keys.surge;
  node->buffer[node->buffer_index].surge_velocity     = asv_get_V(node->asv).keys.surge;
  node->buffer[node->buffer_index].cog_x              = cog_position.keys.x;
  node->buffer[node->buffer_index].cog_y              = cog_position.keys.y;
  node->buffer[node->buffer_index].cog_z              = cog_position.keys.z - (spec.cog.keys.z - spec.T);
  node->buffer[node->buffer_index].heel               = attitude.keys.x * 180.0/PI;
  node->buffer[node->buffer_index].trim               = attitude.keys.y * 180.0/PI;
  node->buffer[node->buffer_index].heading            = attitude.keys.z * 180.0/PI;

  // Check if reached the waypoint
  double proximity_margin = 5.0; // target proximity to waypoint
  int i = node->current_waypoint_index;
  double x = cog_position.keys.x - node->waypoints[i].keys.x;
  double y = cog_position.keys.y - node->waypoints[i].keys.y;
  double distance = sqrt(x*x + y*y);
  if(distance <= proximity_margin)
  {
    // Reached the current waypoint, so increament the index to the next waypoint.
    // if the current_waypoint_index == waypoint.count ==> reached final waypoint.
    ++(node->current_waypoint_index);
  }
  // Increment buffer counter
  ++(node->buffer_index);
  return NULL;
}

static void* simulation_run_per_node_without_time_sync(void* args)
{
  // Extract the arguments passed
  struct Simulation* node = (struct Simulation*)((struct Thread_args*)args)->node;
  char* out_dir = (char*)((struct Thread_args*)args)->out_dir;

  for(node->current_time_index = 0; 
      node->max_time == 0 || node->current_time_index*node->time_step_size/1000.0 < node->max_time; 
      ++(node->current_time_index))
  {
    if(node->current_waypoint_index < node->count_waypoints)
    {
      // Not yet reached the final waypoint, but check if buffer limit reached before further computation.
      // Check if buffer exceeded
      if(node->buffer_index >= OUTPUT_BUFFER_SIZE)
      {
        // Buffer exceeded. Dump buffer to output file.
        if(out_dir)
        {
          simulation_write_output(node, out_dir); 
        }  
        else
        {
          // No out_dir provided. Skip writing output to file.
        }
        // Reset the buffer
        node->buffer_index = 0;
      }
      if(node->error_msg)
      {
        break;
      }
      // If buffer not exceeded and no error.
      simulation_run_per_node_per_time_step((void*)node);
    }
    else
    {
      // Reached the final waypoint
      break;
    }
  }
  return NULL;
}

static void simulation_spawn_nodes_with_time_sync(void* args)
{
  // Extract the arguments passed
  struct Simulation* first_node = (struct Simulation*)((struct Thread_args*)args)->node;
  char* out_dir = (char*)((struct Thread_args*)args)->out_dir;

  bool buffer_exceeded = false;
  for(long t = 0; ; ++t)
  {
    // Variable to check if all reached the destination.
    bool has_all_reached_final_waypoint = true;

    // Create threads
    // int limit_threads = get_nprocs();
    // spawn threads
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      // Set time step for the node
      node->current_time_index = t;

      // Check if simulating for max time and stop if required.
      if(node->max_time == 0 || 
         t*node->time_step_size/1000.0 < node->max_time)
      {
        // Check if asv reached the final waypoint.
        if(node->current_waypoint_index < node->count_waypoints)
        {
          // Not yet reached the final waypoint, but check if buffer limit reached before further computation.
          // Check if buffer exceeded
          if(node->buffer_index >= OUTPUT_BUFFER_SIZE)
          {
            // Buffer exceeded. Dump buffer to output file.
            if(out_dir)
            {
              simulation_write_output(node, out_dir); 
            }  
            else
            {
              // No out_dir provided. Skip writing output to file.
            }      
            // Reset buffer
            node->buffer_index = 0;
          }
          has_all_reached_final_waypoint = false;
          struct Thread_args args;
          args.node = node;
          args.out_dir = out_dir;
          #ifdef DISABLE_MULTI_THREADING
          simulation_run_per_node_per_time_step((void*)&args);
          if(node->error_msg)
          {
            break;
          }
          #else
          pthread_create(&(node->thread), NULL, &simulation_run_per_node_per_time_step, (void*)&args);
          #endif
        }
      }
    }
    // Join threads. Also check if there were errors in any node.
    bool has_error = false;
    #ifndef DISABLE_MULTI_THREADING
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      if(node->current_waypoint_index < node->count_waypoints)
      {
        if(node->error_msg)
        {
          has_error = true;
        }
        pthread_join(node->thread, NULL);
      }
    }
    #endif

    // stop if all reached the destination or if buffer exceeded.
    if(has_all_reached_final_waypoint || buffer_exceeded || has_error)
    {
      break;
    }
  }
}

/**
 * Simulate vehicle dynamics for each time step. This function runs 
 * simultion of each ASV in a independent thread and does not synchronize
 * the simulation for each time step between ASVs. This function is faster
 * compared to the alternative simulate_with_time_sync().
 */
static void simulation_spawn_nodes_without_time_sync(struct Simulation* first_node, char* out_dir)
{
  for(struct Simulation* node = first_node; node != NULL; node = node->next)
  {
    struct Thread_args args;
    args.node = node;
    args.out_dir = out_dir;
    pthread_create(&(node->thread), NULL, &simulation_run_per_node_without_time_sync, (void*)&args);
  }
  // join threads
  for(struct Simulation* node = first_node; node != NULL; node = node->next)
  {
    if(node->current_waypoint_index < node->count_waypoints)
    {
      pthread_join(node->thread, NULL);
    }
  }
}

struct Simulation* simulation_new_node()
{
  // Initialise memory
  struct Simulation* node = (struct Simulation*)malloc(sizeof(struct Simulation));
  node->wave = NULL;
  node->asv = NULL;
  node->controller = NULL;
  node->waypoints = NULL;
  node->buffer = (struct Buffer*)malloc(OUTPUT_BUFFER_SIZE * sizeof(struct Buffer));
  node->previous = NULL;
  node->next = NULL;
  node->error_msg = NULL;
  node->simulation_run = NULL;
  node->count_waypoints = 0;
  node->time_step_size = 40.0;
  node->current_time_index = 0;
  node->max_time = 0.0;
  node->current_waypoint_index = 0;
  return node;
}

struct Simulation* simulation_new()
{
  return simulation_new_node();
}

void simulation_delete(struct Simulation* first_node)
{
  if(first_node)
  {
  for(struct Simulation* current_node = first_node; current_node != NULL;)
    {
      wave_delete(current_node->wave);
      asv_delete(current_node->asv);
      free(current_node->error_msg);
      free(current_node->buffer);
      free(current_node->waypoints);
      free(current_node);
      struct Simulation* next_node = current_node->next;
      current_node->next = NULL;
      current_node = next_node;
    }
  }
}

void simulation_set_input_using_file(struct Simulation* first_node,
                                     char *file,  
                                     double wave_ht, 
                                     double wave_heading, 
                                     long rand_seed,
                                     bool with_time_sync)
{
  char error_buffer[100];
  char* error_missing_table = "Error in input file. Missing %s.";
  char* error_missing_variable = "Error in input file. Missing variable %s in %s[%d].";
  char* error_bad_value = "Error in input file. Bad value for variable %s in %s[%d].";

  if(!first_node)
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  }
  clear_error_msg(first_node->error_msg);

  // buffer to hold raw data from input file.
  const char *raw;

  // Open file to parse
  FILE *fp = fopen(file, "r");
  if (fp == 0)
  {
    if(sizeof(file) > 70)
    {
      snprintf(error_buffer, sizeof(error_buffer), "Cannot open input file.");
    }
    else
    {
      snprintf(error_buffer, sizeof(error_buffer), "Cannot open input file %s.", file);
    }
    set_error_msg(first_node->error_msg, error_buffer);
    return;
  }

  // Read the input file into the root table.
  char errbuf[200];
  toml_table_t *input = toml_parse_file(fp, errbuf, sizeof(errbuf));
  fclose(fp);
  if (input == 0)
  {
    set_error_msg(first_node->error_msg, "Error parsing toml file.");
    return;
  }

  // Read tables [asv]
  toml_array_t *tables = toml_array_in(input, "asv");
  if (tables == 0)
  {
    snprintf(error_buffer, sizeof(error_buffer), error_missing_table, "[[asv]]");
    set_error_msg(first_node->error_msg, error_buffer);
    return;
  }
  
  // get number of asvs
  int count_asvs = toml_array_nelem(tables);
  // iterate each asv table
  struct Simulation* current = first_node;
  for (int n = 0; n < count_asvs; ++n)
  {
    // Create a new entry to the linked list of simulation data.
    if(n != 0)
    {
      struct Simulation* previous = current;
      // Create a new entry to the linked list.
      current = simulation_new_node();
      // Link it to the previous entry in the linked list.
      previous->next = current; 
      current->previous = previous;
    } 
    
    // Set the Pointer to function for executing the simulation.
    if(with_time_sync)
    {
      current->simulation_run = simulation_spawn_nodes_with_time_sync;
    }
    else
    {
      current->simulation_run = simulation_spawn_nodes_without_time_sync;
    }
    // Create and initialise the sea surface
    if(wave_ht)
    {
      int count_wave_spectral_directions  = 5;
      int count_wave_spectral_frequencies = 15;
      current->wave = wave_new(wave_ht, 
                              normalise_angle_2PI(wave_heading * PI/180.0), 
                              rand_seed, 
                              count_wave_spectral_directions, 
                              count_wave_spectral_frequencies);
      if(!current->wave)
      {
        snprintf(error_buffer, sizeof(error_buffer), "Could not create wave with height %lf, heading %lf, rand seed %d", wave_ht, wave_heading, rand_seed);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
    }
    else
    {
      current->wave = NULL;
    }
    
    // ASV specification
    struct Asv_specification asv_spec;
    
    // Get toml table to set the input data
    toml_table_t *table = toml_table_at(tables, n);
    if (table == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_table, "[asv]");
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // Extract values in table [asv]
    // id
    raw = toml_raw_in(table, "id");
    char* id;
    if(raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "id", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtos(raw, &id))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "id", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    else
    {
      strcpy(current->id, id);
    }    

    // L_wl
    raw = toml_raw_in(table, "L_wl");
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "L_wl", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.L_wl)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "L_wl", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // B_wl
    raw = toml_raw_in(table, "B_wl");
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "B_wl", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.B_wl)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "B_wl", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // D
    raw = toml_raw_in(table, "D");
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "D", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.D)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "D", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // T
    raw = toml_raw_in(table, "T");
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "T", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.T)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "T", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // displacement
    raw = toml_raw_in(table, "displacement");
    if (raw == 0) 
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "displacement", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.disp)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "displacement", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // max_speed
    raw = toml_raw_in(table, "max_speed");
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "max_speed", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.max_speed)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "max_speed", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }

    // cog
    toml_array_t *array = toml_array_in(table, "cog");
    if (array == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "cog", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // cog.x
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "cog[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.cog.keys.x)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "cog[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // cog.y
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "cog[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.cog.keys.y)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "cog[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // cog.z
    raw = toml_raw_at(array, 2);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "cog[2]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.cog.keys.z)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "cog[2]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }

    // radius_of_gyration
    array = toml_array_in(table, "radius_of_gyration");
    if (array == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "radius_of_gyration", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // radius_of_gyration.x
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "radius_of_gyration[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.r_roll)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "radius_of_gyration[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // radius_of_gyration.pitch
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "radius_of_gyration[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.r_pitch)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "radius_of_gyration[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // radius_of_gyration.yaw
    raw = toml_raw_at(array, 2);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "radius_of_gyration[2]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(asv_spec.r_yaw)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "radius_of_gyration[2]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }

    // asv_position
    union Coordinates_3D origin_position;
    array = toml_array_in(table, "asv_position");
    if (array == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_position", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // asv_position.x
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_position[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(origin_position.keys.x)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "asv_position[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // asv_position.y
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_position[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(origin_position.keys.y)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "asv_position[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }

    // asv_attitude
    union Coordinates_3D attitude;
    array = toml_array_in(table, "asv_attitude");
    if (array == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_attitude", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // Extract values in table [vehicle_attitude]
    // heel
    double heel = 0.0;
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_attitude[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(heel)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "asv_attitude[0]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    else
    {
      // convert to radians and set value
      attitude.keys.x = heel * PI / 180.0;
    }
    // trim
    double trim = 0.0;
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_attitude[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(trim)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "asv_attitude[1]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    else
    {
      // convert to radians and set value
      attitude.keys.y = trim * PI / 180.0;
    }
    // heading
    double heading = 0.0;
    raw = toml_raw_at(array, 2);
    if (raw == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "asv_attitude[2]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    if (toml_rtod(raw, &(heading)))
    {
      snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "asv_attitude[2]", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    else
    {
      // convert to radians and set value
      attitude.keys.z = normalise_angle_2PI(heading * PI / 180.0);
    }

    // Initialise the asv
    current->asv = asv_new(asv_spec, current->wave, origin_position, attitude);

    // thrusters
    toml_table_t *arrays = toml_array_in(table, "thrusters");
    if (array == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "thrusters", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // get number of thrusters
    int count_thrusters = toml_array_nelem(arrays);
    struct Thruster** thrusters = (struct Thruster**)malloc(sizeof(struct Thruster*) * count_thrusters);
    // Set thruster data
    for (int i = 0; i < count_thrusters; ++i)
    {
      union Coordinates_3D thruster_position;
      array = toml_array_at(arrays, i);
      // x
      raw = toml_raw_at(array, 0);
      if (raw == 0)
      {
        char* thruster_index[16];
        snprintf(thruster_index, sizeof(thruster_index), "thrusters[%d][0]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, thruster_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      if (toml_rtod(raw, &(thruster_position.keys.x)))
      {
        char* thruster_index[16];
        snprintf(thruster_index, sizeof(thruster_index), "thrusters[%d][0]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_bad_value, thruster_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      // y
      raw = toml_raw_at(array, 1);
      if (raw == 0)
      {
        char* thruster_index[16];
        snprintf(thruster_index, sizeof(thruster_index), "thrusters[%d][1]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, thruster_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      if (toml_rtod(raw, &(thruster_position.keys.y)))
      {
        char* thruster_index[16];
        snprintf(thruster_index, sizeof(thruster_index), "thrusters[%d][1]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_bad_value, thruster_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      // z
      raw = toml_raw_at(array, 2);
      if (raw == 0)
      {
        char* thruster_index[16];
        snprintf(thruster_index, sizeof(thruster_index), "thrusters[%d][2]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, thruster_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      if (toml_rtod(raw, &(thruster_position.keys.z)))
      {
        char* thruster_index[16];
        snprintf(thruster_index, sizeof(thruster_index), "thrusters[%d][2]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_bad_value, thruster_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      thrusters[i] = thruster_new(thruster_position);
    }
    // Set the thrusters on the ASV
    asv_set_thrusters(current->asv, thrusters, count_thrusters);
    free(thrusters);

    // waypoints
    arrays = toml_array_in(table, "waypoints");
    if (arrays == 0)
    {
      snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, "waypoints", "[asv]", n);
      set_error_msg(first_node->error_msg, error_buffer);
      return;
    }
    // get number of waypoints
    current->count_waypoints = toml_array_nelem(arrays);
    current->waypoints = (union Coordinates_3D*)malloc(sizeof(union Coordinates_3D) * current->count_waypoints);
    // Set waypoint data
    for (int i = 0; i < current->count_waypoints; ++i)
    {
      array = toml_array_at(arrays, i);
      // x
      raw = toml_raw_at(array, 0);
      if (raw == 0)
      {
        char* waypoint_index[16];
        snprintf(waypoint_index, sizeof(waypoint_index), "waypoints[%d][0]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, waypoint_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      if (toml_rtod(raw, &(current->waypoints[i].keys.x)))
      {
        char* waypoint_index[16];
        snprintf(waypoint_index, sizeof(waypoint_index), "waypoints[%d][0]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_bad_value, waypoint_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      // y
      raw = toml_raw_at(array, 1);
      if (raw == 0)
      {
        char* waypoint_index[16];
        snprintf(waypoint_index, sizeof(waypoint_index), "waypoints[%d][1]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_missing_variable, waypoint_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
      if (toml_rtod(raw, &(current->waypoints[i].keys.y)))
      {
        char* waypoint_index[16];
        snprintf(waypoint_index, sizeof(waypoint_index), "waypoints[%d][1]", i);
        snprintf(error_buffer, sizeof(error_buffer), error_bad_value, waypoint_index, "[asv]", n);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
    }
  }

  // Locate table [clock]
  current->time_step_size = 40.0; // default value for time step size
  toml_table_t* table = toml_table_in(input, "clock");
  if (table != 0)
  {
    // Extract values in table [clock]
    // time_step_size
    raw = toml_raw_in(table, "time_step_size");
    if (raw != 0)
    {
      if (toml_rtod(raw, &current->time_step_size))
      {
        snprintf(error_buffer, sizeof(error_buffer), error_bad_value, "time_step_size", "[clock]", 0);
        set_error_msg(first_node->error_msg, error_buffer);
        return;
      }
    }
  }

  // done reading inputs
  toml_free(input);
}

void simulation_set_input_using_asvs(struct Simulation* first_node,
                                    struct Asv** asvs,  
                                    int count_asvs,
                                    bool with_time_sync)
{
  if(first_node && asvs)
  {
    clear_error_msg(first_node->error_msg);
    struct Simulation* current = first_node;
    for (int n = 0; n < count_asvs; ++n)
    {
      // Create a new entry to the linked list of simulation data.
      if(n != 0)
      {
        struct Simulation* previous = current;
        // Create a new entry to the linked list.
        current = simulation_new_node();
        // Link it to the previous entry in the linked list.
        previous->next = current; 
        current->previous = previous;
      }

      // Set the Pointer to function for executing the simulation.
      if(with_time_sync)
      {
        current->simulation_run = simulation_spawn_nodes_with_time_sync;
      }
      else
      {
        current->simulation_run = simulation_spawn_nodes_without_time_sync;
      }

      // Create and initialise the sea surface
      current->asv = asvs[n];
      current->wave = asv_get_wave(current->asv);
    }  
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
  } 
}

void simulation_set_waypoints_for_asv(struct Simulation* first_node,
                                      struct Asv* asv, 
                                      union Coordinates_3D* waypoints,
                                      int count_waypoints)
{
  if(first_node && asv)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      if(node->count_waypoints < count_waypoints)
      {
        // More waypoints to store. Expand heap. 
        free(node->waypoints);
        node->waypoints = (union Coordinates_3D*)malloc(sizeof(union Coordinates_3D)*count_waypoints);
      }
      if(waypoints)
      {
        for(int i = 0; i < count_waypoints; ++i)
        {
          node->waypoints[i] = waypoints[i];
        }
        node->count_waypoints = count_waypoints;
        node->current_waypoint_index = 0;
      }
      else
      {
        set_error_msg(first_node->error_msg, error_null_pointer);
        return;
      } 
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return;
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  } 
}

void simulation_set_controller(struct Simulation* first_node, double* gain_position, double* gain_heading)
{
  if(first_node && gain_position && gain_heading)
  {
    clear_error_msg(first_node->error_msg);
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      node->controller = controller_new(node->asv);
      // PID controller set gain terms
      double p_position = gain_position[0];
      double i_position = gain_position[1];
      double d_position = gain_position[2];
      controller_set_gains_position(node->controller, p_position, i_position, d_position);
      double p_heading = gain_heading[0];
      double i_heading = gain_heading[1];
      double d_heading = gain_heading[2];
      controller_set_gains_heading(node->controller, p_heading, i_heading, d_heading);
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  } 
}

void simulation_tune_controller(struct Simulation* first_node)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    first_node->controller = controller_new(first_node->asv);
    controller_tune(first_node->controller);
    union Coordinates_3D k_position = controller_get_gains_position(first_node->controller);
    union Coordinates_3D k_heading  = controller_get_gains_heading(first_node->controller);
    // Assuming all asvs will use the controller with same gain terms. 
    simulation_set_controller(first_node->next, k_position.array, k_heading.array);
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  } 
}



void simulation_run_upto_waypoint(struct Simulation* first_node, char* out_dir)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    first_node->simulation_run(first_node, out_dir);

    // Check for any errors during simulation and print it. 
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      if(node->error_msg)
      {
        fprintf(stderr, "ERROR: ASV id = %s. %s\n", node->id, node->error_msg);
      }
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  } 
}

void simulation_run_upto_time(struct Simulation* first_node, double max_time, char* out_dir)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      node->max_time = max_time;
    }
    simulation_run_upto_waypoint(first_node, out_dir);
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  } 
}

void simulation_run_a_timestep(struct Simulation* first_node)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      simulation_run_per_node_per_time_step(first_node);
    }
    // Check for any errors during simulation and print it. 
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      if(node->error_msg)
      {
        fprintf(stderr, "ERROR: ASV id = %s. %s\n", node->id, node->error_msg);
      }
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return;
  } 
}


struct Buffer* simulation_get_buffer(struct Simulation* first_node, struct Asv* asv)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      return node->buffer;
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return NULL;
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return NULL;
  } 
}


long simulation_get_buffer_length(struct Simulation* first_node, struct Asv* asv)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      return node->buffer_index;
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return 0;
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return 0;
  } 
}

long simulation_get_buffer_size()
{
  return OUTPUT_BUFFER_SIZE;
}

union Coordinates_3D simulation_get_waypoint(struct Simulation* first_node, struct Asv* asv)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      return node->waypoints[node->current_waypoint_index];
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return (union Coordinates_3D){0.0,0.0,0.0};
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return (union Coordinates_3D){0.0,0.0,0.0};
  } 
}

int simulation_get_count_waypoints(struct Simulation* first_node, struct Asv* asv)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      return node->count_waypoints;
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return 0;
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return 0;
  } 
}

union Coordinates_3D* simulation_get_waypoints(struct Simulation* first_node, struct Asv* asv)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      return node->waypoints;
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return NULL;
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return NULL;
  } 
}

int simulation_get_count_asvs(struct Simulation* first_node)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    int i = 0;
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      ++i;
    }
    return i;
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return 0;
  } 
}

int simulation_get_asvs(struct Simulation* first_node, struct Asv** asvs)
{
  if(first_node)
  {
    clear_error_msg(first_node->error_msg);
    int i = 0;
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      asvs[i++] = node->asv;
    }
    return i;
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return 0;
  } 
}


union Coordinates_3D simulation_get_asv_position_at(struct Simulation* first_node, struct Asv* asv, int index)
{
  if(first_node && asv)
  {
    clear_error_msg(first_node->error_msg);
    // Find the asv from the linked list
    struct Simulation* node = NULL;
    for(struct Simulation* current_node = first_node; current_node != NULL; current_node = current_node->next)
    {
      if(current_node->asv == asv)
      {
        // Found it. 
        node = current_node;
        break;
      }
    }
    if(node)
    {
      if(index >= 0 && index < node->buffer_index)
      {
        union Coordinates_3D position;
        position.keys.x = node->buffer[index].cog_x;
        position.keys.y = node->buffer[index].cog_y;
        position.keys.z = node->buffer[index].cog_z;
        return position;
      }
    }
    else
    {
      set_error_msg(first_node->error_msg, "Could not find the ASV.");
      return (union Coordinates_3D){0.0,0.0,0.0};
    }
  }
  else
  {
    set_error_msg(first_node->error_msg, error_null_pointer);
    return (union Coordinates_3D){0.0,0.0,0.0};
  } 
}