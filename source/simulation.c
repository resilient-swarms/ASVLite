#include <stdlib.h>
#include "toml.h"
#include "string.h"
#include "simulation.h"
#include <sys/stat.h> // for creating directory

#define OUTPUT_BUFFER_SIZE 200000 /*!< Output buffer size. */

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
  struct PID_controller* pid_controller;
  union Coordinates_3D* waypoints;
  int count_waypoints;
  struct Buffer* buffer;
  // Data related to current time step
  double time_step_size; // milliseconds
  long current_time_index;
  int current_waypoint_index;
  // Link pointers
  struct Simulation* previous; // previous in the linked list.
  struct Simulation* next; // next in the linked list.
};

// Visualisation data
static double sea_surface_edge_length;
static union Coordinates_3D sea_surface_position;
static int count_mesh_cells_along_edge;

double get_sea_surface_edge_length()
{
  return sea_surface_edge_length;
}

int get_count_mesh_cells_along_edge()
{
  return count_mesh_cells_along_edge;
}

union Coordinates_3D get_sea_surface_position()
{
  return sea_surface_position;
}

struct Simulation* simulation_new_node()
{
  // Initialise memory
  struct Simulation* node = (struct Simulation*)malloc(sizeof(struct Simulation));
  node->wave = NULL;
  node->asv = NULL;
  node->pid_controller = NULL;
  node->waypoints = NULL;
  node->buffer = (struct Buffer*)malloc(OUTPUT_BUFFER_SIZE * sizeof(struct Buffer));
  node->previous = NULL;
  node->next = NULL;
  node->count_waypoints = 0;
  node->time_step_size = 40.0;
  node->current_time_index = 0;
  node->current_waypoint_index = 0;
  return node;
}

void simulation_delete(struct Simulation* first_node)
{
  for(struct Simulation* current_node = first_node; current_node != NULL;)
  {
    wave_delete(current_node->wave);
    asv_delete(current_node->asv);
    pid_controller_delete(current_node->pid_controller);
    free(current_node->buffer);
    free(current_node->waypoints);
    free(current_node);
    struct Simulation* next_node = current_node->next;
    current_node->next = NULL;
    current_node = next_node;
  }
}

void simulation_set_input(struct Simulation* first_node,
                               char *file,  
                               double wave_ht, 
                               double wave_heading, 
                               long rand_seed)
{
  // buffer to hold raw data from input file.
  const char *raw;

  // Open file to parse
  FILE *fp = fopen(file, "r");
  if (fp == 0)
  {
    fprintf(stderr, "Error: cannot open file \"%s\".\n", file);
    exit(1);
  }

  // Read the input file into the root table.
  char errbuf[200];
  toml_table_t *input = toml_parse_file(fp, errbuf, sizeof(errbuf));
  fclose(fp);
  if (input == 0)
  {
    fprintf(stderr, "ERROR: %s\n", errbuf);
    exit(1);
  }

  // Read tables [asv]
  toml_array_t *tables = toml_array_in(input, "asv");
  if (tables == 0)
  {
    fprintf(stderr, "ERROR: missing [[asv]].\n");
    toml_free(input);
    exit(1);
  }
  
  // get number of asvs
  int count_asvs = toml_array_nelem(tables);
  // iterate each asv table
  struct Simulation* current = first_node;
  for (int n = 0; n < count_asvs; ++n)
  {
    // Create and initialise the sea surface
    int count_wave_spectral_directions  = 5;
    int count_wave_spectral_frequencies = 15;
    current->wave = wave_new(wave_ht, wave_heading * PI/180.0, rand_seed, count_wave_spectral_directions, count_wave_spectral_frequencies);
    // ASV specification
    struct Asv_specification asv_spec;
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
    
    // Get toml table to set the input data
    toml_table_t *table = toml_table_at(tables, n);
    if (table == 0)
    {
      fprintf(stderr, "ERROR: missing table [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    // Extract values in table [asv]
    // id
    raw = toml_raw_in(table, "id");
    char* id;
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'id' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtos(raw, &id))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].L_wl'\n",n);
      toml_free(input);
      exit(1);
    }
    else
    {
      strcpy(current->id, id);
    }    

    // L_wl
    raw = toml_raw_in(table, "L_wl");
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'L_wl' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.L_wl)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].L_wl'\n",n);
      toml_free(input);
      exit(1);
    }
    // B_wl
    raw = toml_raw_in(table, "B_wl");
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'B_wl' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.B_wl)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].B_wl'\n",n);
      toml_free(input);
      exit(1);
    }
    // D
    raw = toml_raw_in(table, "D");
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'D' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.D)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].D'\n",n);
      toml_free(input);
      exit(1);
    }
    // T
    raw = toml_raw_in(table, "T");
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'T' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.T)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].T'\n",n);
      toml_free(input);
      exit(1);
    }
    // displacement
    raw = toml_raw_in(table, "displacement");
    if (raw == 0) 
    {
      fprintf(stderr, "ERROR: missing 'displacement' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.disp)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].displacement'\n",n);
      toml_free(input);
      exit(1);
    }
    // max_speed
    raw = toml_raw_in(table, "max_speed");
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'max_speed' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.max_speed)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].max_speed'\n",n);
      toml_free(input);
      exit(1);
    }

    // cog
    toml_array_t *array = toml_array_in(table, "cog");
    if (array == 0)
    {
      fprintf(stderr, "ERROR: missing 'cog' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    // cog.x
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'cog[0]' for [asv][%d].cog.\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.cog.keys.x)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].cog[0]'\n",n);
      toml_free(input);
      exit(1);
    }
    // cog.y
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'cog[1]' for [asv][%d].cog.\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.cog.keys.y)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].cog[1]'\n", n);
      toml_free(input);
      exit(1);
    }
    // cog.z
    raw = toml_raw_at(array, 2);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'cog[2]' for [asv][%d].cog.\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.cog.keys.z)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].cog[2]'\n", n);
      toml_free(input);
      exit(1);
    }

    // radius_of_gyration
    array = toml_array_in(table, "radius_of_gyration");
    if (array == 0)
    {
      fprintf(stderr, "ERROR: missing 'radius_of_gyration' in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    // radius_of_gyration.x
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'radius_of_gyration[0]' in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.r_roll)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].radius_of_gyration[0]'\n",n);
      toml_free(input);
      exit(1);
    }
    // radius_of_gyration.pitch
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'radius_of_gyration[1]' in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.r_pitch)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].radius_of_gyration[1]'\n",n);
      toml_free(input);
      exit(1);
    }
    // radius_of_gyration.yaw
    raw = toml_raw_at(array, 2);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'radius_of_gyration[2]' in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(asv_spec.r_yaw)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].radius_of_gyration[2]'\n",n);
      toml_free(input);
      exit(1);
    }

    // asv_position
    union Coordinates_3D origin_position;
    array = toml_array_in(table, "asv_position");
    if (array == 0)
    {
      fprintf(stderr, "ERROR: missing 'asv_position' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    // asv_position.x
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'asv_position[0]' for [asv][%d].asv_position.\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(origin_position.keys.x)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].asv_position[0]'.\n", n);
      toml_free(input);
      exit(1);
    }
    // asv_position.y
    raw = toml_raw_at(array, 1);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'asv_position[1]' for [asv][%d].asv_position.\n",n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(origin_position.keys.y)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].asv_position[1]'.\n", n);
      toml_free(input);
      exit(1);
    }

    // asv_attitude
    union Coordinates_3D attitude;
    array = toml_array_in(table, "asv_attitude");
    if (array == 0)
    {
      fprintf(stderr, "ERROR: missing 'asv_attitude' in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    // Extract values in table [vehicle_attitude]
    // heel
    double heel = 0.0;
    raw = toml_raw_at(array, 0);
    if (raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'asv_attitude[0]' for [asv][%d].asv_attitude.\n", n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(heel)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].asv_attitude[0]'\n",n);
      toml_free(input);
      exit(1);
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
      fprintf(stderr, "ERROR: missing 'asv_attitude[1]' for [asv][%d].asv_attitude.\n", n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(trim)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].asv_attitude[1]'\n",n);
      toml_free(input);
      exit(1);
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
      fprintf(stderr, "ERROR: missing 'asv_attitude[2]' for [asv][%d].asv_attitude.\n", n);
      toml_free(input);
      exit(1);
    }
    if (toml_rtod(raw, &(heading)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].asv_attitude[2]'\n",n);
      toml_free(input);
      exit(1);
    }
    else
    {
      // convert to radians and set value
      attitude.keys.z = heading * PI / 180.0;
    }

    // Initialise the asv
    current->asv = asv_new(asv_spec, current->wave, origin_position, attitude);

    // thrusters
    toml_table_t *arrays = toml_array_in(table, "thrusters");
    if (array == 0)
    {
      fprintf(stderr, "ERROR: missing 'thrusters' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    // get number of thrusters
    int count_propellers = toml_array_nelem(arrays);
    struct Propeller** propellers = (struct Propeller**)malloc(sizeof(struct Propeller*) * count_propellers);
    // Set propeller data
    for (int i = 0; i < count_propellers; ++i)
    {
      union Coordinates_3D propeller_position;
      array = toml_array_at(arrays, i);
      // x
      raw = toml_raw_at(array, 0);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'thrusters[%d][0]' in [asv][%d].\n", i, n);
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(propeller_position.keys.x)))
      {
        fprintf(stderr, "ERROR: bad value in '[asv][%d]thrustes[%d][0]'\n", n, i);
        toml_free(input);
        exit(1);
      }
      // y
      raw = toml_raw_at(array, 1);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'thrusters[%d][1]' in [asv][%d].\n", i, n);
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(propeller_position.keys.y)))
      {
        fprintf(stderr, "ERROR: bad value in '[asv][%d]thrustes[%d][1]'\n", n, i);
        toml_free(input);
        exit(1);
      }
      // z
      raw = toml_raw_at(array, 2);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'thrusters[%d][2]' in [asv][%d].\n", i, n);
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(propeller_position.keys.z)))
      {
        fprintf(stderr, "ERROR: bad value in '[asv][%d]thrustes[%d][2]'\n", n, i);
        toml_free(input);
        exit(1);
      }
      propellers[i] = propeller_new(propeller_position);
    }
    // Set the thrusters on the ASV
    asv_set_propellers(current->asv, propellers, count_propellers);
    free(propellers);

    // PID controller
    current->pid_controller = pid_controller_new(current->asv);
    // PID controller set gain terms
    double p_position = 1.0   * current->time_step_size/1000.0;
    double i_position = 0.1   * current->time_step_size/1000.0;
    double d_position = -10.0 * current->time_step_size/1000.0;
    pid_controller_set_gains_position(current->pid_controller, p_position, i_position, d_position);
    double p_heading = 1.0   * current->time_step_size/1000.0;
    double i_heading = 0.1   * current->time_step_size/1000.0;
    double d_heading = -10.0 * current->time_step_size/1000.0;
    pid_controller_set_gains_heading(current->pid_controller, p_heading, i_heading, d_heading);

    // waypoints
    arrays = toml_array_in(table, "waypoints");
    if (arrays == 0)
    {
      fprintf(stderr, "ERROR: missing waypoints in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
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
        fprintf(stderr, "ERROR: missing 'waypoints[%d][0]' in [asv][%d].\n", i, n);
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(current->waypoints[i].keys.x)))
      {
        fprintf(stderr, "ERROR: bad value in [asv][%d].waypoints[%d][0]'\n", n, i);
        toml_free(input);
        exit(1);
      }
      // y
      raw = toml_raw_at(array, 1);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'waypoints[%d][1]' in [asv][%d].\n", i, n);
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(current->waypoints[i].keys.y)))
      {
        fprintf(stderr, "ERROR: bad value in [asv][%d].waypoints[%d][1]'\n", n, i);
        toml_free(input);
        exit(1);
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
        fprintf(stderr, "ERROR: bad value in 'time_step_size'\n");
        toml_free(input);
        exit(1);
      }
    }
  }

  // Locate table [visualisation]
  table = toml_table_in(input, "visualisation");
  if(table != 0)
  {
    // Extract value in table [visualisation]
    // sea_surface_edge_length
    raw = toml_raw_in(table, "sea_surface_edge_length");
    if(raw != 0)
    {
      if (toml_rtod(raw, &sea_surface_edge_length))
      {
        fprintf(stderr, "ERROR: bad value in 'sea_surface_edge_length'\n");
        toml_free(input);
        exit(1);
      }
    }

    // sea_surface_position
    toml_table_t *array = toml_array_in(table, "sea_surface_position");
    if(array != 0)
    {
      // sea_surface_position.x
      raw = toml_raw_at(array, 0);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'sea_surface_position[0]'.\n");
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(sea_surface_position.keys.x)))
      {
        fprintf(stderr, "ERROR: bad value in 'sea_surface_position[0]'.\n");
        toml_free(input);
        exit(1);
      }
      // sea_surface_position.y
      raw = toml_raw_at(array, 1);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'sea_surface_position[1]'.\n");
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(sea_surface_position.keys.y)))
      {
        fprintf(stderr, "ERROR: bad value in 'sea_surface_position[1]'.\n");
        toml_free(input);
        exit(1);
      }
    }     

    // count_mesh_cells_along_edge
    raw = toml_raw_in(table, "count_mesh_cells_along_edge");
    if(raw != 0)
    {
      if (toml_rtoi(raw, &count_mesh_cells_along_edge))
      {
        fprintf(stderr, "ERROR: bad value in 'count_mesh_cells_along_edge'\n");
        toml_free(input);
        exit(1);
      }
    }
  }

  // done reading inputs
  toml_free(input);
}

void simulation_write_output(struct Simulation* first_node,
                                  char* out, 
                                  double simulation_time)
{
  bool has_multiple_asvs = false;
  // Check if single asv or more
  if(first_node->next != NULL)
  {
    has_multiple_asvs = true;
  }

  // Create director if there are more than one asvs.
  if(has_multiple_asvs)
  {
    // Check if the directory exist
    struct stat st = {0};
    if (stat(out, &st) == -1) 
    {
      mkdir(out, 0700);
    }
  }

  for(struct Simulation* node = first_node; node != NULL; node = node->next)
  {
    double time = node->current_time_index * node->time_step_size/1000.0; //sec
    double performance = time / simulation_time;
    fprintf(stdout, "%s, %f sec, %f sec, %f x \n", node->id, time, simulation_time, performance);

    FILE *fp;
    // Create the output file in the out dir if there are more than one asvs,
    // else create an output file with name in out.
    char file[128];
    strcpy(file, out);
    if(has_multiple_asvs)
    {
      // More than one asv.
      strcpy(file, out);
      strcat(file, "/");
      strcat(file, node->id);
    }
    // Open the file
    if (!(fp = fopen(file, "a")))
    {
      fprintf(stderr, "Error. Cannot open output file %s.\n", file);
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
    for (int i = 0; i < node->current_time_index; ++i)
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
}

// Computes dynamics for current node for the current time step.
void compute_dynamics(void* current_node)
{
  struct Simulation* node = (struct Simulation*)current_node;
  // Current time
  double current_time = (node->current_time_index+1) * node->time_step_size/1000.0; //sec

  // Set differential thrust on each propeller.
  // ------------------------------------------
  // PID controller estimate thrust to be applied on each propeller.
  pid_controller_set_thrust(node->pid_controller, node->waypoints[node->current_waypoint_index]);

  // Compute the dynamics of asv for the current time step
  asv_compute_dynamics(node->asv, node->time_step_size);
  struct Asv_specification spec = asv_get_spec(node->asv);
  union Coordinates_3D cog_position = asv_get_position_cog(node->asv);
  union Coordinates_3D attitude = asv_get_attitude(node->asv);

  // save simulated data to buffer. 
  node->buffer[node->current_time_index].time               = current_time;
  node->buffer[node->current_time_index].sig_wave_ht        = wave_get_significant_height(node->wave);
  node->buffer[node->current_time_index].wave_heading       = wave_get_predominant_heading(node->wave) * 180.0/PI;
  node->buffer[node->current_time_index].wave_elevation     = wave_get_elevation(node->wave, cog_position, current_time);
  node->buffer[node->current_time_index].F_surge            = asv_get_F(node->asv).keys.pitch;
  node->buffer[node->current_time_index].surge_acceleration = asv_get_A(node->asv).keys.surge;
  node->buffer[node->current_time_index].surge_velocity     = asv_get_V(node->asv).keys.surge;
  node->buffer[node->current_time_index].cog_x              = cog_position.keys.x;
  node->buffer[node->current_time_index].cog_y              = cog_position.keys.y;
  node->buffer[node->current_time_index].cog_z              = cog_position.keys.z - (spec.cog.keys.z - spec.T);
  node->buffer[node->current_time_index].heel               = attitude.keys.x * 180.0/PI;
  node->buffer[node->current_time_index].trim               = attitude.keys.y * 180.0/PI;
  node->buffer[node->current_time_index].heading            = attitude.keys.z * 180.0/PI;

  // Check if reached the waypoint
  double proximity_margin = 2.0; // target proximity to waypoint
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
}

void compute_dynamics_per_thread_no_time_sync(void* current_node)
{
  struct Simulation* node = (struct Simulation*)current_node;
  for(node->current_time_index = 0; ; ++(node->current_time_index))
  {
    if(node->current_waypoint_index < node->count_waypoints)
    {
      // Not yet reached the final waypoint, but check if buffer limit reached before further computation.
      // Check if buffer exceeded
      if(node->current_time_index >= OUTPUT_BUFFER_SIZE)
      {
        // buffer exceeded
        fprintf(stderr, "ERROR: output buffer exceeded for asv with id '%s'.\n", node->id);
        break;        
      }
      // If buffer not exceeded.
      compute_dynamics((void*)node);
    }
    else
    {
      // Reached the final waypoint
      break;
    }
  }
}

static void simulation_for_time_step(struct Simulation* first_node, long t, bool* buffer_exceeded, bool* has_all_reached_final_waypoint)
{
  // Create threads
    // int limit_threads = get_nprocs();
    // spawn threads
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      // Set time step for the node
      node->current_time_index = t;

      // Check if asv reached the final waypoint.
      if(node->current_waypoint_index < node->count_waypoints)
      {
        // Not yet reached the final waypoint, but check if buffer limit reached before further computation.
        // Check if buffer exceeded
        if(node->current_time_index >= OUTPUT_BUFFER_SIZE)
        {
          // buffer exceeded
          *buffer_exceeded = true;
          fprintf(stderr, "ERROR: output buffer exceeded for asv with id '%s'.\n", node->id);
          break;        
        }
        *has_all_reached_final_waypoint = false;
        #ifdef DISABLE_MULTI_THREADING
        compute_dynamics((void*)node);
        #else
        pthread_create(&(node->thread), NULL, &compute_dynamics, (void*)node);
        #endif
      }
    }
    #ifndef DISABLE_MULTI_THREADING
    // join threads
    for(struct Simulation* node = first_node; node != NULL; node = node->next)
    {
      if(node->current_waypoint_index < node->count_waypoints)
      {
        pthread_join(node->thread, NULL);
      }
    }
    #endif
}

static void simulation_run_with_time_sync(struct Simulation* first_node)
{
  bool buffer_exceeded = false;
  for(long t = 0; ; ++t)
  {
    // Variable to check if all reached the destination.
    bool has_all_reached_final_waypoint = true;

    simulation_for_time_step(first_node, t, &buffer_exceeded, &has_all_reached_final_waypoint);

    // stop if all reached the destination or if buffer exceeded.
    if(has_all_reached_final_waypoint || buffer_exceeded)
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
static void simulation_run_without_time_sync(struct Simulation* first_node)
{
  for(struct Simulation* node = first_node; node != NULL; node = node->next)
  {
    pthread_create(&(node->thread), NULL, &compute_dynamics_per_thread_no_time_sync, (void*)node);
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

void simulation_run(struct Simulation* first_node)
{
  #ifdef ENABLE_TIME_SYNC
  simulation_run_with_time_sync(first_node);
  #else
  simulation_run_without_time_sync(first_node);
  #endif
}