#include <stdlib.h>
#include "toml.h"
#include "io.h"

void simulation_data_node_init(struct Simulation_data* simulation_data)
{
  simulation_data->previous = NULL;
  simulation_data->next = NULL;
  simulation_data->current_time_index = 0;
  simulation_data->current_waypoint_index = 0;
}

void simulation_data_set_input(struct Simulation_data* simulation_data,
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
  toml_table_t *tables = toml_array_in(input, "asv");
  if (tables == 0)
  {
    fprintf(stderr, "ERROR: missing [[asv]].\n");
    toml_free(input);
    exit(1);
  }
  
  // get number of asvs
  int count_asvs = toml_array_nelem(tables);
  // iterate each asv table
  struct Simulation_data* previous = NULL;
  struct Simulation_data* current = simulation_data;
  for (int n = 0; n < count_asvs; ++n)
  {
    // Create a new entry to the linked list of simulation data.
    if(n != 0)
    {
      previous = current;
      // Create a new entry to the linked list.
      current = (struct Simulation_data*)malloc(sizeof(struct Simulation_data));
      // Link it to the previous entry in the linked list.
      previous->next = current; 
    }
    // Always initialise the instance of Simulation_data before use.
    simulation_data_node_init(current);

    // Init wave for the asv
    if(wave_ht != 0.0)
    {
      current->asv.wave_type = irregular_wave;
      wave_init(&(current->asv.wave), wave_ht, wave_heading * PI/180.0, rand_seed); 
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
    if (toml_rtod(raw, &(current->asv.spec.L_wl)))
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
    if (toml_rtod(raw, &(current->asv.spec.B_wl)))
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
    if (toml_rtod(raw, &(current->asv.spec.D)))
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
    if (toml_rtod(raw, &(current->asv.spec.T)))
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
    if (toml_rtod(raw, &(current->asv.spec.disp)))
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
    if (toml_rtod(raw, &(current->asv.spec.max_speed)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].max_speed'\n",n);
      toml_free(input);
      exit(1);
    }

    // cog
    toml_table_t *array = toml_array_in(table, "cog");
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
    if (toml_rtod(raw, &(current->asv.spec.cog.x)))
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
    if (toml_rtod(raw, &(current->asv.spec.cog.y)))
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
    if (toml_rtod(raw, &(current->asv.spec.cog.z)))
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
    if (toml_rtod(raw, &(current->asv.spec.r_roll)))
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
    if (toml_rtod(raw, &(current->asv.spec.r_pitch)))
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
    if (toml_rtod(raw, &(current->asv.spec.r_yaw)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].radius_of_gyration[2]'\n",n);
      toml_free(input);
      exit(1);
    }

    // thrusters
    toml_table_t *arrays = toml_array_in(table, "thrusters");
    if (array == 0)
    {
      fprintf(stderr, "ERROR: missing 'thrusters' in [asv][%d].\n",n);
      toml_free(input);
      exit(1);
    }
    // get number of thrusters
    current->asv.count_propellers = toml_array_nelem(arrays);
    if (current->asv.count_propellers > COUNT_PROPELLERS_MAX)
    {
      fprintf(stderr, "ERROR: number of thrusters (%d) exceed max limit (%i) in [asv][%d].'\n",
              current->asv.count_propellers, COUNT_PROPELLERS_MAX, n);
      toml_free(input);
      exit(1);
    }
    // Set propeller data
    for (int i = 0; i < current->asv.count_propellers; ++i)
    {
      array = toml_array_at(arrays, i);
      // x
      raw = toml_raw_at(array, 0);
      if (raw == 0)
      {
        fprintf(stderr, "ERROR: missing 'thrusters[%d][0]' in [asv][%d].\n", i, n);
        toml_free(input);
        exit(1);
      }
      if (toml_rtod(raw, &(current->asv.propellers[i].position.x)))
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
      if (toml_rtod(raw, &(current->asv.propellers[i].position.y)))
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
      if (toml_rtod(raw, &(current->asv.propellers[i].position.z)))
      {
        fprintf(stderr, "ERROR: bad value in '[asv][%d]thrustes[%d][2]'\n", n, i);
        toml_free(input);
        exit(1);
      }
    }

    // asv_position
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
    if (toml_rtod(raw, &(current->asv.origin_position.x)))
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
    if (toml_rtod(raw, &(current->asv.origin_position.y)))
    {
      fprintf(stderr, "ERROR: bad value in '[asv][%d].asv_position[1]'.\n", n);
      toml_free(input);
      exit(1);
    }

    // asv_attitude
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
      current->asv.attitude.x = heel * PI / 180.0;
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
      current->asv.attitude.y = trim * PI / 180.0;
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
      current->asv.attitude.z = heading * PI / 180.0;
    }

    // waypoints
    arrays = toml_array_in(table, "waypoints");
    if (arrays == 0)
    {
      fprintf(stderr, "ERROR: missing waypoints in [asv][%d].\n", n);
      toml_free(input);
      exit(1);
    }
    // get number of waypoints
    current->waypoints.count = toml_array_nelem(arrays);
    if (current->waypoints.count > COUNT_WAYPOINTS_MAX)
    {
      fprintf(stderr, "ERROR: number of waypoints (%d) in [asv][%d] exceed max limit (%i)'\n",
              current->waypoints.count, n, COUNT_WAYPOINTS_MAX);
      toml_free(input);
      exit(1);
    }
    // Set waypoint data
    for (int i = 0; i < current->waypoints.count; ++i)
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
      if (toml_rtod(raw, &(current->waypoints.points[i].x)))
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
      if (toml_rtod(raw, &(current->waypoints.points[i].y)))
      {
        fprintf(stderr, "ERROR: bad value in [asv][%d].waypoints[%d][1]'\n", n, i);
        toml_free(input);
        exit(1);
      }
    }
  }

  // Locate table [clock]
  double time_step_size = 40.0; // default value for time step size
  toml_table_t *table = toml_table_in(input, "clock");
  if (table != 0)
  {
    // Extract values in table [clock]
    // time_step_size
    raw = toml_raw_in(table, "time_step_size");
    if (raw != 0)
    {
      if (toml_rtod(raw, &time_step_size))
      {
        fprintf(stderr, "ERROR: bad value in 'time_step_size'\n");
        toml_free(input);
        exit(1);
      }
    }
  }
  // Set time step size in all asvs
  for(struct Simulation_data* data = simulation_data; data != NULL; data = data->next)
  {
    data->asv.dynamics.time_step_size = time_step_size/1000.0; // sec
  }

  // Initialise the asv after setting all inputs.
  asv_init(&(current->asv));

  // done reading inputs
  toml_free(input);
}

void simulation_data_write_output(struct Simulation_data* data,
                                  char* file, 
                                  double simulation_time)
{
  double time = data->current_time_index * data->asv.dynamics.time_step_size; //sec
  double performance = time / simulation_time;
  fprintf(stdout, "%s, %f sec, %f sec, %f x \n", data->id, time, simulation_time, performance);

  FILE *fp;
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
            "sig_wave_ht(m) "
            "wave_heading(deg) "
            "rand_seed "
            "time(sec) "
            "wave_elevation(m) "
            "cog_x(m) "
            "cog_y(m) "
            "cog_z(m) "
            "heel(deg) "
            "trim(deg) "
            "heading(deg) "
            "surge_vel(m/s) "
            "surge_acc(m/s2) "
            "F_surge(N) "
            "F_sway(N)");
  }
  // write buffer to file and close the file.
  for (int i = 0; i <= data->current_time_index; ++i)
  {
    fprintf(fp, "\n%f %f %ld %f %f %f %f %f %f %f %f %f %f %f %f",
            data->buffer[i].sig_wave_ht,
            data->asv.wave.heading * 360.0 / (2.0 * PI),
            data->asv.wave.random_number_seed,
            data->buffer[i].time,
            data->buffer[i].wave_elevation,
            data->buffer[i].cog_x,
            data->buffer[i].cog_y,
            data->buffer[i].cog_z,
            data->buffer[i].heel,
            data->buffer[i].trim,
            data->buffer[i].heading,
            data->buffer[i].surge_velocity,
            data->buffer[i].surge_acceleration,
            data->buffer[i].F_surge,
            data->buffer[i].F_sway);
  }
  fclose(fp);
}
