#include <stdlib.h>
#include "toml.h"
#include "io.h"

struct Buffer buffer[OUTPUT_BUFFER_SIZE];

void set_input(char* file, struct Asv* asv, struct Waypoints* waypoints)
{
  // buffer to hold raw data from input file. 
  const char* raw;

  // Open file to parse
  FILE* fp = fopen(file, "r");
  if(fp == 0)
  {
    fprintf(stderr, "Error: cannot open file \"%s\".\n", file);
    exit(1);
  } 

  // Read the input file into the root table.
  char errbuf[200];
  toml_table_t* input = toml_parse_file(fp, errbuf, sizeof(errbuf));
  fclose(fp);
  if(input == 0)
  {
    fprintf(stderr, "ERROR: %s\n", errbuf);
	  exit(1);
  }

  // Read tables [asv]
  toml_table_t* tables = toml_array_in(input, "asv");
  if(tables == 0)
  {
    fprintf(stderr, "ERROR: missing [[asv]].\n");
	  toml_free(input);
	  exit(1);
  }
  // get number of waypoints
  int count_asvs = toml_array_nelem(tables);
  // iterate each asv table
  for(int n = 0; n < count_asvs; ++n)
  {
  toml_table_t* table = toml_table_at(tables, n);
  if(table == 0)
  {
    fprintf(stderr, "ERROR: missing [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [asv]
  // L_wl
  raw = toml_raw_in(table, "L_wl");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'L_wl' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.L_wl)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.L_wl'\n");
	  toml_free(input);
	  exit(1);
  }
  // B_wl
  raw = toml_raw_in(table, "B_wl");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'B_wl' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.B_wl)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.B_wl'\n");
	  toml_free(input);
	  exit(1);
  }
  // D
  raw = toml_raw_in(table, "D");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'D' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.D)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.D'\n");
	  toml_free(input);
	  exit(1);
  }
  // T
  raw = toml_raw_in(table, "T");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'T' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.T)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.T'\n");
	  toml_free(input);
	  exit(1);
  }
  // displacement
  raw = toml_raw_in(table, "displacement");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'displacement' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.disp)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.displacement'\n");
	  toml_free(input);
	  exit(1);
  }
  // max_speed
  raw = toml_raw_in(table, "max_speed");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'max_speed' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.max_speed)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.max_speed'\n");
	  toml_free(input);
	  exit(1);
  }

  // cog
  toml_table_t* array = toml_array_in(table, "cog");
  if(array == 0)
  {
    fprintf(stderr, "ERROR: missing 'cog' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  // cog.x
  raw = toml_raw_at(array, 0);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing x coordinate for 'cog' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.cog.x)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.cog.x'\n");
	  toml_free(input);
	  exit(1);
  }
  // cog.y
  raw = toml_raw_at(array, 1);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing y coordinate for 'cog' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.cog.y)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.cog.y'\n");
	  toml_free(input);
	  exit(1);
  }
  // cog.z
  raw = toml_raw_at(array, 2);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing z coordinate for 'cog' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.cog.z)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv.cog.z'\n");
	  toml_free(input);
	  exit(1);
  }

  // radius_of_gyration
  array = toml_array_in(table, "radius_of_gyration");
  if(array == 0)
  {
    fprintf(stderr, "ERROR: missing 'radius_of_gyration' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  // radius_of_gyration.x
  raw = toml_raw_at(array, 0);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'radius_of_gyration.roll' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.r_roll)))
  {
    fprintf(stderr, "ERROR: bad value in 'radius_of_gyration.roll'\n");
	  toml_free(input);
	  exit(1);
  }
  // radius_of_gyration.pitch
  raw = toml_raw_at(array, 1);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'radius_of_gyration.pitch' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.r_pitch)))
  {
    fprintf(stderr, "ERROR: bad value in 'radius_of_gyration.pitch'\n");
	  toml_free(input);
	  exit(1);
  }
  // radius_of_gyration.yaw
  raw = toml_raw_at(array, 2);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'radius_of_gyration.yaw' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.r_yaw)))
  {
    fprintf(stderr, "ERROR: bad value in 'radius_of_gyration.yaw'\n");
	  toml_free(input);
	  exit(1);
  }

  // thrusters
  toml_table_t* arrays = toml_array_in(table, "thrusters");
  if(array == 0)
  {
    fprintf(stderr, "ERROR: missing 'thrusters' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  // get number of thrusters
  asv->count_propellers = toml_array_nelem(arrays);
  if(asv->count_propellers > COUNT_PROPELLERS_MAX)
  {
    fprintf(stderr,"ERROR: number of thrusters (%d) exceed max limit (%i)'\n", 
            asv->count_propellers, COUNT_PROPELLERS_MAX);
	  toml_free(input);
	  exit(1);
  }
  // Set propeller data
  for(int i = 0; i < asv->count_propellers; ++i)
  {
    array = toml_array_at(arrays, i);
    // x
    raw = toml_raw_at(array, 0);
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'x' in thrusters[%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(asv->propellers[i].position.x)))
    {
      fprintf(stderr, "ERROR: bad value in 'thrustes[%d].x'\n", i);
	    toml_free(input);
	    exit(1);
    }
    // y
    raw = toml_raw_at(array, 1);
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'y' in thrusters[%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(asv->propellers[i].position.y)))
    {
      fprintf(stderr, "ERROR: bad value in 'thrusters[%d].y'\n", i);
	    toml_free(input);
	    exit(1);
    }
    // z
    raw = toml_raw_at(array, 2);
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'z' in thrusters[%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(asv->propellers[i].position.z)))
    {
      fprintf(stderr, "ERROR: bad value in 'thrusters[%d].z'\n", i);
	    toml_free(input);
	    exit(1);
    }
  }

  // asv_position
  array = toml_array_in(table, "asv_position");
  if(array == 0)
  {
    fprintf(stderr, "ERROR: missing 'asv_position' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  // asv_position.x
  raw = toml_raw_at(array, 0);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing x coordinate for 'asv_position' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->origin_position.x)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv_position.x'\n");
	  toml_free(input);
	  exit(1);
  }
  // asv_position.y
  raw = toml_raw_at(array, 1);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing y coordinate for 'asv_position' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->origin_position.y)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv_position.y'\n");
	  toml_free(input);
	  exit(1);
  }

    // asv_attitude
  array = toml_array_in(table, "asv_attitude");
  if(array == 0)
  {
    fprintf(stderr, "ERROR: missing 'asv_attitude' in [asv].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [vehicle_attitude]
  // heel
  double heel = 0.0;
  raw = toml_raw_at(array, 0);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing heel angle in [asv_attitude].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(heel)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv_attitude.heel'\n");
	  toml_free(input);
	  exit(1);
  }
  else
  {
    // convert to radians and set value
    asv->attitude.x = heel * PI/180.0;
  }
  // trim
  double trim = 0.0;
  raw = toml_raw_at(array, 1);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing trim angle in [asv_attitude].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(trim)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv_attitude.trim'\n");
	  toml_free(input);
	  exit(1);
  }
  else
  {
    // convert to radians and set value
    asv->attitude.y = trim * PI/180.0;
  }
  // heading
  double heading = 0.0;
  raw = toml_raw_at(array, 2);
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing heading angle in [asv_attitude].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(heading)))
  {
    fprintf(stderr, "ERROR: bad value in 'asv_attitude.heading'\n");
	  toml_free(input);
	  exit(1);
  }
  else
  {
    // convert to radians and set value
    asv->attitude.z = heading * PI/180.0;
  }

  // waypoints
  arrays = toml_array_in(table, "waypoints");
  if(arrays == 0)
  {
    fprintf(stderr, "ERROR: missing waypoints.\n");
	  toml_free(input);
	  exit(1);
  }
  // get number of waypoints
  waypoints->count = toml_array_nelem(arrays);
  if(waypoints->count > COUNT_WAYPOINTS_MAX)
  {
    fprintf(stderr,"ERROR: number of waypoints (%d) exceed max limit (%i)'\n", 
            waypoints->count, COUNT_WAYPOINTS_MAX);
	  toml_free(input);
	  exit(1);
  }
  // Set waypoint data
  for(int i = 0; i < waypoints->count; ++i)
  {
    array = toml_array_at(arrays, i);
    // x
    raw = toml_raw_at(array, 0);
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'x' in waypoints[%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(waypoints->points[i].x)))
    {
      fprintf(stderr, "ERROR: bad value in waypoints[%d].x'\n", i);
	    toml_free(input);
	    exit(1);
    }
    // y
    raw = toml_raw_at(array, 1);
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'y' in waypoints[%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(waypoints->points[i].y)))
    {
      fprintf(stderr, "ERROR: bad value in waypoints[%d].y'\n", i);
	    toml_free(input);
	    exit(1);
    }
  }
  }

  // Locate table [clock]
  double time_step_size = 0.0;
  toml_table_t* table = toml_table_in(input, "clock");
  if(table != 0)
  {
    // Extract values in table [clock]
    // time_step_size
    raw = toml_raw_in(table, "time_step_size");
    if(raw != 0)
    {
      if(toml_rtod(raw, &time_step_size))
      {
        fprintf(stderr, "ERROR: bad value in 'time_step_size'\n");
	      toml_free(input);
	      exit(1);
      }
      // If time step size not set in input file, then default to 10 millisec
      asv->dynamics.time_step_size = (time_step_size == 0.0)? 
                                      10.0/1000.0 : time_step_size/1000.0;
    }  
  }
  // done reading inputs
  toml_free(input);
}

void write_output(char* file, 
                  int buffer_length,
                  double wave_ht, 
                  double wave_heading, 
                  long rand_seed,
                  double task_duration,
                  double simulation_time)
{
  // output message
  //fprintf(stdout, "significant wave height = %f m.\n", wave_ht);
  //fprintf(stdout, "wave heading = %f deg.\n", wave_heading);
  //fprintf(stdout, "random number seed = %ld.\n", rand_seed);
  //fprintf(stdout, "task duration = %f sec.\n", task_duration);
  //fprintf(stdout, "time taken for simulation = %f sec. \n\n", simulation_time);
  fprintf(stdout, "%f sec, %f sec, %f x \n", task_duration, simulation_time, task_duration/simulation_time);
  
  FILE* fp;
  if(!(fp = fopen(file, "a")))
  {
    fprintf(stderr, "Error. Cannot open output file %s.\n", file);
    exit(1);
  }
  // Check if the file is empty and add header only for empty file.
  fseek (fp, 0, SEEK_END);
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
           "F_sway(N)"
           );
  }
  // write buffer to file and close the file.
  for(int i = 0; i < buffer_length; ++i)
  {
    fprintf(fp, "\n%f %f %ld %f %f %f %f %f %f %f %f %f %f %f %f", 
            buffer[i].sig_wave_ht,
            wave_heading,
            rand_seed,
            buffer[i].time,
            buffer[i].wave_elevation,
            buffer[i].cog_x, 
            buffer[i].cog_y, 
            buffer[i].cog_z, 
            buffer[i].heel, 
            buffer[i].trim, 
            buffer[i].heading,
            buffer[i].surge_velocity,
            buffer[i].surge_acceleration,
            buffer[i].F_surge,
            buffer[i].F_sway);
  }
  fclose(fp);
}
