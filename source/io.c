#include <stdlib.h>
#include "toml.h"
#include "io.h"

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

  // Locate table [spec]
  toml_table_t* table = toml_table_in(input, "spec");
  if(table == 0)
  {
    fprintf(stderr, "ERROR: missing [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [spec]
  // L_wl
  raw = toml_raw_in(table, "L_wl");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'L_wl' in [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.L_wl)))
  {
    fprintf(stderr, "ERROR: bad value in 'spec.L_wl'\n");
	  toml_free(input);
	  exit(1);
  }
  // B_wl
  raw = toml_raw_in(table, "B_wl");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'B_wl' in [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.B_wl)))
  {
    fprintf(stderr, "ERROR: bad value in 'spec.B_wl'\n");
	  toml_free(input);
	  exit(1);
  }
  // D
  raw = toml_raw_in(table, "D");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'D' in [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.D)))
  {
    fprintf(stderr, "ERROR: bad value in 'spec.D'\n");
	  toml_free(input);
	  exit(1);
  }
  // T
  raw = toml_raw_in(table, "T");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'T' in [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.T)))
  {
    fprintf(stderr, "ERROR: bad value in 'spec.T'\n");
	  toml_free(input);
	  exit(1);
  }
  // displacement
  raw = toml_raw_in(table, "displacement");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'displacement' in [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.disp)))
  {
    fprintf(stderr, "ERROR: bad value in 'spec.displacement'\n");
	  toml_free(input);
	  exit(1);
  }
  // max_speed
  raw = toml_raw_in(table, "max_speed");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'max_speed' in [spec].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.max_speed)))
  {
    fprintf(stderr, "ERROR: bad value in 'spec.max_speed'\n");
	  toml_free(input);
	  exit(1);
  }

  // Locate table [cog]
  table = toml_table_in(input, "cog");
  if(table == 0)
  {
    fprintf(stderr, "ERROR: missing [cog].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [cog]
  // x
  raw = toml_raw_in(table, "x");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'x' in [cog].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.cog.x)))
  {
    fprintf(stderr, "ERROR: bad value in 'cog.x'\n");
	  toml_free(input);
	  exit(1);
  }
  // y
  raw = toml_raw_in(table, "y");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'y' in [cog].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.cog.y)))
  {
    fprintf(stderr, "ERROR: bad value in 'cog.y'\n");
	  toml_free(input);
	  exit(1);
  }
  // z
  raw = toml_raw_in(table, "z");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'z' in [cog].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.cog.z)))
  {
    fprintf(stderr, "ERROR: bad value in 'cog.z'\n");
	  toml_free(input);
	  exit(1);
  }

  // Locate table [radius_of_gyration]
  table = toml_table_in(input, "radius_of_gyration");
  if(table == 0)
  {
    fprintf(stderr, "ERROR: missing [radius_of_gyration].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [radius_of_gyration]
  // roll
  raw = toml_raw_in(table, "roll");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'roll' in [radius_of_gyration].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.r_roll)))
  {
    fprintf(stderr, "ERROR: bad value in 'radius_of_gyration.roll'\n");
	  toml_free(input);
	  exit(1);
  }
  // pitch
  raw = toml_raw_in(table, "pitch");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'pitch' in [radius_of_gyration].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.r_pitch)))
  {
    fprintf(stderr, "ERROR: bad value in 'radius_of_gyration.pitch'\n");
	  toml_free(input);
	  exit(1);
  }
  // yaw
  raw = toml_raw_in(table, "yaw");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'yaw' in [radius_of_gyration].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->spec.r_yaw)))
  {
    fprintf(stderr, "ERROR: bad value in 'radius_of_gyration.yaw'\n");
	  toml_free(input);
	  exit(1);
  }

  // Locate array of tables [propeller]
  toml_array_t* tables = toml_array_in(input, "propeller");
  if(tables == 0)
  {
    fprintf(stderr, "ERROR: missing [[propeller]].\n");
	  toml_free(input);
	  exit(1);
  }
  // get number of propellers
  asv->count_propellers = toml_array_nelem(tables);
  if(asv->count_propellers > COUNT_PROPELLERS_MAX)
  {
    fprintf(stderr,"ERROR: number of propellers (%d) exceed max limit (%i)'\n", 
            asv->count_propellers, COUNT_PROPELLERS_MAX);
	  toml_free(input);
	  exit(1);
  }
  // Set propeller data
  for(int i = 0; i < asv->count_propellers; ++i)
  {
    table = toml_table_at(tables, i);
    // Extract values in table [[propeller]]
    // x
    raw = toml_raw_in(table, "x");
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'x' in [propeller][%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(asv->propellers[i].position.x)))
    {
      fprintf(stderr, "ERROR: bad value in 'propeller.x'\n");
	    toml_free(input);
	    exit(1);
    }
    // y
    raw = toml_raw_in(table, "y");
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'y' in [propeller][%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(asv->propellers[i].position.y)))
    {
      fprintf(stderr, "ERROR: bad value in 'propeller.y'\n");
	    toml_free(input);
	    exit(1);
    }
    // z
    raw = toml_raw_in(table, "z");
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'z' in [propeller][%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(asv->propellers[i].position.z)))
    {
      fprintf(stderr, "ERROR: bad value in 'propeller.z'\n");
	    toml_free(input);
	    exit(1);
    }
  }

  // Locate table [vehicle_position]
  table = toml_table_in(input, "vehicle_position");
  if(table == 0)
  {
    fprintf(stderr, "ERROR: missing [vehicle_position].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [vehicle_position]
  // x
  raw = toml_raw_in(table, "x");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'x' in [vehicle_position].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->origin_position.x)))
  {
    fprintf(stderr, "ERROR: bad value in 'vehicle_position.x'\n");
	  toml_free(input);
	  exit(1);
  }
  // y
  raw = toml_raw_in(table, "y");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'y' in [vehicle_position].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(asv->origin_position.y)))
  {
    fprintf(stderr, "ERROR: bad value in 'vehicle_position.y'\n");
	  toml_free(input);
	  exit(1);
  }

  // Locate table [vehicle_heading]
  table = toml_table_in(input, "vehicle_heading");
  if(table == 0)
  {
    fprintf(stderr, "ERROR: missing [vehicle_heading].\n");
	  toml_free(input);
	  exit(1);
  }
  // Extract values in table [vehicle_heading]
  // heading
  double heading = 0.0;
  raw = toml_raw_in(table, "heading");
  if(raw == 0)
  {
    fprintf(stderr, "ERROR: missing 'heading' in [vehicle_heading].\n");
	  toml_free(input);
	  exit(1);
  }
  if(toml_rtod(raw, &(heading)))
  {
    fprintf(stderr, "ERROR: bad value in 'vehicle_heading.heading'\n");
	  toml_free(input);
	  exit(1);
  }
  else
  {
    // convert to radians and set value
    asv->attitude.heading = heading * PI/180.0;
  }
  

  // Locate array of tables [waypoint]
  tables = toml_array_in(input, "waypoint");
  if(tables == 0)
  {
    fprintf(stderr, "ERROR: missing [[waypoint]].\n");
	  toml_free(input);
	  exit(1);
  }
  // get number of waypoints
  waypoints->count = toml_array_nelem(tables);
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
    table = toml_table_at(tables, i);
    // Extract values in table [[waypoint]]
    // x
    raw = toml_raw_in(table, "x");
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'x' in [waypoint][%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(waypoints->points[i].x)))
    {
      fprintf(stderr, "ERROR: bad value in 'waypoint.x'\n");
	    toml_free(input);
	    exit(1);
    }
    // y
    raw = toml_raw_in(table, "y");
    if(raw == 0)
    {
      fprintf(stderr, "ERROR: missing 'y' in [waypoint][%d].\n", i);
	    toml_free(input);
	    exit(1);
    }
    if(toml_rtod(raw, &(waypoints->points[i].y)))
    {
      fprintf(stderr, "ERROR: bad value in 'waypoint.y'\n");
	    toml_free(input);
	    exit(1);
    }
  }

  // **** Matrix *****

  // Locate table [clock]
  double time_step_size = 0.0;
  table = toml_table_in(input, "clock");
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
                  double task_duration,
                  double simulation_time)
{
  // output message
  fprintf(stdout, "significant wave height = %f m.\n", wave_ht);
  fprintf(stdout, "wave heading = %f m.\n", wave_heading);
  fprintf(stdout, "task duration = %f seconds.\n", task_duration);
  fprintf(stdout, "time taken for simulation = %f sec. \n\n", simulation_time);
  
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
           "time(sec) "
           "wave_elevation(m) " 
           "cog_x(m) "
           "cog_y(m) "
           "cog_z(m) "
           "heel(deg) "
           "trim(deg) "
           "heading(deg) "
           "surge_vel(m/s) "
           "surge_acc(m/s2)" 
           );
  }
  // write buffer to file and close the file.
  for(int i = 0; i < buffer_length; ++i)
  {
    fprintf(fp, "\n%f %f %f %f %f %f %f %f %f %f %f %f", 
            buffer[i].sig_wave_ht,
            buffer[i].wave_heading,
            buffer[i].time,
            buffer[i].wave_elevation,
            buffer[i].cog_x, 
            buffer[i].cog_y, 
            buffer[i].cog_z, 
            buffer[i].heel, 
            buffer[i].trim, 
            buffer[i].heading,
            buffer[i].surge_velocity,
            buffer[i].surge_acceleration);
  }
  fclose(fp);
}
