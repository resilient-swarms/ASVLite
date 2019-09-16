#include <stdlib.h>
#include "input.h"
#include "toml.h"

int set_input(char* file, struct Asv* asv)
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
    if(table == 0)
    {
      fprintf(stderr, "ERROR: missing [[propeller]].\n");
	    toml_free(input);
	    exit(1);
    }
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
      fprintf(stderr, "ERROR: missing 'y' in [propeller][%d].\n");
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
      fprintf(stderr, "ERROR: missing 'z' in [propeller][%d].\n");
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

  // done reading inputs
  toml_free(input);
}