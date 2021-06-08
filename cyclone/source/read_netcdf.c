#include <stdlib.h>
#include <stdio.h>
#include <netcdf.h>
#include "cyclone.h"

// Name of the file to read.
#define FILE_NAME "hs.nc"

// Handle errors by printing an error message and exiting with a non-zero status.
#define ERRCODE 2
#define ERR(e) {printf("Error: %s\n", nc_strerror(e)); exit(ERRCODE);}

int main()
{
   // error index
   int error_id;

   // NetCDF IDs for the file and variable.
   int nc_id, var_id;

   // Current session to simulate a cyclone.
   struct Cyclone cyclone;

   // Open the file. NC_NOWRITE tells netCDF we want read-only access to the file.
   if ((error_id = nc_open(FILE_NAME, NC_NOWRITE, &nc_id)))
      ERR(error_id);
   
   // Explore the structure of the netcdf file.
   // Get the number of dimensions, variables, attributes and the id of the unlimited dimension (-1 implies no unlimited dimension).
   if ((error_id = nc_inq(nc_id, &cyclone.count_dimensions, 
                                 &cyclone.count_vars, 
                                 &cyclone.count_attrs, 
                                 &cyclone.unlimited_dim_id)))
      ERR(error_id);
   // TODO: Check if 1 is longitude and 2 is latitude and 3 is time. If not then throw error message. 
   
   // Create dynamic array for storing dimensions.
   cyclone.dim_sizes = (int*)malloc(sizeof(int) * cyclone.count_dimensions);
   // Get the size for dimensions - longitude, latitude and time:
   for(int i = 0; i < cyclone.count_dimensions; ++i)
   {
      if ((error_id = nc_inq_dimlen(nc_id, i, (cyclone.dim_sizes+i) )))
         ERR(error_id);
   }

   // Create dinamic arrays for the data.
   int size_longitudes = cyclone.dim_sizes[1];
   int size_latitudes = cyclone.dim_sizes[2];
   int size_time = cyclone.dim_sizes[3];
   int total_size_map = size_longitudes * size_latitudes;
   int total_size = total_size_map * size_time;
   cyclone.map = (int*)malloc(sizeof(int) * total_size_map);
   cyclone.hs = (float*)malloc(sizeof(float) * total_size);
   cyclone.fp = (float*)malloc(sizeof(float) * total_size);
   
   // Read map
   // Get the var_id of the data variable, based on its name.
   if ((error_id = nc_inq_varid(nc_id, "MAPSTA", &var_id)))
      ERR(error_id);
   
   // Read the map.
   if ((error_id = nc_get_var_int(nc_id, var_id, cyclone.map)))
      ERR(error_id);

   // Read hs
   // Get the var_id of the data variable, based on its name.
   if ((error_id = nc_inq_varid(nc_id, "hs", &var_id)))
      ERR(error_id);

   // Read the data.
   if ((error_id = nc_get_var_float(nc_id, var_id, cyclone.hs)))
      ERR(error_id);

   // Print the data 
   for(int i = 0; i < size_time; ++i)
   {
      for(int j = 0; j < size_latitudes; ++j)
      {
         for(int k = 0; k < size_longitudes; ++k)
         {
            // Get the value for the cell.
            // If the cell is in water (ie. map[j][k] == 1), then the grid has a hs value, else the value can be shown as 0.0)
            int map_value = cyclone.map[j*size_longitudes + k];
            float value = (map_value == 1)? cyclone.hs[i*size_latitudes*size_longitudes + j*size_longitudes + k] : 0.0;
            fprintf(stdout, "%f ", value);
         }
         fprintf(stdout, "\n");
      }
      fprintf(stdout, "\n");
   }

   // Close the file, freeing all resources.
   if ((error_id = nc_close(nc_id)))
      ERR(error_id);

   return 0;
}