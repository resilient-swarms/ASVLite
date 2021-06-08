#include <stdlib.h>
#include <stdio.h>
#include <netcdf.h>

// Name of the file to read.
#define FILE_NAME "hs.nc"

// Size of the data grid 
#define NX 24 // number of time steps
#define NY 53 // number of latitudes
#define NZ 77 // number of longitudes 

// Handle errors by printing an error message and exiting with a non-zero status.
#define ERRCODE 2
#define ERR(e) {printf("Error: %s\n", nc_strerror(e)); exit(ERRCODE);}

int main()
{
   // error index
   int error_id;

   // NetCDF IDs for the file and variable.
   int nc_id, var_id;

   // Open the file. NC_NOWRITE tells netCDF we want read-only access to the file.
   if ((error_id = nc_open(FILE_NAME, NC_NOWRITE, &nc_id)))
      ERR(error_id);
   
   // Explore the structure of the netcdf file.
   // Get the number of dimensions, variables, attributes and the id of the unlimited dimension (-1 implies no unlimited dimension).
   int count_dimensions, count_vars, count_attrs, unlimited_dim_id;
   if ((error_id = nc_inq(nc_id, &count_dimensions, &count_vars, &count_attrs, &unlimited_dim_id)))
      ERR(error_id);
   // TODO: Check if count_dimensions = 4 and 1 is longitude and 2 is latitude and 3 is time. If not then throw error message. 
   
   // Get the length for dimensions - longitude, latitude and time:
   int dim_sizes[4]; // array to store the size of each dimension.
                     // NOTE: Here we are assuming an netcdf file of a certain structure.
                     // The assumption is as follows:
                     // (1) number of dimensions = 4,
                     // (2) dimensions are (name, id) - (level, 0), (longitude, 1), (latitude, 2), (time, 3)
   for(int i = 0; i < count_dimensions; ++i)
   {
      if ((error_id = nc_inq_dimlen(nc_id, i, (dim_sizes+i) )))
         ERR(error_id);
      fprintf(stdout, "size of dim[%i] = %i \n", i, dim_sizes[i]);
   }

   // TODO: Create dinamic arrays for the data.
   // The hc values are of datatype double.
   int map[NY][NZ]; // Map of the grids. Value 1 for a cell implies that the cell is in water.
                    // If a cell is not in water then there is not value for hs for that cell.
   float data[NX][NY][NZ];
   
   // Read map
   // Get the var_id of the data variable, based on its name.
   if ((error_id = nc_inq_varid(nc_id, "MAPSTA", &var_id)))
      ERR(error_id);
   
   // Read the map.
   if ((error_id = nc_get_var_int(nc_id, var_id, &map)))
      ERR(error_id);

   // Read hs
   // Get the var_id of the data variable, based on its name.
   if ((error_id = nc_inq_varid(nc_id, "hs", &var_id)))
      ERR(error_id);

   // Read the data.
   if ((error_id = nc_get_var_float(nc_id, var_id, &data)))
      ERR(error_id);

   // Print the data 
   for(int i = 0; i < NX; ++i)
   {
      for(int j = 0; j < NY; ++j)
      {
         for(int k = 0; k < NZ; ++k)
         {
            // Get the value for the grid.
            // If the grid is in water (ie. map[j][k] == 1), then the grid has a hs value, else the value can be shown as 0.0)
            float value = (map[j][k] == 1)? data[i][j][k] : 0.0;
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