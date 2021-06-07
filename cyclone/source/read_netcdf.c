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
   // NetCDF IDs for the file and variable.
   int nc_id, var_id;

   // The hc values are of datatype double.
   int map[NY][NZ]; // Map of the grids. Value 1 implies the grid in the water.
   float data[NX][NY][NZ];

   // error index
   int error_id;

   // Open the file. NC_NOWRITE tells netCDF we want read-only access to the file.
   if ((error_id = nc_open(FILE_NAME, NC_NOWRITE, &nc_id)))
      ERR(error_id);
   
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