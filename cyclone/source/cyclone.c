#include <stdlib.h>
#include <stdio.h>
#include <netcdf.h>
#include "cyclone.h"

static void get_sizeof_dimension(int nc_id, char* name, int* size)
{
   int index;
   if(nc_inq_dimid(nc_id, name, &index))
   {
      fprintf(stderr, "ERROR: File does not contain the dimension %s.", name);
      exit(1);
   }
   else
   {
      if(nc_inq_dimlen(nc_id, index, size))
      {
         fprintf(stderr, "ERROR: Cannot get the size of dimension %s.", name);
         exit(1);
      }
   }
}

static void get_data(int nc_id, char* name, void* data)
{
   int index;
   if (nc_inq_varid(nc_id, name, &index))
   {
      fprintf(stderr, "ERROR: File does not contain the variable %s.", name);
      exit(1);
   }
   else
   {
      if(name == "MAPSTA")
      {
         if(nc_get_var_int(nc_id, index, (int*)data))
         {
            fprintf(stderr, "ERROR: Cannot get data for variable %s.", name);
            exit(1);
         }
      }
      else
      {
         if(nc_get_var_float(nc_id, index, (float*)data))
         {
            fprintf(stderr, "ERROR: Cannot get data for variable %s.", name);
            exit(1);
         }
      }
   }
}

/**
 * Initialise the hs OR dp data. 
 * @return  0 if no error.
 *          1 if nc file is not of correct format.
 */
static int init_data(char* path_to_nc, struct Data* data, char* var_name)
{
   // NetCDF IDs for the file and variable.
   int nc_id, var_id;

   // Open the file. NC_NOWRITE tells netCDF we want read-only access to the file.
   if (nc_open(path_to_nc, NC_NOWRITE, &nc_id))
   {
      fprintf(stderr, "ERROR: Cannot open file %s.", path_to_nc);
      exit(1);
   }
   
   get_sizeof_dimension(nc_id, "longitude", &data->count_longitudes);
   get_sizeof_dimension(nc_id, "latitude", &data->count_latitudes);
   get_sizeof_dimension(nc_id, "time", &data->count_time_steps);

   int total_size_map = data->count_longitudes * data->count_latitudes;
   int total_size = total_size_map * data->count_time_steps;
   data->longitudes = (float*)malloc(sizeof(float) * data->count_longitudes);
   data->latitudes  = (float*)malloc(sizeof(float) * data->count_latitudes);
   data->time_steps = (float*)malloc(sizeof(float) * data->count_time_steps);     
   data->map = (int*)malloc(sizeof(int) * total_size_map);
   data->data = (float*)malloc(sizeof(float) * total_size);
   
   // Read data
   get_data(nc_id, "longitude", data->longitudes);
   get_data(nc_id, "latitude", data->latitudes);
   get_data(nc_id, "time", data->time_steps);
   get_data(nc_id, "MAPSTA", data->map);
   get_data(nc_id, var_name, data->data);
   
   // Close the file, freeing all resources.
   nc_close(nc_id);
}

int cyclone_init(struct Cyclone* cyclone, char* path_to_hs_nc, char* path_to_dp_nc)
{
   // Read the netCDF files and initialise significant wave height and wave heading.
   init_data(path_to_hs_nc, &cyclone->hs, "hs");
   init_data(path_to_dp_nc, &cyclone->dp, "dp");

   // TODO: Check if hs and dp files match.

   return 0;
}

void cyclone_clean(struct Cyclone* cyclone)
{
   free(cyclone->dp.map);
   free(cyclone->dp.data);
   free(cyclone->hs.map);
   free(cyclone->hs.data);
}

void cyclone_print_data(struct Cyclone* cyclone)
{
   static const int data_length = 2;
   struct Data* data[] = {&cyclone->hs, &cyclone->dp};
   char* data_names[] = {"hs", "dp"};

   int count_time_steps, count_latitudes, count_longitudes;

   for(int n = 0; n < data_length; ++n)
   {
      fprintf(stdout, "Printing data from netCDF file for %s: \n\n", data_names[n]);

      count_longitudes = data[n]->count_longitudes;
      count_latitudes = data[n]->count_latitudes;
      count_time_steps = data[n]->count_time_steps;

      fprintf(stdout, "Printing %s longitudes: \n", data_names[n]);
      for(int i = 0; i < count_longitudes; ++i)
      {
         fprintf(stdout, "%f, ", data[n]->longitudes[i]);
      }
      fprintf(stdout, "\n\n");

      fprintf(stdout, "Printing %s latitudes: \n", data_names[n]);
      for(int i = 0; i < count_latitudes; ++i)
      {
         fprintf(stdout, "%f, ", data[n]->latitudes[i]);
      }
      fprintf(stdout, "\n\n");

      fprintf(stdout, "Printing %s time steps: \n", data_names[n]);
      for(int i = 0; i < count_latitudes; ++i)
      {
         fprintf(stdout, "%f, ", data[n]->time_steps[i]);
      }
      fprintf(stdout, "\n\n");
      
      fprintf(stdout, "Printing %s map: \n", data_names[n]);
      for(int j = 0; j < count_latitudes; ++j)
      {
         for(int k = 0; k < count_longitudes; ++k)
         {
            // Get the value for the cell.
            int map_value = data[n]->map[j*count_longitudes + k];
            fprintf(stdout, "%i ", map_value);
         }
         fprintf(stdout, "\n");
      }
      fprintf(stdout, "\n");

      fprintf(stdout, "Printing %s data: \n", data_names[n]);
      for(int i = 0; i < count_time_steps; ++i)
      {
         for(int j = 0; j < count_latitudes; ++j)
         {
            for(int k = 0; k < count_longitudes; ++k)
            {
               // Get the value for the cell.
               int map_value = data[n]->map[j*count_longitudes + k];
               float value = (map_value == 1)? data[n]->data[i*count_latitudes*count_longitudes + j*count_longitudes + k] : 0.0;
               fprintf(stdout, "%f ", value);
            }
            fprintf(stdout, "\n");
         }
         fprintf(stdout, "\n");
      }
   }
}

int main()
{
   struct Cyclone cyclone;
   cyclone_init(&cyclone, "hs.nc", "dp.nc");
   cyclone_print_data(&cyclone);
}