#include <stdlib.h>
#include <stdio.h>
#include <netcdf.h>
#include "cyclone.h"

const int days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static int count_leap_years(struct Time t)
{
   int year = t.year;
   int n = 0;
   // Check if the current year needs to be considered for the count of leap years
   year = (t.month <= 2) ? --year : year;
   // Number of leap years
   n = (year/4 - year/100 + year/400);
   return n;   
}

static int count_days_since_epoch(struct Time t)
{
   // Total number of days from 00/00/0000 to t1:
   // year
   int n = t.year* 365;
   // month
   for(int i = 0; i < t.month; ++i)
   {
      n += days_in_month[i];
   }
   // day
   n += t.day;

   // Add 1 day for each leap year since epoch
   n +=  count_leap_years(t);
   
   return n; 
}

static int count_days_between_dates(struct Time t1, struct Time t2)
{
   int n1 = count_days_since_epoch(t1);
   int n2 = count_days_since_epoch(t2);
   int n = abs(n1 - n2);
   return n;
}

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

int cyclone_init(struct Cyclone* cyclone, char** path_to_hs_nc_files, char** path_to_dp_nc_files, int count_sets)
{
   cyclone->count_sets = count_sets;
   // Allocate memory 
   cyclone->hs = (struct Data*)malloc(sizeof(struct Data) * count_sets);
   cyclone->dp = (struct Data*)malloc(sizeof(struct Data) * count_sets);
   // Read the netCDF files and initialise significant wave height and wave heading.
   for(int i = 0; i < count_sets; ++i)
   {
      init_data(path_to_hs_nc_files[i], cyclone->hs+i, "hs");
      init_data(path_to_dp_nc_files[i], cyclone->dp+i, "dp");
   }

   // TODO: Check if hs and dp files match.

   return 0;
}

void cyclone_clean(struct Cyclone* cyclone)
{
   for(int i = 0; i < cyclone->count_sets; ++i)
   {
      free(cyclone->dp[i].map);
      free(cyclone->dp[i].data);
      free(cyclone->hs[i].map);
      free(cyclone->hs[i].data);
   }
   free(cyclone->hs);
   free(cyclone->dp);
}

void cyclone_print_data(struct Cyclone* cyclone)
{
   for(int set = 0; set < cyclone->count_sets; ++set)
   {
      static const int data_length = 2;
      struct Data* data[] = {&cyclone->hs[set], &cyclone->dp[set]};
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
}

static int find_index(float* array, int length, float value)
{
   int index = -1;
   for(int i = 0; i < length - 1; ++i)
   {
      if(value >= array[i] && value < array[i+1])
      {
         index = i;
      }
      else
      {
         if(value == array[length - 1])
         {
            index = length - 1;
         }
      }
   }
   return index;
}

static float get_value_at(struct Data* data, int count_sets, struct Location location, struct Time time)
{
   // Convert time to days since 1-Jan-1990 00:00:00.
   struct Time t_epoch  = {1990, 1, 1, 0};
   int n = count_days_between_dates(time, t_epoch);
   float t = n + time.hour/24.0;
   
   // Get index of time
   int index_set;
   int index_time;
   for(index_set = 0; index_set < count_sets; ++index_set)
   {
      index_time = find_index(data[index_set].time_steps, data[index_set].count_time_steps, t);
      if(index_time != -1)
      {
         // Found index
         break;
      } 
   }
   if (index_time == -1)
   {
      fprintf(stderr, "ERROR: Time %f is beyoud the limits [%f, %f].", t, data[0].time_steps[0], data[count_sets-1].time_steps[data[count_sets-1].count_time_steps - 1]);
      exit(1);
   }
   
   // Get index of latitude
   int index_latitude = find_index(data[index_set].latitudes,  data[index_set].count_latitudes,  location.latitude);
   if (index_latitude == -1)
   {
      fprintf(stderr, "ERROR: Latitude %f is beyoud the limits [%f, %f].", location.latitude, data[0].latitudes[0], data[count_sets-1].latitudes[data[count_sets-1].count_latitudes - 1]);
      exit(1);
   }

   // Get index of latitude
   int index_longitude = find_index(data[index_set].longitudes, data[index_set].count_longitudes, location.longitude);
   if (index_longitude == -1)
   {
      fprintf(stderr, "ERROR: Longitude %f is beyoud the limits [%f, %f].", location.longitude, data[0].longitudes[0], data[count_sets-1].longitudes[data[count_sets-1].count_longitudes - 1]);
      exit(1);
   }   

   // Get the value for the cell.
   int count_longitudes = data[index_set].count_longitudes;
   int count_latitudes  = data[index_set].count_latitudes;
   int count_time_steps = data[index_set].count_time_steps;
   int map_value = data[index_set].map[index_latitude*count_longitudes + index_longitude];
   float value = (map_value == 1)? data[index_set].data[index_time*count_latitudes*count_longitudes + index_latitude*count_longitudes + index_longitude] : 0.0;

   return value;
}

float cyclone_get_wave_height(struct Cyclone* cyclone, struct Location location, struct Time time)
{
   float value = get_value_at(cyclone->hs, cyclone->count_sets, location, time);
   return value;
}

float cyclone_get_wave_heading(struct Cyclone* cyclone, struct Location location, struct Time time)
{
   float value = get_value_at(cyclone->dp, cyclone->count_sets, location, time);
   return value;
}

int main()
{
   struct Cyclone cyclone;
   char* hs_files[] = {"hs1.nc", "hs2.nc", "hs3.nc"};
   char* dp_files[] = {"dp1.nc", "dp2.nc", "dp3.nc"};
   // char* hs_files[] = {"hs.nc"};
   // char* dp_files[] = {"dp.nc"};
   int count_sets = 3;
   cyclone_init(&cyclone, hs_files, dp_files, count_sets);
   //cyclone_print_data(&cyclone);

   struct Location l = {22.3, 262.3};
   struct Time t = {2005, 8, 29, 11};
   cyclone_get_wave_height(&cyclone, l, t);
   cyclone_get_wave_heading(&cyclone, l, t);
}