#ifndef CYCLONE_H
#define CYCLONE_H

/**
 * Structure to represent time.
 */
struct Time
{
  int year;
  int month;
  int day;
  int hour;
};

/**
 * Structure to represent a position on earth.
 */
struct Location
{
  float latitude;
  float longitude;
};

/**
 * Structure to store wave height or wave heading data from the netCDF file. 
 */
struct Data
{
  int count_longitudes; //!< Number of longitudes in the netCDF file. 
  int count_latitudes;  //!< Number of latitudes in the netCDF file.
  int count_time_steps; //!< Number of time steps in the netCDF file.
  float* longitudes; //!< List of longitudes.
  float* latitudes;  //!< List of latitudes.
  float* time_steps; //!< List of time steps.
  int* map;       //!< Array for storing map informantion. A cell with value 1 implies that the cell is in water and therefore will have a hs and fp value.
  float* data;    //!< Array to store hs or dp data.
};

struct Cyclone
{
  // Input variables
  // ---------------
  struct Data* hs; //!< Significant wave heights read from the netCDF file.
  struct Data* dp; //!< Wave headings read from the netCDF file.
  int count_sets;  //!< Number of sets of hs and dp files. 
};

/**
 * Initialise the cyclone to simulate.
 * @param cyclone instance to initialise.
 * @param path_to_hs_nc_files is the list of paths to the netCDF files containing significant wave heights.
 * @param path_to_dp_nc_files is the list of paths to the netCDF files containing predominant wave heading.
 * @param count_sets is the number of sets of hs and dp files.
 * @return 0 if no error encountered. 
 *         1 if files are not of the appropriate format.
 *         2 hs and dp files don't match.
 */
int cyclone_init(struct Cyclone* cyclone, char** path_to_hs_nc_files, char** path_to_dp_nc_files, int count_sets);

/** Free the heap memory.
 * @param cyclone instance to clean.
 */
void cyclone_clean(struct Cyclone* cyclone);

/**
 * Print the map and data for hs and dp.
 */
void cyclone_print_data(struct Cyclone* cyclone);

/**
 * Get the significant wave height at a given location at a given time.
 * @param location for which the wave height is to be obtained. 
 * @param time for which the wave height is to be obtained.
 */
float cyclone_get_wave_height_using_time(struct Cyclone* cyclone, struct Location location, struct Time time);
float cyclone_get_wave_height_using_days(struct Cyclone* cyclone, struct Location location, float time);

/**
 * Get the predominant direction of wave at a given location at a given time.
 * @param location for which the wave height is to be obtained. 
 * @param time for which the wave height is to be obtained.
 */
float cyclone_get_wave_heading_using_time(struct Cyclone* cyclone, struct Location location, struct Time time);
float cyclone_get_wave_heading_using_days(struct Cyclone* cyclone, struct Location location, float time);

#endif // CYCLONE_H