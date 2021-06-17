#ifndef CYCLONE_H
#define CYCLONE_H

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
  struct Data hs; //!< Significant wave heights read from the netCDF file.
  struct Data dp; //!< Wave headings read from the netCDF file.
};

/**
 * Initialise the cyclone to simulate.
 * @param cyclone instance to initialise.
 * @param path_to_hs_nc is the path to the netCDF file containing significant wave heights.
 * @param path_to_dp_nc is the path to the netCDF file containing predominant wave heading.
 * @return 0 if no error encountered. 
 *         1 if files are not of the appropriate format.
 *         2 hs and dp files don't match.
 */
int cyclone_init(struct Cyclone* cyclone, char* path_to_hs_nc, char* path_to_dp_nc);

/** Free the heap memory.
 * @param cyclone instance to clean.
 */
void cyclone_clean(struct Cyclone* cyclone);

/**
 * Print the map and data for hs and dp.
 */
void cyclone_print_data(struct Cyclone* cyclone);

// Function to access print the grid data and map data.
// Function to access a cell value for a give long, lat, time value.

#endif // CYCLONE_H