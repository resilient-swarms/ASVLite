#ifndef CYCLONE_H
#define CYCLONE_H

/**
 * Structure to store wave height or wave heading data from the netCDF file. 
 */
struct Data
{
  int count_dimensions; //!< number of dimensions in the nc file
  int count_vars;       //!< number of variables in the nc file
  int count_attrs;      //!< number of attributes in the nc file
  int unlimited_dim_id; //!< id of the unlimited dimension (-1 implies no unlimited dimension)
  int* dim_sizes; //!< array to store the size of each dimension.
  int* map;       //!< array for storing map informantion. A cell with value 1 implies that the cell is in water and therefore will have a hs and fp value.
  float* data;    //!< array to store hs or dp data.
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