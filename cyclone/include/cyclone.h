#ifndef CYCLONE_H
#define CYCLONE_H

/**
 * Structure to define a cyclone. 
 */
struct Cyclone
{
  // Input variables
  // ---------------
  int count_dimensions; //!< number of dimensions in the nc file
  int count_vars;       //!< number of variables in the nc file
  int count_attrs;      //!< number of attributes in the nc file
  int unlimited_dim_id; //!< id of the unlimited dimension (-1 implies no unlimited dimension)
  int* dim_sizes; //!< array to store the size of each dimension.
  int* map;       //!< 2D array for storing map informantion. A cell with value 1 implies that the cell is in water and therefore will have a hs and fp value.
  float* hs;     //!< 3D array for storing significant wave heights.
  float* fp;     //!< 3D array for storing the peak spectral frequencies
};

#endif // CYCLONE_H