#ifndef GEOMETRY_H
#define GEOMETRY_H

/**
 * Structure to represent a point in 3D space.
 */
struct Point
{
  double x;
  double y;
  double z;
};

/**
 * Class to represent the orientation in 3D space with respect to x, y, z 
 * coordinates.
 */
struct Orientation
{
  double x;
  double y;
  double z;
}; 

#endif
