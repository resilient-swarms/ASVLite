#ifndef GEOMETRY_H
#define GEOMETRY_H

/**
 * Structure to represent a point in 3D space.
 */
struct Point
{
  double x; // x coordinate in meter.
  double y; // y coordinate in meter.
  double z; // z coordinate in meter.
};

/**
 * Structure to represent the orientation in 3D space with respect to x, y, z 
 * axis.
 */
struct Orientation
{
  double x; // Angle with x-axis in radian.
  double y; // Angle with y-axis in radian.
  double z; // Angle with z-axis in radian.
}; 

#endif // GEOMETRY_H
