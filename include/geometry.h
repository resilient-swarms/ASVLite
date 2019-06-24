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
 * Structure to represent the angles.
 */
struct Attitude
{
  double heel;    // Angle in radian with x-axis.
  double trim;    // Angle in radian with y-axis.
  double heading; // Angle in radian with z-axis.
}; 



#endif // GEOMETRY_H
