#ifndef GEOMETRY_H
#define GEOMETRY_H

#define COUNT_COORDINATES 3 /* Count of dimensions in 3D space */ 
/**
 * Cartesian coordinates for a three-dimensional space. 
 */
union Coordinates_3D
{
  struct {
    double x;
    double y;
    double z;
  } keys;
  double array[COUNT_COORDINATES];
};


#define COUNT_DOF 6 /* Count of number of degrees of freedom for a rigid body. */ 
/**
 * Six degrees of freedom for a rigid body in a three-dimensional space. 
 */
union Rigid_body_DOF
{
  struct {
    double surge; // x translational 
    double sway;  // y translational 
    double heave; // z translational 
    double roll;  // x rotational 
    double pitch; // y rotational 
    double yaw;   // z rotational
  } keys;
  double array[COUNT_DOF];
};

#endif // GEOMETRY_H
