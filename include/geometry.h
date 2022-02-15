#ifndef GEOMETRY_H
#define GEOMETRY_H

/**
 * Cartesian coordinates for a three-dimensional space. 
 */
struct Cartesian_coordinate_3D
{
  double x; 
  double y; 
  double z; 
};

/**
 * Six degrees of freedom for a rigid body in a three-dimensional space. 
 */
struct Rigid_body_DOF
{
  struct Cartesian_coordinate_3D translational;
  struct Cartesian_coordinate_3D rotational;
};



#endif // GEOMETRY_H
