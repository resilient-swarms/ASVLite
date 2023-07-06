#ifndef GEOMETRY_H
#define GEOMETRY_H

/**
 * @file
 * Coordinates_3D and Rigid_body_DOF provides mechanisms to access a block
 * of memory as a struct or as an array. Example:
 * \code{.c}
 * const union Coordinates_3D point = asv_get_position_cog(asv); // Returns the position of an asv.
 * 
 * // Access the x, y, z coordinates of point as a struct:
 * double x = point.keys.x;
 * double y = point.keys.y;
 * double z = point.keys.z;
 * 
 * // Access the same x,y,z coordinates of point as an array:
 * double coordinates[3];
 * for(int i = 0; i < COUNT_COORDINATES; ++i)
 * {
 *   coordinates[i] = point.array[i];
 * }
 * \endcode
 * 
 */

#define COUNT_COORDINATES 3 /* Number of dimensions in 3D space */ 
/**
 * Cartesian coordinates for a three-dimensional space. 
 * The members can either be accessed as a struct using keys or as an array using index.
 */
union Coordinates_3D
{
  struct {
    double x; //!< x coordinate
    double y; //!< y coordinate
    double z; //!< z coordinate
  } keys; //!< To access the coordinate values using the keys x, y, z.
  double array[COUNT_COORDINATES]; //!< To access the coordinate values using index 0, 1, 2.
};


#define COUNT_DOF 6 /* Number of degrees of freedom for a rigid body. */ 
/**
 * Six degrees of freedom for a rigid body in a three-dimensional space. 
 * The members can either be accessed as a struct using keys or as an array using index.
 */
union Rigid_body_DOF
{
  struct {
    double surge; //!< x translational 
    double sway;  //!< y translational 
    double heave; //!< z translational 
    double roll;  //!< x rotational 
    double pitch; //!< y rotational 
    double yaw;   //!< z rotational
  } keys; //!< To access the DOF values using the keys surge, sway, heave, roll, pitch, yaw.
  double array[COUNT_DOF]; //!< To access the DOF values using index 0, 1, 2, 3, 4, 5.
};

/**
 * Normalise angle so that -PI < angle <= PI.
 * @param angle in radians.
 * @return normalised angle in radians. 
 */
double normalise_angle_PI(double angle);

/**
 * Normalise angle so that 0 <= angle < 2PI.
 * @param angle in radians.
 * @return normalised angle in radians. 
 */
double normalise_angle_2PI(double angle);

#endif // GEOMETRY_H
