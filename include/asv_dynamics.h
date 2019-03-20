#ifndef ASV_DYNAMICS_H
#define ASV_DYNAMICS_H

#include "wave_spectrum.h"
#include "geometry.h"

namespace asv_swarm
{
namespace Hydrodynamics
{
/**
 * A simple structure to contain all input data about ASV.
 */
struct ASV
{
public:
  Quantity<Units::length> L; // length on load water line
  Quantity<Units::length> B; // beam of ASV at midship
  Quantity<Units::length> T; // draft of ASV at midship
  Quantity<Units::volume> displacement; // displacement at load water line
  Geometry::Point centre_of_gravity; // Also the control point of ASV
  Quantity<Units::length> metacentric_height; // metacentric height from keel
  Quantity<Units::length> r_roll; // roll radius of gyration
  Quantity<Units::length> r_pitch; // pitch radius of gyration
  Quantity<Units::length> r_yaw; // yaw radius of gyration
};

/**
 * Class for calculation the dynamics of ASV in waves, wind and current.
 */
class ASV_dynamics
{
public:
  /**
   * Constructor.
   */
  ASV_dynamics(ASV& asv);

private:
  /**
   * Method to calculate added mass and set set mass matrix.
   */
  void set_mass_matrix();

  /**
   * Method to calculate the damping matrix. 
   * This method is currently just a place holder and does not contain any
   * implementation. Damping, currently, is assumed as zero for the following
   * reasons:
   * - The length of the vessel is much smaller compared to wave length.
   * - The vessel speed is considered small.
   * - The vessel does not create any significant waves because of its small
   *   size and low speed.
   */
  void set_damping_matrix(){;}

  /**
   * Method to calculate the hydrostatic stiffness and set stiffness matrix.
   */
  void set_stiffness_matrix();

private:
  Quantity<Units::time> current_time;
  ASV& asv;
  double M[6][6]; // mass matrix
  double C[6][6]; // damping matrix
  double K[6][6]; // stiffness matrix
};

} //namespace Hydrodynamics

} //namespace asv_swarm

#endif // ASV_DYNAMICS_H
