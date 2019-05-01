#ifndef ASV_DYNAMICS_H
#define ASV_DYNAMICS_H

#include <array>
#include "sea_surface_dynamics.h"
#include "geometry.h"

namespace asv_swarm
{
namespace Hydrodynamics
{
/**
 * A simple structure to contain all input data about ASV.
 */
struct ASV_particulars
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
  Quantity<Units::velocity> max_speed; // maximum operational speed of the ASV.
};

/**
 * A simple structure to contain the state of motion of the ASV.
 */
struct ASV_motion_state
{
public:
  Geometry::Point position; // Position of the ASV in the field
  Geometry::Orientation attitude; // roll, pitch and yaw angles of the ASV
  std::array<Quantity<Units::velocity>, 3> linear_velocity; 
  std::array<Quantity<Units::angular_velocity>, 3> angular_velocity;  
  std::array<Quantity<Units::acceleration>, 3> acceleration; 
  std::array<Quantity<Units::angular_acceleration>, 3> angular_acceleration;
};

/**
 * Class for calculation the dynamics of ASV in waves, wind and current.
 */
class ASV_dynamics
{
public:
  /**
   * Constructor. 
   * @param sea_surface reference to the sea surface simulated.
   * @param asv is the particulars of the ASV.
   * @param initial_state is the initial state of the ASV.
   */
  ASV_dynamics(Sea_surface_dynamics& sea_surface, 
               ASV_particulars asv, 
               ASV_motion_state initial_state);

  /**
   * Method to update the position of the ASV in the global coordinates for the
   * current time step.
   */
  void set_position(Quantity<Units::time> current_time);

  /**
   * Method to update the roll, pitch and yaw angles of the ASV for the current
   * time step.
   */
  void set_attitude(Quantity<Units::time> current_time);

private:
  /**
   * Method to calculate added mass and set set mass matrix.
   */
  void set_mass_matrix();

  /**
   * Method to calculate the viscous damping. 
   */
  void set_damping_matrix();

  /**
   * Method to calculate the wave force and moments due to wave of unit height 
   * for a range of frequencies and heading angles.
   */
  void set_unit_wave_force_spectrum();

  /**
   * Set the wave for matrix for the current time step.
   */
  void set_wave_force_matrix();

  /**
   * Set the propeller force matrix for the current time step.
   */
  void set_propeller_force_matrix();

  /**
   * Set the water current force matrix for the current time step.
   */
  void set_current_force_matrix();

  /**
   * Set the wind force matrix for the current time step.
   */
  void set_wind_force_matrix();
  /**
   * Method to set the restoring force matrix for the current time step. 
   * Restoring force = buoyancy - weight.
   */
  void set_restoring_force_matrix();

  /**
   * Method to get the encounter frequency for a given regular wave.
   * @param asv_speed is he speed of the ASV.
   * @param wave_frequency is the frequency of the regular wave.
   * @param wave_heading is the direction of propagation of the wave with
   * respect to direction of propagation of the ASV.
   */
  Quantity<Units::frequency> get_encounter_frequency(
      Quantity<Units::velocity> asv_speed,
      Quantity<Units::frequency> wave_frequency,
      Quantity<Units::plane_angle> wave_heading);

private:
  ASV_particulars asv; // Reference to the ASV simulated.

  Sea_surface_dynamics& sea_surface; // Reference to the sea surface simulated.
  Quantity<Units::frequency> min_encounter_frequency;
  Quantity<Units::frequency> max_encounter_frequency;
  static const int freq_count {101}; // Number of discrete frequencies
                                     // in the unit wave force spectrum. 
  static const int direction_count {361}; // Number of discrete wave 
                                          // headings in the unit wave force 
                                          // spectrum.
  static const int dof {6}; // degrees of freedom
  
  Quantity<Units::time> current_time;
  ASV_motion_state motion_state; // The current state of motion of the ASV.
  std::array<std::array<double, dof>, dof> M; // Mass matrix. mass + added mass
  std::array<std::array<double, dof>, dof> C; // Damping matrix. Viscous damping 
                                              // coefficient 
  std::array<std::array<std::array<double, dof>, freq_count>, direction_count> 
    F_unit_wave;                         // Unit wave force spectrum.
  std::array<double, dof> F_wave;        // Wave force matrix 
  std::array<double, dof> F_damping;     // Damping force matrix
  std::array<double, dof> F_restoring;   // Restoring force matrix  
  std::array<double, dof> F_propulsion;  // Propeller thrust force matrix
  std::array<double, dof> F_current;     // Water current force matrix
  std::array<double, dof> F_wind;        // wind force matrix
};

} //namespace Hydrodynamics

} //namespace asv_swarm

#endif // ASV_DYNAMICS_H
