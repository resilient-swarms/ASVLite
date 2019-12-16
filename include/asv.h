#ifndef ASV_H
#define ASV_H

#include <stdbool.h>
#include "constants.h"
#include "geometry.h"
#include "wave.h"

// Enum to correctly index the motions in the matrices for asv dynamics.
enum i_dof{surge, sway, heave, roll, pitch, yaw}; // to index the DOF
enum i_axis{x, y, z}; // to index the axis for linear motion
enum i_attitude{heel, trim, heading}; // to index the floating attitude of ASV

/**
 * Struct to hold all the inputs for the propeller.
 */
struct Asv_propeller
{
  struct Dimensions position; // Position of propeller force vector in ASV's 
                              // body-fixed frame.
  struct Dimensions orientation; // Orientation of the force vector of the 
                                 // propeller in body-fixed frame.
  double thrust; // Magnitude of propeller force.
};

/**
 * Struct to hold some of the input value of the vehicle. All 
 * variables in this struct are inputs and should be set before calling 
 * asv_init().
 * Coordinate system: Body-centric frame. The origin of the frame is on the
 * waterline at the aft end centre line.
 */
struct Asv_specification
{
  double L_wl; // Length waterline in m.
  double B_wl; // Breadth waterline in m.
  double D; // Depth of the ASV in m.
  double T; // Draught of the ASV in m.
  double max_speed; // Maximum operational speed of the ASV in m/s.
  double disp; // Displacement of the ASV in m3.
  double r_roll; // roll radius of gyration.
  double r_pitch; // pitch radius of gyration.
  double r_yaw; // yaw radius of gyration.
  struct Dimensions cog; // Centre of gravity in body-fixed frame.
};

/**
 * Struct to contain both inputs and outputs of ASV dynamics. All input
 * variables must be set before calling asv_compute_dynamics().
 */
struct Asv_dynamics
{
  // Input
  double time_step_size; // seconds
  double time; // seconds
  
  // Optional inputs
  double M[COUNT_DOF]; // Mass (+ added mass).
  double C[COUNT_DOF]; // Drag force coefficients.
  double K[COUNT_DOF]; // Stiffness.
  
  // Output
  double X[COUNT_DOF]; // Deflection in body-fixed frame.
  double V[COUNT_DOF]; // Velocity of ASV in body-fixed frame.
  double A[COUNT_DOF]; // Acceleration of ASV in body-fixed frame.
  
  double F[COUNT_DOF]; // Net force.
  double F_wave[COUNT_DOF];
  double F_propeller[COUNT_DOF];
  double F_drag[COUNT_DOF];
  double F_restoring[COUNT_DOF];

  double P_unit_wave[COUNT_ASV_SPECTRAL_FREQUENCIES][2]; // index 0 - freq
                                                         // index 1 - pressure
  double P_unit_wave_freq_min;
  double P_unit_wave_freq_max;
};

/**
 * Stuct to contain both input and output of ASV motion in waves. 
 */
struct Asv
{
  // Input
  struct Asv_specification spec;
  int count_propellers; // Number of propellers attached to ASV.
  struct Asv_propeller propellers[COUNT_PROPELLERS_MAX];
  bool using_waves; // false for still water else true.
  struct Wave wave;

  // Initial used for input but later contains results. 
  struct Dimensions origin_position; // Position of the body-fixed frame in the 
                                // global frame for the current time step.
  struct Dimensions attitude; // The heel and trim are in body-fixed frame and the
                            // heading is in global frame.
  
  // Output
  struct Asv_dynamics dynamics;
  struct Dimensions cog_position; // Position of the centre of gravity of the ASV 
                             // in the global frame for the current time step.
};

/**
 * Function to initialise a model of ASV after setting the vehicle spec. 
 * **Note:** This function should be called only after setting all input values.
 * @param asv is the object to be initialised.
 */
void asv_init(struct Asv* asv);

/**
 * Function to set the position and attitude of the ASV for the given time step.
 * @param asv is the pointer to the asv object for which the position is to be
 * computed. 
 * @param time is the time for which the position is to be computed.
 */
void asv_compute_dynamics(struct Asv* asv, double time);

#endif // ASV_H
