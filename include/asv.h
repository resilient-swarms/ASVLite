#ifndef ASV_H
#define ASV_H

#include <stdbool.h>
#include "constants.h"
#include "geometry.h"
#include "wave.h"

/**
 * Structure to represent the propeller of an ASV.
 */
struct Asv_propeller
{
  struct Point position; // Position of propeller force vector in ASV's 
                         // body-fixed frame.
  struct Attitude orientation; // Orientation of the force vector of the 
                               // propeller in body-fixed frame.
  double thrust; // Magnitude of propeller force.
};

/**
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
  struct Point cog; // Centre of gravity in body-fixed frame.
};

struct Asv_dynamics
{
  double time; // seconds
  double time_step_size; // seconds
  double M[COUNT_DOF]; // Mass (+ added mass).
  double C[COUNT_DOF]; // Drag force coefficients.
  double K[COUNT_DOF]; // Stiffness.
  double X[COUNT_DOF]; // Deflection in body-fixed frame.
  double V[COUNT_DOF]; // Velocity of ASV in body-fixed frame.
  double A[COUNT_DOF]; // Acceleration of ASV in body-fixed frame.
  
  double F[COUNT_DOF]; // Net force.
  double F_wave[COUNT_DOF];
  double F_propeller[COUNT_DOF];
  double F_drag[COUNT_DOF];
  double F_restoring[COUNT_DOF];

  double F_unit_wave[COUNT_ASV_SPECTRAL_FREQUENCIES][COUNT_DOF];
  double F_unit_wave_freq_min;
  double F_unit_wave_freq_max;
};

struct Asv
{
  // Input
  struct Asv_specification spec;
  int count_propellers; // Number of propellers attached to ASV.
  struct Asv_propeller propellers[COUNT_PROPELLERS_MAX];
  bool using_waves; // false for still water else true.
  struct Wave wave;

  // Initial used for input but later contains results. 
  struct Point origin_position; // Position of the body-fixed frame in the 
                                // global frame for the current time step.
  struct Attitude attitude; // The heel and trim are in body-fixed frame and the
                            // heading is in global frame.
  
  // Output
  struct Asv_dynamics dynamics;
  struct Point cog_position; // Position of the centre of gravity of the ASV 
                             // in the global frame for the current time step.
};

/**
 * Function to initialise a model of ASV after setting the vehicle spec. 
 * **Note:** This function should be called only after setting Asv::spec.
 * @param asv is the object to be initialised.
 */
void asv_init(struct Asv* asv);

/**
 * Function to set the position and attitude of the ASV for the given time step.
 * @param asv is the pointer to the asv object for which the position is to be
 * computed. **Note:** Time 0 must always be run first to complete the 
 * initialisation. 
 * @param time is the time for which the position is to be computed.
 */
void asv_compute_dynamics(struct Asv* asv, double time);

#endif // ASV_H
