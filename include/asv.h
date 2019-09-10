#ifndef ASV_H
#define ASV_H

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
  double time;
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
  struct Wave* wave;
  struct Asv_specification spec;
  struct Asv_propeller propellers[COUNT_PROPELLERS_MAX];
  int count_propellers; // Number of propellers attached to ASV.
  struct Asv_dynamics dynamics;
  struct Point origin_position; // Position of the body-fixed frame in the 
                                // global frame for the current time step.
  struct Point cog_position; // Position of the centre of gravity of the ASV 
                             // in the global frame for the current time step.
  struct Attitude attitude; // The heel and trim are in body-fixed frame and the
                            // heading is in global frame.
};

/**
 * Function to initialise a propeller for an ASV.
 * @param propeller object to be initialised.
 * @param position of the propeller in the ASV in body-fixed frame.
 */
void asv_propeller_init(struct Asv_propeller* propeller, 
                        struct Point position); 

/**
 * Function to set the magnitude and direction of thrust force for the
 * propeller.
 * @param propeller for which the thrust is to be set.
 * @param thrust is the magnitude of thrust force in newton.
 * @param orientation is the direction of propeller thrust force in body-fixed
 * frame.
 */
void asv_propeller_set_thrust(struct Asv_propeller* propeller, 
                              double thrust,
                              struct Attitude orientation);

/**
 * Function to set a propeller in an ASV.
 * @param asv to which the propeller is to be fixed.
 * @param propeller to be fixed. Pointer propeller should have at least the same 
 * life time as pointer asv.
 * @param return 1 if successful else 0.
 */
int asv_set_propeller(struct Asv* asv, struct Asv_propeller propeller);


/**
 * Function to initialise a model of ASV. This function initialises all
 * properties of the ASV and also places the ASV in the origin of the global
 * frame at its still water floating condition.
 * @param asv is the object to be initialised.
 * @param spec is the pointer to the specifications. Assumes that spec is not
 * NULL and has a lifetime at least equal to that of pointer asv.
 * @param wave is the pointer to the wave model. Value can be set to NULL if
 * wave forces are not required to be simulated. Pointer wave should have at
 * least the same life time of pointer asv.
 */
void asv_init(struct Asv* asv, 
              struct Asv_specification spec, 
              struct Wave* wave);

/**
 * Function to set the initial position of the ASV in the global frame. The 
 * position of the ASV is set by setting the position of the origin of the 
 * body-fixed frame in the global frame. 
 */
void asv_set_position(struct Asv* asv, struct Point position);

/**
 * Function to set the initial floating attitude of the ASV.
 */
void asv_set_attitude(struct Asv* asv, struct Attitude attitude);

/**
 * Function to set the position and attitude of the ASV for the given time step.
 * @param asv is the pointer to the asv object for which the position is to be
 * computed.
 * @param time is the time for which the position is to be computed.
 */
void asv_set_dynamics(struct Asv* asv, double time);

#endif // ASV_H
