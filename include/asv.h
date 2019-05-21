#ifndef ASV_H
#define ASV_H

#include "geometry.h"

// Forward declarations of wind, wave and current.
struct Wave;
struct Wind;
struct Current;

#define COUNT_DOF 6 /* Number of degrees of freedom for the motion of ASV. */
#define COUNT_ASV_SPECTRAL_DIRECTIONS 360
#define COUNT_ASV_SPECTRAL_FREQUENCIES 100

/**
 * Structure to represent the floating angles of the ASV.
 */
struct Asv_attitude
{
  double heel;    // Angle with x-axis in radian.
  double trim;    // Angle with y-axis in radian.
  double heading; // Angle with z-axis in radian.
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
  double KG; // Distance of centre of gravity from keel in m.
  double disp; // Displacement of the ASV in m3.
  double r_roll; // roll radius of gyration.
  double r_pitch; // pitch radius of gyration.
  double r_yaw; // yaw radius of gyration.

  // TODO: struct Propeller[?] propeller;
};

struct Asv_dynamics
{
  double M[COUNT_DOF]; // Mass (+ added mass).
  double C[COUNT_DOF]; // Drag force coefficients.
  double K[COUNT_DOF]; // Stiffness.
  double X[COUNT_DOF]; // Deflection.
  
  double F[COUNT_DOF]; // Net force.
  double F_wave[COUNT_DOF];
  double F_wind[COUNT_DOF];
  double F_propeller[COUNT_DOF];
  double F_drag[COUNT_DOF];
  double F_restoring[COUNT_DOF];

  double F_unit_wave[COUNT_ASV_SPECTRAL_FREQUENCIES][COUNT_DOF];
  double F_unit_wave_freq_min;
  double F_unit_wave_freq_max;
  double F_wind_all_directions[COUNT_ASV_SPECTRAL_DIRECTIONS][COUNT_DOF];
};

struct Asv
{
  struct Wave* wave;
  struct Wind* wind;
  struct Current* current;
  struct Asv_specification* spec;
  struct Asv_dynamics dynamics;
  double time;
  struct Point position;
  struct Asv_attitude attitude;
};

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
 * @param wind is the pointer to the wind model. Value can be set to NULL if 
 * wind forces are not required to be simulated. Pointer wind should have at
 * least the same life time of pointer asv.
 * @param current is the pointer to the current model. Value can be set to NULL
 * if current forces are not required to be simulated. Pointer current should
 * have at least the same life time as pointer asv.
 */
void asv_init(struct Asv* asv, 
              struct Asv_specification* spec, 
              struct Wave* wave, 
              struct Wind* wind, 
              struct Current* current);

/**
 * Function to set the initial position of the ASV in the global frame. The 
 * position of the ASV is set by setting the position of the origin of the 
 * body-fixed frame in the global frame. 
 */
void asv_set_position(struct Asv* asv, struct Point position);

/**
 * Function to set the initial floating attitude of the ASV.
 */
void asv_set_attitude(struct Asv* asv, struct Asv_attitude attitude);

/**
 * Function to set the position and attitude of the ASV in the global frame for 
 * the given time step.
 * @param asv is the pointer to the asv object for which the position is to be
 * computed.
 * @param time is the time for which the position is to be computed.
 */
void asv_set_dynamics(struct Asv* asv, double time);

#endif // ASV_H
