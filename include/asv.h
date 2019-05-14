#ifndef ASV_H
#define ASV_H

#include "geometry.h"
#include "environment.h"

#define COUNT_DOF 6 /* Number of degrees of freedom for the motion of ASV. */
#define COUNT_ASV_SPECTRAL_DIRECTIONS 360
#define COUNT_ASV_SPECTRAL_FREQUENCIES 100

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
  struct Point cog; // Centre of gravity of the ASV.
  double disp; // Displacement of the ASV in m3.
  double r_roll; // roll radius of gyration
  double r_pitch; // pitch radius of gyration
  double r_yaw; // yaw radius of gyration

  // TODO: struct Propeller[?] propeller;
};

struct Asv_dynamics
{
  double M[COUNT_DOF]; // Mass (+ added mass) matrix.
  double C[COUNT_DOF]; // Drag force coefficients.
  double K[COUNT_DOF]; // Stiffness .
  double X[COUNT_DOF]; // Deflection
  
  double F[COUNT_DOF]; // Net force
  double F_wave[COUNT_DOF];
  double F_wind[COUNT_DOF];
  double F_current[COUNT_DOF];
  double F_propeller[COUNT_DOF];
  double F_drag[COUNT_DOF];
  double F_restoring[COUNT_DOF];

  double F_unit_wave[COUNT_ASV_SPECTRAL_DIRECTIONS]
                    [COUNT_ASV_SPECTRAL_FREQUENCIES]
                    [COUNT_DOF];
  double F_wind_direction[COUNT_ASV_SPECTRAL_DIRECTIONS][COUNT_DOF];
  double F_current_direction[COUNT_ASV_SPECTRAL_DIRECTIONS][COUNT_DOF];
};

struct Asv
{
  struct Asv_specification spec;
  struct Asv_dynamics dynamics;
};

/**
 * Function to initialise a model of ASV. This function initialises all
 * properties of the ASV and also places the ASV in the origin of the global
 * frame at its still water floating condition.
 * @param asv is the object to be initialised.
 * @param spec is the pointer to the specifications. Values are deep copied from
 * spec.
 */
void asv_init(struct Asv* asv, struct Asv_specification* spec);

/**
 * Function to overwrite the mass matrix of the ASV.
 * @param asv is the pointer to the ASV object.
 * @param M is the mass + added mass matrix. Values are deep copied from M.
 */
void asv_set_mass_matrix(struct Asv* asv, double M[COUNT_DOF][COUNT_DOF]);

/**
 * Function to overwrite the damping matrix of the ASV.
 * @param asv is the pointer to the ASV object.
 * @parma C is the damping matrix. Values are deep copied from C.
 */
void asv_set_damping_matrix(struct Asv* asv, double C[COUNT_DOF][COUNT_DOF]);

/**
 * Function to overwrite the stiffness matrix of the ASV.
 * @param asv is the pointer to the ASV object.
 * @param K is the stiffness matrix. Values are deep copied from K.
 */
void asv_set_stiffness_matrix(struct Asv* asv, double K[COUNT_DOF][COUNT_DOF]);

/**
 * Function to get the position of the ASV in the global frame for the given
 * time.
 * @param position is the buffer to contain the return value. The returned value
 * is an array of length 3. The values contained are the x, y and z coordinate
 * values.
 * @param asv is the poinet to the asv object for which the position is to be
 * computed.
 * @param time is the time for which the position is to be computed.
 */
void asv_get_position(double position[3], struct Asv* asv, double time);

#endif // ASV_H
