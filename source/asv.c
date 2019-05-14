#include <math.h>
#include "asv.h"
#include "constants.h"

// Enum to correctly index the motions in the matrices for asv dynamics.
enum i_DOF{surge, sway, heave, roll, pitch, yaw};

// Method to set the mass matrix for the given asv object.
static void set_mass_matrix(struct Asv* asv)
{
  // Initialise all values of mass matrix to zero
  for(int i = 0; i<COUNT_DOF; ++i)
    for(int j = 0; j<COUNT_DOF; ++j)
      asv->dynamics.M[i][j] = 0.0;

  // Mass of the ASV
  double mass = asv->spec.disp * SEA_WATER_DENSITY;

  // Added mass of the ASV
  // ASV shape idealisation:
  // For the purpose of calculating the added mass the shape of the ASV is
  // assumed to be a semi-spheroid, with the transverse cross section as a 
  // semi-circle and the waterline as an ellipse.
  
  // Added mass for semi-spheroid. 
  // Ref: The complete expression for "added mass" of a rigid body moving in an
  // ideal fluid. Frederick H Imlay. Page 16.
  // Note: the formula given in the above reference is for a full spheroid but
  // the shape that is assumed in this implementation is for a semi-spheroid.
  // Therefore multiply added mass by 0.5.
  double a = asv->spec.L_wl/2.0;
  // Find b such the volume of the semi-spheroid is equal to the displacement of
  // the vessel.
  double b = sqrt((3.0/4.0) * (asv->spec.disp/(PI * a))); 

  double e = sqrt(1.0 - pow(b/a, 2.0));
  double alpha_0 = (2.0*(1 - e*e)/(e*e*e)) * (0.5*log10((1+e)/(1-e)) - e);
  double beta_0 = (1.0/(e*e)) - ((1-e*e)/(2*e*e*e)) * log10((1+e)/(1-e));
  
  double added_mass_surge = fabs(
                            0.5 * 
                            (alpha_0/(2.0 - alpha_0)) * 
                            (4.0/3.0) * PI * 
                            SEA_WATER_DENSITY *
                            a * b * b
                            );
  double added_mass_sway = fabs(
                           0.5 * 
                           (beta_0/(2.0 - beta_0)) *  
                           (4.0/3.0) * PI *
                           SEA_WATER_DENSITY * 
                           a * b * b
                           );
  double added_mass_heave = added_mass_sway;
  double added_mass_roll = 0.0;
  double added_mass_pitch = fabs(
                            0.5 *
                            (1.0/5.0) * 
                            pow(b*b - a*a, 2.0)*(alpha_0 - beta_0) / 
                            (2.0*(b*b - a*a) + (b*b + a*a)*(beta_0 - alpha_0)) *
                            (4.0/3.0) * PI *
                            SEA_WATER_DENSITY * 
                            a * b * b
                            );
  double added_mass_yaw = added_mass_pitch;

  asv->dynamics.M[surge][surge] = mass + added_mass_surge;
  asv->dynamics.M[sway][sway]   = mass + added_mass_sway;
  asv->dynamics.M[yaw][yaw]     = mass + added_mass_heave;

  // Roll moment of inertia
  double r_roll = asv->spec.r_roll;
  asv->dynamics.M[roll][roll] = mass * r_roll*r_roll + added_mass_roll;
  
  // Pitch moment of inertia
  double r_pitch = asv->spec.r_pitch;
  asv->dynamics.M[pitch][pitch] = mass * r_pitch*r_pitch + added_mass_pitch;

  // Yaw moment of inertia
  double r_yaw = asv->spec.r_yaw;
  asv->dynamics.M[yaw][yaw] = mass * r_yaw*r_yaw + added_mass_yaw;
}

// Method to set the damping matrix for the given asv object.
static void set_damping_matrix(struct Asv* asv)
{
  // Initialise all values of damping matrix to zero.
  for(int i = 0; i < COUNT_DOF; ++i)
    for(int j = 0; j < COUNT_DOF; ++j)
      asv->dynamics.C[i][j] = 0.0;

  // TODO: Set damping coefficients.
}

// Method to set the stiffness matrix for the given asv object.
static void set_stiffness_matrix(struct Asv* asv)
{
  // Initialise all values of stiffness matrix to zero.
  for(int i = 0; i < COUNT_DOF; ++i)
    for(int j = 0; j < COUNT_DOF; ++j)
      asv->dynamics.K[i][j] = 0.0;

  // Surge stiffness = 0
  // Sway stiffness = 0
  // Yaw stiffness = 0
 
  // Assuming elliptical shape for the water plane area.
  double a = asv->spec.L/2.0;
  double b = asv->spec.B/2.0;
  double A = PI * a * b;
  double I_xx = (PI/4.0) * a * b*b*b;
  double I_yy = (PI/4.0) * a*a*a * b;
  
  // Heave stiffness
  asv->dynamics.K[heave][heave] = A * SEA_WATER_DENSITY * G;

  // Roll stiffness
  // Using the same formula as mentioned for pitch in below ref.
  // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
  asv->dynamics.K[roll][roll] = I_xx * SEA_WATER_DENSITY * G;

  // Pitch stiffness
  // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
  asv->dynamics.K[pitch][pitch] = I_yy * SEA_WATER_DENSITY * G;
}

void asv_init(struct Asv* asv, struct Asv_specification* spec)
{
  // Copy the specification
  asv->spec.L = spec->L;
  asv->spec.L_wl = spec->L_wl;
  asv->spec.B = spec->B;
  asv->spec.B_wl = spec->B_wl;
  asv->spec.D = spec->D;
  asv->spec.T = spec->T;
  asv->spec.max_speed = spec->max_speed;
  asv->spec.disp = spec->disp;
  asv->spec.cog = spec->cog;
  asv->spec.r_roll = spec->r_roll;
  asv->spec.r_pitch = spec->r_pitch;
  asv->spec.r_yaw = spec->r_yaw;
  // TODO: Copy structure for propeller.
  
  // Set the mass matrix
  set_mass_matrix(asv);
  // Set the damping matrix
  set_damping_matrix(asv);
  // Set the stiffness matrix
  set_stiffness_matrix(asv);
}
