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

  // Radius of gyration
  // Assuming the ASV to homogeneous elliptical cylinder for the purpose of
  // calculating the radius of gyrations.
  double a = asv->spec.L/2.0;
  double b = asv->spec.B/2.0;
  double d = asv->spec.D;
  double r_roll =sqrt(9.0 * b*b + 12.0*d*d)/6.0;
  double r_pitch=sqrt(9.0 * a*a + 12.0*d*d)/6.0; 
  double r_yaw = sqrt(a*a + b*b) / 2.0; 
  

  // Added mass of the ASV
  // ASV shape idealisation
  // For the purpose of calculating the added mass the shape of the ASV is
  // assumed to be a semi ellipsoid.
  
  // Added mass for ellipsoid. 
  // Ref: The complete expression for "added mass" of a rigid body moving in an
  // ideal fluid. Frederick H Imlay. Page 15.
  // Note: the formula given in the above reference is for a full ellipsoid but
  // the shape that is assumed in this implementation is for a semi-ellipsoid.
  // Therefore divide the values for added mass by 2.
  double alpha_0 = 1.0;
  double beta_0 = 1.0;
  double gamma_0 = 1.0;
  a = asv->spec.L_wl/2.0;
  b = asv->spec.B_wl/2.0;
  double c = asv->spec.T;
  double added_mass_surge = 0.5 * (alpha_0/(2.0 - alpha_0)) *
                            (4.0/3.0) * PI * 
                            SEA_WATER_DENSITY *
                            a * b * c;
  double added_mass_sway = 0.5 * (beta_0/(2.0 - beta_0)) * 
                           (4.0/3.0) * PI *
                           SEA_WATER_DENSITY * 
                           a * b * c;
  double added_mass_heave = 0.5 * (gamma_0/(2.0 - gamma_0)) * 
                            (4.0/3.0) * PI *
                            SEA_WATER_DENSITY * 
                            a * b * c;
  // Assuming added mass for roll, pitch and yaw to be 0.

  asv->dynamics.M[surge][surge] = mass + added_mass_surge;
  asv->dynamics.M[sway][sway]   = mass + added_mass_sway;
  asv->dynamics.M[yaw][yaw]     = mass + added_mass_heave;

  // Roll moment of inertia
  asv->dynamics.M[roll][roll] = mass * r_roll*r_roll;
  
  // Pitch moment of inertia
  asv->dynamics.M[pitch][pitch] = mass * r_pitch*r_pitch;

  // Yaw moment of inertia
  asv->dynamics.M[yaw][yaw] = mass * r_yaw*r_yaw;
  asv->dynamics.M[surge][surge] += added_mass_surge;
  asv->dynamics.M[sway][sway] += added_mass_sway;
  asv->dynamics.M[heave][heave] += added_mass_heave;

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
  asv->spec.B = spec->B;
  asv->spec.D = spec->D;
  asv->spec.T = spec->T;
  asv->spec.max_speed = spec->max_speed;
  asv->spec.disp = spec->disp;
  asv->spec.cog = spec->cog;
  // TODO: Copy structure for propeller.
  
  // Set the mass matrix
  set_mass_matrix(asv);
  // Set the damping matrix
  set_damping_matrix(asv);
  // Set the stiffness matrix
  set_stiffness_matrix(asv);
}
