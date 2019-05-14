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
    asv->dynamics.M[i] = 0.0;

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

  asv->dynamics.M[surge] = mass + added_mass_surge;
  asv->dynamics.M[sway]  = mass + added_mass_sway;
  asv->dynamics.M[yaw]   = mass + added_mass_heave;

  // Roll moment of inertia
  double r_roll = asv->spec.r_roll;
  asv->dynamics.M[roll] = mass * r_roll*r_roll + added_mass_roll;
  
  // Pitch moment of inertia
  double r_pitch = asv->spec.r_pitch;
  asv->dynamics.M[pitch] = mass * r_pitch*r_pitch + added_mass_pitch;

  // Yaw moment of inertia
  double r_yaw = asv->spec.r_yaw;
  asv->dynamics.M[yaw] = mass * r_yaw*r_yaw + added_mass_yaw;
}

// Method to set the damping matrix for the given asv object.
static void set_damping_matrix(struct Asv* asv)
{
  // Initialise all values of damping matrix to zero.
  for(int i = 0; i < COUNT_DOF; ++i)
    asv->dynamics.C[i] = 0.0;

  // Ref: Recommended practices DNVGL-RP-N103 Modelling and analysis of marine
  // operations. Edition July 2017. Appendix B Table B-1, B-2.
  // Surge drag coefficient - assuming elliptical waterplane area
  double B_L_ratio = asv->spec.B_wl/ asv->spec.L_wl;
  if(B_L_ratio <= 0.125)
  {
    asv->dynamics.C[surge] = 0.22 * asv->spec.T;
  }
  else if (B_L_ratio > 0.125 && B_L_ratio <= 0.25)
  {
    asv->dynamics.C[surge] = 0.22 + (0.3-0.22)/(0.25-0.125)*(B_L_ratio - 0.125)*
                             asv->spec.T;
  }
  else if (B_L_ratio > 0.25 && B_L_ratio <= 0.5)
  {
    asv->dynamics.C[surge] = 0.3+(0.6-0.3)/(0.5-0.25)*(B_L_ratio - 0.25) *
                             asv->spec.T;
  }
  else if (B_L_ratio > 0.5 && B_L_ratio <= 1.0)
  {
    asv->dynamics.C[surge] = 0.6 + (1.0 - 0.6)/(1.0-0.5)*(B_L_ratio - 0.5) * 
                             asv->spec.T;
  }
  else
  {
    asv->dynamics.C[surge] = 1.0 + (1.6 - 1.0)/(2.0-1.0)*(B_L_ratio - 1.0) *
                             asv->spec.T;
  }
  
  // Sway drag coefficient - if the vessel is cylindrical in shape then use the
  // same value as for surge else consider it as a flat plat.
  if(asv->spec.L_wl == asv->spec.B_wl)
  {
    asv->dynamics.C[sway] = asv->dynamics.C[surge];
  }
  else
  {
    double L_T_ratio = asv->spec.L_wl / asv->spec.T;
    if(L_T_ratio <= 1.0)
    {
      asv->dynamics.C[sway] = 1.16;
    }
    else if (L_T_ratio > 1.0 && L_T_ratio <= 5.0)
    {
      asv->dynamics.C[sway] = 1.16 + (1.2 - 1.16)/(5.0 - 1.0)*(L_T_ratio - 1.0);
    } 
    else if (L_T_ratio > 5.0 && L_T_ratio <= 10.0)
    {
      asv->dynamics.C[sway] = 1.2 + (1.5- 1.2)/(10.0 - 5.0)*(L_T_ratio - 5.0);
    }
    else
    {
      asv->dynamics.C[sway] = 1.9;
    }
  }
  
  // Heave drag coefficient - consider it as flat plat perpendicular to flow.
  double L_B_ratio = asv->spec.L_wl / asv->spec.B_wl;
  if(L_B_ratio <= 1.0)
  {
    asv->dynamics.C[heave] = 1.16;
  }
  else if (L_B_ratio > 1.0 && L_B_ratio <= 5.0)
  {
    asv->dynamics.C[heave] = 1.16 + (1.2 - 1.16)/(5.0 - 1.0)*(L_B_ratio - 1.0);
  } 
  else if (L_B_ratio > 5.0 && L_B_ratio <= 10.0)
  {
    asv->dynamics.C[heave] = 1.2 + (1.5- 1.2)/(10.0 - 5.0)*(L_B_ratio - 5.0);
  }
  else
  {
    asv->dynamics.C[heave] = 1.9;
  }

  // roll, pitch and yaw drag coefficient assumed as 0.
}

// Method to set the stiffness matrix for the given asv object.
static void set_stiffness_matrix(struct Asv* asv)
{
  // Initialise all values of stiffness matrix to zero.
  for(int i = 0; i < COUNT_DOF; ++i)
    asv->dynamics.K[i] = 0.0;

  // Surge stiffness = 0
  // Sway stiffness = 0
  // Yaw stiffness = 0
 
  // Assuming elliptical shape for the water plane area.
  double a = asv->spec.L_wl/2.0;
  double b = asv->spec.B_wl/2.0;
  double A = PI * a * b;
  double I_xx = (PI/4.0) * a * b*b*b;
  double I_yy = (PI/4.0) * a*a*a * b;
  
  // Heave stiffness
  asv->dynamics.K[heave] = A * SEA_WATER_DENSITY * G;

  // Roll stiffness
  // Using the same formula as mentioned for pitch in below ref.
  // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
  asv->dynamics.K[roll] = I_xx * SEA_WATER_DENSITY * G;

  // Pitch stiffness
  // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
  asv->dynamics.K[pitch] = I_yy * SEA_WATER_DENSITY * G;
}

void asv_init(struct Asv* asv, struct Asv_specification* spec)
{
  // Copy the specification
  asv->spec.L_wl = spec->L_wl;
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
