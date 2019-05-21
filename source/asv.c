#include <math.h>
#include "asv.h"
#include "constants.h"
#include "wave.h"
#include "wind.h"
#include "current.h"


// Enum to correctly index the motions in the matrices for asv dynamics.
enum i_dof{surge, sway, heave, roll, pitch, yaw}; // to index the DOF
enum i_axis{x, y, z}; // to index the axis for linear motion
enum i_attitude{heel, trim, heading}; // to index the floating attitude of ASV

// Method to set the mass and added mass for the given asv object.
static void set_mass(struct Asv* asv)
{
  // Mass of the ASV
  double mass = asv->spec->disp * SEA_WATER_DENSITY;

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
  double a = asv->spec->L_wl/2.0;
  // Find b such the volume of the semi-spheroid is equal to the displacement of
  // the vessel.
  double b = sqrt((3.0/4.0) * (asv->spec->disp/(PI * a))); 

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
  double r_roll = asv->spec->r_roll;
  asv->dynamics.M[roll] = mass * r_roll*r_roll + added_mass_roll;
  
  // Pitch moment of inertia
  double r_pitch = asv->spec->r_pitch;
  asv->dynamics.M[pitch] = mass * r_pitch*r_pitch + added_mass_pitch;

  // Yaw moment of inertia
  double r_yaw = asv->spec->r_yaw;
  asv->dynamics.M[yaw] = mass * r_yaw*r_yaw + added_mass_yaw;
}

// Method to set the drag coefficient for the given asv object.
static void set_drag_coefficient(struct Asv* asv)
{
  // Ref: Recommended practices DNVGL-RP-N103 Modelling and analysis of marine
  // operations. Edition July 2017. Appendix B Table B-1, B-2.
  // Surge drag coefficient - assuming elliptical waterplane area
  double B_L_ratio = asv->spec->B_wl/ asv->spec->L_wl;
  if(B_L_ratio <= 0.125)
  {
    asv->dynamics.C[surge] = 0.22;
  }
  else if (B_L_ratio > 0.125 && B_L_ratio <= 0.25)
  {
    asv->dynamics.C[surge] = 0.22 + (0.3-0.22)/(0.25-0.125)*(B_L_ratio - 0.125);
  }
  else if (B_L_ratio > 0.25 && B_L_ratio <= 0.5)
  {
    asv->dynamics.C[surge] = 0.3+(0.6-0.3)/(0.5-0.25)*(B_L_ratio - 0.25);
  }
  else if (B_L_ratio > 0.5 && B_L_ratio <= 1.0)
  {
    asv->dynamics.C[surge] = 0.6 + (1.0 - 0.6)/(1.0-0.5)*(B_L_ratio - 0.5);
  }
  else
  {
    asv->dynamics.C[surge] = 1.0 + (1.6 - 1.0)/(2.0-1.0)*(B_L_ratio - 1.0);
  }
  asv->dynamics.C[surge] = 0.5 * 
                           SEA_WATER_DENSITY * 
                           asv->dynamics.C[surge] * 
                           asv->spec->B_wl * asv->spec->T;
  
  // Sway drag coefficient - if the vessel is cylindrical in shape then use the
  // same value as for surge else consider it as a flat plat.
  if(asv->spec->L_wl == asv->spec->B_wl)
  {
    asv->dynamics.C[sway] = asv->dynamics.C[surge];
  }
  else
  {
    double L_T_ratio = asv->spec->L_wl / asv->spec->T;
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
    asv->dynamics.C[sway] = 0.5 * 
                            SEA_WATER_DENSITY * 
                            asv->dynamics.C[sway] * 
                            asv->spec->L_wl * asv->spec->T;
  }
  
  // Heave drag coefficient - consider it as flat plat perpendicular to flow.
  double L_B_ratio = asv->spec->L_wl / asv->spec->B_wl;
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
  asv->dynamics.C[heave] = 0.5 * 
                           SEA_WATER_DENSITY * 
                           asv->dynamics.C[heave] *
                           asv->spec->L_wl * asv->spec->B_wl;

  // roll, pitch and yaw drag coefficient set equal to roll damping coefficient 
  // given in Handbook of Marin Craft Hydrodynamics and motion control, page 125
  asv->dynamics.C[roll] = asv->dynamics.C[pitch] = asv->dynamics.C[yaw] = 0.075; 
}

// Method to set the stiffness for the given asv object.
static void set_stiffness(struct Asv* asv)
{
  // Surge stiffness = 0
  // Sway stiffness = 0
  // Yaw stiffness = 0
 
  // Assuming elliptical shape for the water plane area.
  double a = asv->spec->L_wl/2.0;
  double b = asv->spec->B_wl/2.0;
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

static void set_unit_wave_force(struct Asv* asv)
{
  // Assumptions:
  // 1. Assume the underwater volume to be a semi-ellipsoid with centre of
  // buoyancy at the centroid of the volume.
  // 2. Assume the wave pressure as constant along he projected surface of the 
  // hull.
  // 3. All measurements are with respect to body-frame - origin on waterline at 
  // aft end centreline. Positive measurements towards fore, port side and up.
  // 4. Unit wave height = 1 cm = 0.01 m.
  
  // Dimensions of ellipsoid
  double a = asv->spec->L_wl/ 2.0;
  double b = asv->spec->B_wl/ 2.0;
  double c = asv->spec->T;
  // Distance of centroid of ellipsoid from waterline.
  double z = - 4.0*c/(3.0*PI);

  // Distance of COG from COB
  double BG = fabs((asv->spec->KG - asv->spec->T) - z);
  
  double H_w = 1.0; // unit wave height in m.

  // Calculate the forces for each encounter wave freq
  double freq_step_size = (asv->dynamics.F_unit_wave_freq_max - 
                           asv->dynamics.F_unit_wave_freq_min)/
                          (COUNT_ASV_SPECTRAL_FREQUENCIES - 1); 
  for(int i = 0; i < COUNT_ASV_SPECTRAL_FREQUENCIES; ++i)
  {
    double freq = asv->dynamics.F_unit_wave_freq_min + i * freq_step_size;
    // Create a regular wave for the freq with wave height = 0.01, 
    // direction = 0.0 and phase = 0.0.
    struct Regular_wave wave;
    regular_wave_init(&wave, H_w/2.0, freq, 0.0, 0.0);
    
    // Calculate wave pressure at centre of buoyancy
    double P = SEA_WATER_DENSITY* G* wave.amplitude* exp(wave.wave_number* z);

    // Project water plane area
    double A_heave = 0.5*PI*a*b;
    // Project trans section area at midship
    double A_surge = 0.5*PI*b*c;
    // Projected buttockline area at CL
    double A_sway = 0.5*PI*a*c;

    // Surge force 
    asv->dynamics.F_unit_wave[i][surge] = A_surge * P;
    // Sway force
    asv->dynamics.F_unit_wave[i][sway] = A_sway * P;
    // Heave force
    asv->dynamics.F_unit_wave[i][heave] = A_heave * P;
    // roll, pitch and yaw moments assumed as zero
  }
}

static void set_wind_force_all_directions(struct Asv* asv)
{
  // Assumptions:
  // Longitudinal project area assumed as a triangle.
  // Transverse project area assumed as a rectangle.
  // Wind assumed as blowing at steady speed.
  
  double A_sway = 0.5 * asv->spec->L_wl * (asv->spec->D - asv->spec->T);
  double A_surge = asv->spec->B_wl * (asv->spec->D - asv->spec->T);
  double h_roll = ((1.0/3.0)*(asv->spec->D - asv->spec->T) + asv->spec->T) - 
                  asv->spec->KG;
  double h_pitch = (0.5*(asv->spec->D - asv->spec->T) + asv->spec->T) - 
                  asv->spec->KG;

  for(int i = 0; i < 360; ++i)
  {
    double angle = (PI/180.0) * i; // radians 
    // Resolve the wind velocity along the x and y direction (body-frame used)
    double v_x = asv->wind->speed * cos(angle);
    double v_y = asv->wind->speed * sin(angle);

    // Calculate forces
    double C = 1.16; // Drag coefficient
    double f_surge = 0.5 * AIR_DENSITY * C * A_surge * v_x * v_x;
    double f_sway  = 0.5 * AIR_DENSITY * C * A_sway * v_y * v_y;
    double f_roll  = f_sway * h_roll;
    double f_pitch = f_surge * h_pitch;  
    asv->dynamics.F_wind_all_directions[i][surge] = f_surge;
    asv->dynamics.F_wind_all_directions[i][sway]  = f_sway;
    asv->dynamics.F_wind_all_directions[i][roll]  = f_roll;
    asv->dynamics.F_wind_all_directions[i][pitch] = f_pitch;
    // heave and yaw assumed as zero.
  }
}

static double get_encounter_frequency(double wave_freq, double asv_speed)
{
  return wave_freq - (pow(wave_freq, 2.0)/G) * asv_speed;
}

void asv_init(struct Asv* asv, 
              struct Asv_specification* spec, 
              struct Wave* wave,
              struct Wind* wind,
              struct Current* current)
{
  // Copy pointers. 
  asv->spec = spec; // Should NOT be NULL and should be valid al long as asv is 
                    // valid.
  asv->wave = wave; // Could be NULL.
  asv->wind = wind; // Could be NULL.
  asv->current = current; // Could be NULL.

  // Initialise the position of the ASV
  asv->position.x = 0.0;
  asv->position.y = 0.0j;
  asv->position.z = -asv->spec->T;

  // Initialise the floating attitude of the ASV
  asv->attitude.heel = 0.0;
  asv->attitude.trim = 0.0;
  asv->attitude.heading = 0.0;

  // Initialise time record 
  asv->time = 0.0;

  // Initialise all the vectors matrices to zero.
  for(int i = 0; i < COUNT_ASV_SPECTRAL_DIRECTIONS; ++i)
  {
    for(int j = 0; j < COUNT_ASV_SPECTRAL_FREQUENCIES; ++j)
    {
  		for(int k = 0; k < COUNT_DOF; ++k)
  		{
  		  asv->dynamics.M                             [k] = 0.0;
  		  asv->dynamics.C                             [k] = 0.0;
  		  asv->dynamics.K                             [k] = 0.0;
  		  asv->dynamics.X                             [k] = 0.0;
  		  asv->dynamics.F                             [k] = 0.0;
  		  asv->dynamics.F_wave                        [k] = 0.0;
  		  asv->dynamics.F_wind                        [k] = 0.0;
  		  asv->dynamics.F_propeller                   [k] = 0.0;
  		  asv->dynamics.F_drag                        [k] = 0.0;
  		  asv->dynamics.F_restoring                   [k] = 0.0;
        asv->dynamics.F_wind_all_directions   [i]   [k] = 0.0;
        asv->dynamics.F_unit_wave                [j][k] = 0.0;
  		}
    }
  }

  // Set the mass matrix
  set_mass(asv);
  // Set the drag coefficient matrix
  set_drag_coefficient(asv);
  // Set the stiffness matrix
  set_stiffness(asv);

  if(asv->wave)
  {
    // Set minimum encounter frequency
    asv->dynamics.F_unit_wave_freq_min = get_encounter_frequency(
                                          asv->wave->min_spectral_frequency,
                                          asv->spec->max_speed);
    asv->dynamics.F_unit_wave_freq_max = get_encounter_frequency(
                                          asv->wave->max_spectral_frequency,
                                          -asv->spec->max_speed);
    // Set the wave force for unit waves
    set_unit_wave_force(asv);
  }
  if(asv->wind)
  {
    // Set the wind force for all directions
    set_wind_force_all_directions(asv);
  }
}

void asv_set_position(struct Asv* asv, struct Point position)
{
  asv->position.x = position.x;
  asv->position.y = position.y;
  asv->position.z = position.z;
}

void asv_set_attitude(struct Asv* asv, struct Asv_attitude attitude)
{
  asv->attitude.heel = attitude.heel;
  asv->attitude.trim = attitude.trim;
  asv->attitude.heading = attitude.heading;
}
