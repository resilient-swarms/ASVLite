#include <math.h>
#include <stdlib.h>
#include "asv.h"

// Method to compute the encounter frequency. 
// heading_angle is the heading angle of the wave with respect to positive x
// axis of ASV.
static double get_encounter_frequency(double wave_freq, 
                                      double asv_speed, 
                                      double heading_angle)
{
  return wave_freq - (pow(wave_freq, 2.0)/G) * asv_speed * cos(heading_angle);
}

// Function to set the COG of the ASV in the global frame. 
static void set_cog(struct Asv* asv)
{
  // Match the position of the COG with that of the position of the origin.
  double l = sqrt(pow(asv->spec.cog.x, 2.0) + pow(asv->spec.cog.y, 2.0));
  asv->cog_position.x = asv->origin_position.x + l * sin(asv->attitude.heading);
  asv->cog_position.y = asv->origin_position.y + l * cos(asv->attitude.heading);
  asv->cog_position.z = asv->origin_position.z + asv->spec.cog.z;
}

// Method to set the mass and added mass for the given asv object.
static void set_mass(struct Asv* asv)
{
  // Mass of the ASV
  double mass = asv->spec.disp * SEA_WATER_DENSITY;
  // Roll moment of inertia
  double r_roll = asv->spec.r_roll;
  // Pitch moment of inertia
  double r_pitch = asv->spec.r_pitch;
  // Yaw moment of inertia
  double r_yaw = asv->spec.r_yaw;

  double added_mass_surge = 0.0;
  double added_mass_sway  = 0.0;
  double added_mass_heave = 0.0;
  double added_mass_roll  = 0.0;
  double added_mass_pitch = 0.0;
  double added_mass_yaw   = 0.0;

  // Added mass of the ASV
  // ---------------------
  // ASV shape idealisation:
  // For the purpose of calculating the added mass the shape of the ASV is
  // assumed to be a semi-spheroid, with the transverse cross section as a 
  // semi-circle and the waterline as an ellipse.
  double a = asv->spec.L_wl/2.0;
  // Find b such the volume of the semi-spheroid is equal to the displacement 
  // of the vessel.
  double b = sqrt(((3.0/4.0) * (2.0*asv->spec.disp/(PI * a)))); 

  double e = sqrt(1.0 - pow(b/a, 2.0));
  double alpha_0 = (2.0*(1 - e*e)/(e*e*e)) * (0.5*log10((1+e)/(1-e)) - e);
  double beta_0 = (1.0/(e*e)) - ((1-e*e)/(2*e*e*e)) * log10((1+e)/(1-e));

  // Ref: DNVGL-RP-N103 Table A-2 (page 210)
  // Surge added mass = rho * Ca_axial * disp
  double C_a_axial = 0.0;
  double C_a_lateral = 0.0;
  if(a/b <= 1.0)
  {
    C_a_axial = 0.5;
    C_a_lateral = 0.5;
  }
  else if(a/b > 1.0 && a/b <= 1.5)
  {
    C_a_axial = 0.5 - (0.5 - 0.304)/(0.5)*(a/b - 1.0);
    C_a_lateral = 0.5 + (0.622 - 0.5)/(0.5)*(a/b - 1.0);
  }
  else if(a/b > 1.5 && a/b <= 2.0)
  {
    C_a_axial = 0.304 - (0.304 - 0.210)/(0.5)*(a/b - 1.5);
    C_a_lateral = 0.622 + (0.704 - 0.622)/(0.5)*(a/b - 1.5);
  }
  else if(a/b > 2.0 && a/b <= 2.5)
  {
    C_a_axial = 0.210 - (0.210 - 0.156)/(0.5)*(a/b - 2.0);
    C_a_lateral = 0.704 + (0.762- 0.704)/(0.5)*(a/b - 2.0);
  }
  else if(a/b > 2.5 && a/b <= 4.0)
  {
    C_a_axial = 0.156 - (0.156 - 0.082)/(1.5)*(a/b - 2.5);
    C_a_lateral = 0.762 + (0.86 - 0.762)/(1.5)*(a/b - 2.5);
  }
  else if(a/b > 4.0 && a/b <= 5.0)
  {
    C_a_axial = 0.082 - (0.082 - 0.059)/(1.0)*(a/b - 4.0);
    C_a_lateral = 0.86 + (0.894 - 0.86)/(1.0)*(a/b - 4.0);
  }
  else if(a/b > 5.0 && a/b <= 6.0)
  {
    C_a_axial = 0.059 - (0.059 - 0.045)/(1.0)*(a/b - 5.0);
    C_a_lateral = 0.894 + (0.917 - 0.894)/(1.0)*(a/b - 5.0);
  }
  else if(a/b > 6.0 && a/b <= 7.0)
  {
    C_a_axial = 0.045 - (0.045 - 0.036)/(1.0)*(a/b - 6.0);
    C_a_lateral = 0.917 + (0.933 - 0.917)/(1.0)*(a/b - 6.0);
  }
  else
  {
    C_a_axial = 0.036 - (0.036 - 0.029)/(1.0)*(a/b - 7.0);
    C_a_lateral = 0.933 + (0.945 - 0.933)/(1.0)*(a/b - 7.0);
  }
  added_mass_surge= 0.5 * SEA_WATER_DENSITY * C_a_axial * asv->spec.disp;
  added_mass_sway = 0.5 *SEA_WATER_DENSITY * C_a_lateral * asv->spec.disp;
  added_mass_heave= SEA_WATER_DENSITY * C_a_lateral * asv->spec.disp;
  
  // Added mass for rotational motion for a hemispheroid. 
  // Ref: The complete expression for "added mass" of a rigid body moving in an
  // ideal fluid. Frederick H Imlay. Page 16-17.
  // Note: the formula given in the above reference is for a full spheroid but
  // the shape that is assumed in this implementation is for a hemispheroid.
  // Therefore multiply added mass by 0.5.
  added_mass_roll = 0.0;
  added_mass_pitch = fabs(
                          0.5 *
                          (1.0/5.0) *
                          pow(b*b - a*a, 2.0)*(alpha_0 - beta_0) / 
                          (2.0*(b*b - a*a) + (b*b + a*a)*(beta_0 - alpha_0)) *
                          (4.0/3.0) * PI * SEA_WATER_DENSITY * a * b*b
                          );
  added_mass_yaw = added_mass_pitch;

  asv->dynamics.M[surge] = mass + added_mass_surge;
  asv->dynamics.M[sway]  = mass + added_mass_sway;
  asv->dynamics.M[heave] = mass + added_mass_heave;
  asv->dynamics.M[roll] = mass * r_roll*r_roll + added_mass_roll;
  asv->dynamics.M[pitch] = mass * r_pitch*r_pitch + added_mass_pitch;
  asv->dynamics.M[yaw] = mass * r_yaw*r_yaw + added_mass_yaw;
}

// Method to compute the drag coefficient for an ellipse based on 
// Ref: Recommended practices DNVGL-RP-N103 Modelling and analysis of marine
//      operations. Edition July 2017. Appendix B page-215
// L is the dimension parallel to flow 
// D is the dimension perpendicular to flow
static double get_drag_coefficient_ellipse(double L, double D)
{
  double ratio = D/L;
  if(ratio <= 0.125)
  {
    return 0.22;
  }
  else if (ratio > 0.125 && ratio <= 0.25)
  {
    return 0.22 + (0.3-0.22)/(0.25-0.125)*(ratio - 0.125);
  }
  else if (ratio > 0.25 && ratio <= 0.5)
  {
    return 0.3+(0.6-0.3)/(0.5-0.25)*(ratio - 0.25);
  }
  else if (ratio > 0.5 && ratio <= 1.0)
  {
    return 0.6 + (1.0 - 0.6)/(1.0-0.5)*(ratio - 0.5);
  }
  else
  {
    return 1.0 + (1.6 - 1.0)/(2.0-1.0)*(ratio - 1.0);
  }
}

// Method to set the drag coefficient for the given asv object.
static void set_drag_coefficient(struct Asv* asv)
{
  // Ref: Recommended practices DNVGL-RP-N103 Modelling and analysis of marine
  // operations. Edition July 2017. Appendix B Table B-1, B-2.
  
  // Surge drag coefficient - assuming elliptical waterplane area
  double C_DS = get_drag_coefficient_ellipse(asv->spec.L_wl, asv->spec.B_wl);
  asv->dynamics.C[surge] = 0.5 * SEA_WATER_DENSITY * C_DS * 
                           asv->spec.B_wl * asv->spec.T;
  
  // Sway drag coefficient - assuming elliptical waterplane area
  C_DS = get_drag_coefficient_ellipse(asv->spec.B_wl, asv->spec.L_wl);
  asv->dynamics.C[sway] = 0.5 * SEA_WATER_DENSITY * C_DS * 
                           asv->spec.L_wl * asv->spec.T;
  
  // Heave drag coefficient - consider it as flat plat perpendicular to flow.
  // Ref: Recommended practices DNVGL-RP-N103 Modelling and analysis of marine
  //      operations. Edition July 2017. Appendix B page-213
  // Convert the waterplate to an equivalent rectangle with length equal to 
  // length of ASV and area equal to waterplane area of ASV.
  // Assuming elliptical waterplane 
  double A_wl = PI *(asv->spec.L_wl/2.0)*(asv->spec.B_wl/2.0); // area of 
                                                      // equivalent rectangle.
  double D = A_wl/(asv->spec.L_wl); // width of equivalent rectangle.
  C_DS = 1.9;
  asv->dynamics.C[heave] = 0.5 * SEA_WATER_DENSITY * C_DS * 
                           D * asv->spec.L_wl;

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

static void set_unit_wave_force(struct Asv* asv)
{
  // Assumptions:
  // 1. Assume the underwater volume to be a semi-ellipsoid with centre of
  // buoyancy at the centroid of the volume.
  // 2. Assume the wave pressure as constant along he projected surface of the 
  // hull.
  // 3. All measurements are with respect to body-frame - origin on waterline at 
  // aft end centreline. Positive measurements towards fore, port side and up.
  // 4. Unit wave height = 1 m.
  
  // Dimensions of ellipsoid
  double a = asv->spec.L_wl/ 2.0;
  double b = asv->spec.B_wl/ 2.0;
  double c = asv->spec.T;
  // Distance of centroid of ellipsoid from waterline.
  double z = - 4.0*c/(3.0*PI);

  // Distance of COG from COB
  double BG = fabs((asv->spec.cog.z - asv->spec.T) - z);
  
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

    // Heave force
    // The heave force is maximum when the wave peak of the wave is at midship
    double A_heave = PI*a*b; // Projected waterplane area
    asv->dynamics.F_unit_wave[i][heave] = A_heave * P;

    // Surge force
    double A_surge = 0.5*PI*b*c;;
    asv->dynamics.F_unit_wave[i][surge] = A_surge * P;

    // Sway force
    double A_sway = 0.5*PI*a*c;
    asv->dynamics.F_unit_wave[i][sway] = A_sway * P;

    // roll, pitch and yaw moments assumed as zero
  }
}

// Function to compute the wave force for the current time step.
static void set_wave_force(struct Asv* asv)
{
  // Reset the wave force to all zeros
  for(int k = 0; k < COUNT_DOF; ++k)
  {
    asv->dynamics.F_wave[k] = 0.0;
  }

  // For each wave in the wave spectrum
  for(int i = 0; i < COUNT_WAVE_SPECTRAL_DIRECTIONS; ++i)
  {
    for(int j = 0; j < COUNT_WAVE_SPECTRAL_FREQUENCIES; ++j)
    {
      // Compute the encounter frequency
      double angle = asv->wave.spectrum[i][j].direction - asv->attitude.heading;
      // Better to keep angle +ve
      angle = (angle < 0.0)? 2*PI + angle : angle;
      // Get encounter frequency
      double freq = get_encounter_frequency(asv->wave.spectrum[i][j].frequency,
                                            asv->dynamics.V[surge], angle);

      // Get the index for unit wave force for the encounter frequency
      double nf = COUNT_ASV_SPECTRAL_FREQUENCIES;
      double freq_step_size = (asv->dynamics.F_unit_wave_freq_max - 
                               asv->dynamics.F_unit_wave_freq_min) /
                              (COUNT_ASV_SPECTRAL_FREQUENCIES - 1.0);
      int index = round((freq - asv->dynamics.F_unit_wave_freq_min)/
                         freq_step_size);

      // Compute the scaling factor to compute the wave force from unit wave
      double scale = asv->wave.spectrum[i][j].amplitude * 2.0;

      // Assume the wave force to be have zero phase lag with the wave
      // wave phase at the cog position.
      double phase_cog = regular_wave_get_phase(&(asv->wave.spectrum[i][j]), 
                                                &asv->cog_position, 
                                                asv->dynamics.time); 
      // wave phase at the aft-CL position.
      struct Point point_aft = asv->cog_position;
      point_aft.x -= (asv->spec.L_wl*0.25)*sin(asv->attitude.heading);
      point_aft.y -= (asv->spec.L_wl*0.25)*cos(asv->attitude.heading);
      double phase_aft = regular_wave_get_phase(&(asv->wave.spectrum[i][j]), 
                                                &point_aft, 
                                                asv->dynamics.time);
      // wave phase at the fore-CL position.
      struct Point point_fore = asv->cog_position;
      point_fore.x += (asv->spec.L_wl*0.25)*sin(asv->attitude.heading);
      point_fore.y += (asv->spec.L_wl*0.25)*cos(asv->attitude.heading);
      double phase_fore = regular_wave_get_phase(&(asv->wave.spectrum[i][j]), 
                                                 &point_fore, 
                                                 asv->dynamics.time);
      // wave phase at the mid-PS position.
      struct Point point_ps = asv->cog_position;
      point_ps.x -= (asv->spec.B_wl*0.25)*cos(asv->attitude.heading);
      point_ps.y += (asv->spec.B_wl*0.25)*sin(asv->attitude.heading);
      double phase_ps = regular_wave_get_phase(&(asv->wave.spectrum[i][j]), 
                                               &point_ps, 
                                               asv->dynamics.time);
      // wave phase at the mid-SB position.
      struct Point point_sb = asv->cog_position;
      point_sb.x += (asv->spec.B_wl*0.25)*cos(asv->attitude.heading);
      point_sb.y -= (asv->spec.B_wl*0.25)*sin(asv->attitude.heading);
      double phase_sb = regular_wave_get_phase(&(asv->wave.spectrum[i][j]), 
                                               &point_sb, 
                                               asv->dynamics.time);
      
      // Compute wave force
      asv->dynamics.F_wave[surge] += 
        scale * asv->dynamics.F_unit_wave[index][surge] * 
        (cos(phase_aft) - cos(phase_fore));
      asv->dynamics.F_wave[sway] += 
        scale * asv->dynamics.F_unit_wave[index][sway] * 
        (cos(phase_ps) - cos(phase_sb));
      asv->dynamics.F_wave[heave] += 
        scale * asv->dynamics.F_unit_wave[index][heave] * cos(phase_cog);
      // roll moment = differential_heave_force * lever
      // differential_heave_force = F_heave_ps - F_heave_sb
      // lever = (2/3B) - 0.5B = B/6.0
      asv->dynamics.F_wave[roll] += 
        scale * asv->dynamics.F_unit_wave[index][heave] *
        (cos(phase_ps) - cos(phase_sb)) * asv->spec.B_wl/6.0;
      // pitch moment = differential_heave_force * lever
      // differential_heave_force = F_heave_aft - F_heave_fore
      // lever = (2/3L) - 0.5L = L/6.0
      asv->dynamics.F_wave[pitch] += 
        scale * asv->dynamics.F_unit_wave[index][heave] *
        (cos(phase_aft) - cos(phase_fore)) * asv->spec.L_wl/6.0;
      // yaw moment = 0.0
    }
  } 
}

// Function to calculate the propeller force for the current time step.
static void set_propeller_force(struct Asv* asv)
{
  // Reset the propeller force to 0.
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F_propeller[i] = 0.0;
  }

  // Calculate force from each propeller.
  for(int i = 0; i < asv->count_propellers; ++i)
  {
    double thrust = asv->propellers[i].thrust;
    double trim = asv->propellers[i].orientation.trim;
    double prop_angle = asv->propellers[i].orientation.heading;
    double prop_cog_angle = atan(
        (asv->propellers[i].position.y - asv->spec.cog.y)/
        (asv->spec.cog.x - asv->propellers[i].position.x));
    double thrust_cog_angle = prop_cog_angle + prop_angle;
    
    double F_prop_to_cog = thrust * cos(thrust_cog_angle);
    double F_x = F_prop_to_cog*cos(asv->propellers[i].orientation.trim) *
                               cos(prop_cog_angle);
    double F_y = -F_prop_to_cog*cos(asv->propellers[i].orientation.trim) *
                               sin(prop_cog_angle);
    double F_z = F_prop_to_cog*sin(asv->propellers[i].orientation.trim);
    
    double x = asv->spec.cog.x - asv->propellers[i].position.x;
    double y = asv->spec.cog.y - asv->propellers[i].position.y;
    double z = asv->propellers[i].position.z - asv->spec.cog.z;
 
    double F_perp_to_cog = thrust * sin(thrust_cog_angle);
    double M_x = F_perp_to_cog * 
                 cos(asv->propellers[i].orientation.trim) * 
                 cos(prop_cog_angle) * z + 
                 F_perp_to_cog * sin(asv->propellers[i].orientation.trim) * y;
    double M_y = F_perp_to_cog * 
                 cos(asv->propellers[i].orientation.trim) *
                 sin(prop_cog_angle) * z + 
                 F_perp_to_cog * sin(asv->propellers[i].orientation.trim) * x; 
    double M_z = F_perp_to_cog * sqrt(x*x + y*y);
    if(asv->propellers[i].position.x < asv->spec.cog.x &&
       asv->propellers[i].orientation.heading < PI)
    {
      M_z = -M_z;
    }

    asv->dynamics.F_propeller[surge]  += F_x;
    asv->dynamics.F_propeller[sway]   += F_y;
    asv->dynamics.F_propeller[heave]  += F_z;
    asv->dynamics.F_propeller[roll]   += M_x;
    asv->dynamics.F_propeller[pitch]  += M_y;
    asv->dynamics.F_propeller[yaw]    += M_z;
  }
}

// Function to compute the drag force for the current time step.
static void set_drag_force(struct Asv* asv)
{
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F_drag[i] = -asv->dynamics.C[i] * 
                               asv->dynamics.V[i]*
                               fabs(asv->dynamics.V[i]);
  }
}

static void set_restoring_force(struct Asv* asv)
{
  // Heave restoring force
  // Distance of current COG position from still water floating position.
  double dist = (asv->spec.cog.z -asv->spec.T) - asv->cog_position.z;
  asv->dynamics.F_restoring[heave] = asv->dynamics.K[heave] * dist;
  
  // Roll restoring force 
  asv->dynamics.F_restoring[roll] = -asv->dynamics.K[roll] * asv->attitude.heel;

  // Pitch restoring force
  asv->dynamics.F_restoring[pitch]= -asv->dynamics.K[pitch]* asv->attitude.trim;
  
  // No restoring force for sway, yaw and surge. 
}

static void set_net_force(struct Asv* asv)
{
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F[i] = asv->dynamics.F_wave[i]            
                         + asv->dynamics.F_propeller[i]  
                         + asv->dynamics.F_drag[i]       
                         + asv->dynamics.F_restoring[i];
  }
}

static void set_acceleration(struct Asv* asv)
{
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.A[i] = asv->dynamics.F[i] / asv->dynamics.M[i];
  }
}

static void set_velocity(struct Asv* asv)
{
  // compute the velocity at the end of the time step
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.V[i] += asv->dynamics.A[i] * asv->dynamics.time_step_size; 
  }
}

static void set_deflection(struct Asv* asv)
{
  // compute the deflection at the end of the time step
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.X[i] = asv->dynamics.V[i] * asv->dynamics.time_step_size; 
  }
}

// Compute deflection in global frame and set position of origin and cog.
static void set_position(struct Asv* asv)
{
  double deflection_x = asv->dynamics.X[surge]*sin(asv->attitude.heading) -
                        asv->dynamics.X[sway]*cos(asv->attitude.heading);
  double deflection_y = asv->dynamics.X[surge]*cos(asv->attitude.heading) + 
                        asv->dynamics.X[sway]*sin(asv->attitude.heading);
                        
  double deflection_z = asv->dynamics.X[heave];
  
  // Update origin position 
  asv->cog_position.x += deflection_x;
  asv->cog_position.y += deflection_y;
  asv->cog_position.z += deflection_z;

  // Update origin position
  double l = sqrt(pow(asv->spec.cog.x, 2.0) + pow(asv->spec.cog.y, 2.0));
  asv->origin_position.x = asv->cog_position.x - l * sin(asv->attitude.heading);
  asv->origin_position.y = asv->cog_position.y - l * cos(asv->attitude.heading);
  asv->origin_position.z = asv->cog_position.z - asv->spec.cog.z;
  
}

// Compute the attitude for the current time step
static void set_attitude(struct Asv* asv)
{
  asv->attitude.heading += asv->dynamics.X[yaw];
  asv->attitude.heel    += asv->dynamics.X[roll];
  asv->attitude.trim    += asv->dynamics.X[pitch];
}

void asv_init(struct Asv* asv)
{ 
  // Initialise time record 
  asv->dynamics.time = 0.0;

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
        asv->dynamics.V                             [k] = 0.0;
        asv->dynamics.A                             [k] = 0.0;
  		  asv->dynamics.F                             [k] = 0.0;
  		  asv->dynamics.F_wave                        [k] = 0.0;
  		  asv->dynamics.F_propeller                   [k] = 0.0;
  		  asv->dynamics.F_drag                        [k] = 0.0;
  		  asv->dynamics.F_restoring                   [k] = 0.0;
        asv->dynamics.F_unit_wave                [j][k] = 0.0;
  		}
    }
  }

  // Place the asv vertically in the correct position W.R.T wave
  asv->origin_position.z = 
    wave_get_elevation(&asv->wave, &asv->cog_position, 0.0)
    -asv->spec.T; 
  // Reset the cog position.
  set_cog(asv); // Match the position of the cog with that of origin
  // Set minimum and maximum encounter frequency
  double max_speed_for_spectrum = 2.0 * asv->spec.max_speed;
  asv->dynamics.F_unit_wave_freq_min = get_encounter_frequency(
                                      asv->wave.min_spectral_frequency,
                                      max_speed_for_spectrum, 0);
  asv->dynamics.F_unit_wave_freq_max = get_encounter_frequency(
                                      asv->wave.max_spectral_frequency,
                                      max_speed_for_spectrum, PI);

  // Set the mass matrix
  set_mass(asv);
  // Set the drag coefficient matrix
  set_drag_coefficient(asv);
  // Set the stiffness matrix
  set_stiffness(asv);
  // Set the wave force for unit waves
  set_unit_wave_force(asv);
}

void asv_compute_dynamics(struct Asv* asv, double time)
{
  // Update the time
  asv->dynamics.time = time;

  if(asv->using_waves)
  {
    // Get the wave force for the current time step
    set_wave_force(asv);
  }
  
  // Get the propeller force for the current time step
  set_propeller_force(asv);
  
  // Compute the drag force for the current time step based on velocity reading
  set_drag_force(asv);
  
  // Compute the restoring force for the current time step based on the position
  // reading
  set_restoring_force(asv);

  // Compute the net force for the current time step
  set_net_force(asv);
  
  // Compute the acceleration for the current time step
  set_acceleration(asv);
  
  // Compute the velocity for the current time step
  set_velocity(asv);
  
  // Compute the deflection for the current time step in body-fixed frame
  set_deflection(asv);
   
  // Compute the new attitude
  set_attitude(asv);
  
  // Translate the deflection to global frame and compute the new position.
  set_position(asv);
}

void asv_propeller_init(struct Asv_propeller* propeller,
                        struct Point position)
{
  propeller->position.x = position.x;
  propeller->position.y = position.y;
  propeller->position.z = position.z;
  
  propeller->thrust = 0.0;
}

int asv_set_propeller(struct Asv* asv, struct Asv_propeller propeller)
{
  if(asv->count_propellers == COUNT_PROPELLERS_MAX)
  {
    // Limit reached.
    return 0;
  }
  
  asv->propellers[asv->count_propellers] = propeller;
  ++asv->count_propellers;
  
  return 1;
}
