#include <stdio.h>
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
  asv->cog_position.x = asv->origin_position.x + l * sin(asv->attitude.z);
  asv->cog_position.y = asv->origin_position.y + l * cos(asv->attitude.z);
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
  // assumed to be a elliptic-cylinder.
  double a = asv->spec.L_wl/2.0;
  double b = asv->spec.B_wl/2.0; 
  double c = asv->spec.T;

  // Ref: DNVGL-RP-N103 Table A-1 (page 205)
  // Surge added mass = rho * Ca * Ar
  double C_a = 1.0;
  double Ar_surge = PI*b*b;
  double Ar_sway = PI*a*a;
  double Ar_heave = PI*a*b;
  added_mass_surge= SEA_WATER_DENSITY * C_a * Ar_surge * c;
  added_mass_sway= SEA_WATER_DENSITY * C_a * Ar_sway * c;
  added_mass_heave= SEA_WATER_DENSITY * C_a * Ar_heave * c;

  // Moment of inertia for angular motions
  double I_roll = mass * r_roll * r_roll;
  double I_pitch = mass * r_pitch * r_pitch;
  double I_yaw = mass * r_yaw * r_yaw;
  
  added_mass_roll = SEA_WATER_DENSITY * PI/8.0 * b * (2*a);
  added_mass_pitch = SEA_WATER_DENSITY * PI/8.0 * a * (2*b);
  added_mass_yaw = SEA_WATER_DENSITY * PI/8.0 * (a*a-b*b)*(a*a-b*b);
  added_mass_yaw = (added_mass_yaw > I_yaw) ? I_yaw : added_mass_yaw;
  
  asv->dynamics.M[surge] = mass + added_mass_surge;
  asv->dynamics.M[sway]  = mass + added_mass_sway;
  asv->dynamics.M[heave] = mass + added_mass_heave;
  asv->dynamics.M[roll] = mass * r_roll*r_roll /* + added_mass_roll */;
  asv->dynamics.M[pitch] = mass * r_pitch*r_pitch /*+ added_mass_pitch */;
  asv->dynamics.M[yaw] = mass * r_yaw*r_yaw /* + added_mass_yaw */;
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

static void set_unit_wave_pressure(struct Asv* asv)
{
  double H_w = 1.0; // unit wave height in m.

  // Calculate the pressure for each encounter wave freq
  double freq_step_size = (asv->dynamics.P_unit_wave_freq_max - 
                           asv->dynamics.P_unit_wave_freq_min)/
                          (COUNT_ASV_SPECTRAL_FREQUENCIES - 1); 
  if(asv->wave_type == irregular_wave)
  {
    for(int i = 0; i < COUNT_ASV_SPECTRAL_FREQUENCIES; ++i)
    {
      double freq = asv->dynamics.P_unit_wave_freq_min + i * freq_step_size;
      asv->dynamics.P_unit_wave[i][0] = freq;
      // Create a regular wave for the freq with wave height = 0.01, 
      // direction = 0.0 and phase = 0.0.
      struct Regular_wave wave;
      regular_wave_init(&wave, H_w/2.0, freq, 0.0, 0.0);
    
      // Calculate wave pressure amplitude for the regular wave at the cog depth
      asv->dynamics.P_unit_wave[i][1] = 
        regular_wave_get_pressure_amp(&wave, -asv->spec.T);
    }
  }
  else if(asv->wave_type == regular_wave)
  {
    double freq = asv->regular_wave.frequency;
    struct Regular_wave wave;
    regular_wave_init(&wave, H_w/2.0, freq, 0.0, 0.0);
    asv->dynamics.P_unit_regular_wave = 
      regular_wave_get_pressure_amp(&wave, -asv->spec.T);
  }
}

// Function to compute the wave force for the current time step.
static void set_wave_force(struct Asv* asv)
{
  // Dimensions of ellipsoid
  double a = asv->spec.L_wl/ 2.0;
  double b = asv->spec.B_wl/ 2.0;
  double c = asv->spec.T;
  double A_trans = PI/2.0 * b * c;
  double A_profile = PI/2.0 * a * c;
  double A_waterplane = PI * a * b;

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
      // Regular wave
      struct Regular_wave* wave = 0;
      if(asv->wave_type == regular_wave)
      {
        wave = &(asv->regular_wave);
      }
      if(asv->wave_type == irregular_wave)
      {
        wave = &(asv->wave.spectrum[i][j]);
      }

      // Compute the encounter frequency
      double angle = wave->direction - asv->attitude.z;
      // Better to keep angle +ve
      angle = (angle < 0.0)? 2*PI + angle : angle;
      // Get encounter frequency
      double freq = get_encounter_frequency(wave->frequency,
                                            asv->dynamics.V[surge], angle);
      int index = 0;
      // Compute the scaling factor to compute the wave force from unit wave
      double scale = wave->amplitude * 2.0;
      if(asv->wave_type == irregular_wave)
      {
        // Get the index for unit wave force for the encounter frequency
        double nf = COUNT_ASV_SPECTRAL_FREQUENCIES;
        double freq_step_size = (asv->dynamics.P_unit_wave_freq_max - 
                                 asv->dynamics.P_unit_wave_freq_min) /
                                (COUNT_ASV_SPECTRAL_FREQUENCIES - 1.0);
        index = round((freq - asv->dynamics.P_unit_wave_freq_min)/
                           freq_step_size);
        if(index >= COUNT_ASV_SPECTRAL_FREQUENCIES || index < 0)
        {
          fprintf(stderr, "FATAL ERROR! Array index out of bounds. \n");
          exit(1);
        }
      }

      // Assume the wave force to be have zero phase lag with the wave
      // wave phase at the cog position.
      double phase_cog = regular_wave_get_phase(wave, 
                                                &asv->cog_position, 
                                                asv->dynamics.time); 
      // wave phase at the aft-CL position.
      struct Dimensions point_aft = asv->cog_position;
      point_aft.x -= (a/3.0)*sin(asv->attitude.z);
      point_aft.y -= (a/3.0)*cos(asv->attitude.z);
      point_aft.z = regular_wave_get_elevation(wave, 
                                               &point_aft, 
                                               asv->dynamics.time);
      double phase_aft = regular_wave_get_phase(wave, 
                                                &point_aft, 
                                                asv->dynamics.time);
      // wave phase at the fore-CL position.
      struct Dimensions point_fore = asv->cog_position;
      point_fore.x += (a/3.0)*sin(asv->attitude.z);
      point_fore.y += (a/3.0)*cos(asv->attitude.z);
      point_fore.z = regular_wave_get_elevation(wave, 
                                               &point_fore, 
                                               asv->dynamics.time);
      double phase_fore = regular_wave_get_phase(wave, 
                                                 &point_fore, 
                                                 asv->dynamics.time);
      // wave phase at the mid-PS position.
      struct Dimensions point_ps = asv->cog_position;
      point_ps.x -= (b/3.0)*cos(asv->attitude.z);
      point_ps.y += (b/3.0)*sin(asv->attitude.z);
      point_ps.z = regular_wave_get_elevation(wave, 
                                               &point_ps, 
                                               asv->dynamics.time);
      double phase_ps = regular_wave_get_phase(wave, 
                                               &point_ps, 
                                               asv->dynamics.time);
      // wave phase at the mid-SB position.
      struct Dimensions point_sb = asv->cog_position;
      point_sb.x += (b/3.0)*cos(asv->attitude.z);
      point_sb.y -= (b/3.0)*sin(asv->attitude.z);
      point_sb.z = regular_wave_get_elevation(wave, 
                                               &point_sb, 
                                               asv->dynamics.time);
      double phase_sb = regular_wave_get_phase(wave, 
                                               &point_sb, 
                                               asv->dynamics.time);

      // Compute the difference between the points
      double lever_trans = b / 3.0;
      //double lever_vertical_trans = z - asv->cog_position.z;
      double lever_long = a / 3.0;
      //double lever_vertical_long = z - asv->cog_position.z;
      
      // Compute the pressure difference between fore and aft point
      double P = 0.0;
      if(asv->wave_type == irregular_wave)
      {
        P = asv->dynamics.P_unit_wave[index][1];
      }
      else if(asv->wave_type == regular_wave)
      {
        P = asv->dynamics.P_unit_regular_wave;
      }
      double P_diff_long = P *(cos(phase_fore) - cos(phase_aft));
      // Compute the pressure difference between SB and PS point
      double P_diff_trans = P * (cos(phase_sb) - cos(phase_ps));
      
      // Compute wave force
      asv->dynamics.F_wave[heave] += 
        scale * (P * cos(phase_cog)) * A_waterplane;
      asv->dynamics.F_wave[surge] += scale * P_diff_long * A_trans;
      asv->dynamics.F_wave[sway]  += scale * P_diff_trans * A_profile;
      asv->dynamics.F_wave[roll] += scale * P_diff_trans * A_waterplane * lever_trans;
      asv->dynamics.F_wave[pitch] += scale * P_diff_long * A_waterplane * lever_long;
      asv->dynamics.F_wave[yaw] += scale * P_diff_trans * (A_profile/2.0) * lever_long;
      
      if(asv->wave_type == regular_wave)
        break;
    }
    if(asv->wave_type == regular_wave)
      break;
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
    double trim = asv->propellers[i].orientation.y;
    double prop_angle = asv->propellers[i].orientation.z;
    double prop_cog_angle = atan(
        (asv->propellers[i].position.y - asv->spec.cog.y)/
        (asv->spec.cog.x - asv->propellers[i].position.x));
    double thrust_cog_angle = prop_cog_angle + prop_angle;
    
    double F_prop_to_cog = thrust * cos(thrust_cog_angle);
    double F_x = F_prop_to_cog*cos(asv->propellers[i].orientation.y) *
                               cos(prop_cog_angle);
    double F_y = -F_prop_to_cog*cos(asv->propellers[i].orientation.y) *
                               sin(prop_cog_angle);
    double F_z = F_prop_to_cog*sin(asv->propellers[i].orientation.y);
    
    double x = asv->spec.cog.x - asv->propellers[i].position.x;
    double y = asv->spec.cog.y - asv->propellers[i].position.y;
    double z = asv->propellers[i].position.z - asv->spec.cog.z;
 
    double F_perp_to_cog = thrust * sin(thrust_cog_angle);
    double M_x = F_perp_to_cog * 
                 cos(asv->propellers[i].orientation.y) * 
                 cos(prop_cog_angle) * z + 
                 F_perp_to_cog * sin(asv->propellers[i].orientation.y) * y;
    double M_y = F_perp_to_cog * 
                 cos(asv->propellers[i].orientation.y) *
                 sin(prop_cog_angle) * z + 
                 F_perp_to_cog * sin(asv->propellers[i].orientation.y) * x; 
    double M_z = F_perp_to_cog * sqrt(x*x + y*y);
    if(asv->propellers[i].position.x < asv->spec.cog.x &&
       asv->propellers[i].orientation.z < PI)
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
  asv->dynamics.F_restoring[roll] = -asv->dynamics.K[roll] * asv->attitude.x;

  // Pitch restoring force
  asv->dynamics.F_restoring[pitch]= -asv->dynamics.K[pitch]* asv->attitude.y;
  
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
  double deflection_x = asv->dynamics.X[surge]*sin(asv->attitude.z) -
                        asv->dynamics.X[sway]*cos(asv->attitude.z);
  double deflection_y = asv->dynamics.X[surge]*cos(asv->attitude.z) + 
                        asv->dynamics.X[sway]*sin(asv->attitude.z);
                        
  double deflection_z = asv->dynamics.X[heave];
  
  // Update origin position 
  asv->cog_position.x += deflection_x;
  asv->cog_position.y += deflection_y;
  asv->cog_position.z += deflection_z;

  // Update origin position
  double l = sqrt(pow(asv->spec.cog.x, 2.0) + pow(asv->spec.cog.y, 2.0));
  asv->origin_position.x = asv->cog_position.x - l * sin(asv->attitude.z);
  asv->origin_position.y = asv->cog_position.y - l * cos(asv->attitude.z);
  asv->origin_position.z = asv->cog_position.z - asv->spec.cog.z;
  
}

// Compute the attitude for the current time step
static void set_attitude(struct Asv* asv)
{
  asv->attitude.z += asv->dynamics.X[yaw];
  asv->attitude.x    += asv->dynamics.X[roll];
  asv->attitude.y    += asv->dynamics.X[pitch];
}

void asv_init(struct Asv* asv)
{ 
  // Initialise time record 
  asv->dynamics.time = 0.0;

  // Initialise all the vectors matrices to zero.
  for(int j = 0; j < COUNT_ASV_SPECTRAL_FREQUENCIES; ++j)
  {
    asv->dynamics.P_unit_wave[j][0] = 0.0;
    asv->dynamics.P_unit_wave[j][1] = 0.0;
  }
  for(int k = 0; k < COUNT_DOF; ++k)
  {
  	asv->dynamics.M          [k] = 0.0;
  	asv->dynamics.C          [k] = 0.0;
  	asv->dynamics.K          [k] = 0.0;
  	asv->dynamics.X          [k] = 0.0;
    asv->dynamics.V          [k] = 0.0;
    asv->dynamics.A          [k] = 0.0;
  	asv->dynamics.F          [k] = 0.0;
  	asv->dynamics.F_wave     [k] = 0.0;
  	asv->dynamics.F_propeller[k] = 0.0;
  	asv->dynamics.F_drag     [k] = 0.0;
  	asv->dynamics.F_restoring[k] = 0.0;
  }

  // Place the asv vertically in the correct position W.R.T wave
  asv->origin_position.z = 
    wave_get_elevation(&asv->wave, &asv->origin_position, 0.0) - asv->spec.T; 
  // Reset the cog position.
  set_cog(asv); // Match the position of the cog with that of origin

  // Set minimum and maximum encounter frequency
  double max_speed_for_spectrum = 2.0 * asv->spec.max_speed;
  asv->dynamics.P_unit_wave_freq_min = get_encounter_frequency(
                                      asv->wave.min_spectral_frequency,
                                      max_speed_for_spectrum, 0);
  asv->dynamics.P_unit_wave_freq_max = get_encounter_frequency(
                                      asv->wave.max_spectral_frequency,
                                      max_speed_for_spectrum, PI);

  // Set the mass matrix
  set_mass(asv);
  // Set the drag coefficient matrix
  set_drag_coefficient(asv);
  // Set the stiffness matrix
  set_stiffness(asv);
  // Set the wave force for unit waves
  set_unit_wave_pressure(asv);
}

void asv_compute_dynamics(struct Asv* asv, double time)
{
  // Update the time
  asv->dynamics.time = time;

  if(asv->wave_type == irregular_wave ||
     asv->wave_type == regular_wave)
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
                        struct Dimensions position)
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
