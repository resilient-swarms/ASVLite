#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "asv.h"
#include "sea_surface.h"
#include "regular_wave.h"
#include "errors.h"
#include "constants.h"
#include "geometry.h"

#define COUNT_ASV_SPECTRAL_DIRECTIONS 360  /*!< Number of directions in the wave force spectrum. */
#define COUNT_ASV_SPECTRAL_FREQUENCIES 100 /*!< Number of frequencies in the wave force spectrum. */

/**
 * Struct to hold all the inputs for the thruster.
 */
struct Thruster
{
  union Coordinates_3D position;    //!< Position of thrust vector in ASV's body-fixed frame.
  union Coordinates_3D orientation; //!< Orientation of the thrust vector in body-fixed frame.
  double thrust;    //!< Magnitude of thrust in Newton.
  char* error_msg;  //!< Error message, if any.
};

/**
 * Struct to contain both inputs and outputs of ASV dynamics. All input
 * variables must be set before calling asv_compute_dynamics().
 */
struct Asv_dynamics
{
  // Input
  double time_step_size; //!< Time step size in milliseconds.
  double time; //!< Time since start of simulation in seconds.
  double tuning_factor_thrust; //!< Thrust tuning factor for wave glider.
  union Rigid_body_DOF M; //!< Mass + added mass in Kg. 
  union Rigid_body_DOF C; //!< Drag force coefficients. 
  union Rigid_body_DOF K; //!< Stiffness. 
  
  // Output
  union Rigid_body_DOF X; //!< Deflection in body-fixed frame. 
  union Rigid_body_DOF V; //!< Velocity of ASV in body-fixed frame. 
  union Rigid_body_DOF A; //!< Acceleration of ASV in body-fixed frame. 
  
  union Rigid_body_DOF F; //!< Net force. 
  union Rigid_body_DOF F_wave; //!< Wave force. 
  union Rigid_body_DOF F_thruster; //!< Thruster force. 
  union Rigid_body_DOF F_drag; //!< Quadratic force force. 
  union Rigid_body_DOF F_restoring; //!< Hydrostatic restoring force. 

  double* P_unit_wave; //!< 2D array with wave pressure amplitude. Size is COUNT_ASV_SPECTRAL_FREQUENCIES * 2.
                       //!< Index 0 is wave freq and index 1 gives the corresponding wave pressure amplitude. 
  double P_unit_regular_wave; //!< Pressure amplitude when wave_type is set as regular_wave.
  double P_unit_wave_freq_min;//!< Minimum wave frequency considered in array P_unit_wave.
  double P_unit_wave_freq_max;//!< Maximum wave frequency considered in array P_unit_wave.
  char* error_msg;  //!< Error message, if any.
};

/**
 * Stuct to contain both input and output of ASV motion in waves. 
 */
struct Asv
{
  // Input
  struct Asv_specification spec; //!< ASV specification.
  int count_thrusters;           //!< Number of thrusters attached to the ASV.
  struct Thruster** thrusters;   //!< Array of thrusters.
  struct Sea_surface* sea_surface;       //!< Irregular sea_surface instance. 
  double zonal_velocity;
  double meridional_velocity;
  
  // Initial used for input but later contains results. 
  union Coordinates_3D origin_position; //!< Position of the origin of the body-fixed frame in 
                                        //!< the global frame for the current time step.
  union Coordinates_3D attitude; //!< The heel and trim are in body-fixed 
                                 //!< frame and the heading is in global frame.
  
  // Output
  struct Asv_dynamics dynamics; //!< ASV dynamics variables. 
  union Coordinates_3D cog_position; //!< Position of the centre of gravity of the ASV in  
                                     //!< the global frame for the current time step.
  char* error_msg;  //!< Error message, if any.
};

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
  double l = sqrt(asv->spec.cog.keys.x*asv->spec.cog.keys.x + asv->spec.cog.keys.y*asv->spec.cog.keys.y);
  asv->cog_position.keys.x = asv->origin_position.keys.x + l * sin(asv->attitude.keys.z);
  asv->cog_position.keys.y = asv->origin_position.keys.y + l * cos(asv->attitude.keys.z);
  asv->cog_position.keys.z = asv->origin_position.keys.z + asv->spec.cog.keys.z;
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
  added_mass_surge= SEA_WATER_DENSITY * C_a * Ar_surge * (2.0*a);
  added_mass_sway= SEA_WATER_DENSITY * C_a * Ar_sway * (2.0*b);
  added_mass_heave= SEA_WATER_DENSITY * C_a * Ar_heave * c;

  // Moment of inertia for angular motions
  double I_roll = mass * r_roll * r_roll;
  double I_pitch = mass * r_pitch * r_pitch;
  double I_yaw = mass * r_yaw * r_yaw;
  
  added_mass_roll = SEA_WATER_DENSITY * PI/8.0 * b * (2*a);
  added_mass_pitch = SEA_WATER_DENSITY * PI/8.0 * a * (2*b);
  added_mass_yaw = SEA_WATER_DENSITY * PI/8.0 * (a*a-b*b)*(a*a-b*b);
  added_mass_yaw = (added_mass_yaw > I_yaw) ? I_yaw : added_mass_yaw;
  
  asv->dynamics.M.keys.surge = mass + added_mass_surge;
  asv->dynamics.M.keys.sway  = mass + added_mass_sway;
  asv->dynamics.M.keys.heave = mass + added_mass_heave;
  asv->dynamics.M.keys.roll = mass * r_roll*r_roll /* + added_mass_roll */;
  asv->dynamics.M.keys.pitch = mass * r_pitch*r_pitch /*+ added_mass_pitch */;
  asv->dynamics.M.keys.yaw = mass * r_yaw*r_yaw /* + added_mass_yaw */;
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
  // double C_DS = get_drag_coefficient_ellipse(asv->spec.L_wl, asv->spec.B_wl);
  double C_DS = 1.9;
  asv->dynamics.C.keys.surge = 0.5 * SEA_WATER_DENSITY * C_DS * asv->spec.B_wl * asv->spec.T;
  
  // Sway drag coefficient - assuming elliptical waterplane area
  // C_DS = get_drag_coefficient_ellipse(asv->spec.B_wl, asv->spec.L_wl);
  asv->dynamics.C.keys.sway = 0.5 * SEA_WATER_DENSITY * C_DS * asv->spec.L_wl * asv->spec.T;
  
  // Heave drag coefficient - consider it as flat plat perpendicular to flow.
  // C_DS = 1.9;
  asv->dynamics.C.keys.heave = 0.5 * SEA_WATER_DENSITY * C_DS * asv->spec.L_wl * asv->spec.B_wl;

  // roll, pitch and yaw drag coefficient set equal to roll damping coefficient 
  // given in Handbook of Marin Craft Hydrodynamics and motion control, page 125
  
  // Use these for SMARTY
  // asv->dynamics.C.keys.roll = asv->dynamics.C.keys.pitch = asv->dynamics.C.keys.yaw = 0.075; 

  // Else, use these for wave glider. 
  asv->dynamics.C.keys.roll = asv->dynamics.C.keys.pitch = asv->dynamics.C.keys.yaw = asv->dynamics.C.keys.heave;
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
  asv->dynamics.K.keys.heave = A * SEA_WATER_DENSITY * G;

  // Roll stiffness
  // Using the same formula as mentioned for pitch in below ref.
  // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
  asv->dynamics.K.keys.roll = I_xx * SEA_WATER_DENSITY * G;

  // Pitch stiffness
  // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
  asv->dynamics.K.keys.pitch = I_yy * SEA_WATER_DENSITY * G;
}

static void set_unit_wave_pressure(struct Asv* asv)
{
  if(asv->sea_surface)
  {
    const char* error_computation_failed = "Unit pressure computation failed.";
    double H_w = 1.0; // unit wave height in m.

    // Calculate the pressure for each encounter wave freq
    double freq_step_size = (asv->dynamics.P_unit_wave_freq_max - 
                            asv->dynamics.P_unit_wave_freq_min)/
                            (COUNT_ASV_SPECTRAL_FREQUENCIES - 1); 

    for(int i = 0; i < COUNT_ASV_SPECTRAL_FREQUENCIES; ++i)
    {
      double freq = asv->dynamics.P_unit_wave_freq_min + (i * freq_step_size);
      asv->dynamics.P_unit_wave[i*2 + 0] = freq;
      // Create a regular wave for the freq with wave height = 0.01, 
      // direction = 0.0 and phase = 0.0.
      struct Regular_wave* wave = regular_wave_new(H_w/2.0, freq, 0.0, 0.0);
      if(wave)
      {
        // Calculate wave pressure amplitude for the regular wave at the cog depth
        double P_unit_wave = regular_wave_get_pressure_amp(wave, asv->spec.T);
        const char* error_msg = regular_wave_get_error_msg(wave);
        if(error_msg)
        {
          set_error_msg(&asv->error_msg, error_computation_failed);
          regular_wave_delete(wave);
          return;
        }
        asv->dynamics.P_unit_wave[i*2 + 1] = P_unit_wave;
        regular_wave_delete(wave);
      }
      else
      {
        set_error_msg(&asv->error_msg, error_computation_failed);
        return;
      }
    }
  }
}

// Function to compute the wave force for the current time step.
static void set_wave_force(struct Asv* asv)
{
  if(asv->sea_surface)
  {
    const char* error_computation_failed = "Wave force computation failed.";

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
      asv->dynamics.F_wave.array[k] = 0.0;
    }

    // For each wave in the wave spectrum
    int count_waves  = sea_surface_get_count_component_waves(asv->sea_surface); 
    for(int i = 0; i < count_waves; ++i)
    {
      // Regular wave
      const struct Regular_wave* wave = sea_surface_get_regular_wave_at(asv->sea_surface, i);

      // Compute the encounter frequency
      double wave_direction = regular_wave_get_direction(wave);
      double angle = normalise_angle_2PI(wave_direction - asv->attitude.keys.z);
      // Get encounter frequency
      double wave_frequency = regular_wave_get_frequency(wave);
      double freq = get_encounter_frequency(wave_frequency, asv->dynamics.V.keys.surge, angle);
      int index = 0;
      // Compute the scaling factor to compute the wave force from unit wave
      double wave_amplitude = regular_wave_get_amplitude(wave);
      double scale = (wave_amplitude * 2.0 < asv->spec.D)? (wave_amplitude * 2.0) : asv->spec.D;
      scale = scale / count_waves;

      // Get the index for unit wave force for the encounter frequency
      double nf = COUNT_ASV_SPECTRAL_FREQUENCIES;
      double freq_step_size = (asv->dynamics.P_unit_wave_freq_max - 
                                asv->dynamics.P_unit_wave_freq_min) /
                              (COUNT_ASV_SPECTRAL_FREQUENCIES - 1.0);
      index = round((freq - asv->dynamics.P_unit_wave_freq_min)/
                          freq_step_size);
      if(index >= COUNT_ASV_SPECTRAL_FREQUENCIES || index < 0)
      {
        set_error_msg(&asv->error_msg, error_computation_failed);
        return;

        // fprintf(stderr, "FATAL ERROR! Array index out of bounds.\n");
        // fprintf(stderr, "array index = %i \n", index);
        // fprintf(stderr, "V[surge] = %f \n", asv->dynamics.V.keys.surge);
        // fprintf(stderr, "F_thruster[surge] = %f \n", asv->dynamics.F_thruster.keys.surge);
        // fprintf(stderr, "encounter freq = %f \n", freq);
        // fprintf(stderr, "freq_step_size = %f \n", freq_step_size);
        // fprintf(stderr, "P_unit_wave_freq_min = %f \n", asv->dynamics.P_unit_wave_freq_min);
        // fprintf(stderr, "wave->frequency = %f \n", wave_frequency);
        // fprintf(stderr, "angle = %f \n", angle);
        // fprintf(stderr, "wave->direction = %f \n", wave_direction);
        // fprintf(stderr, "asv->attitude.z = %f \n", asv->attitude.keys.z);
        // exit(1);
      }

      // Assume the wave force to be have zero phase lag with the wave
      // wave phase at the cog position.
      double phase_cog = regular_wave_get_phase(wave, asv->cog_position, asv->dynamics.time); 
      // wave phase at the aft-CL position.
      union Coordinates_3D point_aft = asv->cog_position;
      point_aft.keys.x -= (a/4.0)*sin(asv->attitude.keys.z);
      point_aft.keys.y -= (a/4.0)*cos(asv->attitude.keys.z);
      double phase_aft = regular_wave_get_phase(wave, point_aft, asv->dynamics.time);
      // wave phase at the fore-CL position.
      union Coordinates_3D point_fore = asv->cog_position;
      point_fore.keys.x += (a/4.0)*sin(asv->attitude.keys.z);
      point_fore.keys.y += (a/4.0)*cos(asv->attitude.keys.z);
      double phase_fore = regular_wave_get_phase(wave, point_fore, asv->dynamics.time);
      // wave phase at the mid-PS position.
      union Coordinates_3D point_ps = asv->cog_position;
      point_ps.keys.x -= (b/4.0)*cos(asv->attitude.keys.z);
      point_ps.keys.y += (b/4.0)*sin(asv->attitude.keys.z);
      double phase_ps = regular_wave_get_phase(wave, point_ps, asv->dynamics.time);
      // wave phase at the mid-SB position.
      union Coordinates_3D point_sb = asv->cog_position;
      point_sb.keys.x += (b/4.0)*cos(asv->attitude.keys.z);
      point_sb.keys.y -= (b/4.0)*sin(asv->attitude.keys.z);
      double phase_sb = regular_wave_get_phase(wave, point_sb, asv->dynamics.time);

      // Compute the difference between the points
      double lever_trans = b / 4.0;
      //double lever_vertical_trans = z - asv->cog_position.z;
      double lever_long = a / 4.0;
      //double lever_vertical_long = z - asv->cog_position.z;
      
      // Compute the pressure difference between fore and aft point
      double P = asv->dynamics.P_unit_wave[index*2 + 1];

      double P_diff_long = P *(cos(phase_fore) - cos(phase_aft));
      // Compute the pressure difference between SB and PS point
      double P_diff_trans = P * (cos(phase_sb) - cos(phase_ps));
      
      // Compute wave force
      asv->dynamics.F_wave.keys.heave += scale * (P * cos(phase_cog)) * A_waterplane;
      asv->dynamics.F_wave.keys.surge += scale * P_diff_long * A_trans;
      asv->dynamics.F_wave.keys.sway  += scale * P_diff_trans * A_profile;
      asv->dynamics.F_wave.keys.roll  += scale * P_diff_trans * (A_waterplane/2.0) * lever_trans;
      asv->dynamics.F_wave.keys.pitch += scale * P_diff_long * (A_waterplane/2.0) * lever_long;
      asv->dynamics.F_wave.keys.yaw   += scale * P_diff_long * (A_profile/2.0) * lever_long * 0.0; // <-- CONSTRAIN YAW MOTION. 
    } 
  }
  else
  {
    // Still water simulation.
    asv->dynamics.F_wave.keys.heave = 0.0;
    asv->dynamics.F_wave.keys.surge = 0.0;
    asv->dynamics.F_wave.keys.sway  = 0.0;
    asv->dynamics.F_wave.keys.roll  = 0.0;
    asv->dynamics.F_wave.keys.pitch = 0.0;
    asv->dynamics.F_wave.keys.yaw   = 0.0;
  }
}

// Function to calculate the thruster force for the current time step.
static void set_thruster_force(struct Asv* asv)
{
  // Reset the thruster force to 0.
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F_thruster.array[i] = 0.0;
  }

  // Calculate force from each thruster.
  for(int i = 0; i < asv->count_thrusters; ++i)
  {
    double thrust = asv->thrusters[i]->thrust;
    double trim = asv->thrusters[i]->orientation.keys.y;
    double prop_angle = asv->thrusters[i]->orientation.keys.z;
    
    double F_x =  thrust * cos(asv->thrusters[i]->orientation.keys.z);
    double F_y =  thrust * sin(asv->thrusters[i]->orientation.keys.z);
    double F_z =  thrust * sin(asv->thrusters[i]->orientation.keys.y);
    
    double x = asv->spec.cog.keys.x - asv->thrusters[i]->position.keys.x;
    double y = asv->spec.cog.keys.y - asv->thrusters[i]->position.keys.y;
    double z = asv->thrusters[i]->position.keys.z - asv->spec.cog.keys.z;
 
    double M_x = F_y * z + F_z * y;
    double M_y = F_x * z + F_z * x; 
    double M_z = F_x * y + F_y * x;

    asv->dynamics.F_thruster.keys.surge  += F_x;
    asv->dynamics.F_thruster.keys.sway   += F_y;
    asv->dynamics.F_thruster.keys.heave  += F_z;
    asv->dynamics.F_thruster.keys.roll   += M_x;
    asv->dynamics.F_thruster.keys.pitch  += M_y;
    asv->dynamics.F_thruster.keys.yaw    += M_z;
  }
}

// Function to calculate the thruster force for the current time step.
static void set_wave_glider_thrust(struct Asv* asv, double rudder_angle)
{
  // Reset the thruster force to 0.
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F_thruster.array[i] = 0.0;
  }

  // Ref: Dynamic modeling and simulations of the wave glider, Peng Wang, Xinliang Tian, Wenyue Lu, Zhihuan Hu, Yong Luo

  // Compute the thrust generated by the glider
  // Glider details:
  // Number of hydrofoils = 6
  // Area of one hydrofoil (A) = 0.113 m2
  // Angle of attack (alpha_k) = 18 deg
  // Aspect ration (lambda) = 2
  // Cross flow damping coefficient (C_DC) = 0.6
  // 1/4 angle of sweepback (chi) = 7 deg
  // Lift force from one hydrofoil (F_L) = 0.5 * rho * C_L * A * V^2
  // where: 
  // C_L = (1.8 * PI * lambda * alpha_k) / (cos(chi) * sqrt(lambda^2/cos^4(chi) + 4) + 1.8) + (C_DC * alpha_k^2 / lambda)
  // V = heave velocity
  int count_hydrofoils = 6;
  double A = 0.113; // m2
  double alpha_k = 18.0 * PI/180.0; // radians
  double alpha_f = 45.0 * PI/180.0; // radians
  double chi = 7.0 * PI/180.0; // radian
  double lambda = 2.0;
  double C_DC = 0.6;
  double C_DO = 0.008;
  double C_L = (1.8 * PI * lambda * alpha_k) / (cos(chi) * sqrt(lambda*lambda/pow(cos(chi), 4) + 4) + 1.8) + (C_DC/ lambda * alpha_k*alpha_k);
  double C_D = C_DO + C_L*C_L / (0.9 * PI * lambda);
  double V = asv->dynamics.V.keys.heave;
  double F_L = 0.5 * SEA_WATER_DENSITY * C_L * A * V * V;
  double F_D = 0.5 * SEA_WATER_DENSITY * C_D * A * V * V;
  double thrust_per_hydrofoil = F_L * sin(alpha_f) - F_D * cos(alpha_f);
  double thrust = count_hydrofoils * thrust_per_hydrofoil;
  asv->dynamics.F_thruster.keys.surge = asv->dynamics.tuning_factor_thrust * thrust;

  // Compute the yaw moment generated by the rudder
  // Assuming the rudder area = area of a hydrofoil
  alpha_f = fabs(rudder_angle); // radians
  V = asv->dynamics.V.keys.surge;
  C_L = (1.8 * PI * lambda * alpha_k) / (cos(chi) * sqrt(lambda*lambda/pow(cos(chi), 4) + 4) + 1.8) + (C_DC/ lambda * alpha_k*alpha_k);
  double F_L_rudder = 0.5 * SEA_WATER_DENSITY * C_L * A * V * V;
  double yaw_moment = F_L_rudder * sin(alpha_f) * asv->spec.L_wl/2.0;
  yaw_moment = (rudder_angle < 0.0)? -yaw_moment: yaw_moment;
  asv->dynamics.F_thruster.keys.yaw = yaw_moment;
}

// Function to compute the drag force for the current time step.
static void set_drag_force(struct Asv* asv)
{
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F_drag.array[i] = -asv->dynamics.C.array[i] * asv->dynamics.V.array[i] * fabs(asv->dynamics.V.array[i]);
  }
}

static void set_restoring_force(struct Asv* asv)
{
  // Heave restoring force
  // Distance of current COG position from still water floating position.
  double surface_elevation = sea_surface_get_elevation(asv->sea_surface, asv->cog_position, asv->dynamics.time);
  double still_water_cog = asv->spec.cog.keys.z;
  double current_relative_cog = asv->cog_position.keys.z - surface_elevation;
  double dist = still_water_cog - current_relative_cog;
  if(dist > asv->spec.D)
  {
    dist = asv->spec.D;
  }
  if(dist < -asv->spec.D)
  {
    dist = -asv->spec.D;
  }
  asv->dynamics.F_restoring.keys.heave = asv->dynamics.K.keys.heave * dist;
  
  // Roll restoring force 
  asv->dynamics.F_restoring.keys.roll = -asv->dynamics.K.keys.roll * asv->attitude.keys.x;

  // Pitch restoring force
  asv->dynamics.F_restoring.keys.pitch = -asv->dynamics.K.keys.pitch * asv->attitude.keys.y;
  
  // No restoring force for sway, yaw and surge. 
}

static void set_net_force(struct Asv* asv)
{
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.F.array[i] = asv->dynamics.F_wave.array[i]            
                             + asv->dynamics.F_thruster.array[i]  
                             + asv->dynamics.F_drag.array[i]       
                             + asv->dynamics.F_restoring.array[i];
  }
}

static void set_acceleration(struct Asv* asv)
{
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.A.array[i] = asv->dynamics.F.array[i] / asv->dynamics.M.array[i];
  }
}

static void set_velocity(struct Asv* asv)
{
  // compute the velocity at the end of the time step
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.V.array[i] += asv->dynamics.A.array[i] * asv->dynamics.time_step_size/1000.0; 
  }
}

static void set_deflection(struct Asv* asv)
{
  // compute the deflection at the end of the time step
  for(int i = 0; i < COUNT_DOF; ++i)
  {
    asv->dynamics.X.array[i] = asv->dynamics.V.array[i] * asv->dynamics.time_step_size/1000.0; 
  }
}

// Compute deflection in global frame and set position of origin and cog.
static void set_position(struct Asv* asv)
{
  double deflection_x = asv->dynamics.X.keys.surge * sin(asv->attitude.keys.z) -
                        asv->dynamics.X.keys.sway  * cos(asv->attitude.keys.z) + 
                        asv->zonal_velocity * asv->dynamics.time_step_size/1000.0;
  double deflection_y = asv->dynamics.X.keys.surge * cos(asv->attitude.keys.z) + 
                        asv->dynamics.X.keys.sway  * sin(asv->attitude.keys.z) +
                        asv->meridional_velocity * asv->dynamics.time_step_size/1000.0;              
  double deflection_z = asv->dynamics.X.keys.heave;
  
  // Update cog position 
  asv->cog_position.keys.x += deflection_x;
  asv->cog_position.keys.y += deflection_y;
  asv->cog_position.keys.z += deflection_z;

  // Update origin position
  double l = sqrt(pow(asv->spec.cog.keys.x, 2.0) + pow(asv->spec.cog.keys.y, 2.0));
  asv->origin_position.keys.x = asv->cog_position.keys.x - l * sin(asv->attitude.keys.z);
  asv->origin_position.keys.y = asv->cog_position.keys.y - l * cos(asv->attitude.keys.z);
  asv->origin_position.keys.z = asv->cog_position.keys.z - asv->spec.cog.keys.z;
  
}

// Compute the attitude for the current time step
static void set_attitude(struct Asv* asv)
{
  asv->attitude.keys.z += asv->dynamics.X.keys.yaw;
  asv->attitude.keys.z = normalise_angle_2PI(asv->attitude.keys.z);
  asv->attitude.keys.x += asv->dynamics.X.keys.roll;
  asv->attitude.keys.y += asv->dynamics.X.keys.pitch;
}

struct Thruster* thruster_new(const union Coordinates_3D position)
{
  struct Thruster* thruster = NULL;
  if(thruster = (struct Thruster*)malloc(sizeof(struct Thruster)))
  {
    thruster->error_msg = NULL;
    thruster->position = position;
    for(int i = 0; i < COUNT_COORDINATES; ++i)
    {
      thruster->orientation.array[i] = 0.0;
    }
    thruster->thrust = 0.0;
  }

  return thruster;
}

void thruster_delete(struct Thruster* thruster)
{
  if(thruster)
  {
    free(thruster->error_msg);
    free(thruster);
    thruster = NULL;
  }
}

const char* thruster_get_error_msg(const struct Thruster* thruster)
{
  if(thruster)
  {
    return thruster->error_msg;
  }
  else
  {
    return NULL;
  }
}

void thruster_set_thrust(struct Thruster* thruster, const union Coordinates_3D orientation, double magnitude)
{
  if(thruster)
  {
    clear_error_msg(&thruster->error_msg);
    thruster->orientation = orientation;
    thruster->orientation.keys.z = normalise_angle_2PI(thruster->orientation.keys.z); 
    thruster->thrust = magnitude;
  }
  else
  {
    set_error_msg(&thruster->error_msg, error_null_pointer);
  }
} 

struct Asv* asv_new(const struct Asv_specification specification, 
                    const struct Sea_surface* sea_surface, 
                    union Coordinates_3D position, 
                    union Coordinates_3D attitude)
{
  struct Asv* asv = NULL;
  if(asv = (struct Asv*)malloc(sizeof(struct Asv)))
  {
    asv->error_msg = NULL;
    asv->spec = specification;
    // Thrusters set to null
    asv->thrusters = NULL;
    asv->count_thrusters = 0;
    // Initialise time record 
    asv->dynamics.time = 0.0;

    // set the sea_surface for the ASV
    asv->sea_surface = (struct Sea_surface*)sea_surface;
    asv->zonal_velocity = 0.0;
    asv->meridional_velocity = 0.0;

    // Initialise all the vectors matrices to zero.
    if(asv->dynamics.P_unit_wave = (double*)malloc(sizeof(double) * COUNT_ASV_SPECTRAL_FREQUENCIES * 2))
    {
      for(int i = 0; i < COUNT_ASV_SPECTRAL_FREQUENCIES; ++i)
      {
        for(int j = 0; j < 2; ++j)
        {
          asv->dynamics.P_unit_wave[i*2 + j] = 0.0;
        }
      }
    }
    else
    {
      // Memory allocation failed.
      free(asv);
      return NULL;
    }
    
    asv->dynamics.tuning_factor_thrust = 1.0;
    for(int k = 0; k < COUNT_DOF; ++k)
    {
      asv->dynamics.M.array          [k] = 0.0;
      asv->dynamics.C.array          [k] = 0.0;
      asv->dynamics.K.array          [k] = 0.0;
      asv->dynamics.X.array          [k] = 0.0;
      asv->dynamics.V.array          [k] = 0.0;
      asv->dynamics.A.array          [k] = 0.0;
      asv->dynamics.F.array          [k] = 0.0;
      asv->dynamics.F_wave.array     [k] = 0.0;
      asv->dynamics.F_thruster.array[k] = 0.0;
      asv->dynamics.F_drag.array     [k] = 0.0;
      asv->dynamics.F_restoring.array[k] = 0.0;
    }

    asv->attitude = attitude;
    asv->attitude.keys.z = normalise_angle_2PI(asv->attitude.keys.z);
    asv->origin_position = position;
    if(asv->sea_surface)
    {
      // Place the asv vertically in the correct position W.R.T sea_surface
      asv->origin_position.keys.z = sea_surface_get_elevation(asv->sea_surface, asv->origin_position, asv->dynamics.time) - asv->spec.T; 
      const char* error_msg = sea_surface_get_error_msg(asv->sea_surface);
      if(error_msg)
      {
        asv_delete(asv);
        return NULL;
      }

      // Set minimum and maximum encounter frequency
      double max_speed_for_spectrum = 2.0 * asv->spec.max_speed;
      double wave_min_spectral_frequency = sea_surface_get_min_spectral_frequency(asv->sea_surface); 
      double wave_max_spectral_frequency = sea_surface_get_max_spectral_frequency(asv->sea_surface); 
      asv->dynamics.P_unit_wave_freq_min = get_encounter_frequency(wave_min_spectral_frequency, max_speed_for_spectrum, 0);
      asv->dynamics.P_unit_wave_freq_max = get_encounter_frequency(wave_max_spectral_frequency, max_speed_for_spectrum, PI);
      // Set the wave force for unit waves
      set_unit_wave_pressure(asv);
      error_msg = asv_get_error_msg(asv);
      if(error_msg)
      {
        asv_delete(asv);
        return NULL;
      }
    }
    // Set the cog position.
    set_cog(asv); // Match the position of the cog with that of origin
    
    // Set the mass matrix
    set_mass(asv);
    // Set the drag coefficient matrix
    set_drag_coefficient(asv);
    // Set the stiffness matrix
    set_stiffness(asv);
    return asv;
  }
  else
  {
    // Memory allocation failed.
    return NULL;
  }
}

void asv_set_sea_state(struct Asv* asv, const struct Sea_surface* sea_surface)
{ 
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    const struct Sea_surface* old_sea_surface = asv->sea_surface;
    // set the sea_surface for the ASV
    asv->sea_surface = (struct Sea_surface*)sea_surface;

    // Initialise all the vectors matrices to zero.
    for(int i = 0; i < COUNT_ASV_SPECTRAL_FREQUENCIES; ++i)
    {
      for(int j = 0; j < 2; ++j)
      {
        asv->dynamics.P_unit_wave[i*2 + j] = 0.0;
      }
    }

    if(asv->sea_surface)
    {
      // Place the asv vertically in the correct position W.R.T sea_surface
      asv->origin_position.keys.z = sea_surface_get_elevation(asv->sea_surface, asv->origin_position, asv->dynamics.time) - asv->spec.T; 
      const char* error_msg = sea_surface_get_error_msg(asv->sea_surface);
      if(error_msg)
      {
        set_error_msg(&asv->error_msg, error_msg);
        asv_set_sea_state(asv, old_sea_surface);
        return;
      }

      // Set minimum and maximum encounter frequency
      double max_speed_for_spectrum = 2.0 * asv->spec.max_speed;
      double wave_min_spectral_frequency = sea_surface_get_min_spectral_frequency(asv->sea_surface); 
      double wave_max_spectral_frequency = sea_surface_get_max_spectral_frequency(asv->sea_surface); 
      asv->dynamics.P_unit_wave_freq_min = get_encounter_frequency(wave_min_spectral_frequency, max_speed_for_spectrum, 0);
      asv->dynamics.P_unit_wave_freq_max = get_encounter_frequency(wave_max_spectral_frequency, max_speed_for_spectrum, PI);
      
      // Set the wave force for unit waves
      set_unit_wave_pressure(asv);
      if(asv->error_msg)
      {
        // Some error occurred in the call to set_unit_wave_pressure.
        asv_set_sea_state(asv, old_sea_surface);
        return;
      }
    }
    // Set the cog position.
    set_cog(asv); // Match the position of the cog with that of origin
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

void asv_set_ocean_current(struct Asv* asv, double zonal_velocity, double meridional_velocity)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    asv->zonal_velocity = zonal_velocity;
    asv->meridional_velocity = meridional_velocity;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

void asv_delete(struct Asv* asv)
{
  if(asv)
  {
    free(asv->error_msg);
    free(asv->dynamics.P_unit_wave);
    free(asv->thrusters);
    free(asv);
    asv = NULL;
  }
}

const char* asv_get_error_msg(const struct Asv* asv)
{
  if(asv)
  {
    return asv->error_msg;
  }
  else
  {
    return NULL;
  }
}

void asv_set_thrusters(struct Asv* asv, struct Thruster** thrusters, int count_thrusters)
{
  if(asv && thrusters)
  {
    clear_error_msg(&asv->error_msg);
    // Need more memory
    free(asv->thrusters);
    if(asv->thrusters = (struct Thruster**)malloc(sizeof(struct Thruster*) * count_thrusters))
    {
      for(int i = 0; i<count_thrusters; ++i)
      {
        asv->thrusters[i] = thrusters[i];
      }
      asv->count_thrusters = count_thrusters;
    }
    else
    {
      set_error_msg(&asv->error_msg, error_malloc_failed);
    }
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

struct Thruster** asv_get_thrusters(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->thrusters;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return NULL;
  }
}

const union Coordinates_3D thruster_get_position(struct Thruster* thruster)
{
  if(thruster)
  {
    clear_error_msg(&thruster->error_msg);
    return thruster->position;
  }
  else
  {
    set_error_msg(&thruster->error_msg, error_null_pointer);
    return (union Coordinates_3D){0.0, 0.0, 0.0};
  }
}

int asv_get_count_thrusters(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->count_thrusters;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return 0;
  }
}

struct Sea_surface* asv_get_sea_surface(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->sea_surface;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return NULL;
  }
}

union Coordinates_3D asv_get_position_cog(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->cog_position;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return (union Coordinates_3D){0.0, 0.0, 0.0};
  }
}

union Coordinates_3D asv_get_position_origin(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->origin_position;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return (union Coordinates_3D){0.0, 0.0, 0.0};
  }
}

union Coordinates_3D asv_get_attitude(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->attitude;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return (union Coordinates_3D){0.0, 0.0, 0.0};
  }
}

union Rigid_body_DOF asv_get_F(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->dynamics.F;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return (union Rigid_body_DOF){0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
}

union Rigid_body_DOF asv_get_A(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->dynamics.A;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return (union Rigid_body_DOF){0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
}

union Rigid_body_DOF asv_get_V(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->dynamics.V;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
    return (union Rigid_body_DOF){0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
}

struct Asv_specification asv_get_spec(struct Asv* asv)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    return asv->spec;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

void wave_glider_set_thrust_tuning_factor(struct Asv* asv, double tuning_factor)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    asv->dynamics.tuning_factor_thrust = tuning_factor;
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

static void set_dynamics(struct Asv* asv, bool is_wave_glider, double rudder_angle, double time_step_size)
{
  if(asv)
  {
    double old_time_step_size = asv->dynamics.time_step_size;
    double old_time = asv->dynamics.time;
    // Update the time
    asv->dynamics.time_step_size = time_step_size; // milliseconds
    asv->dynamics.time += time_step_size/1000.0; // seconds

    // Get the wave force for the current time step
    set_wave_force(asv);
    if(asv->error_msg)
    {
      // Some error occurred.
      // Reset time
      asv->dynamics.time_step_size = old_time_step_size; // milliseconds
      asv->dynamics.time = old_time; // seconds
      return;
    }
    
    // Get the thruster force for the current time step
    if(is_wave_glider)
    {
      set_wave_glider_thrust(asv, rudder_angle);
    }
    else
    {
      set_thruster_force(asv);
    }
    
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
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

void asv_compute_dynamics(struct Asv* asv, double time_step_size)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    if(time_step_size)
    {
      set_dynamics(asv, false, 0.0, time_step_size);
    }
  }
  
}

void wave_glider_compute_dynamics(struct Asv* asv, double rudder_angle, double time_step_size)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    if(rudder_angle >= -PI/2.0 && rudder_angle <= PI/2.0)
    {
      if(time_step_size)
      {
        set_dynamics(asv, true, rudder_angle, time_step_size);
      }
    }
    else
    {
      // Rudder angle out of range. 
      set_error_msg(&asv->error_msg, error_incorrect_rudder_angle);
    }
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}

void wave_glider_run(struct Asv* asv, bool(*callback_precompute)(double*), void(*callback_postcompute)(void), double time_step_size)
{
  if(asv)
  {
    clear_error_msg(&asv->error_msg);
    double rudder_angle = 0.0;
    while(callback_precompute(&rudder_angle))
    {
      wave_glider_compute_dynamics(asv, rudder_angle, time_step_size);
      callback_postcompute();
      if(asv->error_msg)
      {
        // Some error occurred.
        return;
      }
    }
  }
  else
  {
    set_error_msg(&asv->error_msg, error_null_pointer);
  }
}