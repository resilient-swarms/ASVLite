#include "asv_dynamics.h"
#include "exception.h"
#include <algorithm>

using namespace asv_swarm;
using namespace asv_swarm::Hydrodynamics;

/**
 * A simple enum to correctly index motion in the matrices defined for equation 
 * of motion.
 */
enum DOF{surge=0, sway=1, heave=2, roll=3, pitch=4, yaw=5};

ASV_dynamics::ASV_dynamics(
    Sea_surface_dynamics& sea_surface,
    ASV_particulars asv,
    ASV_motion_state initial_state):
  sea_surface{sea_surface},
  asv{asv},
  motion_state{initial_state}
{
  // Check if the asv inputs are valid
  if( asv.L.value() <= 0.0                                             ||
      asv.B.value() <= 0.0                                             ||
      asv.T.value() <= 0.0                                             ||
      asv.displacement.value() <= 0.0                                  ||
      (asv.displacement/(asv.L* asv.B* asv.T)).value() > 1.0           ||
      asv.metacentric_height.value() <= asv.centre_of_gravity.z.value()||
      asv.metacentric_height.value() <= asv.T.value()                  ||
      asv.max_speed.value() > 0.0                                      
    )
  {
    throw Exception::ValueError("Constructor error. Class: ASV_dynamics." 
                                "Invalid input.");
  }

  // TODO: Check if the initial position provided for the ASV is valid, ie:
  // check if the position is out of the field.

  // current time = 0
  current_time = 0.0*Units::second;

  // Set min and max encounter frequency
  Wave_spectrum& wave_spectrum = sea_surface.get_wave_spectrum();
  Quantity<Units::frequency> min_wave_freq = wave_spectrum.get_min_frequency();
  Quantity<Units::frequency> max_wave_freq = wave_spectrum.get_max_frequency();
  min_encounter_frequency = get_encounter_frequency(asv.max_speed, 
                                                    min_wave_freq,
                                                    0.0*Units::radian);
  max_encounter_frequency = get_encounter_frequency(asv.max_speed,
                                                    max_wave_freq,
                                                    Constant::PI*Units::radian);

  // Initialise mass matrix
  set_mass_matrix();
  // Initialise damping matrix
  set_damping_matrix();
  // Initialise stiffness matrix
  set_stiffness_matrix();
  // Initialise the restoring force matrix
  set_restoring_force_matrix();
  // Initialise the wave force spectrum
  set_unit_wave_force_spectrum();
  // Initialise the wave force matrix
  set_wave_force_matrix();
  // Initialise the propeller force matrix
  set_propeller_force_matrix();
  // Initialise the current force matrix
  set_current_force_matrix();
  // Initialise the wind force matrix
  set_wind_force_matrix(); 
}

void ASV_dynamics::set_mass_matrix()
{
  double PI = Constant::PI.value();
  double rho = Constant::RHO_SEA_WATER.value();
  double g = Constant::G.value();
  double L = asv.L.value();
  double B = asv.B.value();
  double T = asv.T.value();
  double KM = asv.metacentric_height.value();
  double x_c = asv.centre_of_gravity.x.value();
  double y_c = asv.centre_of_gravity.y.value();
  double z_c = asv.centre_of_gravity.z.value();
  double mass = asv.displacement.value() * rho;
  
  M[DOF::surge][DOF::surge] = 
    M[DOF::sway][DOF::sway] = 
    M[DOF::heave][DOF::heave] = mass;
  
  // Roll moment of inertia 
  double r_roll = asv.r_roll.value();
  M[DOF::roll][DOF::roll] = mass * r_roll * r_roll;
  
  // Pitch moment of inertia
  double r_pitch = asv.r_pitch.value();
  M[DOF::pitch][DOF::pitch] = mass * r_pitch * r_pitch;
  
  // Yaw moment of inertia
  double r_yaw = asv.r_yaw.value();
  M[DOF::yaw][DOF::yaw] = mass * r_yaw * r_yaw;

  // Set added mass and inertia in the matrix
  //
  // Shape idealisation: 
  // For the purpose of calculating added mass the shape of ASV is assumed as:
  // - waterline - elliptical, with major axis along the length of the
  //   vessel and major axis length equal to asv's load waterline length.
  // - transverse section - rectangular, with length of the rectangle equal to
  //   beam of the asv width of rectangle equal to draft of the asv.
  
  // Heave added mass
  // Heave added mass = integral(0.5 * rho * PI * y^2).    ----->[equ:1]
  // But for the ellipse:
  // (x^2/ a^2) + (y^2/ b^2) = 1
  // where a = major axis = L/2
  //       b = minor axis = B/2
  // which implies:
  // y = (b/a) * sqrt(a^2 - x^2)
  // applying the above equation of y in [equ:1]
  // ie: heave added mass = (PI/12) rho B^2 L
  double added_mass_heave = (PI/12.0) * rho * B*B * L;
  M[DOF::heave][DOF::heave] += added_mass_heave;

  // Sway added mass 
  // Sway added mass = 0.5 rho PI T^2 L
  double added_mass_sway = 0.5 * rho * PI * T*T * L;
  M[DOF::sway][DOF::sway] += added_mass_sway;

  // Surge added mass
  // Generally surge added mass is assumed as zero. But for a cylindrical ASV, 
  // this may not be appropriate. In this case we set surge added mass = sway
  // added mass.
  // So, for ASV with L=B, surge added mass = sway added mass
  if(L == B)
  {
    M[DOF::surge][DOF::surge] += added_mass_sway;
  }

  // Roll added mass inertia 
  // add mass inertial = rho (PI/24) (KM^2 + T^2) B^2 L
  double added_mass_roll = (rho * PI/ 24.0)* (KM*KM + T*T) * B*B * L;
  M[DOF::roll][DOF::roll] += added_mass_roll;

  // Pitch added mass inertia  
  // pitch add mass inertia = (1/15) rho PI B^2/4 L^3/4
  double added_mass_pitch = (1.0/15.0)*rho*PI* (B*B/4.0) * (L*L*L/4.0);
  M[DOF::pitch][DOF::pitch] += added_mass_pitch;

  // Yaw added mass inertia
  // yaw added mass inertia = (1/24) * rho * PI * T^2 * L^3
  double added_mass_yaw = (1.0/24.0) * rho * PI * T*T * L*L*L;
  M[DOF::yaw][DOF::yaw] += added_mass_yaw;

  // Motion coupling 
  /*
  M[DOF::surge][DOF::pitch] = M[DOF::pitch][DOF::surge] = mass  * z_c;
  M[DOF::sway ][DOF::roll ] = M[DOF::roll ][DOF::sway ] = -mass * z_c;
  M[DOF::sway ][DOF::yaw  ] = M[DOF::yaw  ][DOF::sway ] = mass  * x_c;
  M[DOF::heave][DOF::pitch] = M[DOF::pitch][DOF::heave] = -mass * x_c;
  */
}

void ASV_dynamics::set_damping_matrix()
{
  // TODO: Implement
}

void ASV_dynamics::set_stiffness_matrix()
{
  double PI = Constant::PI.value();
  double rho = Constant::RHO_SEA_WATER.value();
  double g = Constant::G.value();
  double L = asv.L.value();
  double B = asv.B.value();
  double T = asv.T.value();
  double KM = asv.metacentric_height.value();

  // For the purpose of estimating stiffness, we assume the water plane to be of
  // elliptical shape.
  
  // Surge stiffness = 0
  // Sway stiffness = 0
  
  // Heave stiffness 
  // heave stiffness = water plane area * rho * g
  // water plane area (considering elliptical shape) = PI/4 * L*B
  double stiffness_heave = (PI/4.0) * L * B * rho * g;
  K[DOF::heave][DOF::heave] = stiffness_heave;

  // Roll stiffness
  // roll stiffness = restoring moment
  // restoring moment = rho * g * displacement * GM
  double displacement = asv.displacement.value();
  double GM = KM - asv.centre_of_gravity.z.value();
  double stiffness_roll = rho * g * displacement * GM;
  K[DOF::roll][DOF::roll] = stiffness_roll;

  // Pitch stiffness
  // pitch stiffness = rho * g * I_y
  // for ellipse I_y = (PI/4) a^3 b
  // where a = L/2
  // and b = B/2
  double stiffness_pitch = rho * g * (PI/4.0) * L*L*L/8.0 * B/2.0;
  K[DOF::pitch][DOF::pitch] = stiffness_pitch;

  // Yaw stiffness = 0.0
}

void ASV_dynamics::set_unit_wave_force_spectrum()
{
  // The response spectrum is to be calculated for heading directions ranging
  // from 0 deg to 360 deg with 1 deg as the angle step size. Here the angle is
  // calculated with respect to the ASV and not with respect to global
  // direction.
  for(int i = 0; i < direction_count; ++i)
  {
    // convert the angle to radians
    Quantity<Units::plane_angle> heading = 
      (i * Constant::PI)/180 * Units::radian;

    // encounter frequency step size 
    int freq_band_count = freq_count - 1; 
    Quantity<Units::frequency> encounter_freq_step_size =  
      (max_encounter_frequency - min_encounter_frequency) / 
      (freq_band_count * Units::si_dimensionless);
    for(int j = 0; j < freq_count; ++j)
    {
      Quantity<Units::frequency> frequency = min_encounter_frequency + 
                                             encounter_freq_step_size * 
                                             (j * Units::si_dimensionless);
      // Surge and sway force, and yaw moment due to wave is considered as 0
      F_unit_wave[i][j][DOF::surge] = 
        F_unit_wave[i][j][DOF::sway] = 
        F_unit_wave[i][j][DOF::yaw] = 0.0;

      // Calculate and set heave force and roll and pitch moments.
      double heave_pressure_force = 0.0;
      double pitch_moment = 0.0;  
      double roll_moment = 0.0;
      double wave_height = 0.01; // wave height = 1 cm.
      double circular_freq = 2.0 * Constant::PI.value() * frequency.value();
      double k = circular_freq*circular_freq / Constant::G.value();
      double z = -asv.T.value();
      double mu = heading.value();
      //  The waterline view of the asv is like an ellipse.
      //  find the major and minor axis of the asv
      double major_axis = asv.L.value()/2.0;
      double minor_axis = asv.B.value()/2.0;

      // Divide the length into 100 strips
      double delta_x = asv.L.value()/100.0;
      for(double x = 0.0; x < asv.L.value(); x += delta_x)
      {
        double x_mid_strip = x + delta_x/2.0;
        // find the corresponding width of the strip from the equation of 
        // ellipse
        double b_x = (major_axis / minor_axis) * sqrt(major_axis*major_axis - 
                                                x_mid_strip*x_mid_strip);
        double force = Constant::RHO_SEA_WATER.value() * 
                       Constant::G.value() * 
                       wave_height * 
                       exp(k * z) * 
                       sin(k * b_x * sin(mu)) /
                       (k * sin(mu)) * 
                       delta_x;
        heave_pressure_force += force;
        pitch_moment += force * (major_axis - x_mid_strip); 
        // For calculating roll moment divide the strip in the y direction into 100 
        // strips
        double delta_y = asv.B.value()/100.0;
        for(double y = - minor_axis; y < major_axis; y += delta_y)
        {
          double y_mid = y + delta_y/2.0;
          roll_moment += (2.0/3.0) * 
                         Constant::RHO_SEA_WATER.value() *
                         Constant::G.value() * 
                         k *
                         wave_height/2.0 *
                         sin(mu) *
                         cos(k * (x*cos(mu) + y_mid*sin(mu))) * 
                         pow(y_mid,3) * 
                         delta_y *
                         delta_x;
        }
      } 
      F_unit_wave[i][j][DOF::heave] = heave_pressure_force;
      F_unit_wave[i][j][DOF::pitch] = pitch_moment;
      F_unit_wave[i][j][DOF::roll]  = roll_moment;
    }
  }
}

void ASV_dynamics::set_wave_force_matrix()
{  
  double pi = Constant::PI.value();
  double g = Constant::G.value();
  double x = motion_state.position.x.value();
  double y = motion_state.position.y.value();
  double asv_heading = motion_state.attitude.z.value();
  double asv_speed = motion_state.linear_velocity[DOF::surge].value();
  
  // Reset F_wave to 0.0
  for(int i=0; i<6; ++i)
  {
    F_wave[i] = 0.0;
  }

  // For each wave 
  Wave_spectrum& wave_spectrum = sea_surface.get_wave_spectrum();
  std::vector<std::vector<Regular_wave>> waves = wave_spectrum.get_spectrum();
  for(auto directional_waves : waves)
  {
    for(auto wave : directional_waves)
    {
      // Get wave direction W.R.T vessel heading
      double wave_direction = wave.get_direction().value() - asv_heading;
      wave_direction = (wave_direction < 0.0)? 2.0*pi - wave_direction : 
                                               wave_direction; 
      // Get encounter frequency
      double wave_freq = wave.get_wave_frequency().value();
      double encounter_freq = 
        wave_freq - (wave_freq*wave_freq/g)*asv_speed*cos(wave_direction);
      // Get the wave elevation at the point for the current time step
      double wave_elevation = wave.get_wave_elevation(
        motion_state.position.x, motion_state.position.y, current_time).value();
      // Get unit wave force
      double freq_band_count = freq_count - 1.0;
      double frequency_step_size = (max_encounter_frequency.value() - 
                                    min_encounter_frequency.value()) / 
                                    freq_band_count;
      int frequency_index = round(encounter_freq/frequency_step_size);
      std::array<double, dof> unit_wave_force = 
        F_unit_wave[std::lround(wave_direction)][frequency_index];
      // Get scaled wave force
      for(int i = 0; i<dof; ++i)
      {
        F_wave[i] += wave_elevation * unit_wave_force[i];
      }
    }
  }
}

void ASV_dynamics::set_restoring_force_matrix()
{
  // TODO: Implement
}

void ASV_dynamics::set_propeller_force_matrix()
{
  // TODO: Implement
}

void ASV_dynamics::set_current_force_matrix()
{
  // TODO: Implement
}

void ASV_dynamics::set_wind_force_matrix()
{
  // TODO: Implement
}

void set_position(Quantity<Units::time> current_time)
{
  // TODO: Implement
}

void set_attitude(Quantity<Units::time> current_time)
{
  // TODO: Implement
}

/*
void ASV_dynamics::set_asv_position()
{
  if(current_time.value() == 0.0)
  {
    // Set the initial position of the ASV at centre of the field.
    Quantity<Units::length> field_length = sea_surface.get_field_length();
    position.x = field_length / 2.0;
    position.y = position.x;
    position.z = 0.0*Units::meter;
    // z should not be 0. It should be equal to: 
    // the wave elevation at the point + distance of COG to waterline.
    // Get wave elevation at the point 
    Quantity<Units::length> elevation = 
      sea_surface.get_current_elevation_at(position); 
    // Calculate the distance of COG to WL
    Quantity<Units::length> dist_cog_wl = asv.centre_of_gravity.z - asv.T;
    position.z = elevation + dist_cog_wl;  
  }
  else
  {
    // TODO: Iterate position value for each time step after start.
  }
}

void ASV_dynamics::set_asv_attitude()
{
  if(current_time.value() == 0.0)
  {
    // TODO: Implement the initial attitude of ASV
  }
  else
  {
    // TODO: Implement attitude of ASV for all time steps after start.
  }
}
*/

Quantity<Units::frequency> ASV_dynamics::get_encounter_frequency(
    Quantity<Units::velocity> asv_speed,
    Quantity<Units::frequency> wave_frequency,
    Quantity<Units::plane_angle> wave_heading)
{
  return wave_frequency - 
         wave_frequency*wave_frequency/Constant::G * 
         asv_speed * cos(wave_heading);
}
