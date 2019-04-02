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

ASV_dynamics::ASV_dynamics(ASV& asv, Wave_spectrum* wave_spectrum):
  asv{asv}, 
  wave_spectrum{wave_spectrum}
{
  // Check if the asv inputs are valid
  if( asv.L.value() <= 0.0                                             ||
      asv.B.value() <= 0.0                                             ||
      asv.T.value() <= 0.0                                             ||
      asv.displacement.value() <= 0.0                                  ||
      (asv.displacement/(asv.L* asv.B* asv.T)).value() > 1.0           ||
      asv.metacentric_height.value() <= asv.centre_of_gravity.z.value()||
      asv.metacentric_height.value() <= asv.T.value()                  ||
      asv.max_speed.value() > 0.0                                      ||
      wave_spectrum == nullptr
    )
  {
    throw Exception::ValueError("Constructor error. Class: ASV_dynamics." 
                                "Invalid input.");
  }
  // current time = 0
  current_time = 0.0*Units::second;

  // Initialise all matrix to zero.
  for(unsigned int i{0u}; i<6; ++i)
  {
    for(unsigned int j{0u}; j<6; ++j)
    {
      M[i][j] = 0.0;
      C[i][j] = 0.0;
      K[i][j] = 0.0;
    }
  }
  // Set mass matrix
  set_mass_matrix();
  // Set damping matrix
  set_damping_matrix();
  // Set stiffness matrix
  set_stiffness_matrix();
  // Set min and max encounter frequency
  Quantity<Units::frequency> min_wave_freq = wave_spectrum->get_min_frequency();
  Quantity<Units::frequency> max_wave_freq = wave_spectrum->get_max_frequency();
  min_encounter_frequency = min_wave_freq - 
                            (min_wave_freq*min_wave_freq)/Constant::G *
                            asv.max_speed;
  max_encounter_frequency = max_wave_freq +
                            (max_wave_freq*max_wave_freq)/Constant::G *
                            asv.max_speed;
  // Set number of bands in the response spectrum.
  encounter_freq_band_count = 100;
  encounter_wave_direction_count = 360;
  // Set the RAO spectrum
  set_wave_force_matrix();
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

  // Set the mass and inertia of ASV in the matrix
  double mass = asv.displacement.value() * rho;
  M[DOF::surge][DOF::surge] = 
  M[DOF::sway][DOF::sway]   = 
  M[DOF::heave][DOF::heave] = mass;
  // Roll moment of inertia 
  double r_roll = asv.r_roll.value();
  M[DOF::roll][DOF::roll] = mass * r_roll * r_roll;
  // Pitch moment of inertia
  double r_pitch = asv.r_pitch.value();
  M[DOF::pitch][DOF::pitch] = mass * r_pitch * r_pitch;
  // Yaw moment of intertia
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
  M[DOF::surge][DOF::pitch] = M[DOF::pitch][DOF::surge] = mass  * z_c;
  M[DOF::sway][DOF::roll]   = M[DOF::roll][DOF::sway]   = -mass * z_c;
  M[DOF::sway][DOF::yaw]    = M[DOF::yaw][DOF::sway]    = mass  * x_c;
  M[DOF::heave][DOF::pitch] = M[DOF::pitch][DOF::heave] = -mass * x_c;
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

std::array<double, 3> ASV_dynamics::get_wave_heave_force_pitch_roll_moment(
    Quantity<Units::frequency> frequency, 
    Quantity<Units::plane_angle> angle)
{
  double heave_pressure_force = 0.0;
  double pitch_moment = 0.0;  
  double roll_moment = 0.0;
  double wave_height = 0.01; // wave height = 1 cm.
  double circular_freq = 2.0 * Constant::PI.value() * frequency.value();
  double k = circular_freq*circular_freq / Constant::G.value();
  double z = -asv.T.value();
  double mu = angle.value();
  //  The waterline view of the asv is like an ellipse.
  //  find the major and minor axis of the asv
  double major_axis = asv.L.value()/2.0;
  double minor_axis = asv.B.value()/2.0;

  // Divide the length into 100 strips
  double delta_x = asv.L.value()/100.0;
  for(double x = 0.0; x < asv.L.value(); x += delta_x)
  {
    double x_mid_strip = x + delta_x/2.0;
    // find the corresponding width of the strip from the equation of ellipse
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
  std::array<double, 3>return_value = {heave_pressure_force, 
                                       pitch_moment,
                                       roll_moment};
  return return_value;
}

double ASV_dynamics::get_wave_surge_force(Quantity<Units::frequency> frequency,
                                          Quantity<Units::plane_angle> angle)
{
  // We consider wave surge force as 0.
  return 0.0;
}

double ASV_dynamics::get_wave_sway_force(Quantity<Units::frequency> frequency,
                                         Quantity<Units::plane_angle> angle)
{
  // We consider wave sway force as 0.
  return 0.0;
}

double ASV_dynamics::get_wave_yaw_moment(Quantity<Units::frequency> frequency,
                                         Quantity<Units::plane_angle> angle)
{
  // We consider wave yaw moment as 0.
  return 0.0;
}

void ASV_dynamics::set_wave_force_matrix()
{
  // The response spectrum is to be calculated for heading directions ranging
  // from 0 deg to 360 deg with 1 deg as the angle step size.
  for(int i = 0; i < 360; ++i)
  {
    // convert the angle to radians
    Quantity<Units::plane_angle> heading = 
      (i * Constant::PI)/180 * Units::radian;

    // encounter frequency step size 
    Quantity<Units::frequency> encounter_freq_step_size =  
      (max_encounter_frequency - min_encounter_frequency) / 
      (100.0 * Units::si_dimensionless);
    for(int j = 0; j < 100; ++j)
    {
      Quantity<Units::frequency> frequency = min_encounter_frequency + 
                                             encounter_freq_step_size * 
                                             (j * Units::si_dimensionless);
      // set wave force
      std::array<double, 3> heave_pitch_roll = 
        get_wave_heave_force_pitch_roll_moment(frequency, heading);
      F_wave[i][j][DOF::heave] = heave_pitch_roll[0];
      F_wave[i][j][DOF::pitch] = heave_pitch_roll[1];
      F_wave[i][j][DOF::roll]  = heave_pitch_roll[2];
      F_wave[i][j][DOF::surge] = get_wave_surge_force (frequency, heading);
      F_wave[i][j][DOF::sway]  = get_wave_sway_force  (frequency, heading);
      F_wave[i][j][DOF::yaw]   = get_wave_yaw_moment  (frequency, heading);
    }
  }
}
