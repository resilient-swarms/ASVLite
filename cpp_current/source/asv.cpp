#include "ASVLite/asv.h"
#include "ASVLite/constants.h"
#include <stdexcept>
#include <algorithm>


// Method to compute the encounter frequency. 
// heading_angle is the angle of the wave propagation w.r.t positive x-axis of ASV.
static double get_encounter_frequency(double wave_freq, double asv_speed, double heading_angle) {
    return wave_freq - (pow(wave_freq, 2.0)/ASVLite::Constants::G) * asv_speed * cos(heading_angle);
}


void ASVLite::Asv::set_mass() {
    // Mass of the ASV
    const double asv_mass = spec.L_wl * spec.B_wl * spec.T * Constants::SEA_WATER_DENSITY;

    // Added mass of the ASV
    // ---------------------
    // ASV shape idealisation:
    // For the purpose of calculating the added mass the shape of the ASV is
    // assumed to be a elliptic-cylinder.
    const double a = spec.L_wl/2.0;
    const double b = spec.B_wl/2.0; 
    const double c = -std::clamp(dynamics.submersion_depth, -spec.D, 0.0);

    // Ref: DNVGL-RP-N103 Table A-1 (page 205)
    // Surge added mass = rho * Ca * Ar
    constexpr double C_a = 1.0;
    const double Ar_surge = M_PI*b*b;
    const double Ar_sway  = M_PI*a*a;
    const double Ar_heave = M_PI*a*b;
    const double added_mass_surge = Constants::SEA_WATER_DENSITY * C_a * Ar_surge * (2.0*a);
    const double added_mass_sway  = Constants::SEA_WATER_DENSITY * C_a * Ar_sway * (2.0*b);
    const double added_mass_heave = Constants::SEA_WATER_DENSITY * C_a * Ar_heave * c;

    // Moment of inertia for angular motions
    const double I_roll = (1.0 / 12.0) * asv_mass * (spec.B_wl * spec.B_wl + spec.D * spec.D); // Roll
    const double I_pitch = (1.0 / 12.0) * asv_mass * (spec.L_wl * spec.L_wl + spec.D * spec.D);  // Pitch
    const double I_yaw = (1.0 / 12.0) * asv_mass * (spec.L_wl * spec.L_wl + spec.B_wl * spec.B_wl);  // Yaw
  
    const double added_mass_roll  = Constants::SEA_WATER_DENSITY * M_PI/8.0 * b * (2*a);
    const double added_mass_pitch = Constants::SEA_WATER_DENSITY * M_PI/8.0 * a * (2*b);
    const double added_mass_yaw   = Constants::SEA_WATER_DENSITY * M_PI/8.0 * (a*a-b*b)*(a*a-b*b);

    // Set the mass matrixs
    dynamics.M(0, 0) = asv_mass;// + added_mass_surge;
    dynamics.M(1, 1) = asv_mass;// + added_mass_sway;
    dynamics.M(2, 2) = asv_mass;// + added_mass_heave;
    dynamics.M(3, 3) = I_roll  ;// + added_mass_roll;
    dynamics.M(4, 4) = I_pitch ;// + added_mass_pitch;
    dynamics.M(5, 5) = I_yaw   ;// + added_mass_yaw;
}


void ASVLite::Asv::set_damping_coefficient() {
    // Ref: Recommended practices DNVGL-RP-N103 Modelling and analysis of marine
    // operations. Edition July 2017. Appendix B Table B-1, B-2.
  
    // Surge drag coefficient - assuming elliptical waterplane area
    const double C_DS = (dynamics.submersion_depth < 0.0)? 1.9 : 0.001;
    const double C_surge = 0.5 * Constants::SEA_WATER_DENSITY * C_DS * spec.B_wl * spec.T;
  
    // Sway drag coefficient - assuming elliptical waterplane area
    const double C_sway = 0.5 * Constants::SEA_WATER_DENSITY * C_DS * spec.L_wl * spec.T;
  
    // Heave drag coefficient - consider it as flat plat perpendicular to flow.
    const double C_heave = 0.5 * Constants::SEA_WATER_DENSITY * C_DS * spec.L_wl * spec.B_wl;

    // roll, pitch and yaw drag coefficient set equal to roll damping coefficient 
    // given in Handbook of Marin Craft Hydrodynamics and motion control, page 125
    const double C_roll  = (dynamics.submersion_depth < 0.0)? 1.9 : 0.001;
    const double C_pitch = (dynamics.submersion_depth < 0.0)? 1.9 : 0.001;
    const double C_yaw   = (dynamics.submersion_depth < 0.0)? 1.9 : 0.001;

    // Set the drage coeff matrix
    dynamics.C(0, 0) = C_surge;
    dynamics.C(1, 1) = C_sway;
    dynamics.C(2, 2) = C_heave;
    dynamics.C(3, 3) = C_roll;
    dynamics.C(4, 4) = C_pitch;
    dynamics.C(5, 5) = C_yaw;
}


void ASVLite::Asv::set_stiffness() {
    constexpr double K_surge = 0.0;
    constexpr double K_sway  = 0.0;
    constexpr double K_yaw   = 0.0;
   
    // Assuming elliptical shape for the water plane area.
    const double a = spec.L_wl / 2.0;
    const double b = spec.B_wl / 2.0;
    const double A = M_PI * a * b;
    const double I_xx = M_PI * a * pow(b, 3) / 4;
    const double I_yy = M_PI * b * pow(a, 3) / 4;
    
    // Heave stiffness
    double K_heave = A * Constants::SEA_WATER_DENSITY * Constants::G;
    const double submersion_depth = std::clamp(dynamics.submersion_depth, -spec.D, 0.0);
    if(submersion_depth > -spec.T) {
        // tapper down the buoyancy to avoid lifting of ASV from surface.
        K_heave += K_heave * sin(fabs(submersion_depth) / spec.T) * sin(fabs(submersion_depth) / spec.T);
    }
  
    // Roll stiffness
    // Using the same formula as mentioned for pitch in below ref.
    // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
    const double K_roll = I_xx * Constants::SEA_WATER_DENSITY * Constants::G;
  
    // Pitch stiffness
    // Ref: Dynamics of Marine Vehicles, R. Bhattacharyya, page 66
    const double K_pitch = I_yy * Constants::SEA_WATER_DENSITY * Constants::G;
  
    // Set the stiffeness matrix
    dynamics.K(0, 0) = K_surge;
    dynamics.K(1, 1) = K_sway;
    dynamics.K(2, 2) = K_heave;
    dynamics.K(3, 3) = K_roll;
    dynamics.K(4, 4) = K_pitch;
    dynamics.K(5, 5) = K_yaw;
}


void ASVLite::Asv::set_wave_force() {
    // Dimensions of ellipsoid
    const double a = spec.L_wl/ 2.0;
    const double b = spec.B_wl/ 2.0;
    const double c = -std::clamp(dynamics.submersion_depth, -spec.D, 0.0);
    const double A_trans = M_PI/2.0 * b * c;
    const double A_profile = M_PI/2.0 * a * c;
    const double A_waterplane = M_PI/2 * a * b;

    // Reset the wave force to all zeros
    dynamics.F_wave = Eigen::Matrix<double, 6, 1>::Zero();

    if(dynamics.submersion_depth >= 0.0) {
        dynamics.F_wave = Eigen::Matrix<double, 6, 1>::Zero();
    } else {
        // For each wave in the wave spectrum
        for(const RegularWave& wave : sea_surface->component_waves) {
            // Compute relative wave heading
            const double wave_heading_global   = Geometry::normalise_angle_PI(wave.heading);
            const double wave_heading_relative = Geometry::normalise_angle_PI(wave_heading_global - dynamics.attitude.keys.z);
            // Get encounter frequency
            const double surge_velocity = dynamics.V(0,0);
            const double encounter_freq = get_encounter_frequency(wave.frequency, surge_velocity, wave_heading_relative);
            // Compute the coordinates of fore, aft, port side, starboard side and centre position of the vehicle for calculating wave pressure.
            // Step 1: Create rotation matrix (intrinsic Z-Y-X: yaw -> pitch -> roll)
            const Eigen::Matrix3d R (Eigen::AngleAxisd(dynamics.attitude.keys.z, Eigen::Vector3d::UnitZ())*  // yaw  
                                    Eigen::AngleAxisd(dynamics.attitude.keys.y, Eigen::Vector3d::UnitY())*  // pitch 
                                    Eigen::AngleAxisd(dynamics.attitude.keys.x, Eigen::Vector3d::UnitX())); // roll 
            // Step 2: Define the direction vectors in the body frame
            const Eigen::Vector3d forward_direction_local_frame(1.0, 0.0, 0.0);
            const Eigen::Vector3d aft_direction_local_frame(-1.0, 0.0, 0.0);
            const Eigen::Vector3d starboard_direction_local_frame(0.0, 1.0, 0.0);
            const Eigen::Vector3d portside_direction_local_frame(1.0, -1.0, 0.0);
            // Step 3: Rotate direction vectors into world frame
            const Eigen::Vector3d forward_direction_world_frame   = R * forward_direction_local_frame;
            const Eigen::Vector3d aft_direction_world_frame       = R * aft_direction_local_frame;
            const Eigen::Vector3d starboard_direction_world_frame = R * starboard_direction_local_frame;
            const Eigen::Vector3d portside_direction_world_frame  = R * portside_direction_local_frame;
            // Step 4: Compute coordinates of the positions in world frame
            const Eigen::Vector3d position_centre(dynamics.position.keys.x, dynamics.position.keys.y, dynamics.position.keys.z);
            const Eigen::Vector3d position_forward   = position_centre + (a/2 * forward_direction_world_frame);
            const Eigen::Vector3d position_aft       = position_centre + (a/2 * aft_direction_world_frame);
            const Eigen::Vector3d position_starboard = position_centre + (b/2 * starboard_direction_world_frame);
            const Eigen::Vector3d position_portside  = position_centre + (b/2 * portside_direction_world_frame);
            // Construct Coordinate3D objects for these positions
            const Geometry::Coordinates3D pos_centre{position_centre(0), position_centre(1), position_centre(2)};
            const Geometry::Coordinates3D pos_forward{position_forward(0), position_forward(1), position_forward(2)};
            const Geometry::Coordinates3D pos_aft{position_aft(0), position_aft(1), position_aft(2)};
            const Geometry::Coordinates3D pos_starboard{position_starboard(0), position_starboard(1), position_starboard(2)};
            const Geometry::Coordinates3D pos_porside{position_portside(0), position_portside(1), position_portside(2)};
            // Construct the encountered wave
            const RegularWave encountered_wave(wave.amplitude, encounter_freq, wave.phase_lag, wave.heading);
            // Get the wave pressure amplitude for the encountered wave
            const double wave_pressure_centre    = encountered_wave.get_wave_pressure(pos_centre, dynamics.time); 
            const double wave_pressure_forward   = encountered_wave.get_wave_pressure(pos_forward, dynamics.time); 
            const double wave_pressure_aft       = encountered_wave.get_wave_pressure(pos_aft, dynamics.time); 
            const double wave_pressure_starboard = encountered_wave.get_wave_pressure(pos_starboard, dynamics.time); 
            const double wave_pressure_portside  = encountered_wave.get_wave_pressure(pos_porside, dynamics.time); 
            // Lever
            const double lever_trans = b / 8;
            const double lever_long  = a / 8;
            // Set the wave pressue force matrix
            const double scale = 1.0;
            dynamics.F_wave(0) += (wave_pressure_forward - wave_pressure_aft) * A_trans * scale; // surge
            dynamics.F_wave(1) += (wave_pressure_starboard - wave_pressure_portside) * A_profile * scale; // sway
            // dynamics.F_wave(2) += wave_pressure_centre * A_waterplane * scale; // heave
            dynamics.F_wave(3) += (wave_pressure_starboard - wave_pressure_portside) * A_waterplane * lever_trans * scale; // roll
            dynamics.F_wave(4) += (wave_pressure_forward - wave_pressure_aft) * A_waterplane * lever_long * scale; // pitch
            // dynamics.F_wave(5) += (wave_pressure_forward - wave_pressure_aft) * A_profile * lever_long * scale; // yaw
        }
    } 

}


void ASVLite::Asv::set_thrust(const Geometry::Coordinates3D& thrust_position, const Geometry::Coordinates3D& thrust_magnitude) {
    // Reset the thrust to all zeros
    dynamics.F_thrust = Eigen::Matrix<double, 6, 1>::Zero();

    if(dynamics.submersion_depth < 0.0) {
        const double x = thrust_position.keys.x;
        const double y = thrust_position.keys.y;
        const double z = thrust_position.keys.z;

        const double M_x = thrust_magnitude.keys.y * z + thrust_magnitude.keys.z * y;
        const double M_y = thrust_magnitude.keys.x * z + thrust_magnitude.keys.z * x; 
        const double M_z = thrust_magnitude.keys.x * y + thrust_magnitude.keys.y * x;

        // Set the thrust matrix
        dynamics.F_thrust(0) = thrust_magnitude.keys.x;
        dynamics.F_thrust(1) = thrust_magnitude.keys.y;
        dynamics.F_thrust(2) = thrust_magnitude.keys.z;
        dynamics.F_thrust(3) = M_x;
        dynamics.F_thrust(4) = M_y;
        dynamics.F_thrust(5) = M_z; 
    }
}


void ASVLite::Asv::set_damping_force() {
    set_damping_coefficient();

    // Compute the quadratic velocity term explicitly to avoid unnecessary temporaries
    Eigen::VectorXd velocity_square = dynamics.V.cwiseProduct(dynamics.V.cwiseAbs());

    // Set the drag force matrix
    dynamics.F_damping = -dynamics.C * velocity_square;
}


void ASVLite::Asv::set_restoring_force() {
    set_stiffness();

    if(dynamics.submersion_depth >= 0.0) {
        dynamics.F_restoring(2) = -dynamics.K(2,2) * spec.T;
    } else {    
        // Heave restoring force
        const double delta_T = dynamics.position.keys.z - sea_surface->get_elevation(dynamics.position, dynamics.time);
        const Eigen::Vector3d elongation {delta_T, dynamics.attitude.keys.x, dynamics.attitude.keys.y};
        
        // Set the restoring force matrix
        const Eigen::Matrix3d K_sub = dynamics.K.block<3,3>(2,2); // Extract the relevant 3x3 submatrix from K
        dynamics.F_restoring.segment(2,3) = -K_sub * elongation; // heave, roll, pitch
        // No restoring force for sway, yaw and surge.
    }
}

void ASVLite::Asv::set_net_force() {
    // Set the net force matrix
    dynamics.F = dynamics.F_thrust + dynamics.F_wave + dynamics.F_damping + dynamics.F_restoring;
    if(halt_surge_and_sway) {
        dynamics.F(0) = 0.0;
        dynamics.F(1) = 0.0;
    }
}


void ASVLite::Asv::set_acceleration() {
    // Set acceleration matrix
    dynamics.A = dynamics.M.inverse() * dynamics.F;
}


void ASVLite::Asv::set_velocity() {
    // Set velocity matrix
    dynamics.V = dynamics.V + (dynamics.A * dynamics.time_step_size/1000.0);
}


void ASVLite::Asv::set_deflection() {
    // Construct a resultant velocity matrix in body frame considering ocean current
    // Create rotation matrix (intrinsic Z-Y-X: yaw -> pitch -> roll)
    const Eigen::Matrix3d R (Eigen::AngleAxisd(dynamics.attitude.keys.z, Eigen::Vector3d::UnitZ())*  // yaw  
                             Eigen::AngleAxisd(dynamics.attitude.keys.y, Eigen::Vector3d::UnitY())*  // pitch 
                             Eigen::AngleAxisd(dynamics.attitude.keys.x, Eigen::Vector3d::UnitX())); // roll 
    // Global velocity in world frame (only X and Y are given)
    const Eigen::Vector3d V_current_global(ocean_current.first, ocean_current.second, 0.0);
    // Convert global velocity to body frame (R^T * V_current_global)
    const Eigen::Vector3d V_current_body = R.transpose() * V_current_global;
    
    // Compute Net Velocity in Body Frame
    Eigen::Matrix<double, 6, 1> V_net = dynamics.V;
    V_net.head(3) += V_current_body;  // Add only the linear velocity components

    // Set deflection matrix
    dynamics.X = V_net * dynamics.time_step_size/1000.0;
}


void ASVLite::Asv::set_pose() {
    // First set attitude
    dynamics.attitude.keys.x += Geometry::normalise_angle_PI(dynamics.X(3));
    dynamics.attitude.keys.y += Geometry::normalise_angle_PI(dynamics.X(4));
    dynamics.attitude.keys.z += Geometry::normalise_angle_PI(dynamics.X(5)); 
    dynamics.attitude.keys.x = Geometry::normalise_angle_PI(dynamics.attitude.keys.x);
    dynamics.attitude.keys.y = Geometry::normalise_angle_PI(dynamics.attitude.keys.y);
    dynamics.attitude.keys.z = Geometry::normalise_angle_PI(dynamics.attitude.keys.z);

    // Create rotation matrix (intrinsic Z-Y-X: yaw -> pitch -> roll)
    const Eigen::Matrix3d R (Eigen::AngleAxisd(dynamics.attitude.keys.z, Eigen::Vector3d::UnitZ())*  // yaw  
                             Eigen::AngleAxisd(dynamics.attitude.keys.y, Eigen::Vector3d::UnitY())*  // pitch 
                             Eigen::AngleAxisd(dynamics.attitude.keys.x, Eigen::Vector3d::UnitX())); // roll 
    // Rotate Deflection Vector from Body Frame to Global Frame
    const Eigen::Vector3d X_global = R * dynamics.X.topRows(3);
    // Compute New Position in Global Frame
    const Eigen::Vector3d current_position {dynamics.position.keys.x, dynamics.position.keys.y, dynamics.position.keys.z};
    const Eigen::Vector3d new_position = current_position + X_global;

    dynamics.position.keys.x = new_position(0);
    dynamics.position.keys.y = new_position(1);
    dynamics.position.keys.z = new_position(2);
}


ASVLite::Asv::Asv(const AsvSpecification& spec, const SeaSurface* sea_surface, const Geometry::Coordinates3D& position, const Geometry::Coordinates3D& attitude) :
    spec {spec} {
    if(sea_surface == nullptr) {
        throw std::invalid_argument("Sea surface cannot be nullptr.");
    }

    this->sea_surface = sea_surface;

    // Place the asv vertically in the correct position W.R.T sea_surface
    dynamics.position = position;
    dynamics.position.keys.z = sea_surface->get_elevation(position, dynamics.time);
    dynamics.attitude.keys.x = Geometry::normalise_angle_PI(attitude.keys.x);
    dynamics.attitude.keys.y = Geometry::normalise_angle_PI(attitude.keys.y);
    dynamics.attitude.keys.z = Geometry::normalise_angle_PI(attitude.keys.z);
}


void ASVLite::Asv::step_simulation(const Geometry::Coordinates3D& thrust_position, const Geometry::Coordinates3D& thrust_magnitude) {
    // Advance time
    dynamics.time += dynamics.time_step_size/1000.0; // seconds

    dynamics.submersion_depth = (dynamics.position.keys.z - spec.T) - sea_surface->get_elevation(dynamics.position, dynamics.time);
    
    set_mass();
    set_wave_force();
    set_thrust(thrust_position, thrust_magnitude);
    set_damping_force();
    set_restoring_force();
    set_net_force();
    set_acceleration();
    set_velocity();
    set_deflection();
    set_pose();
}


void ASVLite::Asv::set_sea_state(const SeaSurface* sea_surface) { 
    if(sea_surface == nullptr) {
        throw std::invalid_argument("Sea surface cannot be nullptr.");
    }

    // Calculate the current submersion depth before changing the sea surface
    const double submersion_depth = sea_surface->get_elevation(dynamics.position, dynamics.time) - dynamics.position.keys.z;
    // set the sea_surface for the ASV
    this->sea_surface = sea_surface;
    // Place the asv vertically in the correct position W.R.T new sea_surface
    dynamics.position.keys.z = sea_surface->get_elevation(dynamics.position, dynamics.time) + submersion_depth;
}


std::pair<ASVLite::Geometry::Coordinates3D, ASVLite::Geometry::Coordinates3D> ASVLite::get_wave_glider_thrust(const Asv& wave_glider, double rudder_angle) {
    auto wave_glider_spec = wave_glider.get_spec(); 
    
    std::pair<Geometry::Coordinates3D, Geometry::Coordinates3D> return_value;
    return_value.first = {-wave_glider_spec.L_wl/2, 0, 0}; // Thrust position
    return_value.second = {0, 0, 0}; // Thrust magnitude

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
    const int count_hydrofoils = 6;
    const double A = 0.113; // m2
    const double alpha_k = 18.0 * M_PI/180.0; // radians
    const double alpha_f_1 = 45.0 * M_PI/180.0; // radians
    const double chi = 7.0 * M_PI/180.0; // radian
    const double lambda = 2.0;
    const double C_DC = 0.6;
    const double C_DO = 0.008;
    const double C_L_1 = (1.8 * M_PI * lambda * alpha_k) / (cos(chi) * sqrt(lambda*lambda/pow(cos(chi), 4) + 4) + 1.8) + (C_DC/ lambda * alpha_k*alpha_k);
    const double C_D = C_DO + C_L_1*C_L_1 / (0.9 * M_PI * lambda);
    const double V_heave = wave_glider.get_velocity().keys.heave;
    const double F_L = 0.5 * Constants::SEA_WATER_DENSITY * C_L_1 * A * V_heave * V_heave;
    const double F_D = 0.5 * Constants::SEA_WATER_DENSITY * C_D * A * V_heave * V_heave;
    const double thrust_per_hydrofoil = F_L * sin(alpha_f_1) - F_D * cos(alpha_f_1);
    const double thrust = count_hydrofoils * thrust_per_hydrofoil;
    return_value.second.keys.x = thrust;

    // Compute the thrust generated by the rudder
    // Assuming the rudder area = area of a hydrofoil
    const double A_rudder = 0.4 * 0.2 ; // m2
    const double alpha_f_2 = fabs(rudder_angle); // radians
    const double V_surge = wave_glider.get_velocity().keys.surge;
    const double C_L_2 = (1.8 * M_PI * lambda * alpha_k) / (cos(chi) * sqrt(lambda*lambda/pow(cos(chi), 4) + 4) + 1.8) + (C_DC/ lambda * alpha_k*alpha_k);
    const double F_L_rudder = 0.5 * Constants::SEA_WATER_DENSITY * C_L_2 * A_rudder * V_surge * V_surge;
    double rudder_thrust = F_L_rudder * sin(alpha_f_2);
    rudder_thrust = (rudder_angle < 0.0)? -rudder_thrust: rudder_thrust;
    return_value.second.keys.y = rudder_thrust;

    return return_value;
}