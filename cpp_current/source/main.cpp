#include "ASVLite/asv.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <stdexcept>

using namespace ASVLite;

int main() {
    std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    std::filesystem::path results_dir = root_dir/"results";
    if (!std::filesystem::exists(results_dir)) {
        std::filesystem::create_directory(results_dir);
    }
    else {
        std::cout << results_dir << " already exists." << std::endl;
        return 1;
    }
    std::filesystem::path result_file_path = results_dir/("simulation_data.csv");
    std::ofstream file(result_file_path); 

    // Initialise the sea surface
    const double wave_ht = 7.50; // m
    const double wave_dp = M_PI/3.0; // rad
    const int count_component_waves = 15;
    const int wave_rand_seed = 1;
    const SeaSurface sea_surface {wave_ht, wave_dp, wave_rand_seed, count_component_waves};

    // Set ASV spec
    AsvSpecification asv_spec {
        .L_wl = 2.1, // m
        .B_wl = 0.6, // m
        .D = 0.25,   // m
        .T = 0.15,   // m
    };

    // Init ASV
    const Geometry::Coordinates3D position {100.0, 100.0, 0.0};
    const Geometry::Coordinates3D attitude {0, 0, 0};
    Asv asv {asv_spec, &sea_surface, position, attitude};

    // Run simulation
    file << "x,y,z,submersion_depth,F_wave,F_damping,F_restoring,F_thrust,F_net\n";
    const double simulation_duration = 60 * 60; // sec
    int i = 0;
    while(asv.get_time() < simulation_duration) {
        ++i;
        auto [thrust_position, thrust_magnitude] = get_wave_glider_thrust(asv, 0.0, sea_surface.significant_wave_height);
        thrust_magnitude = {0.0, 0.0, 0.0};
        asv.step_simulation(thrust_position, thrust_magnitude);
        
        file    << asv.get_position().keys.x << "," 
                << asv.get_position().keys.y << "," 
                << asv.get_position().keys.z << ","
                << asv.get_submersion_depth() << ","
                << asv.get_wave_force().keys.heave << ","
                << asv.get_damping_force().keys.heave << ","
                << asv.get_restoring_force().keys.heave << ","
                << asv.get_propulsive_thrust().keys.heave << ","
                << asv.get_net_force().keys.heave << ","
                << asv.get_velocity().keys.heave << ","
                << asv.get_attitude().keys.x << ","
                << asv.get_velocity().keys.roll << "\n";
    }

    return 0;
}