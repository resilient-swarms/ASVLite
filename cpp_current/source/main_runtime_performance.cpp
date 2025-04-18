#include "ASVLite/asv.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <ctime>
#include <algorithm>

using namespace ASVLite;

int main() {
    std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    std::filesystem::path results_dir = root_dir/"results";
    if (!std::filesystem::exists(results_dir)) {
        std::filesystem::create_directory(results_dir);
    }
    
    std::filesystem::path result_file_path = results_dir/("simulation_data.csv");
    std::ofstream file(result_file_path); 

    std::vector<double> simulation_speeds;
    const int num_simulations = 100;

    for(size_t n = 0; n < num_simulations; ++n) {
        // Initialise the sea surface
        constexpr size_t count_component_waves = 15;
        const double wave_ht = 7.50; // m
        const double wave_dp = M_PI/3.0; // rad
        const int wave_rand_seed = 1;
        const SeaSurface<count_component_waves> sea_surface {wave_ht, wave_dp, wave_rand_seed};

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
        file << "x,y,z,submersion_depth,F_wave,F_drag,F_restoring,F_thrust,F_net,M_surge,M_sway,M_heave,I_roll,I_pitch,I_yaw,a_surge,a_sway,a_heave,a_roll,a_pitch,a_yaw,v_surge,v_swary,v_heave,v_roll,v_pitch,v_yaw,roll,pitch,yaw\n";
        const double simulation_duration = 60 * 60; // sec
        int i = 0;
        std::clock_t start = std::clock();
        while(asv.get_time() < simulation_duration) {
            ++i;
            auto [thrust_position, thrust_magnitude] = get_wave_glider_thrust(asv, 0.0, sea_surface.significant_wave_height);
            asv.step_simulation(thrust_position, thrust_magnitude);
            
            // file<< asv.get_position().keys.x << "," 
            //     << asv.get_position().keys.y << "," 
            //     << asv.get_position().keys.z << ","
            //     << asv.get_submersion_depth() << ","
            //     << asv.get_wave_force().keys.heave << ","
            //     << asv.get_drag_force().keys.heave << ","
            //     << asv.get_restoring_force().keys.heave << ","
            //     << asv.get_propulsive_thrust().keys.heave << ","
            //     << asv.get_net_force().keys.heave << ","
            //     << asv.get_mass().keys.surge << ","
            //     << asv.get_mass().keys.sway << ","
            //     << asv.get_mass().keys.heave << ","
            //     << asv.get_mass().keys.roll << ","
            //     << asv.get_mass().keys.pitch << ","
            //     << asv.get_mass().keys.yaw << ","
            //     << asv.get_acceleration().keys.surge << ","
            //     << asv.get_acceleration().keys.sway << ","
            //     << asv.get_acceleration().keys.heave << ","
            //     << asv.get_acceleration().keys.roll << ","
            //     << asv.get_acceleration().keys.pitch << ","
            //     << asv.get_acceleration().keys.yaw << ","
            //     << asv.get_velocity().keys.surge << ","
            //     << asv.get_velocity().keys.sway << ","
            //     << asv.get_velocity().keys.heave << ","
            //     << asv.get_velocity().keys.roll << ","
            //     << asv.get_velocity().keys.pitch << ","
            //     << asv.get_velocity().keys.yaw << ","
            //     << asv.get_attitude().keys.x * 180.0/M_PI << ","
            //     << asv.get_attitude().keys.y * 180.0/M_PI << ","
            //     << asv.get_attitude().keys.z * 180.0/M_PI << "\n";
        }
        std::clock_t end = std::clock();
        double cpu_time = double(end - start) / CLOCKS_PER_SEC; // sec
        std::cout << "Simulation completed in " << cpu_time << " seconds." << std::endl;

        simulation_speeds.push_back(simulation_duration/cpu_time); // x realtime speed
    }
    std::cout << "Simulation speed " << std::accumulate(simulation_speeds.begin(), simulation_speeds.end(), 0.0)/simulation_speeds.size() << " X realtime speed." << std::endl;

    return 0;
}