#include "ASVLite/sea_surface.h"
#include "ASVLite/asv.h"
#include "ASVLite/rudder_controller.h"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>


int main() {
    // Set ASV spec
    ASVLite::AsvSpecification asv_spec {
        .L_wl = 2.1, // m
        .B_wl = 0.6, // m
        .D = 0.25,   // m
        .T = 0.15,   // m
    };

    ASVLite::RudderController rudder_controller(asv_spec, {1.0, 1.0, 1.0});
    rudder_controller.tune_controller_local_search(0, 5, 0.25);
    // rudder_controller.tune_controller_exhaustive_search(0, 5, 0.25);


    // Simulate waypoint navigation
    std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    std::filesystem::path results_dir = root_dir/"data"/"rudder_controller_tuning";
    if (!std::filesystem::exists(results_dir)) {
        std::filesystem::create_directory(results_dir);
    }
    
    std::filesystem::path result_file_path = results_dir/("waypoint_navigation.csv");
    std::ofstream file(result_file_path); 
    file << "x,y\n";

    // Waypoints
    const std::vector<ASVLite::Geometry::Coordinates3D> waypoints = {
        {100.0, 300.0, 0.0},
        {300.0, 300.0, 0.0},
        {300.0, 100.0, 0.0},
        {600.0, 100.0, 0.0},
        {600.0, 300.0, 0.0},
    };

    // Initialise the sea surface
    const size_t count_component_waves = 15;
    const double wave_ht = 3.50; // m
    const double wave_dp = M_PI/3.0; // rad
    const int wave_rand_seed = 1;
    const ASVLite::SeaSurface<count_component_waves> sea_surface {wave_ht, wave_dp, wave_rand_seed};

    // Init ASV
    const ASVLite::Geometry::Coordinates3D position {100.0, 100.0, 0.0};
    const ASVLite::Geometry::Coordinates3D attitude {0, 0, 0};
    ASVLite::Asv asv {asv_spec, &sea_surface, position, attitude};

    // Run simulation
    const double simulation_duration = 60.0 * 60.0; // sec
    int i = 0;
    while(asv.get_time() < simulation_duration && i < waypoints.size()) {
        const double rudder_angle = rudder_controller.get_rudder_angle(asv.get_position(), asv.get_attitude(), waypoints[i]);
        auto [thrust_position, thrust_magnitude] = get_wave_glider_thrust(asv, rudder_angle,sea_surface.significant_wave_height);
        asv.step_simulation(thrust_position, thrust_magnitude);

        const ASVLite::Geometry::Coordinates3D current_position = asv.get_position();
        const double delta_x = waypoints[i].keys.x - current_position.keys.x;
        const double delta_y = waypoints[i].keys.y - current_position.keys.y;
        const double dist = sqrt(delta_x*delta_x + delta_y*delta_y);
        if (dist < 5.0) {
            ++i;
        }
        file << current_position.keys.x << "," << current_position.keys.y << "\n";
    }

    file.close();


    return 0;
}