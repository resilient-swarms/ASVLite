#include "ASVLite/asv.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <string>

using namespace ASVLite;

int main() {
    std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    std::filesystem::path data_dir = root_dir/"data";
    std::filesystem::path results_dir = root_dir/"results_thrust";
    if (!std::filesystem::exists(results_dir)) {
        std::filesystem::create_directory(results_dir);
    }
    else {
        std::cout << results_dir << " already exists." << std::endl;
        return 1;
    }
    // Open the results file to write data
    std::filesystem::path result_file_path = results_dir/("simulation_data.csv");
    std::ofstream result_file(result_file_path); 

    if (!result_file.is_open()) {
        std::cerr << "Error: could not open result file" << std::endl;
        return 1;
    } else {
        result_file << "y1,x1,wave_ht,tuning_factor\n";
    }
    
    // Open the onboard data file.
    std::filesystem::path data_file_path = data_dir/("tuning_data.csv");
    std::ifstream data_file(data_file_path);
    
    if (!data_file.is_open()) {
        std::cerr << "Error: could not open data file" << std::endl;
        return 1;
    }

    std::string line;
    bool has_read_header = false;
    // Process each line in the file
    int line_count = 0;
    while (std::getline(data_file, line)) {
        line_count++;
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        
        // Split the line by commas
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        
        // Extract row data
        if(!has_read_header) {
            // reading header
            has_read_header = true;
            continue;
        } else {
            // Convert text to numbers
            const double y1 = std::stod(row[2]); // m
            const double x1 = std::stod(row[3]); // m
            const double wave_ht = std::stod(row[6]); // m
            const double sim_duration = std::stod(row[7]); // sec
            const double target_speed = std::stod(row[9]); // m/s

            // Initialise the sea surface
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

            double tuning_factor = 0.5;
            bool found_optimal_tuning = false;
            while(!found_optimal_tuning){
                // Init ASV
                const Geometry::Coordinates3D position {x1, y1, 0.0};
                const Geometry::Coordinates3D attitude {0, 0, 0};
                Asv asv {asv_spec, &sea_surface, position, attitude};

                // Run simulation
                int i = 0;
                while(asv.get_time() < sim_duration) {
                    ++i;
                    auto [thrust_position, thrust_magnitude] = get_wave_glider_thrust(asv, 0.0);
                    thrust_magnitude.keys.x = tuning_factor * thrust_magnitude.keys.x;
                    asv.step_simulation(thrust_position, thrust_magnitude);
                }
                const double delta_x = asv.get_position().keys.x - x1;
                const double delta_y = asv.get_position().keys.y - y1;
                const double dist = sqrt(delta_x*delta_x + delta_y*delta_y);
                const double sim_speed = dist / sim_duration; // m/s
                std::cout << "tuning factor = " << tuning_factor << " Target speed = " << target_speed << " Sim speed = " << sim_speed << "\n"; 

                const double speed_ratio = sim_speed / target_speed;
                if(speed_ratio >= 0.95 && speed_ratio <= 1.05) {
                    found_optimal_tuning = true;
                    std::cout << line_count << " Optimal tuning for x1 = " << x1 << " y1 = " << y1 << " tuning factor = " << tuning_factor << "\n\n";
                    result_file << y1 << "," << x1 << "," << wave_ht << "," << tuning_factor << "\n";
                } else {
                    tuning_factor = tuning_factor / speed_ratio;
                }

            }

        }
    }
    
    data_file.close();
    result_file.close();

    return 0;
}