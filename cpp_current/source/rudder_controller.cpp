#include "ASVLite/rudder_controller.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <future>
#include <thread>


ASVLite::RudderController::RudderController(const ASVLite::AsvSpecification& asv_spec, const Eigen::Vector3d& initial_K) :
asv_spec {asv_spec}, K {initial_K} {
}


double ASVLite::RudderController::get_relative_heading(const ASVLite::Asv& asv, const ASVLite::Geometry::Coordinates3D& waypoint) const {
    const Geometry::Coordinates3D& p1 = asv.get_position();
    const Geometry::Coordinates3D& attitude = asv.get_attitude();
    // Instantiate a Rotation2D object
    Eigen::Rotation2D<double> rot(attitude.keys.z);
    // Define the direction vector for heading
    const Eigen::Vector2d v1 = rot * Eigen::Vector2d(1.0, 0.0);

    const Eigen::Vector2d v_asv_position(p1.keys.x, p1.keys.y);
    const Eigen::Vector2d v_waypoint(waypoint.keys.x, waypoint.keys.y);

    // Compute direction vector
    const Eigen::Vector2d v2 = v_waypoint - v_asv_position;

    // Angle between the two vectors
    const double theta_1 = std::atan2(v1.y(), v1.x());
    const double theta_2 = std::atan2(v2.y(), v2.x());
    const double theta = theta_1 - theta_2;

    return theta;
}


double ASVLite::RudderController::get_rudder_angle(const Asv& asv, const ASVLite::Geometry::Coordinates3D& waypoint) {
    // Compute the relative angle between the vehicle heading and the waypoint.
    const double theta = get_relative_heading(asv, waypoint);
    // Set error as the difference of the current heading and the desired heading.
    previous_error = error;
    error = theta;
    constexpr double gamma = 0.7; // Rate at which the past errors reduces.
    cumulative_error = error + (gamma * cumulative_error);
    delta_error = error - previous_error;
    // Compute the rudder angle
    const Eigen::Vector3d E(error, cumulative_error, delta_error); // P, I, D errors.
    double phi = std::clamp(K.dot(E), -max_rudder_angle, max_rudder_angle); // Limit the rudder angle within the range (-PI/6, PI/6)

    return phi; // radians
}


double ASVLite::RudderController::simulate_wave_glider(const double significant_wave_ht, const double asv_heading, const double P, const double I, const double D) const {
    // Init waves
    const int rng_seed = 1;
    const int num_component_waves = 15;
    const double predominant_wave_heading = 0.0;
    const SeaSurface sea_surface {significant_wave_ht, predominant_wave_heading, rng_seed, num_component_waves};
    // Init ASV
    const Geometry::Coordinates3D start_position {100.0, 100.0, 0.0};
    const Geometry::Coordinates3D attitude {0.0, 0.0, asv_heading};
    Asv asv {asv_spec, &sea_surface, start_position, attitude};
    // Init rudder controller
    RudderController rudder_controller {asv_spec, {P, I, D}};
    // Simulate
    const Geometry::Coordinates3D waypoint {100.0, 10000.0, 0.0};
    const double sim_duration = 2.0 * 60.0; // Sec
    double heading_error = 0.0;
    while(asv.get_time() < sim_duration) {
        const double rudder_angle = rudder_controller.get_rudder_angle(asv, waypoint);
        const std::pair<Geometry::Coordinates3D, Geometry::Coordinates3D> thrust_position_magnitude = get_wave_glider_thrust(asv, rudder_angle, significant_wave_ht);
        asv.step_simulation(thrust_position_magnitude.first, thrust_position_magnitude.second);
        // Compute error in heading
        const double error = rudder_controller.get_relative_heading(asv, waypoint);
        heading_error += (error*error);
    }
    const int num_sim_steps = sim_duration / asv.get_time_step_size() * 1000;
    double rms_error = sqrt(heading_error/num_sim_steps);
    return rms_error;
}


void ASVLite::RudderController::tune_controller_gradient_descent(const double lower_bound, const double upper_bound, const double step_size) {
    std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    std::filesystem::path results_dir = root_dir/"data"/"rudder_controller_tuning";
    if (!std::filesystem::exists(results_dir)) {
        std::filesystem::create_directory(results_dir);
    }
    
    // Open the results file to write data
    std::filesystem::path result_file_path = results_dir/("gradient_descent.csv");
    std::ofstream result_file(result_file_path); 

    if (!result_file.is_open()) {
        throw std::runtime_error("Could not open result file - " + result_file_path.string());
    } else {
        result_file << "P,I,D,error_avg\n";
    }

    const double delta = step_size;
    // double P_current = K[0], I_current = K[1], D_current = K[2];
    // Create a thread-local random engine
    const int rand_seed = 1;
    thread_local std::mt19937 rng(rand_seed); 
    // Define a uniform distribution
    std::uniform_int_distribution<int> dist(lower_bound, upper_bound);
    // Generate a random number
    double P_current = dist(rng);
    double I_current = dist(rng);
    double D_current = dist(rng);

    const size_t num_iterations = 30;
    for (int n = 0; n < num_iterations; ++n) {

        std::vector<std::vector<double>> PIDs;
        for (double P : {P_current - delta, P_current, P_current + delta}) {
            for (double I : {I_current - delta, I_current, I_current + delta}) {
                for (double D : {D_current - delta, D_current, D_current + delta}) {
                    PIDs.push_back({P, I, D});
                }
            }
        }

        std::vector<std::vector<double>> costs;
        for (auto& PID : PIDs) {
            double P = std::max(PID[0], 0.0);// Prevent -ve value
            double I = std::max(PID[1], 0.0);// Prevent -ve value
            double D = std::max(PID[2], 0.0);// Prevent -ve value
            std::vector<std::future<double>> futures;
            for (double significant_wave_ht = 1.0; significant_wave_ht < 10.0; significant_wave_ht += 2.0) {
                for (double asv_heading = 0.0; asv_heading < 360.0; asv_heading += 45.0) {
                    futures.push_back(std::async(std::launch::async, &ASVLite::RudderController::simulate_wave_glider, this, significant_wave_ht, asv_heading * M_PI / 180, P, I, D));
                }
            }
            std::vector<double> results;
            for (auto& f : futures) results.push_back(f.get());
            double avg_cost = std::accumulate(results.begin(), results.end(), 0.0) / results.size();
            costs.push_back({P, I, D, avg_cost});
        }

        const std::vector<double> min_K = *std::min_element(costs.begin(), costs.end(), [](const std::vector<double>& a, const std::vector<double>& b) {
            return a[3] < b[3];
        });

        P_current = min_K[0];
        I_current = min_K[1];
        D_current = min_K[2];
        K = {P_current, I_current, D_current};

        result_file << P_current << "," << I_current << "," << D_current << "," << min_K[3] << "\n";
        std::cout << "P = " << P_current << ", I = " << I_current << ", D = " << D_current << ", error = " << min_K[3] << "\n";
    }
    result_file.close();
}


void ASVLite::RudderController::tune_controller_exhaustive_search(const double lower_bound, const double upper_bound, const double step_size) {
    std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    std::filesystem::path results_dir = root_dir/"data"/"rudder_controller_tuning";
    if (!std::filesystem::exists(results_dir)) {
        std::filesystem::create_directory(results_dir);
    }
    
    // Open the results file to write data
    std::filesystem::path result_file_path = results_dir/("exhaustive_search.csv");
    std::ofstream result_file(result_file_path); 

    if (!result_file.is_open()) {
        throw std::runtime_error("Could not open result file - " + result_file_path.string());
    } else {
        result_file << "P,I,D,error_avg\n";
    }


    std::vector<std::vector<double>> PIDs;
    for (double P = lower_bound; P < upper_bound; P+=step_size) {
        for (double I = lower_bound; I < upper_bound; I+=step_size) {
            for (double D = lower_bound; D < upper_bound; D+=step_size) {
                PIDs.push_back({P, I, D});
            }
        }
    }

    std::vector<std::vector<double>> costs;
    for (auto& PID : PIDs) {
        double P = PID[0], I = PID[1], D = PID[2];
        std::vector<std::future<double>> futures;
        for (double significant_wave_ht = 1.0; significant_wave_ht < 10.0; significant_wave_ht += 2.0) {
            for (double asv_heading = 0.0; asv_heading < 360.0; asv_heading += 45.0) {
                futures.push_back(std::async(std::launch::async, &ASVLite::RudderController::simulate_wave_glider, this, significant_wave_ht, asv_heading * M_PI / 180, P, I, D));
            }
        }
        std::vector<double> results;
        for (auto& f : futures) results.push_back(f.get());
        double avg_cost = std::accumulate(results.begin(), results.end(), 0.0) / results.size();
        costs.push_back({P, I, D, avg_cost});
        result_file << P << "," << I << "," << D << "," << avg_cost << "\n";
        std::cout << "P = " << P << ", I = " << I << ", D = " << D << ", error = " << avg_cost << "\n";
    }

    const std::vector<double> min_K = *std::min_element(costs.begin(), costs.end(), [](const std::vector<double>& a, const std::vector<double>& b) {
        return a[3] < b[3];
    });

    K = {min_K[0], min_K[1], min_K[2]};
    std::cout << "Best result - P = " << K[0] << ", I = " << K[1] << ", D = " << K[2] << ", error = " << K[3] << "\n";

    result_file.close();
}