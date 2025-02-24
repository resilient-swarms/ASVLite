#include "ASVLite/rudder_controller.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>


ASVLite::RudderController::RudderController(const ASVLite::AsvSpecification& asv_spec, const Eigen::Vector3d& initial_K) :
asv_spec {asv_spec} {
    this->K = initial_K;
    // std::filesystem::path root_dir = std::filesystem::current_path().parent_path();
    // std::filesystem::path out_dir = root_dir/ "results" / "rudder_controller" / "tuning";
    // if (!std::filesystem::exists(out_dir)) {
    //     std::filesystem::create_directory(out_dir);
    // }
    // else {
    //     throw std::runtime_error("Could not create output directory - " + out_dir.string());
    // }
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

    // Compute dot product
    const double dot_product = v1.dot(v2);

    // Compute magnitudes
    const double magnitude_v1 = v1.norm();
    const double magnitude_v2 = v2.norm();

    // Ensure no division by zero
    if (magnitude_v1 == 0 || magnitude_v2 == 0) {
        throw std::runtime_error("One of the lines has zero length!");
    }

    // Compute cosine of the angle
    const double cos_theta = std::clamp(dot_product / (magnitude_v1 * magnitude_v2), -1.0, 1.0); // Clamp value to avoid numerical instability

    // Compute angle in radians
    double theta = std::acos(cos_theta);

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

