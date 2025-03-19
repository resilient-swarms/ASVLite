#pragma once

#include <filesystem>
#include <Eigen/Dense>
#include "constants.h"
#include "geometry.h"
#include "asv.h"


namespace ASVLite {

    class RudderController {
        public:
            RudderController(const AsvSpecification& asv_spec, const Eigen::Vector3d& initial_K);
            double get_rudder_angle(const Asv& asv, const Geometry::Coordinates3D& waypoint);
            void tune_controller_gradient_descent(const double lower_bound, const double upper_bound, const double step_size);
            void tune_controller_exhaustive_search(const double lower_bound, const double upper_bound, const double step_size);
        
        private:
            double get_relative_heading(const Asv& asv, const Geometry::Coordinates3D& waypoint) const;
            double simulate_wave_glider(const double significant_wave_ht, const double asv_heading, const double P, const double I, const double D) const;
        
        private:
            const AsvSpecification asv_spec;
            constexpr static double max_rudder_angle = M_PI / 6.0; // 30 degrees
            Eigen::Vector3d K;
            double error = 0.0;
            double previous_error = 0.0;
            double cumulative_error = 0.0;
            double delta_error = 0.0;
        };

}


