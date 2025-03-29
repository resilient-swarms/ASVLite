#pragma once

#include <filesystem>
#include <Eigen/Dense>
#include "constants.h"
#include "geometry.h"
#include "asv.h"


namespace ASVLite {

    /**
     * @class RudderController
     * @brief Controls the rudder angle for the ASV based on its position, attitude, and waypoint.
     * 
     * This class implements a rudder controller that calculates the optimal rudder angle to steer the ASV 
     * towards a desired waypoint. It uses a combination of proportional, integral, and derivative (PID) control 
     * strategies, with methods for both gradient descent and exhaustive search for tuning the control gains.
     */
    class RudderController {
        public:
            /**
             * @brief Constructor for the RudderController class.
             * 
             * Initialises the rudder controller with the ASV specification and initial control gains.
             * 
             * @param asv_spec Specification of the ASV, including geometry and other parameters.
             * @param initial_K Initial values for the control gains (P, I, D).
             */
            RudderController(const AsvSpecification& asv_spec, const Eigen::Vector3d& initial_K);
            
            /**
             * @brief Calculates the rudder angle to steer the ASV towards the waypoint.
             * 
             * This function computes the required rudder angle based on the current position, attitude, 
             * and the target waypoint.
             * 
             * @param asv_position Current position of the ASV.
             * @param asv_attitude Current attitude (roll, pitch, yaw) of the ASV.
             * @param waypoint Target waypoint coordinates.
             * @return double The calculated rudder angle in radians.
             */
            double get_rudder_angle(const Geometry::Coordinates3D& asv_position, 
                                    const Geometry::Coordinates3D& asv_attitude, 
                                    const Geometry::Coordinates3D& waypoint);
            
            /**
             * @brief Computes the rudder angle to steer the ASV towards the desired heading using PID control.
             * 
             * @param desired_heading The target heading the ASV needs to achieve (in radians).
             * @param asv_attitude Current attitude of the ASV, including roll, pitch, and yaw (in radians).
             * @return double The required rudder angle (in radians) to steer the ASV towards the desired heading.
             */
            double get_rudder_angle( const double desired_heading, const Geometry::Coordinates3D& asv_attitude);
            
            /**
             * @brief Tunes the controller using a local search strategy.
             * 
             * This function tunes the controller's proportional, integral, and derivative (PID) gains 
             * by performing a grid-based local search around the current parameter values. 
             * For each iteration, it evaluates the average control error across a range of simulated conditions 
             * and selects the parameter set with the lowest cost.
             * 
             * This is not a true gradient descent method, as it does not compute or follow gradients, 
             * but instead uses a discrete search in the local neighborhood.
             * 
             * @param lower_bound Lower bound of the search range for the gains.
             * @param upper_bound Upper bound of the search range for the gains.
             * @param step_size Size of the neighborhood search step.
             */
            void tune_controller_local_search(const double lower_bound, const double upper_bound, const double step_size);
                        
            /**
             * @brief Tunes the controller using exhaustive search.
             * 
             * This function tunes the controller's gains by exhaustively testing all possible values within the 
             * specified range. The best performing gains are selected based on the error.
             * 
             * @param lower_bound Lower bound of the search range for the gains.
             * @param upper_bound Upper bound of the search range for the gains.
             * @param step_size Step size for the exhaustive search.
             */
            void tune_controller_exhaustive_search(const double lower_bound, const double upper_bound, const double step_size);
        
        private:
            /**
             * @brief Computes the relative heading between the ASV and the waypoint.
             * 
             * This function calculates the relative heading between the ASV’s current position and the desired waypoint.
             * 
             * @param asv_position Current position of the ASV.
             * @param asv_attitude Current attitude of the ASV.
             * @param waypoint Target waypoint coordinates.
             * @return double The calculated relative heading in radians.
             */
            double get_relative_heading(const Geometry::Coordinates3D& asv_position, 
                                        const Geometry::Coordinates3D& asv_attitude, 
                                        const Geometry::Coordinates3D& waypoint) const;
            
            /**
             * @brief Simulates the wave glider's behavior based on wave height, heading, and PID gains.
             * 
             * This function simulates the wave glider's performance for a given significant wave height, heading, 
             * and control gains (P, I, D), and returns a performance metric.
             * 
             * @param significant_wave_ht Significant wave height affecting the wave glider’s performance.
             * @param asv_heading The heading of the ASV.
             * @param P Proportional gain.
             * @param I Integral gain.
             * @param D Derivative gain.
             * @return double A performance metric based on the simulation.
             */
            double simulate_wave_glider(const double significant_wave_ht, const double asv_heading, const double P, const double I, const double D) const;
        
        private:
            /** @brief Specification of the ASV (geometry and other parameters). */
            const AsvSpecification asv_spec;
            
            /** @brief Maximum allowable rudder angle (30 degrees). */
            constexpr static double max_rudder_angle = M_PI / 6.0;
            
            /** @brief Vector of control gains (P, I, D). */
            Eigen::Vector3d K;
            
            /** @brief Current error value for the control loop. */
            double error = 0.0;
            
            /** @brief Previous error value for the control loop. */
            double previous_error = 0.0;
            
            /** @brief Cumulative error for the integral term in PID control. */
            double cumulative_error = 0.0;
            
            /** @brief Change in error for the derivative term in PID control. */
            double delta_error = 0.0;
    };

}


