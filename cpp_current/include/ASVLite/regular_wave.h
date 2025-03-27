#pragma once

#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include "ASVLite/constants.h"
#include "geometry.h"

namespace ASVLite {

    template<size_t N>
    class RegularWave {

        public:

            /**
             * @brief Constructs a collection of regular ocean waves.
             * 
             * @param amplitude Wave amplitudes in meters.
             * @param frequency Wave frequencies in Hz.
             * @param phase_lag Phase lags in radians.
             * @param heading Directions of wave propagation in radians, clockwise from geographic north.
             */
            RegularWave(const Eigen::Vector<double, N> amplitude, 
                        const Eigen::Vector<double, N> frequency, 
                        const Eigen::Vector<double, N> phase_lag, 
                        const Eigen::Vector<double, N> heading) :
            amplitude {amplitude},
            frequency {frequency},
            phase_lag {phase_lag},
            heading {heading.unaryExpr(&ASVLite::Geometry::normalise_angle_2PI)},
            height {2.0 * amplitude},
            time_period {frequency.array().inverse()},
            wave_length {(ASVLite::Constants::G * time_period.array().square())/(2.0 * M_PI)}, 
            wave_number {(2.0 * M_PI) * wave_length.array().inverse()} {
            }


            /**
             * @brief Computes the phase of the wave at a specific location and time.
             * 
             * @param location 3D coordinates (in meters) where the wave phase is evaluated.
             * @param time Time in seconds since the start of the simulation (must be non-negative).
             * @return Eigen::Vector<double, N> Wave phase in radians at the given location and time.
             * 
             * @throws std::invalid_argument if time is negative.
             */
            Eigen::Vector<double, N> get_phase(const ASVLite::Geometry::Coordinates3D& location, const double time) const {
                if(time < 0.0) {
                    throw std::invalid_argument("Time cannot be negative.");
                }
                // elevation = amplitude * cos(A - B + phase)
                // where:
                // A = wave_number * (x * cos(direction) + y * sin(direction))
                // B = 2 * PI * frequency * time
                // NOTE:
                // In the coordinate system that we use here, angular measurements are made 
                // with respect to north which is represented by y-axis and not x-axis.
                // Therefore the above formula needs to be modified as:
                // A = wave_number * (x * sin(direction) + y * cos(direction))  
                const Eigen::Vector<double, N> A = wave_number.array() * (location.keys.x * heading.array().sin() + location.keys.y * heading.array().cos());
                const Eigen::Vector<double, N> B = 2.0 * M_PI * frequency * time;
                return (A - B + phase_lag);
            }


            /**
             * @brief Computes the wave elevation at a specific location and time.
             * 
             * @param location 3D coordinates (in meters) where the elevation is evaluated.
             * @param time Time in seconds since the start of the simulation (must be non-negative).
             * @return Eigen::Vector<double, N> Wave elevation in meters at the given location and time.
             * 
             * @throws std::invalid_argument if time is negative.
             */
            Eigen::Vector<double, N> get_elevation(const ASVLite::Geometry::Coordinates3D& location, const double time) const {
                if(time < 0.0) {
                    throw std::invalid_argument("Time cannot be negative.");
                }
                const Eigen::Vector<double, N> wave_phase = get_phase(location, time);
                return (amplitude.array() * wave_phase.array().cos());
            }


            /**
             * @brief Computes the wave pressure amplitude at a given location and time.
             * 
             * This function calculates the dynamic pressure component due to wave motion,
             * assuming linear wave theory and neglecting depth attenuation.
             * 
             * @param location 3D coordinates (in meters) where the pressure is evaluated.
             * @param time Time in seconds since the start of the simulation (must be non-negative).
             * @return Eigen::Vector<double, N> Wave pressure amplitude in N/m² at the specified location and time.
             */
            Eigen::Vector<double, N> get_wave_pressure(const Geometry::Coordinates3D& location, const double time) const {

                const Eigen::Vector<double, N> phase = get_phase(location, time);
                return -Constants::SEA_WATER_DENSITY * Constants::G * amplitude.array() * phase.array().cos(); 
            }

            
            // Input variables
            // ---------------
            /** @brief Amplitudes of the wave components (m). */
            const Eigen::Vector<double, N> amplitude;

            /** @brief Frequencies of the wave components (Hz). */
            const Eigen::Vector<double, N> frequency;

            /** @brief Phase lags of the wave components (radian). */
            const Eigen::Vector<double, N> phase_lag;

            /** @brief Directions of wave propagation (radian, clockwise from geographic north). */
            const Eigen::Vector<double, N> heading;

            // Calculated variables
            // --------------------
            /** @brief Wave heights, 2 × amplitude (m). */
            const Eigen::Vector<double, N> height;

            /** @brief Time periods, inverse of frequency (sec). */
            const Eigen::Vector<double, N> time_period;

            /** @brief Wavelengths computed via linear wave theory (m). */
            const Eigen::Vector<double, N> wave_length;

            /** @brief Wave numbers, 2π ÷ wavelength. */
            const Eigen::Vector<double, N> wave_number;
    };

}