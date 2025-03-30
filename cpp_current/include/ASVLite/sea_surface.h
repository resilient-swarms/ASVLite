#pragma once

#include <cmath>
#include <stdexcept>
#include <thread>
#include <random>
#include "geometry.h"
#include "regular_wave.h"
#include "ASVLite/constants.h"


namespace ASVLite {

    /**
     * @brief Models an irregular sea surface as a superposition of N regular component waves.
     * 
     * @tparam N Number of regular component waves in the wave spectrum. Must be an odd number >= 3.
     */
    template<size_t N>
    class SeaSurface {

        public:

            /**
             * @brief Constructs a sea surface model with a given wave height and heading.
             * 
             * @param significant_wave_height Significant wave height (in meters) of the irregular sea surface (must be non-negative).
             * @param predominant_wave_heading Predominant wave heading in radians, measured clockwise from geographic north.
             * @param random_number_seed Seed for the random number generator used in wave spectrum generation.
             */
            SeaSurface(const double significant_wave_height, const double predominant_wave_heading, const int random_number_seed) : 
            significant_wave_height {significant_wave_height},
            predominant_wave_heading {Geometry::switch_angle_frame(predominant_wave_heading)}, // Covert angle to counter-clockwise from geographic east (x-axis). 
            random_number_seed {random_number_seed},
            peak_spectral_frequency {calculate_peak_spectral_frequency()},
            min_spectral_frequency {0.652 * peak_spectral_frequency},
            max_spectral_frequency {5.946 * peak_spectral_frequency},
            min_spectral_wave_heading {Geometry::normalise_angle_PI(predominant_wave_heading - M_PI/2.0)},
            max_spectral_wave_heading {Geometry::normalise_angle_PI(predominant_wave_heading + M_PI/2.0)},
            component_waves {calculate_wave_spectrum()} {
            }


            /**
             * @brief Computes the sea surface elevation at a given location and time.
             * 
             * @param location 3D coordinates (in meters) where elevation is evaluated.
             * @param time Time in seconds since the start of the simulation (must be non-negative).
             * @return double Sea surface elevation in meters at the specified location and time.
             * 
             * @throws std::invalid_argument if time is negative.
             */
            double get_elevation(const Geometry::Coordinates3D& location, const double time) const {
                if(time < 0.0) {
                    throw std::invalid_argument("Time cannot be negative.");
                }
                const Eigen::Vector<double, N> component_waves_elevation = component_waves.get_elevation(location, time);
                double elevation = component_waves_elevation.sum();
                return elevation;
            }


            /**
             * @brief Computes the mean wavenumber for the sea state.
             * 
             * @return double Mean wavenumber computed as the average of all component wave numbers.
             */
            double get_mean_wavenumber() const {
                double mean_wavenumber = component_waves.wave_number.sum()/N;
                return mean_wavenumber;
            }


            // Input variables
            // ---------------
            /** @brief Significant wave height of the sea state (in meters). */
            const double significant_wave_height;

            /** @brief Predominant wave heading in radians, measured clockwise from geographic north. */
            const double predominant_wave_heading;

            /** @brief Seed for the random number generator used in wave component generation. */
            const long random_number_seed;

            // Calculated variables
            // --------------------
            /** @brief Peak spectral frequency of the wave energy distribution (in Hz). */
            const double peak_spectral_frequency;

            /** @brief Minimum spectral frequency considered in the wave spectrum (in Hz). */
            const double min_spectral_frequency;

            /** @brief Maximum spectral frequency considered in the wave spectrum (in Hz). */
            const double max_spectral_frequency;

            /** @brief Minimum wave heading considered in the wave spectrum (in radians). */
            const double min_spectral_wave_heading;

            /** @brief Maximum wave heading considered in the wave spectrum (in radians). */
            const double max_spectral_wave_heading;

            /** @brief Collection of N regular component waves representing the sea surface. */
            const RegularWave<N> component_waves;


        private:

            /**
             * @brief Calculates the peak spectral frequency of the sea state.
             * 
             * @return double Peak spectral frequency in Hz.
             */
            double calculate_peak_spectral_frequency() const {
                constexpr double alpha = 0.0081;
                const double B = 4.0 * alpha * Constants::G*Constants::G / (pow(2.0*M_PI, 4.0) * significant_wave_height*significant_wave_height);
                const double f_p = 0.946 * pow(B, 0.25);
                return f_p;
            }

            
            /**
             * @brief Generates a set of N regular wave components representing an irregular sea state.
             * 
             * The function divides the frequency range symmetrically around the peak spectral frequency
             * and assigns each frequency band a wave heading and amplitude using the Bretschneider spectrum.
             * Random phase lags are sampled uniformly from [0, PI]. The result is a spectrum of waves
             * with realistic distribution in both frequency and direction.
             * 
             * The wave spectrum is constructed as follows:
             * - Frequencies are distributed on either sides of the peak.
             * - Wave headings are spread across +-PI/2 around the predominant wave heading.
             * - The peak frequency aligns with the predominant wave direction, with energy diminishing symmetrically toward both sides.
             * - Amplitudes are computed from spectral density using the Bretschneider model.
             * - Phase lags are randomized.
             * 
             * @return RegularWave<N> A collection of N wave components modeling the sea surface.
             * 
             * @throws std::invalid_argument if N is not an odd number greater than or equal to 3.
             * 
             * @ref Proceedings of the 23rd ITTC - Vol II, Tables A.2, A.3.
             */
            RegularWave<N> calculate_wave_spectrum() const {
                if(N % 2 == 0 or N < 3) {
                    throw std::invalid_argument("Number of component waves must be an odd number greater than or equal to 3.");
                }
                // Init random number generator
                // Create a thread-local random engine
                thread_local std::mt19937 rng(random_number_seed); 
                // Define a uniform distribution
                std::uniform_real_distribution<double> dist(0.0, M_PI);
                // Compute step size for frequency and heading
                const int half_count = (N-1)/2; // Half of the component wave count
                const double frequency_band_size_peak = (max_spectral_frequency - min_spectral_frequency) / N;
                const double peak_freq_band_low_limit = peak_spectral_frequency - frequency_band_size_peak/2;
                const double peak_freq_band_upp_limit = peak_spectral_frequency + frequency_band_size_peak/2;
                const double frequency_band_size_min_to_peak = (peak_freq_band_low_limit  - min_spectral_frequency) / half_count;
                const double frequency_band_size_peak_to_max = (max_spectral_frequency - peak_freq_band_upp_limit) / half_count;
                const double wave_heading_increment = M_PI/N;
                
                // Lambda function to construct the wave
                Eigen::Vector<double, N> amplitudes;
                Eigen::Vector<double, N> frequencys;
                Eigen::Vector<double, N> phases;
                Eigen::Vector<double, N> wave_headings;
                auto construct_regular_wave_parameters = [&](const double freq, const double freq_band_size, const double wave_heading, size_t i) {
                    // Bretschneider spectrum
                    // Ref: Proceedings of the 23rd ITTC - Vol II, Table A.2, A.3.
                    // S(f) = (A/f^5) exp(-B/f^4)
                    // A = alpha g^2 (2 PI)^-4
                    // B = beta (2PI U/g)^-4
                    // alpha = 0.0081
                    // beta = 0.74
                    // f_p = 0.946 B^(1/4)
                    // U = wind speed in m/s
                    constexpr double alpha = 0.0081;
                    constexpr double beta = 0.74;
                    const double A = alpha * Constants::G*Constants::G * pow(2.0*M_PI, -4.0);
                    const double B = 4.0 * alpha * Constants::G*Constants::G / (pow(2.0*M_PI, 4.0) * significant_wave_height*significant_wave_height);
                    const double S = (A / pow(freq, 5.0)) * exp(-B / pow(freq, 4.0)) * freq_band_size;
                    const double amplitude = sqrt(2.0 * S); 
                    const double phase = dist(rng);  // Generate a random number
                    amplitudes(i) = amplitude;
                    frequencys(i) = freq;
                    phases(i) = phase;
                    wave_headings(i) = Geometry::switch_angle_frame(wave_heading); // Convert the heading to be relative to North, as the regular wave interface expects it in this format.
                };
            
                // Create wave parameters for - min to peak freq
                double mu = -M_PI/2.0;
                for(size_t i = 0; i < half_count; ++i) {
                    const double freq = min_spectral_frequency + (i * frequency_band_size_min_to_peak) + frequency_band_size_min_to_peak/2.0;
                    const double mu = M_PI/2.0 - (i * wave_heading_increment) + wave_heading_increment/2.0;
                    const double wave_heading = Geometry::normalise_angle_PI(predominant_wave_heading + mu);
                    construct_regular_wave_parameters(freq, frequency_band_size_min_to_peak, wave_heading, i);
                }
                // Create wave parameters for - peak
                construct_regular_wave_parameters(peak_spectral_frequency, frequency_band_size_peak, predominant_wave_heading, half_count);
                // Create wave parameters for - peak to max freq
                for(size_t i = 0; i < half_count; ++i) {
                    const double freq = peak_freq_band_upp_limit + (i * frequency_band_size_peak_to_max) + frequency_band_size_peak_to_max/2.0;
                    const double mu = (i * wave_heading_increment) + wave_heading_increment/2.0;
                    const double wave_heading = Geometry::normalise_angle_PI(predominant_wave_heading - mu);
                    construct_regular_wave_parameters(freq, frequency_band_size_peak_to_max, wave_heading, half_count+i);
                }
                // Create regular waves
                RegularWave<N> spectrum {amplitudes, frequencys, phases, wave_headings}; 
                return spectrum;
            }
            
    };
    
}