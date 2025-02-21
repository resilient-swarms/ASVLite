#pragma once

#include "geometry.h"
#include "regular_wave.h"
#include <vector>
#include <random>

namespace ASVLite {

    class SeaSurface {
        public:
            SeaSurface(const double significant_wave_height, const double predominant_wave_heading, const int random_number_seed, const int num_component_waves);
            double get_elevation(const Geometry::Coordinates3D& location, const double time) const;
            double get_mean_wavenumber() const;

        public:
            // Input variables
            // ---------------
            const double significant_wave_height; // m
            const double predominant_wave_heading; // radians.
            const long random_number_seed;
            const int num_component_waves;   

            // Calculated variables
            // --------------------  
            const double peak_spectral_frequency; // Hz
            const double min_spectral_frequency; // Hz
            const double max_spectral_frequency; // Hz
            const double min_spectral_wave_heading; // radians
            const double max_spectral_wave_heading; // radians
            const std::vector<RegularWave> component_waves;

        private:
            double calculate_peak_spectral_frequency() const;
            std::vector<RegularWave> calculate_wave_spectrum() const;
    };
    
}