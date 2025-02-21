#pragma once

#include "geometry.h"
#include "regular_wave.h"
#include <vector>
#include <random>

namespace ASVLite {

    class SeaSurface {
        public:
            // Constructor
            // param sig_wave_height is the significant wave height, in meter, of the irregular sea surface. Value should be non-negative.
            // param wave_heading is the predominant wave heading with respect to the geographic north. The angle measured is positive 
            // in the clockwise direction such that the geographic east is at PI/2 radians to the north.
            // param rand_seed is the seed for random number generator. 
            // param count_component_waves is the number of regular component waves in the wave spectrum. Value should be an odd number.
            SeaSurface(const double significant_wave_height, const double predominant_wave_heading, const int random_number_seed, const int num_component_waves);

            // Get sea surface elevation at the given location for the given time. 
            // param location with coordinates in meter, at which the elevation is to be computed.
            // param time for which the elevation is to be computed. Time is measured in seconds from start of simulation.
            // returns wave elevation in meter. 
            double get_elevation(const Geometry::Coordinates3D& location, const double time) const;

            // Function to get the mean wavenumber for the sea state.
            double get_mean_wavenumber() const;

        public:
            // Input variables
            // ---------------
            const double significant_wave_height;  // m
            const double predominant_wave_heading; // radians.
            const long random_number_seed;
            const int num_component_waves;   

            // Calculated variables
            // --------------------  
            const double peak_spectral_frequency; // Hz
            const double min_spectral_frequency;  // Hz
            const double max_spectral_frequency;  // Hz
            const double min_spectral_wave_heading; // radians
            const double max_spectral_wave_heading; // radians
            const std::vector<RegularWave> component_waves;

        private:
            double calculate_peak_spectral_frequency() const;
            std::vector<RegularWave> calculate_wave_spectrum() const;
    };
    
}