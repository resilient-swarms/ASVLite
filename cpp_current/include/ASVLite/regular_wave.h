#pragma once

#include "geometry.h"

namespace ASVLite {

    class RegularWave {
        public:
            // Constructor
            RegularWave(const double amplitude, const double frequency, const double phase_lag, const double heading);

            double get_phase(const Geometry::Coordinates3D& location, const double time) const;
            double get_elevation(const Geometry::Coordinates3D& location, const double time) const;
            double get_wave_pressure(const Geometry::Coordinates3D& location, const double time) const;

            // Input variables
            // ---------------
            const double amplitude; // m
            const double frequency; // Hz
            const double phase_lag; // radian
            const double heading; // radian
            // Calculated variables
            // --------------------
            const double height; // m
            const double time_period; // sec
            const double wave_length; // m
            const double wave_number;
    };

}