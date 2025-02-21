#pragma once

#include "geometry.h"

namespace ASVLite {

    class RegularWave {
        public:
            // Constructor
            // param amplitude of the wave in meter.
            // param frequency of the wave in Hz.
            // param phase of the wave in radians. 
            // param direction of propagation of the wave in radians with respect to the
            // geographic north. The angle measured is positive in the clockwise direction such that 
            // the geographic east is at PI/2 radians to the north.
            RegularWave(const double amplitude, const double frequency, const double phase_lag, const double heading);

            // Get the phase of the wave at a given point for a given time.
            // param location with all coordinates in meter, at which the phase is to be calculated.
            // param time for which the phase is to be calculated. Time is measured in seconds from 
            // the start of simulation. Time should be non-negative.
            // returns wave phase in radian.
            double get_phase(const Geometry::Coordinates3D& location, const double time) const;

            // Get elevation of the wave at a given point for a given time.
            // param location with all coordinates in meter, at which the elevation is to be computed.
            // param time for which elevation is to be computed. Time is measured in 
            // seconds from the start of simulation. Time should be non-negative.
            // returns wave elevation in meter.
            double get_elevation(const Geometry::Coordinates3D& location, const double time) const;

            // Get wave pressure amplitude at a given depth.
            // param depth in meter at which the pressure amplitude is to be computed. Depth 
            // should be positive value. 
            // returns pressure amplitude in N/m2.
            double get_wave_pressure(const Geometry::Coordinates3D& location, const double time) const;

            // Input variables
            // ---------------
            const double amplitude; // m
            const double frequency; // Hz
            const double phase_lag; // radian
            const double heading;   // radian
            // Calculated variables
            // --------------------
            const double height;      // m
            const double time_period; // sec
            const double wave_length; // m
            const double wave_number;
    };

}