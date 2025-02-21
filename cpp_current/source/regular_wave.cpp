#include "ASVLite/constants.h"
#include "ASVLite/regular_wave.h"
#include <cmath>
#include <stdexcept>


ASVLite::RegularWave::RegularWave(const double amplitude, const double frequency, const double phase_lag, const double heading) :
amplitude {amplitude},
frequency {frequency},
phase_lag {phase_lag},
heading {ASVLite::Geometry::normalise_angle_2PI(heading)},
height {2.0 * amplitude},
time_period {1/frequency},
wave_length {(ASVLite::Constants::G * time_period * time_period)/(2.0 * M_PI)}, 
wave_number {(2.0 * M_PI)/wave_length} {
}


double ASVLite::RegularWave::get_phase(const ASVLite::Geometry::Coordinates3D& location, const double time) const {
    if(time < 0.0) {
        throw std::invalid_argument("Time cannot be negative.");
    }

    // elevation = amplitude * cos(A - B + phase)
    // where:
    // A = wave_number * (x * cos(direction) + y * sin(direction))
    // B = 2 * PI * frequency * time
    //
    // NOTE:
    // In the coordinate system that we use here, angular measurements are made 
    // with respect to north which is represented by y-axis and not x-axis.
    // Therefore the above formula needs to be modified as:
    // A = wave_number * (x * sin(direction) + y * cos(direction))  
    double A = wave_number * (location.keys.x * sin(heading) + location.keys.y * cos(heading));
    double B = 2.0 * M_PI * frequency * time;
    return (A - B + phase_lag);
}


double ASVLite::RegularWave::get_elevation(const ASVLite::Geometry::Coordinates3D& location, const double time) const {
    if(time < 0.0) {
        throw std::invalid_argument("Time cannot be negative.");
    }

    double wave_phase = get_phase(location, time);
    return (amplitude * cos(wave_phase));
}


double ASVLite::RegularWave::get_wave_pressure(const Geometry::Coordinates3D& location, const double time) const {

    const double phase = get_phase(location, time);
    return -Constants::SEA_WATER_DENSITY * Constants::G * amplitude * cos(phase); 
}