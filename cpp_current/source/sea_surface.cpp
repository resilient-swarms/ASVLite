#include "ASVLite/constants.h"
#include "ASVLite/sea_surface.h"
#include <cmath>
#include <stdexcept>

ASVLite::SeaSurface::SeaSurface(const double significant_wave_height, const double predominant_wave_heading, const int random_number_seed, const int num_component_waves) : 
significant_wave_height {significant_wave_height},
predominant_wave_heading {Geometry::normalise_angle_2PI(predominant_wave_heading)},
random_number_seed {random_number_seed},
num_component_waves {num_component_waves},
peak_spectral_frequency {calculate_peak_spectral_frequency()},
min_spectral_frequency {0.652 * peak_spectral_frequency},
max_spectral_frequency {5.946 * peak_spectral_frequency},
min_spectral_wave_heading {Geometry::normalise_angle_2PI(predominant_wave_heading - M_PI/2.0)},
max_spectral_wave_heading {Geometry::normalise_angle_2PI(predominant_wave_heading + M_PI/2.0)},
component_waves {calculate_wave_spectrum()} {
}


double ASVLite::SeaSurface::calculate_peak_spectral_frequency() const {
    constexpr double alpha = 0.0081;
    const double B = 4.0 * alpha * Constants::G*Constants::G / (pow(2.0*M_PI, 4.0) * significant_wave_height*significant_wave_height);
    const double f_p = 0.946 * pow(B, 0.25);
    return f_p;
}

std::vector<ASVLite::RegularWave> ASVLite::SeaSurface::calculate_wave_spectrum() const {
    if(num_component_waves % 2 == 0 or num_component_waves < 3) {
        throw std::invalid_argument("Number of component waves must be an odd number greater than or equal to 3.");
    }

    // Create regular waves
    std::vector<ASVLite::RegularWave> spectrum; 
    spectrum.reserve(num_component_waves);

    // Compute step size for frequency and heading
    const int half_count = (num_component_waves-1)/2; // Half of the component wave count
    const double frequency_band_size_peak = (max_spectral_frequency - min_spectral_frequency) / num_component_waves;
    const double peak_freq_band_low_limit = peak_spectral_frequency - frequency_band_size_peak/2;
    const double peak_freq_band_upp_limit = peak_spectral_frequency + frequency_band_size_peak/2;
    const double frequency_band_size_min_to_peak = (peak_freq_band_low_limit  - min_spectral_frequency) / half_count;
    const double frequency_band_size_peak_to_max = (max_spectral_frequency - peak_freq_band_upp_limit) / half_count;
    const double wave_heading_increment = M_PI/num_component_waves;

    // Init random number generator
    static std::mt19937 gen(random_number_seed); // Mersenne Twister PRNG
    static std::uniform_real_distribution<double> rand_from_uniform_dist(0.0, 2.0 * M_PI); // Random number generator that generates a random number in [0, 2PI]

    // Lambda to construct the wave
    auto construct_regular_wave = [&](const double freq, const double freq_band_size, const double wave_heading) {
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
        const double phase = rand_from_uniform_dist(gen);  // Generate a random number
        spectrum.emplace_back(RegularWave(amplitude, freq, phase, wave_heading));
    };

    // Create waves - min to peak freq
    double mu = -M_PI/2.0;
    for(size_t i = 0; i < half_count; ++i) {
        const double freq = min_spectral_frequency + (i * frequency_band_size_min_to_peak) + frequency_band_size_min_to_peak/2.0;
        const double mu = -M_PI/2.0 + (i * wave_heading_increment) + wave_heading_increment/2.0;
        const double wave_heading = Geometry::normalise_angle_2PI(mu + predominant_wave_heading);
        construct_regular_wave(freq, frequency_band_size_min_to_peak, wave_heading);
    }
    // Create wave - peak
    construct_regular_wave(peak_spectral_frequency, frequency_band_size_peak, predominant_wave_heading);
    // Create waves - peak to max freq
    for(size_t i = 0; i < half_count; ++i) {
        const double freq = peak_freq_band_upp_limit + (i * frequency_band_size_peak_to_max) + frequency_band_size_peak_to_max/2.0;
        const double mu = wave_heading_increment/2.0 + i * wave_heading_increment + wave_heading_increment/2.0;
        const double wave_heading = Geometry::normalise_angle_2PI(mu + predominant_wave_heading);
        construct_regular_wave(freq, frequency_band_size_peak_to_max, wave_heading);
    }

    return spectrum;
}


double ASVLite::SeaSurface::get_elevation(const Geometry::Coordinates3D& location, const double time) const {
    if(time < 0.0) {
        throw std::invalid_argument("Time cannot be negative.");
    }

    double elevation = 0.0;
    for(const auto& regular_wave : component_waves) {
        const double regular_wave_elevation = regular_wave.get_elevation(location, time);
        elevation += regular_wave_elevation;
    }
    return elevation;
}


double ASVLite::SeaSurface::get_mean_wavenumber() const {
    double cumulative_wavenumber = 0.0;
    for(const auto& regular_wave : component_waves) {
        const double wave_number = regular_wave.wave_number;
        cumulative_wavenumber += wave_number;
    }
    double mean_wavenumber = cumulative_wavenumber/num_component_waves;
    return mean_wavenumber;
}