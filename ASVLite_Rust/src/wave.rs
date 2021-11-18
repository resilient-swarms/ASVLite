//! Implementation of an irregular wave.

use libc;
use super::error::ValueError;
use super::geometry::Dimension;
use super::regular_wave::RegularWave;
use super::constants::{PI, G};

extern {
    fn srand(seed: libc::c_uint);
    fn rand() -> libc::c_int;
}

/// Structure to define an irregular wave. The members of the struct is initialized 
/// by the associated function new(). The member variables are all public and can be directly
/// accessed for reading. However, the member variables should not be directly accessed for writing. 
/// The struct does not provide any member function for editing the values of a member variable after 
/// the value is set. So to edit a member variable value, drop the current instance of the struct and 
/// create a new one with the desired value. Recommend assigning the return of new() to an immutable variable.
#[derive(Debug, Clone, PartialEq)]
pub struct Wave {
    /// Significant wave height in meter.
    pub significant_wave_height: f64,
    /// Wave heading in radians.
    pub heading: f64,
    /// Random number generator seed.
    pub random_number_seed: u32,
    /// Lower limit (0.1%) of the spectral energy threshold.
    pub min_spectral_frequency: f64,
    /// Upper limit (99.9%) of the spectral energy threshold.
    pub max_spectral_frequency: f64,
    /// Spectral peak frequency in Hz.
    pub peak_spectral_frequency: f64,
    /// Minimum angle, in radians, for wave heading in the spectrum.
    pub min_spectral_wave_heading: f64,
    /// Maximum angle, in radians, for wave heading in the spectrum. 
    pub max_spectral_wave_heading: f64,
    /// Wave spectrum
    pub spectrum: Vec<Vec<RegularWave>>,
}

impl Wave {
    /// Initialize a new irregular wave.
    pub fn new(significant_wave_height: f64, 
                heading: f64, 
                random_number_seed: u32, 
                count_spectral_directions: usize, 
                count_spectral_frequencies: usize) -> Result<Wave,ValueError> {
        match significant_wave_height <= 0.0 {
            true => Err(ValueError::new("Significant wave height should not be <= 0.0.")),
            false => {
                let mut min_spectral_wave_heading = heading - PI/2.0;
                let mut max_spectral_wave_heading = heading + PI/2.0;
                // wave directions should be in the range (0, 2PI)
                if min_spectral_wave_heading < 0.0 {
                    min_spectral_wave_heading += 2.0*PI; 
                }
                if max_spectral_wave_heading >= 2.0*PI {
                    max_spectral_wave_heading -= 2.0*PI;
                }

                // Bretschneider spectrum
                // Ref: Proceedings of the 23rd ITTC - Vol II, Table A.2, A.3.
                // S(f) = (A/f^5) exp(-B/f^4)
                // A = alpha g^2 (2 PI)^-4
                // B = 4 alpha g^2 / [(2 PI)^4 H_s^2]
                // alpha = 0.0081
                // beta = 0.74
                // f_p = 0.946 B^(1/4)
                // U = wind speed in m/s
                let alpha = 0.0081;
                // let beta = 0.74;
                let a = alpha * G*G * (2.0 * PI).powf(-4.0);
                let h_s = significant_wave_height;
                let b = 4.0*alpha*G*G / ((2.0*PI).powf(4.0) * h_s*h_s);
                let f_p = 0.946 * b.powf(0.25);
                let min_spectral_frequency = 0.652 * f_p;
                let max_spectral_frequency = 5.946 * f_p;

                // Create the regular waves
                let wave_0 = RegularWave::new(1.0, 1.0, 0.0, 0.0).unwrap(); // A temp wave to initialise the spectrum.
                let mut spectrum = vec![vec![wave_0; count_spectral_frequencies]; count_spectral_directions];
                let wave_heading_step_size = PI/((count_spectral_directions as f64) - 1.0);
                let frequency_step_size = (max_spectral_frequency - min_spectral_frequency) /
                                          ((count_spectral_frequencies as f64) - 1.0);
                let mut mu = -PI/2.0;
                for i in 0 .. count_spectral_directions {
                    mu += wave_heading_step_size;
                    for j in 0 .. count_spectral_frequencies {
                        let f = min_spectral_frequency + (j as f64) * frequency_step_size;
                        let s = (a/f.powf(5.0)) * (-b/f.powf(4.0)).exp() * frequency_step_size;

                        // Direction function G = (2/PI) * cos(mu)*cos(mu) * delta_mu
                        // delta_mu = wave_heading_step_size
                        let g_spectrum = (2.0/PI) * mu.cos()*mu.cos() * wave_heading_step_size;

                        // Create a wave
                        let amplitude = (2.0 * s * g_spectrum).sqrt(); 
                        unsafe{ srand(random_number_seed) };
                        let phase = unsafe{ rand() } as f64;
                        let mut wave_heading = mu + heading;
                        // wave directions should be in the range (0, 2PI)
                        while wave_heading < 0.0 {
                            wave_heading += 2.0 * PI; 
                        }
                        while wave_heading >= 2.0*PI {
                            wave_heading -= 2.0*PI;
                        }
                        spectrum[i][j] = RegularWave::new(amplitude, f, phase, wave_heading).unwrap();
                    }
                }

                Ok(
                    Wave{
                    significant_wave_height: h_s,
                    heading,
                    random_number_seed,
                    spectrum,
                    min_spectral_frequency,
                    max_spectral_frequency,
                    peak_spectral_frequency: f_p,
                    min_spectral_wave_heading,
                    max_spectral_wave_heading,
                    }
                )
            },
        }
    }

    /// Get wave elevation at a given location at a given time. 
    pub fn get_elevation(&self, location: &Dimension, time: f64) -> Result<f64,ValueError> {
        match time < 0.0 {
            true => Err(ValueError::new("Time should not be < 0.0.")),
            false => {
                let mut elevation = 0.0;
                let count_spectral_directions = self.spectrum.len();
                for i in 0 .. count_spectral_directions {
                    let count_spectral_frequencies = self.spectrum[i].len();
                    for j in 0 .. count_spectral_frequencies {
                        elevation += self.spectrum[i][j].get_elevation(location, time).unwrap();
                    }
                }
                Ok(elevation)
            },
        }
    }
}