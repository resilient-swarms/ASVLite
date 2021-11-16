use super::error::ValueError;
use super::geometry::Dimension;
use super::constants::{PI, G, SEA_WATER_DENSITY};

/// Structure to define a regular wave. The members of the struct is initialised by
/// the associated function new(), and it is the only function that should write value 
/// into the variables. To edit a member variable value, drop the current instance of 
/// the struct and create a new instance with the desired values. Recommend assigning 
/// the return of new() to an immutable variable. The members are public and hence can 
/// be directly accessed for reading. 
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RegularWave {
    /// Wave amplitude in meter.
    pub amplitude: f64,
    /// Wave frequency in Hz.
    pub frequency: f64,
    /// Phase lag in radian.
    pub phase_lag: f64,
    /// Direction of propagation of wave w.r.t geographic North. 
    /// Angle measure +ve in clockwise direction such that the 
    /// East is at PI/2 radian to North.
    pub direction: f64,
    /// Time period in seconds.
    pub time_period: f64,
    /// Wave length in meter.
    pub wave_length: f64,
    /// Wave number; dimensionless.
    pub wave_number: f64,
}

impl RegularWave {
    /// Initialise a regular wave.
    pub fn new(amplitude: f64, frequency: f64, phase_lag: f64, direction: f64) -> Result<RegularWave, ValueError> {
        match (amplitude<=0.0, frequency<=0.0) {
            (true, true) => Err(ValueError::new("Amplitude and frequency should not be <= 0.0.")),
            (true, false) => Err(ValueError::new("Amplitude should not be <= 0.0.")),
            (false, true) => Err(ValueError::new("Frequency should not be <= 0.0.")),
            (false, false) => {
                let time_period = 1.0/frequency;
                let wave_length = (G * time_period * time_period)/(2.0 * PI);
                let wave_number = (2.0 * PI)/wave_length;
                Ok(
                    RegularWave {
                    amplitude, 
                    frequency, 
                    phase_lag, 
                    direction,
                    time_period,
                    wave_length,
                    wave_number,
                    }
                )
            },
        }
    }

    /// Get the phase, in radian, of the wave for a given location at a given time.
    pub fn get_phase(&self, location: &Dimension, time: f64) -> Result<f64,ValueError> {
        match time < 0.0 {
            true => Err(ValueError::new("Time should not be < 0.0.")),
            false => {
                // elevation = amplitude * cos(A - B + phase)
                // where:
                // a = wave_number * (x * cos(direction) + y * sin(direction))
                // b = 2 * PI * frequency * time
                // NOTE:
                // In the coordinate system that we use here, angular measurements are made 
                // with respect to north which is represented by y-axis and not x-axis.
                // Therefore the above formula needs to be modified as:
                // a = wave_number * (x * sin(direction) + y * cos(direction))
                let a = self.wave_number * (location.x * self.direction.sin() + location.y * self.direction.cos());
                let b = 2.0 * PI * self.frequency * time;
                Ok(a - b + self.phase_lag)
            },
        }
    }

     /// Get the wave elevation, in meter, of the wave for a given location at a given time.
     pub fn get_elevation(&self, location: &Dimension, time: f64) -> Result<f64,ValueError> {
        match time < 0.0 {
            true => Err(ValueError::new("Time should not be < 0.0.")),
            false => {
                let phase = self.get_phase(location, time).unwrap();
                Ok(self.amplitude * phase.cos())
            },
        }
    }

    /// Get the wave pressure amplitude at a given depth.
    pub fn get_pressure_amp(&self, z: f64) -> f64 {
        SEA_WATER_DENSITY * G * self.amplitude * (self.wave_number * z).exp()
    }
}

#[test]
fn new() {
    let amp = 1.0;
    let f = 2.0;
    let phase = 3.0;
    let dir = 4.0;
    let t = 1.0/f;
    let l = (G * t*t )/(2.0*PI);
    let n = (2.0 * PI)/l;

    // A good wave
    let wave = RegularWave::new(amp, f, phase, dir).unwrap();
    assert_eq!(wave, RegularWave{
        amplitude: amp, 
        frequency: f, 
        phase_lag: phase, 
        direction: dir, 
        time_period: t,
        wave_length: l,
        wave_number: n,
    });

    // Bad wave with -ve amplitude and/or frequency
    assert_eq!(RegularWave::new(-1.0, -1.0, phase, dir).err().unwrap(), 
        ValueError::new("Amplitude and frequency should not be <= 0.0."));
    assert_eq!(RegularWave::new(-1.0, f, phase, dir).err().unwrap(), 
        ValueError::new("Amplitude should not be <= 0.0."));
    assert_eq!(RegularWave::new(amp, -1.0, phase, dir).err().unwrap(), 
        ValueError::new("Frequency should not be <= 0.0."));
}

#[test]
fn get_phase() {
    let amp = 1.0;
    let f = 2.0;
    let dir = 4.0;
    let location = Dimension{x: 0.0, y:0.0, z:0.0};

    let wave = RegularWave::new(amp, f, 0.0, dir).unwrap();

    // Good time.
    let phi = wave.get_phase(&location, 0.0).unwrap();
    assert_eq!(phi, 0.0);

    // Bad time
    assert_eq!(wave.get_phase(&location, -4.0).err().unwrap(), 
        ValueError::new("Time should not be < 0.0."));
}

#[test]
fn get_elevation() {
    let amp = 1.0;
    let f = 2.0;
    let dir = 4.0;
    let location = Dimension{x: 0.0, y:0.0, z:0.0};

    let wave = RegularWave::new(amp, f, 0.0, dir).unwrap();

    // Good time.
    let elevation = wave.get_elevation(&location, 0.0).unwrap();
    assert_eq!(elevation, 1.0);

    // Bad time
    assert_eq!(wave.get_elevation(&location, -4.0).err().unwrap(), 
    ValueError::new("Time should not be < 0.0."));
}