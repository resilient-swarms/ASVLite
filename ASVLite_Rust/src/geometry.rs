//! Definition of geometric components used in ASVLite-Rust.

/// A structure to represent a 3D quantity.
#[derive(Debug, Clone, PartialEq)]
pub struct Dimension {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A structure to represent the 6 degrees of freedom.
#[derive(Debug, Clone, PartialEq)]
pub struct Dof {
    pub linear: Dimension,
    pub angular: Dimension,
}