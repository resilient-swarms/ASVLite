
/// A structure to represent a 3D quantity.
#[derive(Debug, Copy, Clone, Eq)]
pub struct Dimension {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A structure to represent the 6 degrees of freedom.
#[derive(Debug, Copy, Clone, Eq)]
pub struct Dof {
    pub linear: Dimension,
    pub angular: Dimension,
}