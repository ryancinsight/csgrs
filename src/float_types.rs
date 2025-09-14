// Re-export parry and rapier for the appropriate float size
// Prioritize f64 if both features are enabled (maintains backward compatibility)
#[cfg(any(feature = "f64", all(feature = "f32", not(feature = "f64"))))]
pub use parry3d_f64 as parry3d;
#[cfg(any(feature = "f64", all(feature = "f32", not(feature = "f64"))))]
pub use rapier3d_f64 as rapier3d;

#[cfg(all(feature = "f32", not(feature = "f64")))]
pub use parry3d;
#[cfg(all(feature = "f32", not(feature = "f64")))]
pub use rapier3d;

/// Module for precision-aware constants - zero-cost abstraction without trait complexity
/// This provides compile-time precision selection while avoiding trait system overhead
/// that can cause stack overflow in recursive algorithms.
#[cfg(any(feature = "f64", all(feature = "f32", not(feature = "f64"))))]
pub mod constants {
    use super::Real;

    /// Epsilon value for floating-point comparisons (f64 precision)
    pub const EPSILON: Real = 1e-8;

    /// Archimedes' constant (π) for f64 precision
    pub const PI: Real = core::f64::consts::PI;

    /// π/2 for f64 precision
    pub const FRAC_PI_2: Real = core::f64::consts::FRAC_PI_2;

    /// The full circle constant (τ = 2π) for f64 precision
    pub const TAU: Real = core::f64::consts::TAU;

    /// Unit conversion constants
    pub const INCH: Real = 25.4;
    pub const FOOT: Real = 25.4 * 12.0;
    pub const YARD: Real = 25.4 * 36.0;
    pub const MM: Real = 1.0;
    pub const CM: Real = 10.0;
    pub const METER: Real = 1000.0;
}

#[cfg(all(feature = "f32", not(feature = "f64")))]
pub mod constants {
    use super::Real;

    /// Epsilon value for floating-point comparisons (f32 precision)
    pub const EPSILON: Real = 1e-4;

    /// Archimedes' constant (π) for f32 precision
    pub const PI: Real = core::f32::consts::PI;

    /// π/2 for f32 precision
    pub const FRAC_PI_2: Real = core::f32::consts::FRAC_PI_2;

    /// The full circle constant (τ = 2π) for f32 precision
    pub const TAU: Real = core::f32::consts::TAU;

    /// Unit conversion constants
    pub const INCH: Real = 25.4;
    pub const FOOT: Real = 25.4 * 12.0;
    pub const YARD: Real = 25.4 * 36.0;
    pub const MM: Real = 1.0;
    pub const CM: Real = 10.0;
    pub const METER: Real = 1000.0;
}

// Our Real scalar type - concrete type for current precision
// Prioritize f64 if both features are enabled
#[cfg(any(feature = "f64", all(feature = "f32", not(feature = "f64"))))]
pub type Real = f64;
#[cfg(all(feature = "f32", not(feature = "f64")))]
pub type Real = f32;

// Legacy constants for backward compatibility
// These now delegate to the precision-aware module - zero-cost abstraction
/// A small epsilon for geometric comparisons, adjusted per precision.
pub const EPSILON: Real = constants::EPSILON;

/// Archimedes' constant (π)
pub const PI: Real = constants::PI;

/// π/2
pub const FRAC_PI_2: Real = constants::FRAC_PI_2;

/// The full circle constant (τ)
pub const TAU: Real = constants::TAU;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Unit conversion
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
pub const INCH: Real = constants::INCH;
pub const FOOT: Real = constants::FOOT;
pub const YARD: Real = constants::YARD;
pub const MM: Real = constants::MM;
pub const CM: Real = constants::CM;
pub const METER: Real = constants::METER;
