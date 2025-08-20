// Re-export parry and rapier for the appropriate float size
#[cfg(feature = "f64")]
pub use parry3d_f64 as parry3d;
#[cfg(feature = "f64")]
pub use rapier3d_f64 as rapier3d;

#[cfg(feature = "f32")]
pub use parry3d;
#[cfg(feature = "f32")]
pub use rapier3d;

// Our Real scalar type:
#[cfg(feature = "f32")]
pub type Real = f32;
#[cfg(feature = "f64")]
pub type Real = f64;

/// A small epsilon for geometric comparisons, adjusted per precision.
/// This is the base epsilon - use adaptive_epsilon() for scale-aware tolerance.
#[cfg(feature = "f32")]
pub const EPSILON: Real = 1e-4;
/// A small epsilon for geometric comparisons, adjusted per precision.
/// This is the base epsilon - use adaptive_epsilon() for scale-aware tolerance.
#[cfg(feature = "f64")]
pub const EPSILON: Real = 1e-8;

/// Minimum epsilon to prevent underflow in adaptive calculations
#[cfg(feature = "f32")]
pub const MIN_EPSILON: Real = 1e-6;
#[cfg(feature = "f64")]
pub const MIN_EPSILON: Real = 1e-12;

/// Maximum epsilon to prevent excessive tolerance
#[cfg(feature = "f32")]
pub const MAX_EPSILON: Real = 1e-2;
#[cfg(feature = "f64")]
pub const MAX_EPSILON: Real = 1e-4;

/// Calculate geometry-aware epsilon based on characteristic length scale.
///
/// This implements adaptive tolerance as recommended in computational geometry literature
/// (Hoffmann 1989, Shewchuk 1997) to handle scale-dependent numerical precision issues.
///
/// # Arguments
/// * `characteristic_length` - Typical dimension of the geometry (e.g., bounding box diagonal)
///
/// # Returns
/// * Adaptive epsilon scaled appropriately for the geometry
///
/// # Mathematical Foundation
/// For robust geometric computations, tolerance should scale with geometry:
/// - Too small: floating-point precision errors dominate
/// - Too large: legitimate geometric features are lost
/// - Optimal: proportional to geometry scale with machine precision limits
pub fn adaptive_epsilon(characteristic_length: Real) -> Real {
    if characteristic_length <= 0.0 {
        return EPSILON;
    }

    // Scale epsilon proportionally to geometry size
    // Factor of 1e-10 for f64, 1e-6 for f32 provides good balance
    #[cfg(feature = "f64")]
    let scale_factor = 1e-10;
    #[cfg(feature = "f32")]
    let scale_factor = 1e-6;

    let adaptive_eps = characteristic_length * scale_factor;

    // Clamp to reasonable bounds to prevent extreme values
    adaptive_eps.clamp(MIN_EPSILON, MAX_EPSILON)
}

/// Calculate adaptive epsilon for a bounding box.
/// Uses the diagonal length as the characteristic scale.
pub fn adaptive_epsilon_for_bbox(min_point: &nalgebra::Point3<Real>, max_point: &nalgebra::Point3<Real>) -> Real {
    let diagonal = (max_point - min_point).norm();
    adaptive_epsilon(diagonal)
}

// Pi
/// Archimedes' constant (π)
#[cfg(feature = "f32")]
pub const PI: Real = core::f32::consts::PI;
/// Archimedes' constant (π)
#[cfg(feature = "f64")]
pub const PI: Real = core::f64::consts::PI;

// Frac Pi 2
/// π/2
#[cfg(feature = "f32")]
pub const FRAC_PI_2: Real = core::f32::consts::FRAC_PI_2;
/// π/2
#[cfg(feature = "f64")]
pub const FRAC_PI_2: Real = core::f64::consts::FRAC_PI_2;

// Tau
/// The full circle constant (τ)
#[cfg(feature = "f32")]
pub const TAU: Real = core::f32::consts::TAU;
/// The full circle constant (τ)
#[cfg(feature = "f64")]
pub const TAU: Real = core::f64::consts::TAU;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Unit conversion
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
pub const INCH: Real = 25.4;
pub const FOOT: Real = 25.4 * 12.0;
pub const YARD: Real = 25.4 * 36.0;
pub const MM: Real = 1.0;
pub const CM: Real = 10.0;
pub const METER: Real = 1000.0;
