//! Mathematical utility functions for 2D primitive generation
//!
//! This module contains helper functions used in the generation of complex 2D shapes,
//! particularly for gear profiles and parametric curves.

use crate::core::float_types::Real;

/// Classic parametric involute of a circle calculation.
/// 
/// # Parameters
/// - `rb`: base-circle radius
/// - `phi`: involute parameter
/// 
/// # Returns
/// Cartesian coordinates (x, y) of the involute point
#[inline]
pub fn involute_xy(rb: Real, phi: Real) -> (Real, Real) {
    // Classic parametric involute of a circle (rb = base‑circle radius).
    // x = rb( cosφ + φ·sinφ )
    // y = rb( sinφ – φ·cosφ )
    (
        rb * (phi.cos() + phi * phi.sin()),
        rb * (phi.sin() - phi * phi.cos()),
    )
}

/// Calculate the involute angle at a given radius.
/// 
/// # Parameters
/// - `r`: radius at which to calculate the angle
/// - `rb`: base circle radius
/// 
/// # Returns
/// The involute angle φ = sqrt((r/rb)² - 1)
#[inline]
pub fn involute_angle_at_radius(r: Real, rb: Real) -> Real {
    // φ = sqrt( (r/rb)^2 – 1 )
    ((r / rb).powi(2) - 1.0).max(0.0).sqrt()
}

/// Generate epicycloid coordinates for gear tooth profiles.
/// 
/// # Parameters
/// - `r_g`: pitch-circle radius
/// - `r_p`: pin circle (generating circle) radius
/// - `theta`: parameter angle
/// 
/// # Returns
/// Cartesian coordinates (x, y) of the epicycloid point
#[inline]
pub fn epicycloid_xy(r_g: Real, r_p: Real, theta: Real) -> (Real, Real) {
    // r_g : pitch‑circle radius, r_p : pin circle (generating circle) radius
    // x = (r_g + r_p) (cos θ) – r_p cos((r_g + r_p)/r_p · θ)
    // y = (r_g + r_p) (sin θ) – r_p sin((r_g + r_p)/r_p · θ)
    let k = (r_g + r_p) / r_p;
    (
        (r_g + r_p) * theta.cos() - r_p * (k * theta).cos(),
        (r_g + r_p) * theta.sin() - r_p * (k * theta).sin(),
    )
}

/// Generate hypocycloid coordinates for gear root flanks.
/// 
/// # Parameters
/// - `r_g`: pitch-circle radius
/// - `r_p`: pin circle (generating circle) radius
/// - `theta`: parameter angle
/// 
/// # Returns
/// Cartesian coordinates (x, y) of the hypocycloid point
#[inline]
pub fn hypocycloid_xy(r_g: Real, r_p: Real, theta: Real) -> (Real, Real) {
    // For root flank of a cycloidal tooth
    let k = (r_g - r_p) / r_p;
    (
        (r_g - r_p) * theta.cos() + r_p * (k * theta).cos(),
        (r_g - r_p) * theta.sin() - r_p * (k * theta).sin(),
    )
} 
