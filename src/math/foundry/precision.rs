//! **Numerical Precision Utilities (The Soul)**
//!
//! This module provides canonical implementations for numerical precision and stability
//! operations. It defines the traits and utilities that govern how mathematical
//! operations handle floating-point precision throughout the codebase.
//!
//! ## **Design Philosophy**
//!
//! Numerical precision is the "soul" of mathematical computation - it defines the
//! fundamental contracts for how numbers behave and interact. This module ensures:
//! - **Consistent epsilon handling** across all comparisons
//! - **Robust numerical operations** that handle edge cases gracefully
//! - **Stable algorithms** that minimize accumulation of floating-point errors

use crate::core::float_types::{Real, EPSILON};
use nalgebra::{Vector3, Point3};

/// **Check if two floating-point values are approximately equal**
///
/// This is the canonical implementation for floating-point equality testing
/// throughout the codebase. It handles the inherent imprecision of floating-point
/// arithmetic by using a relative tolerance approach.
///
/// ## **Mathematical Foundation**
/// Two values a and b are considered approximately equal if:
/// ```text
/// |a - b| ≤ max(ε, ε × max(|a|, |b|))
/// ```
/// where ε is the epsilon tolerance.
///
/// # Arguments
/// * `a` - First value
/// * `b` - Second value
/// * `epsilon` - Tolerance for comparison (optional, defaults to EPSILON)
///
/// # Returns
/// * `bool` - True if values are approximately equal
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::approximately_equal;
///
/// assert!(approximately_equal(1.0, 1.0, None));
/// assert!(!approximately_equal(1.0, 1.1, None));
/// assert!(approximately_equal(1.0, 1.01, Some(0.02)));
/// ```
#[inline]
pub fn approximately_equal(a: Real, b: Real, epsilon: Option<Real>) -> bool {
    let eps = epsilon.unwrap_or(EPSILON);
    let diff = (a - b).abs();
    
    // Handle exact equality (including both zero)
    if diff == 0.0 {
        return true;
    }
    
    // Use relative tolerance for non-zero values
    let max_val = a.abs().max(b.abs());
    diff <= eps.max(eps * max_val)
}

/// **Check if a floating-point value is approximately zero**
///
/// This function tests whether a value is close enough to zero to be considered
/// zero for practical purposes, handling floating-point precision issues.
///
/// # Arguments
/// * `value` - The value to test
/// * `epsilon` - Tolerance for comparison (optional, defaults to EPSILON)
///
/// # Returns
/// * `bool` - True if value is approximately zero
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::approximately_zero;
///
/// assert!(approximately_zero(0.0, None));
/// assert!(approximately_zero(1e-15, None));
/// assert!(!approximately_zero(0.1, None));
/// ```
#[inline]
pub fn approximately_zero(value: Real, epsilon: Option<Real>) -> bool {
    let eps = epsilon.unwrap_or(EPSILON);
    value.abs() <= eps
}

/// **Safely normalize a vector, handling zero-length cases**
///
/// This function normalizes a vector while gracefully handling the case where
/// the vector has zero or near-zero length, which would cause division by zero
/// or numerical instability in naive implementations.
///
/// # Arguments
/// * `vector` - The vector to normalize
/// * `epsilon` - Tolerance for zero-length detection (optional, defaults to EPSILON)
///
/// # Returns
/// * `Option<Vector3<Real>>` - The normalized vector, or None if length is too small
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::safe_normalize;
/// use nalgebra::Vector3;
///
/// let v1 = Vector3::new(3.0, 4.0, 0.0);
/// let normalized = safe_normalize(&v1, None).unwrap();
/// assert!((normalized.norm() - 1.0).abs() < 1e-10);
///
/// let v2 = Vector3::new(1e-20, 1e-20, 1e-20);
/// assert!(safe_normalize(&v2, None).is_none());
/// ```
pub fn safe_normalize(vector: &Vector3<Real>, epsilon: Option<Real>) -> Option<Vector3<Real>> {
    let eps = epsilon.unwrap_or(EPSILON);
    let length_squared = vector.norm_squared();
    
    if length_squared <= eps * eps {
        None
    } else {
        let length = length_squared.sqrt();
        Some(vector / length)
    }
}

/// **Compute cross product with numerical stability**
///
/// This function computes the cross product of two vectors while handling
/// numerical precision issues that can arise with nearly parallel vectors.
///
/// # Arguments
/// * `a` - First vector
/// * `b` - Second vector
/// * `epsilon` - Tolerance for degenerate case detection (optional, defaults to EPSILON)
///
/// # Returns
/// * `Option<Vector3<Real>>` - The cross product, or None if vectors are nearly parallel
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::robust_cross_product;
/// use nalgebra::Vector3;
///
/// let v1 = Vector3::new(1.0, 0.0, 0.0);
/// let v2 = Vector3::new(0.0, 1.0, 0.0);
/// let cross = robust_cross_product(&v1, &v2, None).unwrap();
/// assert!((cross - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-10);
///
/// let v3 = Vector3::new(1.0, 0.0, 0.0);
/// let v4 = Vector3::new(1.000000001, 0.0, 0.0); // Nearly parallel
/// assert!(robust_cross_product(&v3, &v4, None).is_none());
/// ```
pub fn robust_cross_product(
    a: &Vector3<Real>, 
    b: &Vector3<Real>, 
    epsilon: Option<Real>
) -> Option<Vector3<Real>> {
    let eps = epsilon.unwrap_or(EPSILON);
    let cross = a.cross(b);
    
    // Check if the cross product is too small (vectors nearly parallel)
    if cross.norm_squared() <= eps * eps {
        None
    } else {
        Some(cross)
    }
}

/// **Clamp a value to a specified range**
///
/// This function ensures a value stays within specified bounds, with proper
/// handling of edge cases and NaN values.
///
/// # Arguments
/// * `value` - The value to clamp
/// * `min` - Minimum allowed value
/// * `max` - Maximum allowed value
///
/// # Returns
/// * `Real` - The clamped value
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::clamp;
///
/// assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
/// assert_eq!(clamp(-5.0, 0.0, 10.0), 0.0);
/// assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
/// ```
#[inline]
pub fn clamp(value: Real, min: Real, max: Real) -> Real {
    debug_assert!(min <= max, "clamp: min must be <= max");
    
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// **Linear interpolation between two values**
///
/// This function performs linear interpolation with proper handling of
/// edge cases and numerical precision.
///
/// # Arguments
/// * `a` - Start value (when t = 0)
/// * `b` - End value (when t = 1)
/// * `t` - Interpolation parameter
///
/// # Returns
/// * `Real` - The interpolated value
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::lerp;
///
/// assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
/// assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
/// assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
/// ```
#[inline]
pub fn lerp(a: Real, b: Real, t: Real) -> Real {
    a + t * (b - a)
}

/// **Safe division with zero-denominator handling**
///
/// This function performs division while gracefully handling the case where
/// the denominator is zero or very close to zero.
///
/// # Arguments
/// * `numerator` - The numerator
/// * `denominator` - The denominator
/// * `epsilon` - Tolerance for zero detection (optional, defaults to EPSILON)
///
/// # Returns
/// * `Option<Real>` - The result of division, or None if denominator is too small
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::safe_divide;
///
/// assert_eq!(safe_divide(10.0, 2.0, None), Some(5.0));
/// assert_eq!(safe_divide(10.0, 0.0, None), None);
/// assert_eq!(safe_divide(10.0, 1e-20, None), None);
/// ```
#[inline]
pub fn safe_divide(numerator: Real, denominator: Real, epsilon: Option<Real>) -> Option<Real> {
    let eps = epsilon.unwrap_or(EPSILON);
    
    if denominator.abs() <= eps {
        None
    } else {
        Some(numerator / denominator)
    }
}

/// **Round a value to a specified number of decimal places**
///
/// This function rounds a floating-point value to a specified precision,
/// useful for display purposes or reducing numerical noise.
///
/// # Arguments
/// * `value` - The value to round
/// * `decimal_places` - Number of decimal places to round to
///
/// # Returns
/// * `Real` - The rounded value
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::precision::round_to_places;
///
/// assert_eq!(round_to_places(3.14159, 2), 3.14);
/// assert_eq!(round_to_places(3.14159, 4), 3.1416);
/// ```
#[inline]
pub fn round_to_places(value: Real, decimal_places: u32) -> Real {
    let multiplier = 10.0_f64.powi(decimal_places as i32) as Real;
    (value * multiplier).round() / multiplier
}

/// **Check if a point is approximately on a plane**
///
/// This function tests whether a point lies on a plane within numerical tolerance.
///
/// # Arguments
/// * `point` - The point to test
/// * `plane_point` - A point on the plane
/// * `plane_normal` - The plane's normal vector (should be normalized)
/// * `epsilon` - Tolerance for the test (optional, defaults to EPSILON)
///
/// # Returns
/// * `bool` - True if the point is approximately on the plane
pub fn point_on_plane(
    point: &Point3<Real>,
    plane_point: &Point3<Real>,
    plane_normal: &Vector3<Real>,
    epsilon: Option<Real>
) -> bool {
    let eps = epsilon.unwrap_or(EPSILON);
    let to_point = point - plane_point;
    let distance = to_point.dot(plane_normal).abs();
    distance <= eps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_approximately_equal() {
        assert!(approximately_equal(1.0, 1.0, None));
        assert!(approximately_equal(1.0, 1.0 + EPSILON * 0.5, None));
        assert!(!approximately_equal(1.0, 1.1, None));
        assert!(approximately_equal(0.0, 0.0, None));
    }

    #[test]
    fn test_approximately_zero() {
        assert!(approximately_zero(0.0, None));
        assert!(approximately_zero(EPSILON * 0.5, None));
        assert!(!approximately_zero(0.1, None));
    }

    #[test]
    fn test_safe_normalize() {
        let v1 = Vector3::new(3.0, 4.0, 0.0);
        let normalized = safe_normalize(&v1, None).unwrap();
        assert!((normalized.norm() - 1.0).abs() < EPSILON);

        let v2 = Vector3::new(0.0, 0.0, 0.0);
        assert!(safe_normalize(&v2, None).is_none());
    }

    #[test]
    fn test_robust_cross_product() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let cross = robust_cross_product(&v1, &v2, None).unwrap();
        assert!((cross - Vector3::new(0.0, 0.0, 1.0)).norm() < EPSILON);

        let v3 = Vector3::new(1.0, 0.0, 0.0);
        let v4 = Vector3::new(1.0, 0.0, 0.0);
        assert!(robust_cross_product(&v3, &v4, None).is_none());
    }

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn test_lerp() {
        assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
        assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
        assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
    }

    #[test]
    fn test_safe_divide() {
        assert_eq!(safe_divide(10.0, 2.0, None), Some(5.0));
        assert_eq!(safe_divide(10.0, 0.0, None), None);
        assert_eq!(safe_divide(10.0, EPSILON * 0.1, None), None);
    }
}
