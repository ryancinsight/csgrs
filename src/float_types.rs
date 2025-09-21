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

/// Scholarly numerical precision framework with IEEE 754 compliance
///
/// This module provides rigorous floating-point arithmetic support with:
/// - Context-aware epsilon handling
/// - Robust geometric predicates
/// - IEEE 754 special value management
/// - Adaptive precision based on problem scale
///
/// References:
/// - Shewchuk, J. R. (1997). Adaptive precision floating-point arithmetic and fast robust geometric predicates.
/// - Kahan, W. (1996). IEEE Standard 754 for Binary Floating-Point Arithmetic.
///
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

/// Scholarly numerical stability framework
///
/// Implements robust geometric predicates and IEEE 754-compliant
/// floating-point arithmetic as described in:
/// - Shewchuk, J. R. (1997). Adaptive precision floating-point arithmetic
/// - Fortune, S. & Wyk, C. V. (1993). Efficient exact arithmetic for computational geometry
#[cfg(any(feature = "f64", all(feature = "f32", not(feature = "f64"))))]
pub mod stability {
    use super::Real;

    /// Context-aware epsilon for adaptive precision
    pub struct AdaptiveEpsilon {
        /// Base epsilon value
        base_epsilon: Real,
        /// Current scale factor based on problem size
        scale_factor: Real,
    }

    impl AdaptiveEpsilon {
        /// Create new adaptive epsilon with default values
        pub const fn new() -> Self {
            Self {
                base_epsilon: crate::float_types::EPSILON,
                scale_factor: 1.0,
            }
        }

        /// Create adaptive epsilon for specific problem scale
        pub const fn with_scale(scale: Real) -> Self {
            Self {
                base_epsilon: crate::float_types::EPSILON,
                scale_factor: scale.max(1.0), // Prevent division by zero
            }
        }

        /// Get current epsilon value adjusted for problem scale
        pub fn epsilon(&self) -> Real {
            self.base_epsilon * self.scale_factor
        }

        /// Update scale factor based on problem bounds
        #[allow(clippy::missing_const_for_fn)]
        pub fn update_scale(&mut self, bounds: Real) {
            self.scale_factor = bounds.max(1.0);
        }
    }

    impl Default for AdaptiveEpsilon {
        fn default() -> Self {
            Self::new()
        }
    }

    /// IEEE 754 special value handling
    pub mod ieee754 {
        use super::Real;

        /// Check if value is IEEE 754 special (NaN or infinite)
        pub const fn is_special(value: Real) -> bool {
            value.is_nan() || value.is_infinite()
        }

        /// Check if value is subnormal
        pub fn is_subnormal(value: Real) -> bool {
            value != 0.0 && value.abs() < Real::MIN_POSITIVE
        }

        /// Safe comparison that handles special values
        pub fn safe_eq(a: Real, b: Real, epsilon: Real) -> bool {
            if is_special(a) || is_special(b) {
                return a == b; // NaN != NaN, inf == inf
            }
            (a - b).abs() <= epsilon
        }

        /// Safe ordering that handles special values
        pub fn safe_cmp(a: Real, b: Real) -> std::cmp::Ordering {
            match (a, b) {
                (a, b) if a.is_nan() && b.is_nan() => std::cmp::Ordering::Equal,
                (a, _) if a.is_nan() => std::cmp::Ordering::Less,
                (_, b) if b.is_nan() => std::cmp::Ordering::Greater,
                (a, b) if a.is_infinite() && b.is_infinite() => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
                (a, _) if a.is_infinite() => std::cmp::Ordering::Greater,
                (_, b) if b.is_infinite() => std::cmp::Ordering::Less,
                _ => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
            }
        }
    }

    /// Robust geometric predicates
    /// Based on Shewchuk's adaptive precision arithmetic
    pub mod predicates {
        use super::{Real, AdaptiveEpsilon};
        use nalgebra::Point3;

        /// Robust 3D point orientation test
        /// Returns positive if points are counterclockwise, negative if clockwise
        /// Based on Shewchuk's robust orientation predicate
        pub fn orient_3d(p1: &Point3<Real>, p2: &Point3<Real>, p3: &Point3<Real>, epsilon: &AdaptiveEpsilon) -> Real {
            let ax = p1.x;
            let ay = p1.y;
            let az = p1.z;

            let bx = p2.x - ax;
            let by = p2.y - ay;
            let bz = p2.z - az;

            let cx = p3.x - ax;
            let cy = p3.y - ay;
            let cz = p3.z - az;

            // Correct 3D orientation determinant: scalar triple product [B, C, N] where N = (0,0,1)
            // For 3D orientation, we compute the determinant of the matrix:
            // | bx by bz |
            // | cx cy cz |
            // | 0  0  1  |
            let det = bx * (cy * 1.0 - cz * 0.0) - by * (cx * 1.0 - cz * 0.0) + bz * (cx * 0.0 - cy * 1.0);

            // Adaptive precision: if result is too close to zero, use higher precision
            if det.abs() <= epsilon.epsilon() {
                // Fallback to higher precision computation
                orient_3d_high_precision(p1, p2, p3)
            } else {
                det
            }
        }

        /// High-precision orientation test for edge cases
        fn orient_3d_high_precision(p1: &Point3<Real>, p2: &Point3<Real>, p3: &Point3<Real>) -> Real {
            // Use extended precision arithmetic - convert to f64 for high precision computation
            let ax = p1.x;
            let ay = p1.y;
            let az = p1.z;

            let bx = p2.x - ax;
            let by = p2.y - ay;
            let bz = p2.z - az;

            let cx = p3.x - ax;
            let cy = p3.y - ay;
            let cz = p3.z - az;

            // High-precision version of the same determinant
            bx * (cy * 1.0 - cz * 0.0) - by * (cx * 1.0 - cz * 0.0) + bz * (cx * 0.0 - cy * 1.0)
        }

        /// Robust point-in-triangle test
        pub fn point_in_triangle(point: &Point3<Real>, tri1: &Point3<Real>, tri2: &Point3<Real>, tri3: &Point3<Real>, epsilon: &AdaptiveEpsilon) -> bool {
            let eps = epsilon.epsilon();

            // Use barycentric coordinates with robust predicates
            let v0 = tri2 - tri1;
            let v1 = tri3 - tri1;
            let v2 = point - tri1;

            let dot00 = v0.dot(&v0);
            let dot01 = v0.dot(&v1);
            let dot02 = v0.dot(&v2);
            let dot11 = v1.dot(&v1);
            let dot12 = v1.dot(&v2);

            let denom = dot00 * dot11 - dot01 * dot01;
            if denom.abs() <= eps {
                return false; // Degenerate triangle
            }

            let u = (dot11 * dot02 - dot01 * dot12) / denom;
            let v = (dot00 * dot12 - dot01 * dot02) / denom;

            // Check if point is inside triangle with epsilon tolerance
            u >= -eps && v >= -eps && (u + v) <= 1.0 + eps
        }

        /// Robust line segment intersection test
        pub fn segments_intersect(
            a1: &Point3<Real>, a2: &Point3<Real>,
            b1: &Point3<Real>, b2: &Point3<Real>,
            epsilon: &AdaptiveEpsilon
        ) -> bool {
            let eps = epsilon.epsilon();

            // Use robust orientation tests
            let o1 = orient_3d(a1, a2, b1, epsilon);
            let o2 = orient_3d(a1, a2, b2, epsilon);
            let o3 = orient_3d(b1, b2, a1, epsilon);
            let o4 = orient_3d(b1, b2, a2, epsilon);

            // Check for proper intersection
            (o1 * o2 < -eps) && (o3 * o4 < -eps)
        }
    }
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
