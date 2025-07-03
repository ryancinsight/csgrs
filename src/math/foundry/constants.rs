//! **Mathematical Constants (The Skeleton)**
//!
//! This module provides canonical mathematical constants used throughout the codebase.
//! It serves as the single source of truth for mathematical values, ensuring consistency
//! and precision across all calculations.
//!
//! ## **Design Philosophy**
//!
//! Mathematical constants should be:
//! - **Precisely defined** with maximum available precision
//! - **Consistently named** following mathematical conventions
//! - **Well-documented** with their mathematical significance
//! - **Centrally located** to avoid duplication and inconsistency

use crate::core::float_types::Real;

/// **The Golden Ratio (φ)**
///
/// The golden ratio φ = (1 + √5) / 2 ≈ 1.618033988749...
/// 
/// ## **Mathematical Properties**
/// - φ² = φ + 1
/// - 1/φ = φ - 1
/// - φ = lim(n→∞) F(n+1)/F(n) where F(n) is the nth Fibonacci number
///
/// ## **Applications**
/// - Geometric constructions and proportions
/// - Fibonacci spirals and phyllotaxis
/// - Optimization algorithms (golden section search)
/// - Aesthetic proportions in design
pub const GOLDEN_RATIO: Real = 1.618033988749894848204586834365638117720309179805762862135;

/// **Square root of 2 (√2)**
///
/// √2 ≈ 1.414213562373...
///
/// ## **Mathematical Significance**
/// - Diagonal of a unit square
/// - First known irrational number (proved by ancient Greeks)
/// - Appears in many geometric constructions
pub const SQRT_2: Real = 1.4142135623730950488016887242096980785696718753769480731767;

/// **Square root of 3 (√3)**
///
/// √3 ≈ 1.732050807568...
///
/// ## **Mathematical Significance**
/// - Height of an equilateral triangle with unit side
/// - Appears in hexagonal and triangular geometry
/// - Used in crystallography and materials science
pub const SQRT_3: Real = 1.7320508075688772935274463415058723669428052538103806280558;

/// **Square root of 5 (√5)**
///
/// √5 ≈ 2.236067977499...
///
/// ## **Mathematical Significance**
/// - Appears in the golden ratio formula: φ = (1 + √5) / 2
/// - Diagonal of a 1×2 rectangle
/// - Related to pentagon geometry
pub const SQRT_5: Real = 2.2360679774997896964091736687312762354406183596115257242709;

/// **Half of π (π/2)**
///
/// π/2 ≈ 1.570796326794...
///
/// ## **Mathematical Significance**
/// - 90 degrees in radians
/// - Quarter circle arc length for unit radius
/// - Appears frequently in trigonometric calculations
pub const PI_2: Real = std::f64::consts::FRAC_PI_2 as Real;

/// **Quarter of π (π/4)**
///
/// π/4 ≈ 0.785398163397...
///
/// ## **Mathematical Significance**
/// - 45 degrees in radians
/// - Eighth circle arc length for unit radius
/// - Common angle in geometric constructions
pub const PI_4: Real = std::f64::consts::FRAC_PI_4 as Real;

/// **Tau (τ = 2π)**
///
/// τ = 2π ≈ 6.283185307179...
///
/// ## **Mathematical Significance**
/// - Full circle circumference for unit radius
/// - Some mathematicians argue this is more fundamental than π
/// - Simplifies many circular and periodic formulas
pub const TAU: Real = std::f64::consts::TAU as Real;

/// **Euler's number (e)**
///
/// e ≈ 2.718281828459...
///
/// ## **Mathematical Significance**
/// - Base of natural logarithm
/// - Limit of (1 + 1/n)^n as n approaches infinity
/// - Fundamental to exponential growth and decay
pub const E: Real = std::f64::consts::E as Real;

/// **Natural logarithm of 2 (ln(2))**
///
/// ln(2) ≈ 0.693147180559...
///
/// ## **Mathematical Significance**
/// - Appears in binary logarithms: log₂(x) = ln(x) / ln(2)
/// - Half-life calculations
/// - Information theory and entropy
pub const LN_2: Real = std::f64::consts::LN_2 as Real;

/// **Natural logarithm of 10 (ln(10))**
///
/// ln(10) ≈ 2.302585092994...
///
/// ## **Mathematical Significance**
/// - Conversion between natural and common logarithms
/// - Appears in pH calculations and decibel scales
pub const LN_10: Real = std::f64::consts::LN_10 as Real;

/// **Reciprocal of the golden ratio (1/φ)**
///
/// 1/φ = φ - 1 ≈ 0.618033988749...
///
/// ## **Mathematical Properties**
/// - φ × (1/φ) = 1
/// - 1/φ = φ - 1 (unique property of the golden ratio)
/// - Appears in golden rectangle constructions
pub const GOLDEN_RATIO_RECIPROCAL: Real = GOLDEN_RATIO - 1.0;

/// **Degrees to radians conversion factor**
///
/// π/180 ≈ 0.017453292519...
///
/// ## **Usage**
/// To convert degrees to radians: radians = degrees × DEG_TO_RAD
pub const DEG_TO_RAD: Real = std::f64::consts::PI as Real / 180.0;

/// **Radians to degrees conversion factor**
///
/// 180/π ≈ 57.295779513082...
///
/// ## **Usage**
/// To convert radians to degrees: degrees = radians × RAD_TO_DEG
pub const RAD_TO_DEG: Real = 180.0 / std::f64::consts::PI as Real;

// Physics constants are commented out for now to avoid feature gate warnings
// Uncomment and add physics-constants feature to Cargo.toml if needed

// /// **Planck's constant (h)**
// ///
// /// h ≈ 6.62607015 × 10⁻³⁴ J⋅s
// ///
// /// ## **Physical Significance**
// /// - Fundamental constant of quantum mechanics
// /// - Relates energy and frequency: E = hf
// /// - Defines the scale of quantum effects
// pub const PLANCK_CONSTANT: Real = 6.62607015e-34;

// /// **Speed of light in vacuum (c)**
// ///
// /// c = 299,792,458 m/s (exact by definition)
// ///
// /// ## **Physical Significance**
// /// - Fundamental constant of relativity
// /// - Maximum speed of information transfer
// /// - Relates energy and mass: E = mc²
// pub const SPEED_OF_LIGHT: Real = 299792458.0;

// /// **Gravitational constant (G)**
// ///
// /// G ≈ 6.67430 × 10⁻¹¹ m³⋅kg⁻¹⋅s⁻²
// ///
// /// ## **Physical Significance**
// /// - Fundamental constant of gravity
// /// - Appears in Newton's law of universal gravitation
// /// - Determines the strength of gravitational interactions
// pub const GRAVITATIONAL_CONSTANT: Real = 6.67430e-11;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::float_types::EPSILON;

    #[test]
    fn test_golden_ratio_properties() {
        // Test φ² = φ + 1
        let phi_squared = GOLDEN_RATIO * GOLDEN_RATIO;
        let phi_plus_one = GOLDEN_RATIO + 1.0;
        assert!((phi_squared - phi_plus_one).abs() < EPSILON);

        // Test 1/φ = φ - 1
        let reciprocal = 1.0 / GOLDEN_RATIO;
        assert!((reciprocal - GOLDEN_RATIO_RECIPROCAL).abs() < EPSILON);
    }

    #[test]
    fn test_sqrt_constants() {
        // Test that our constants match computed values
        assert!((SQRT_2 - 2.0_f64.sqrt() as Real).abs() < EPSILON);
        assert!((SQRT_3 - 3.0_f64.sqrt() as Real).abs() < EPSILON);
        assert!((SQRT_5 - 5.0_f64.sqrt() as Real).abs() < EPSILON);
    }

    #[test]
    fn test_angle_conversions() {
        // Test 90 degrees = π/2 radians
        let ninety_degrees_in_radians = 90.0 * DEG_TO_RAD;
        assert!((ninety_degrees_in_radians - PI_2).abs() < EPSILON);

        // Test π radians = 180 degrees
        let pi_in_degrees = std::f64::consts::PI as Real * RAD_TO_DEG;
        assert!((pi_in_degrees - 180.0).abs() < EPSILON);
    }

    #[test]
    fn test_tau_relationship() {
        // Test τ = 2π
        let two_pi = 2.0 * std::f64::consts::PI as Real;
        assert!((TAU - two_pi).abs() < EPSILON);
    }
}
