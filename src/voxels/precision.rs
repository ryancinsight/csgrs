//! Fixed-Precision Arithmetic Configuration
//!
//! This module provides configuration for fixed-precision arithmetic operations
//! to ensure robust geometric computations in the sparse voxel octree system.
//! Based on research-proven techniques for reliable CSG operations.

use crate::float_types::{Real, EPSILON};
use std::fmt::Debug;

/// Configuration for fixed-precision arithmetic operations
/// 
/// Provides settings for scaling floating-point operations to fixed-point
/// arithmetic to improve numerical robustness in geometric computations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrecisionConfig {
    /// Scale factor for converting floating-point to fixed-point
    /// Higher values provide more precision but may cause overflow
    pub scale_factor: i32,
    
    /// Scaled epsilon value for fixed-point comparisons
    /// Computed as (EPSILON * scale_factor) to maintain relative precision
    pub epsilon_scaled: i64,
}

impl Default for PrecisionConfig {
    fn default() -> Self {
        let scale_factor = 1_000_000; // 1e6 provides good balance of precision and range
        let epsilon_scaled = ((EPSILON * scale_factor as Real) as i64).max(1);
        Self {
            scale_factor,
            epsilon_scaled,
        }
    }
}

impl PrecisionConfig {
    /// Create a new precision configuration with specified scale factor
    pub fn new(scale_factor: i32) -> Self {
        let epsilon_scaled = ((EPSILON * scale_factor as Real) as i64).max(1);
        Self {
            scale_factor,
            epsilon_scaled,
        }
    }
    
    /// Convert floating-point value to scaled fixed-point
    #[inline]
    pub fn to_fixed(&self, value: Real) -> i64 {
        (value * self.scale_factor as Real) as i64
    }
    
    /// Convert scaled fixed-point value back to floating-point
    #[inline]
    pub fn from_fixed(&self, value: i64) -> Real {
        value as Real / self.scale_factor as Real
    }
    
    /// Check if two fixed-point values are approximately equal
    #[inline]
    pub fn fixed_eq(&self, a: i64, b: i64) -> bool {
        (a - b).abs() <= self.epsilon_scaled
    }
    
    /// Check if fixed-point value is approximately zero
    #[inline]
    pub fn fixed_is_zero(&self, value: i64) -> bool {
        value.abs() <= self.epsilon_scaled
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_precision_config_default() {
        let config = PrecisionConfig::default();
        assert_eq!(config.scale_factor, 1_000_000);
        assert!(config.epsilon_scaled > 0);
    }
    
    #[test]
    fn test_fixed_point_conversion() {
        let config = PrecisionConfig::new(1000);
        let value = 3.14159;
        let fixed = config.to_fixed(value);
        let recovered = config.from_fixed(fixed);
        
        assert!((value - recovered).abs() < 0.001);
    }
    
    #[test]
    fn test_fixed_equality() {
        let config = PrecisionConfig::new(1000);
        let a = config.to_fixed(1.0);
        let b = config.to_fixed(1.0 + EPSILON / 2.0);
        
        assert!(config.fixed_eq(a, b));
    }
}