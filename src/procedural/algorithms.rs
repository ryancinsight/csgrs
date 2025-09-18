//! Core algorithms for procedural generation
//!
//! This module contains the fundamental algorithms used by procedural
//! generation, including noise functions, interpolation, and geometric operations.

/// Linear interpolation
pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Smooth step interpolation (3t² - 2t³)
pub fn smooth_step(t: f64) -> f64 {
    t * t * (3.0 - 2.0 * t)
}

/// Smoother step interpolation (6t⁵ - 15t⁴ + 10t³)
pub fn smoother_step(t: f64) -> f64 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// Clamp value to range
pub fn clamp(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Map value from one range to another
pub fn map_range(value: f64, from_min: f64, from_max: f64, to_min: f64, to_max: f64) -> f64 {
    let normalized = (value - from_min) / (from_max - from_min);
    to_min + normalized * (to_max - to_min)
}

/// Generate random number using simple hash
pub fn hash_noise(x: i64, y: i64, seed: u64) -> f64 {
    let mut hash = seed as u64;
    hash = hash.wrapping_mul(6364136223846793005).wrapping_add(1);
    hash ^= x as u64;
    hash = hash.wrapping_mul(6364136223846793005).wrapping_add(1);
    hash ^= y as u64;
    (hash % 1000) as f64 / 1000.0
}

/// Bilinear interpolation of 4 values
pub fn bilinear_interpolate(v00: f64, v10: f64, v01: f64, v11: f64, tx: f64, ty: f64) -> f64 {
    lerp(lerp(v00, v10, tx), lerp(v01, v11, tx), ty)
}

/// Trilinear interpolation of 8 values
pub fn trilinear_interpolate(
    v000: f64, v100: f64, v010: f64, v110: f64,
    v001: f64, v101: f64, v011: f64, v111: f64,
    tx: f64, ty: f64, tz: f64,
) -> f64 {
    lerp(
        bilinear_interpolate(v000, v100, v010, v110, tx, ty),
        bilinear_interpolate(v001, v101, v011, v111, tx, ty),
        tz,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lerp() {
        assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
        assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
        assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
    }

    #[test]
    fn test_smooth_step() {
        assert_eq!(smooth_step(0.0), 0.0);
        assert_eq!(smooth_step(1.0), 1.0);
        assert_eq!(smooth_step(0.5), 0.5); // 3*(0.5)^2 - 2*(0.5)^3 = 0.5
    }

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn test_map_range() {
        assert_eq!(map_range(5.0, 0.0, 10.0, 0.0, 100.0), 50.0);
        assert_eq!(map_range(0.0, 0.0, 10.0, 0.0, 100.0), 0.0);
        assert_eq!(map_range(10.0, 0.0, 10.0, 0.0, 100.0), 100.0);
    }
}
