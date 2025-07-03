//! **Minkowski Difference Operations (The Mind)**
//!
//! This module implements Minkowski difference operations following Cathedral Engineering
//! principles where the implementation represents the "mind" that performs the
//! computational work for difference operations.
//!
//! ## **Mathematical Foundation**
//!
//! ### **Minkowski Difference Definition**
//! For sets A and B in Euclidean space, the Minkowski difference A ⊖ B is defined as:
//! ```text
//! A ⊖ B = {a - b | a ∈ A, b ∈ B}
//! ```
//!
//! ### **Relationship to Minkowski Sum**
//! The Minkowski difference can be computed as:
//! ```text
//! A ⊖ B = A ⊕ (-B)
//! ```
//! where -B is the reflection of B through the origin.
//!
//! ### **Applications**
//! - **Erosion Operations**: Shrinking shapes by a structuring element
//! - **Collision Detection**: Computing forbidden regions for moving objects
//! - **Tolerance Analysis**: Determining clearance zones in mechanical design

use crate::csg::CSG;
use super::errors::{MinkowskiError, MinkowskiResult};
use super::models::MinkowskiConfig;
use std::fmt::Debug;

/// **Compute the Minkowski difference of two CSG objects**
///
/// This function computes A ⊖ B by first reflecting B through the origin
/// and then computing the Minkowski sum A ⊕ (-B).
///
/// ## **Mathematical Foundation**
/// The Minkowski difference A ⊖ B = {a - b | a ∈ A, b ∈ B} is equivalent
/// to A ⊕ (-B) where -B is the reflection of B through the origin.
///
/// # Arguments
/// * `csg_a` - First operand (minuend)
/// * `csg_b` - Second operand (subtrahend)
///
/// # Returns
/// * `CSG<S>` - The Minkowski difference result
///
/// # Examples
/// ```rust
/// use csgrs::CSG;
/// use csgrs::math::minkowski::difference;
///
/// let cube: CSG<()> = CSG::cube(2.0, None);
/// let sphere: CSG<()> = CSG::sphere(0.5, 16, 8, None);
/// let result = difference::compute(&cube, &sphere);
/// ```
pub fn compute<S>(csg_a: &CSG<S>, csg_b: &CSG<S>) -> CSG<S>
where
    S: Clone + Debug + Send + Sync,
{
    compute_with_config(csg_a, csg_b, &MinkowskiConfig::default())
}

/// **Compute Minkowski difference with custom configuration**
///
/// # Arguments
/// * `csg_a` - First operand (minuend)
/// * `csg_b` - Second operand (subtrahend)
/// * `config` - Configuration parameters for the operation
///
/// # Returns
/// * `CSG<S>` - The Minkowski difference result
pub fn compute_with_config<S>(csg_a: &CSG<S>, csg_b: &CSG<S>, config: &MinkowskiConfig) -> CSG<S>
where
    S: Clone + Debug + Send + Sync,
{
    match compute_with_result(csg_a, csg_b, config) {
        Ok(result) => result,
        Err(_) => CSG::new(), // Fallback for compatibility
    }
}

/// **Compute Minkowski difference with detailed error reporting**
///
/// # Arguments
/// * `csg_a` - First operand (minuend)
/// * `csg_b` - Second operand (subtrahend)
/// * `config` - Configuration parameters for the operation
///
/// # Returns
/// * `MinkowskiResult<CSG<S>>` - The result or detailed error information
pub fn compute_with_result<S>(
    csg_a: &CSG<S>, 
    csg_b: &CSG<S>, 
    config: &MinkowskiConfig
) -> MinkowskiResult<CSG<S>>
where
    S: Clone + Debug + Send + Sync,
{
    // Step 1: Reflect csg_b through the origin to get -B
    let reflected_b = reflect_through_origin(csg_b)?;

    // Step 2: Compute Minkowski sum A ⊕ (-B)
    super::sum::compute_with_result(csg_a, &reflected_b, config)
}

/// **Reflect a CSG object through the origin**
///
/// This function creates a new CSG where every vertex position (x, y, z)
/// is transformed to (-x, -y, -z), effectively reflecting the object
/// through the origin.
///
/// # Arguments
/// * `csg` - The CSG object to reflect
///
/// # Returns
/// * `MinkowskiResult<CSG<S>>` - The reflected CSG object
fn reflect_through_origin<S>(csg: &CSG<S>) -> MinkowskiResult<CSG<S>>
where
    S: Clone + Debug + Send + Sync,
{
    if csg.polygons.is_empty() {
        return Err(MinkowskiError::empty_geometry("input"));
    }

    // Create a reflection transformation matrix
    let reflection_matrix = nalgebra::Matrix4::new(
        -1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );

    // Apply the transformation using the existing transform method
    Ok(csg.transform(&reflection_matrix))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minkowski_difference_basic() {
        let cube: CSG<()> = CSG::cube(2.0, None);
        let sphere: CSG<()> = CSG::sphere(0.5, 8, 6, None);
        
        let _result = compute(&cube, &sphere);
        // The result should be valid (not necessarily non-empty for difference)
    }

    #[test]
    fn test_minkowski_difference_empty_input() {
        let cube: CSG<()> = CSG::cube(1.0, None);
        let empty: CSG<()> = CSG::new();
        
        let result = compute(&cube, &empty);
        assert!(result.polygons.is_empty(), "Result should be empty for empty input");
    }

    #[test]
    fn test_reflect_through_origin() {
        let cube: CSG<()> = CSG::cube(1.0, None).translate(1.0, 1.0, 1.0);
        let reflected = reflect_through_origin(&cube).unwrap();
        
        let original_bbox = cube.bounding_box();
        let reflected_bbox = reflected.bounding_box();
        
        // The reflected bounding box should be the negative of the original
        assert!((original_bbox.mins.x + reflected_bbox.maxs.x).abs() < 1e-10);
        assert!((original_bbox.mins.y + reflected_bbox.maxs.y).abs() < 1e-10);
        assert!((original_bbox.mins.z + reflected_bbox.maxs.z).abs() < 1e-10);
    }

    #[test]
    fn test_minkowski_difference_with_config() {
        let cube: CSG<()> = CSG::cube(2.0, None);
        let sphere: CSG<()> = CSG::sphere(0.5, 8, 6, None);
        let config = MinkowskiConfig::high_precision();
        
        let _result = compute_with_config(&cube, &sphere, &config);
        // Result should be valid
    }
}
