//! **Minkowski Sum Operations (The Mind)**
//!
//! This module implements Minkowski sum operations following Cathedral Engineering
//! principles where the implementation represents the "mind" that performs the
//! actual computational work of the architectural space.
//!
//! ## **Mathematical Foundation**
//!
//! ### **Minkowski Sum Definition**
//! For sets A and B in Euclidean space, the Minkowski sum A ⊕ B is defined as:
//! ```text
//! A ⊕ B = {a + b | a ∈ A, b ∈ B}
//! ```
//!
//! ### **Convex Hull Theorem**
//! For convex sets A and B, the Minkowski sum equals the convex hull of all
//! pairwise vertex sums. This theorem enables efficient computation by reducing
//! the problem to convex hull computation.
//!
//! ### **Algorithmic Complexity**
//! - **Vertex Enumeration**: O(|A| × |B|) for generating all pairwise sums
//! - **Convex Hull Computation**: O(n log n) where n = |A| × |B|
//! - **Total Complexity**: O(|A| × |B| + n log n)

use crate::core::float_types::Real;
use crate::csg::CSG;
use crate::geometry::{Polygon, Vertex};
use super::errors::{MinkowskiError, MinkowskiResult};
use super::models::{MinkowskiConfig, MinkowskiState};
use chull::ConvexHullWrapper;
use nalgebra::Point3;
use std::fmt::Debug;
use std::time::Instant;

/// **Compute the Minkowski sum of two CSG objects**
///
/// This is the primary entry point for Minkowski sum computation, providing
/// a clean interface that handles all the complexity of the operation while
/// maintaining mathematical correctness.
///
/// ## **Mathematical Foundation**
/// For CSG objects A and B, computes A ⊕ B = {a + b | a ∈ A, b ∈ B}.
/// The result is the convex hull of all pairwise vertex sums.
///
/// ## **Algorithm**
/// 1. Extract vertices from both CSG objects
/// 2. Validate inputs for mathematical correctness
/// 3. Compute all pairwise vertex sums
/// 4. Generate convex hull of the sum points
/// 5. Reconstruct CSG from hull triangulation
///
/// # Arguments
/// * `csg_a` - First operand for the Minkowski sum
/// * `csg_b` - Second operand for the Minkowski sum
///
/// # Returns
/// * `CSG<S>` - The Minkowski sum result
///
/// # Examples
/// ```rust
/// use csgrs::CSG;
/// use csgrs::math::minkowski::sum;
///
/// let cube: CSG<()> = CSG::cube(1.0, None);
/// let sphere: CSG<()> = CSG::sphere(0.5, 16, 8, None);
/// let result = sum::compute(&cube, &sphere);
/// ```
pub fn compute<S>(csg_a: &CSG<S>, csg_b: &CSG<S>) -> CSG<S>
where
    S: Clone + Debug + Send + Sync,
{
    compute_with_config(csg_a, csg_b, &MinkowskiConfig::default())
}

/// **Compute Minkowski sum with custom configuration**
///
/// This function provides fine-grained control over the Minkowski sum computation
/// through configuration parameters, enabling optimization for specific use cases.
///
/// # Arguments
/// * `csg_a` - First operand for the Minkowski sum
/// * `csg_b` - Second operand for the Minkowski sum
/// * `config` - Configuration parameters for the operation
///
/// # Returns
/// * `CSG<S>` - The Minkowski sum result
pub fn compute_with_config<S>(csg_a: &CSG<S>, csg_b: &CSG<S>, config: &MinkowskiConfig) -> CSG<S>
where
    S: Clone + Debug + Send + Sync,
{
    match compute_with_result(csg_a, csg_b, config) {
        Ok(result) => result,
        Err(_) => {
            // Fallback to empty CSG for compatibility with existing API
            // In a future version, this could be changed to return Result<CSG<S>, MinkowskiError>
            CSG::new()
        }
    }
}

/// **Compute Minkowski sum with detailed error reporting**
///
/// This function provides the most comprehensive interface, returning detailed
/// error information when operations fail, enabling robust error handling.
///
/// # Arguments
/// * `csg_a` - First operand for the Minkowski sum
/// * `csg_b` - Second operand for the Minkowski sum
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
    let start_time = Instant::now();
    let mut state = MinkowskiState::new(config.clone());

    // Step 1: Extract vertices from both CSG objects
    extract_vertices(csg_a, csg_b, &mut state)?;

    // Step 2: Validate inputs
    validate_inputs(&state)?;

    // Step 3: Compute pairwise vertex sums
    compute_vertex_sums(&mut state)?;

    // Step 4: Generate convex hull
    let hull_start = Instant::now();
    let hull = compute_convex_hull(&state)?;
    state.metrics.convex_hull_time_ns = hull_start.elapsed().as_nanos() as u64;

    // Step 5: Reconstruct CSG from hull
    let result = reconstruct_csg_from_hull(hull)?;

    // Update final metrics
    state.metrics.vertex_enumeration_time_ns = 
        start_time.elapsed().as_nanos() as u64 - state.metrics.convex_hull_time_ns;

    Ok(result)
}

/// **Extract vertices from CSG objects into computation state**
fn extract_vertices<S>(
    csg_a: &CSG<S>, 
    csg_b: &CSG<S>, 
    state: &mut MinkowskiState
) -> MinkowskiResult<()>
where
    S: Clone + Debug + Send + Sync,
{
    // Extract vertices using iterator chains for better performance
    state.vertices_a = csg_a
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
        .collect();

    state.vertices_b = csg_b
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
        .collect();

    // Check vertex count limits
    if state.config.max_vertices > 0 {
        let total_vertices = state.vertices_a.len() + state.vertices_b.len();
        if total_vertices > state.config.max_vertices {
            return Err(MinkowskiError::complexity_limit(
                total_vertices,
                state.config.max_vertices,
            ));
        }
    }

    Ok(())
}

/// **Validate inputs for mathematical correctness**
fn validate_inputs(state: &MinkowskiState) -> MinkowskiResult<()> {
    // Check for empty geometries
    if state.vertices_a.is_empty() {
        return Err(MinkowskiError::empty_geometry("first"));
    }
    if state.vertices_b.is_empty() {
        return Err(MinkowskiError::empty_geometry("second"));
    }

    // Validate configuration
    if let Err(config_error) = state.config.validate() {
        return Err(MinkowskiError::invalid_input(config_error));
    }

    Ok(())
}

/// **Compute all pairwise vertex sums**
fn compute_vertex_sums(state: &mut MinkowskiState) -> MinkowskiResult<()> {
    // Pre-allocate capacity for better memory performance
    let expected_capacity = state.vertices_a.len() * state.vertices_b.len();
    
    // Check memory requirements
    let estimated_memory = expected_capacity * std::mem::size_of::<Point3<Real>>();
    if estimated_memory > 1_000_000_000 { // 1GB limit
        return Err(MinkowskiError::insufficient_memory(estimated_memory));
    }

    state.sum_points = Vec::with_capacity(expected_capacity);

    // Compute Minkowski sum using optimized iterator pattern
    // Mathematical theorem: A ⊕ B = {a + b | a ∈ A, b ∈ B}
    for a in &state.vertices_a {
        for b in &state.vertices_b {
            state.sum_points.push(a + b.coords);
            state.metrics.vertex_pairs_processed += 1;
        }
    }

    // Validate that we generated points
    if state.sum_points.is_empty() {
        return Err(MinkowskiError::invalid_input(
            "No sum points generated from input vertices"
        ));
    }

    Ok(())
}

/// **Compute convex hull of sum points**
fn compute_convex_hull(state: &MinkowskiState) -> MinkowskiResult<ConvexHullWrapper<Real>> {
    // Convert to format expected by hull library
    let points_for_hull: Vec<Vec<Real>> = state
        .sum_points
        .iter()
        .map(|p| vec![p.x, p.y, p.z])
        .collect();

    // Compute convex hull with proper error handling
    ConvexHullWrapper::try_new(&points_for_hull, None)
        .map_err(|e| MinkowskiError::convex_hull_failure(
            format!("Hull computation failed: {:?}", e),
            points_for_hull.len(),
        ))
}

/// **Reconstruct CSG from convex hull triangulation**
fn reconstruct_csg_from_hull<S>(hull: ConvexHullWrapper<Real>) -> MinkowskiResult<CSG<S>>
where
    S: Clone + Debug + Send + Sync,
{
    let (verts, indices) = hull.vertices_indices();

    // Reconstruct polygons with proper normal vector calculation
    let polygons: Vec<Polygon<S>> = indices
        .chunks_exact(3)
        .filter_map(|tri| {
            let v0 = &verts[tri[0]];
            let v1 = &verts[tri[1]];
            let v2 = &verts[tri[2]];

            let p0 = Point3::new(v0[0], v0[1], v0[2]);
            let p1 = Point3::new(v1[0], v1[1], v1[2]);
            let p2 = Point3::new(v2[0], v2[1], v2[2]);

            // Calculate proper normal vector using cross product
            let edge1 = p1 - p0;
            let edge2 = p2 - p0;
            let normal = edge1.cross(&edge2);

            // Filter out degenerate triangles
            if normal.norm_squared() > Real::EPSILON {
                let normalized_normal = normal.normalize();
                let vv0 = Vertex::new(p0, normalized_normal);
                let vv1 = Vertex::new(p1, normalized_normal);
                let vv2 = Vertex::new(p2, normalized_normal);
                Some(Polygon::new(vec![vv0, vv1, vv2], None))
            } else {
                None
            }
        })
        .collect();

    Ok(CSG::from_polygons(&polygons))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minkowski_sum_basic() {
        let cube: CSG<()> = CSG::cube(1.0, None);
        let sphere: CSG<()> = CSG::sphere(0.5, 8, 6, None);
        
        let result = compute(&cube, &sphere);
        assert!(!result.polygons.is_empty(), "Result should not be empty");
    }

    #[test]
    fn test_minkowski_sum_empty_input() {
        let cube: CSG<()> = CSG::cube(1.0, None);
        let empty: CSG<()> = CSG::new();
        
        let result = compute(&cube, &empty);
        assert!(result.polygons.is_empty(), "Result should be empty for empty input");
    }

    #[test]
    fn test_minkowski_sum_with_config() {
        let cube: CSG<()> = CSG::cube(1.0, None);
        let sphere: CSG<()> = CSG::sphere(0.5, 8, 6, None);
        let config = MinkowskiConfig::high_performance();
        
        let result = compute_with_config(&cube, &sphere, &config);
        assert!(!result.polygons.is_empty(), "Result should not be empty");
    }
}
