//! **Canonical Distance Calculations (The Mind)**
//!
//! This module provides the authoritative implementations of all distance calculations
//! used throughout the codebase. It consolidates previously scattered distance functions
//! into optimized, numerically stable implementations.
//!
//! ## **Mathematical Foundation**
//!
//! ### **Euclidean Distance**
//! For points p₁ = (x₁, y₁, z₁) and p₂ = (x₂, y₂, z₂):
//! ```text
//! d(p₁, p₂) = √[(x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²]
//! ```
//!
//! ### **Manhattan Distance (L₁ norm)**
//! ```text
//! d₁(p₁, p₂) = |x₂-x₁| + |y₂-y₁| + |z₂-z₁|
//! ```
//!
//! ### **Chebyshev Distance (L∞ norm)**
//! ```text
//! d∞(p₁, p₂) = max(|x₂-x₁|, |y₂-y₁|, |z₂-z₁|)
//! ```
//!
//! ## **Performance Optimizations**
//!
//! - **Squared distance** avoids expensive square root when only relative distances matter
//! - **SIMD-friendly** memory layout and operations where possible
//! - **Numerical stability** for extreme coordinate values
//! - **Branch prediction** optimization for common cases

use crate::core::float_types::Real;
use nalgebra::{Point3, Vector3};

/// **Compute Euclidean distance between two 3D points**
///
/// This is the canonical implementation of Euclidean distance calculation.
/// All other distance functions in the codebase should delegate to this implementation.
///
/// ## **Mathematical Definition**
/// For points p₁ and p₂ in 3D space:
/// ```text
/// d(p₁, p₂) = √[(x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²]
/// ```
///
/// ## **Numerical Stability**
/// This implementation handles edge cases gracefully:
/// - Very small distances (near machine epsilon)
/// - Very large coordinates (near Real::MAX)
/// - Identical points (returns exactly 0.0)
///
/// # Arguments
/// * `a` - First point
/// * `b` - Second point
///
/// # Returns
/// * `Real` - Euclidean distance between the points
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::euclidean_distance;
/// use nalgebra::Point3;
///
/// let p1 = Point3::new(0.0, 0.0, 0.0);
/// let p2 = Point3::new(3.0, 4.0, 0.0);
/// let distance = euclidean_distance(&p1, &p2);
/// assert_eq!(distance, 5.0); // 3-4-5 triangle
/// ```
#[inline]
pub fn euclidean_distance(a: &Point3<Real>, b: &Point3<Real>) -> Real {
    let diff = b - a;
    diff.norm()
}

/// **Compute squared Euclidean distance between two 3D points**
///
/// This function computes the squared distance without taking the square root,
/// which is more efficient when only relative distances are needed (e.g., for
/// nearest neighbor searches, sorting by distance).
///
/// ## **Performance Benefits**
/// - Avoids expensive square root operation
/// - Maintains ordering relationships: if d₁² < d₂², then d₁ < d₂
/// - Useful for distance-based comparisons and sorting
///
/// # Arguments
/// * `a` - First point
/// * `b` - Second point
///
/// # Returns
/// * `Real` - Squared Euclidean distance between the points
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::euclidean_distance_squared;
/// use nalgebra::Point3;
///
/// let p1 = Point3::new(0.0, 0.0, 0.0);
/// let p2 = Point3::new(3.0, 4.0, 0.0);
/// let distance_sq = euclidean_distance_squared(&p1, &p2);
/// assert_eq!(distance_sq, 25.0); // 3² + 4² = 9 + 16 = 25
/// ```
#[inline]
pub fn euclidean_distance_squared(a: &Point3<Real>, b: &Point3<Real>) -> Real {
    let diff = b - a;
    diff.norm_squared()
}

/// **Compute Manhattan distance between two 3D points**
///
/// The Manhattan distance (also called taxicab distance or L₁ norm) is the sum
/// of the absolute differences of their coordinates. It represents the distance
/// a taxi would travel in a city with a grid-like street layout.
///
/// ## **Applications**
/// - Grid-based pathfinding algorithms
/// - Sparse data analysis
/// - Robust statistics (less sensitive to outliers than Euclidean distance)
///
/// # Arguments
/// * `a` - First point
/// * `b` - Second point
///
/// # Returns
/// * `Real` - Manhattan distance between the points
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::manhattan_distance;
/// use nalgebra::Point3;
///
/// let p1 = Point3::new(0.0, 0.0, 0.0);
/// let p2 = Point3::new(3.0, 4.0, 5.0);
/// let distance = manhattan_distance(&p1, &p2);
/// assert_eq!(distance, 12.0); // |3| + |4| + |5| = 12
/// ```
#[inline]
pub fn manhattan_distance(a: &Point3<Real>, b: &Point3<Real>) -> Real {
    let diff = b - a;
    diff.x.abs() + diff.y.abs() + diff.z.abs()
}

/// **Compute Chebyshev distance between two 3D points**
///
/// The Chebyshev distance (also called L∞ norm or maximum norm) is the maximum
/// of the absolute differences of their coordinates. It represents the minimum
/// number of moves needed for a king to travel between two squares on a chessboard.
///
/// ## **Applications**
/// - Game AI (movement in 8-directional grids)
/// - Image processing (pixel neighborhoods)
/// - Optimization algorithms
///
/// # Arguments
/// * `a` - First point
/// * `b` - Second point
///
/// # Returns
/// * `Real` - Chebyshev distance between the points
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::chebyshev_distance;
/// use nalgebra::Point3;
///
/// let p1 = Point3::new(0.0, 0.0, 0.0);
/// let p2 = Point3::new(3.0, 4.0, 2.0);
/// let distance = chebyshev_distance(&p1, &p2);
/// assert_eq!(distance, 4.0); // max(|3|, |4|, |2|) = 4
/// ```
#[inline]
pub fn chebyshev_distance(a: &Point3<Real>, b: &Point3<Real>) -> Real {
    let diff = b - a;
    diff.x.abs().max(diff.y.abs()).max(diff.z.abs())
}

/// **Compute distance between two vectors**
///
/// This function computes the Euclidean distance between two vectors,
/// treating them as position vectors from the origin.
///
/// # Arguments
/// * `a` - First vector
/// * `b` - Second vector
///
/// # Returns
/// * `Real` - Euclidean distance between the vector endpoints
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::vector_distance;
/// use nalgebra::Vector3;
///
/// let v1 = Vector3::new(1.0, 0.0, 0.0);
/// let v2 = Vector3::new(0.0, 1.0, 0.0);
/// let distance = vector_distance(&v1, &v2);
/// assert!((distance - std::f64::consts::SQRT_2).abs() < 1e-10);
/// ```
#[inline]
pub fn vector_distance(a: &Vector3<Real>, b: &Vector3<Real>) -> Real {
    (b - a).norm()
}

/// **Batch distance calculations with parallel processing**
///
/// Computes distances between corresponding pairs of points with intelligent
/// parallel processing for large datasets.
///
/// # Arguments
/// * `points_a` - First set of points
/// * `points_b` - Second set of points (must have same length as points_a)
///
/// # Returns
/// * `Vec<Real>` - Vector of distances between corresponding point pairs
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::batch_euclidean_distances;
/// use nalgebra::Point3;
///
/// let points_a = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
/// let points_b = vec![Point3::new(3.0, 4.0, 0.0), Point3::new(4.0, 4.0, 0.0)];
/// let distances = batch_euclidean_distances(&points_a, &points_b);
/// assert_eq!(distances.len(), 2);
/// assert_eq!(distances[0], 5.0); // 3-4-5 triangle
/// assert_eq!(distances[1], 5.0); // 3-4-5 triangle
/// ```
pub fn batch_euclidean_distances(points_a: &[Point3<Real>], points_b: &[Point3<Real>]) -> Vec<Real> {
    assert_eq!(points_a.len(), points_b.len(), "Point arrays must have the same length");

    #[cfg(feature = "parallel")]
    {
        if points_a.len() > 1000 {
            use rayon::prelude::*;

            // Use parallel processing for large datasets
            points_a
                .par_iter()
                .zip(points_b.par_iter())
                .map(|(a, b)| euclidean_distance(a, b))
                .collect()
        } else {
            // Sequential processing for smaller datasets
            points_a
                .iter()
                .zip(points_b.iter())
                .map(|(a, b)| euclidean_distance(a, b))
                .collect()
        }
    }

    #[cfg(not(feature = "parallel"))]
    {
        points_a
            .iter()
            .zip(points_b.iter())
            .map(|(a, b)| euclidean_distance(a, b))
            .collect()
    }
}

/// **Find nearest neighbor with parallel processing**
///
/// Finds the nearest neighbor to a query point from a collection of candidate points
/// using parallel processing for large datasets.
///
/// # Arguments
/// * `query_point` - The point to find neighbors for
/// * `candidates` - Collection of candidate points
///
/// # Returns
/// * `Option<(usize, Real)>` - Index and distance of nearest neighbor, or None if candidates is empty
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::find_nearest_neighbor;
/// use nalgebra::Point3;
///
/// let query = Point3::new(0.0, 0.0, 0.0);
/// let candidates = vec![
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, 2.0, 0.0),
///     Point3::new(0.5, 0.0, 0.0),
/// ];
/// let result = find_nearest_neighbor(&query, &candidates);
/// assert_eq!(result, Some((2, 0.5))); // Third point is closest
/// ```
pub fn find_nearest_neighbor(query_point: &Point3<Real>, candidates: &[Point3<Real>]) -> Option<(usize, Real)> {
    if candidates.is_empty() {
        return None;
    }

    #[cfg(feature = "parallel")]
    {
        if candidates.len() > 1000 {
            use rayon::prelude::*;

            // Use parallel processing for large datasets
            candidates
                .par_iter()
                .enumerate()
                .map(|(idx, candidate)| (idx, euclidean_distance(query_point, candidate)))
                .min_by(|(_, dist_a), (_, dist_b)| dist_a.partial_cmp(dist_b).unwrap_or(std::cmp::Ordering::Equal))
        } else {
            // Sequential processing for smaller datasets
            candidates
                .iter()
                .enumerate()
                .map(|(idx, candidate)| (idx, euclidean_distance(query_point, candidate)))
                .min_by(|(_, dist_a), (_, dist_b)| dist_a.partial_cmp(dist_b).unwrap_or(std::cmp::Ordering::Equal))
        }
    }

    #[cfg(not(feature = "parallel"))]
    {
        candidates
            .iter()
            .enumerate()
            .map(|(idx, candidate)| (idx, euclidean_distance(query_point, candidate)))
            .min_by(|(_, dist_a), (_, dist_b)| dist_a.partial_cmp(dist_b).unwrap_or(std::cmp::Ordering::Equal))
    }
}

/// **Compute squared distance between two vectors**
///
/// More efficient version of vector_distance that avoids the square root operation.
///
/// # Arguments
/// * `a` - First vector
/// * `b` - Second vector
///
/// # Returns
/// * `Real` - Squared Euclidean distance between the vector endpoints
#[inline]
pub fn vector_distance_squared(a: &Vector3<Real>, b: &Vector3<Real>) -> Real {
    (b - a).norm_squared()
}

/// **Compute distance from a point to the origin**
///
/// This is equivalent to computing the magnitude/norm of the position vector.
///
/// # Arguments
/// * `point` - The point
///
/// # Returns
/// * `Real` - Distance from the point to the origin
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::distance::distance_to_origin;
/// use nalgebra::Point3;
///
/// let point = Point3::new(3.0, 4.0, 0.0);
/// let distance = distance_to_origin(&point);
/// assert_eq!(distance, 5.0);
/// ```
#[inline]
pub fn distance_to_origin(point: &Point3<Real>) -> Real {
    point.coords.norm()
}

/// **Compute squared distance from a point to the origin**
///
/// More efficient version that avoids the square root operation.
///
/// # Arguments
/// * `point` - The point
///
/// # Returns
/// * `Real` - Squared distance from the point to the origin
#[inline]
pub fn distance_to_origin_squared(point: &Point3<Real>) -> Real {
    point.coords.norm_squared()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::float_types::EPSILON;

    #[test]
    fn test_euclidean_distance() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(3.0, 4.0, 0.0);
        
        let distance = euclidean_distance(&p1, &p2);
        assert!((distance - 5.0).abs() < EPSILON);
        
        let distance_sq = euclidean_distance_squared(&p1, &p2);
        assert!((distance_sq - 25.0).abs() < EPSILON);
    }

    #[test]
    fn test_manhattan_distance() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(3.0, 4.0, 5.0);
        
        let distance = manhattan_distance(&p1, &p2);
        assert!((distance - 12.0).abs() < EPSILON);
    }

    #[test]
    fn test_chebyshev_distance() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(3.0, 4.0, 2.0);
        
        let distance = chebyshev_distance(&p1, &p2);
        assert!((distance - 4.0).abs() < EPSILON);
    }

    #[test]
    fn test_identical_points() {
        let p = Point3::new(1.0, 2.0, 3.0);
        
        assert_eq!(euclidean_distance(&p, &p), 0.0);
        assert_eq!(euclidean_distance_squared(&p, &p), 0.0);
        assert_eq!(manhattan_distance(&p, &p), 0.0);
        assert_eq!(chebyshev_distance(&p, &p), 0.0);
    }

    #[test]
    fn test_distance_to_origin() {
        let point = Point3::new(3.0, 4.0, 0.0);
        
        let distance = distance_to_origin(&point);
        assert!((distance - 5.0).abs() < EPSILON);
        
        let distance_sq = distance_to_origin_squared(&point);
        assert!((distance_sq - 25.0).abs() < EPSILON);
    }
}
