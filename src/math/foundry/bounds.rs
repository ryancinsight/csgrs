//! **Canonical Bounding Operations (The Mind)**
//!
//! This module provides the authoritative implementations of all bounding box and
//! spatial bound operations used throughout the codebase. It consolidates previously
//! scattered bounding functions into optimized, mathematically correct implementations.
//!
//! ## **Mathematical Foundation**
//!
//! ### **Axis-Aligned Bounding Box (AABB)**
//! An AABB is defined by two points: minimum corner (min_x, min_y, min_z) and
//! maximum corner (max_x, max_y, max_z).
//!
//! ### **Union Operation**
//! For AABBs A and B:
//! ```text
//! Union(A, B) = (min(A.min, B.min), max(A.max, B.max))
//! ```
//!
//! ### **Intersection Operation**
//! ```text
//! Intersection(A, B) = (max(A.min, B.min), min(A.max, B.max))
//! ```
//! Valid only if max(A.min, B.min) ≤ min(A.max, B.max) in all dimensions.

use crate::core::float_types::{parry3d::bounding_volume::{Aabb, BoundingVolume}, Real};
use nalgebra::Point3;

/// **Compute the union of multiple AABBs with parallel processing**
///
/// This function computes the smallest AABB that contains all input AABBs.
/// It's the canonical implementation for AABB union operations with intelligent
/// parallel processing for large datasets.
///
/// ## **Mathematical Definition**
/// For AABBs A₁, A₂, ..., Aₙ:
/// ```text
/// Union(A₁, A₂, ..., Aₙ) = (
///     min(A₁.min, A₂.min, ..., Aₙ.min),
///     max(A₁.max, A₂.max, ..., Aₙ.max)
/// )
/// ```
///
/// # Arguments
/// * `aabbs` - Slice of AABBs to union
///
/// # Returns
/// * `Option<Aabb>` - The union AABB, or None if input is empty
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_union;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
/// let aabb2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
/// let union = aabb_union(&[aabb1, aabb2]).unwrap();
/// 
/// assert_eq!(union.mins, Point3::new(0.0, 0.0, 0.0));
/// assert_eq!(union.maxs, Point3::new(1.5, 1.5, 1.5));
/// ```
pub fn aabb_union(aabbs: &[Aabb]) -> Option<Aabb> {
    if aabbs.is_empty() {
        return None;
    }

    #[cfg(feature = "parallel")]
    {
        if aabbs.len() > 1000 {
            use rayon::prelude::*;

            // Use parallel processing for large datasets
            let result = aabbs[1..]
                .par_iter()
                .fold(
                    || aabbs[0],
                    |acc, aabb| acc.merged(aabb)
                )
                .reduce(
                    || aabbs[0],
                    |a, b| a.merged(&b)
                );
            return Some(result);
        }
    }

    // Sequential processing for smaller datasets or when parallel feature is disabled
    let result = aabbs[1..]
        .iter()
        .fold(aabbs[0], |acc, aabb| acc.merged(aabb));
    Some(result)
}

/// **Compute the intersection of two AABBs**
///
/// This function computes the AABB representing the intersection of two input AABBs.
/// Returns None if the AABBs don't intersect.
///
/// ## **Mathematical Definition**
/// For AABBs A and B:
/// ```text
/// Intersection(A, B) = (max(A.min, B.min), min(A.max, B.max))
/// ```
/// Valid only if the result has positive volume in all dimensions.
///
/// # Arguments
/// * `a` - First AABB
/// * `b` - Second AABB
///
/// # Returns
/// * `Option<Aabb>` - The intersection AABB, or None if no intersection
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_intersection;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
/// let aabb2 = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
/// let intersection = aabb_intersection(&aabb1, &aabb2).unwrap();
/// 
/// assert_eq!(intersection.mins, Point3::new(1.0, 1.0, 1.0));
/// assert_eq!(intersection.maxs, Point3::new(2.0, 2.0, 2.0));
/// ```
pub fn aabb_intersection(a: &Aabb, b: &Aabb) -> Option<Aabb> {
    let min_point = Point3::new(
        a.mins.x.max(b.mins.x),
        a.mins.y.max(b.mins.y),
        a.mins.z.max(b.mins.z),
    );
    let max_point = Point3::new(
        a.maxs.x.min(b.maxs.x),
        a.maxs.y.min(b.maxs.y),
        a.maxs.z.min(b.maxs.z),
    );

    // Check if intersection is valid (positive volume)
    if min_point.x <= max_point.x && min_point.y <= max_point.y && min_point.z <= max_point.z {
        Some(Aabb::new(min_point, max_point))
    } else {
        None
    }
}

/// **Check if an AABB contains a point**
///
/// This function tests whether a point lies within or on the boundary of an AABB.
///
/// ## **Mathematical Definition**
/// Point p is contained in AABB A if:
/// ```text
/// A.min.x ≤ p.x ≤ A.max.x AND
/// A.min.y ≤ p.y ≤ A.max.y AND
/// A.min.z ≤ p.z ≤ A.max.z
/// ```
///
/// # Arguments
/// * `aabb` - The AABB to test
/// * `point` - The point to test
///
/// # Returns
/// * `bool` - True if the point is contained in the AABB
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_contains_point;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
/// let point_inside = Point3::new(0.5, 0.5, 0.5);
/// let point_outside = Point3::new(1.5, 0.5, 0.5);
/// 
/// assert!(aabb_contains_point(&aabb, &point_inside));
/// assert!(!aabb_contains_point(&aabb, &point_outside));
/// ```
#[inline]
pub fn aabb_contains_point(aabb: &Aabb, point: &Point3<Real>) -> bool {
    point.x >= aabb.mins.x && point.x <= aabb.maxs.x &&
    point.y >= aabb.mins.y && point.y <= aabb.maxs.y &&
    point.z >= aabb.mins.z && point.z <= aabb.maxs.z
}

/// **Compute the volume of an AABB**
///
/// This function calculates the volume (width × height × depth) of an AABB.
///
/// ## **Mathematical Definition**
/// For AABB A:
/// ```text
/// Volume(A) = (A.max.x - A.min.x) × (A.max.y - A.min.y) × (A.max.z - A.min.z)
/// ```
///
/// # Arguments
/// * `aabb` - The AABB to measure
///
/// # Returns
/// * `Real` - The volume of the AABB
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_volume;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
/// let volume = aabb_volume(&aabb);
/// assert_eq!(volume, 24.0); // 2 × 3 × 4 = 24
/// ```
#[inline]
pub fn aabb_volume(aabb: &Aabb) -> Real {
    let extents = aabb.maxs - aabb.mins;
    extents.x * extents.y * extents.z
}

/// **Compute the surface area of an AABB**
///
/// This function calculates the total surface area of an AABB.
///
/// ## **Mathematical Definition**
/// For AABB A with dimensions w, h, d:
/// ```text
/// SurfaceArea(A) = 2 × (w×h + w×d + h×d)
/// ```
///
/// # Arguments
/// * `aabb` - The AABB to measure
///
/// # Returns
/// * `Real` - The surface area of the AABB
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_surface_area;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
/// let surface_area = aabb_surface_area(&aabb);
/// assert_eq!(surface_area, 52.0); // 2×(2×3 + 2×4 + 3×4) = 2×(6+8+12) = 52
/// ```
#[inline]
pub fn aabb_surface_area(aabb: &Aabb) -> Real {
    let extents = aabb.maxs - aabb.mins;
    let w = extents.x;
    let h = extents.y;
    let d = extents.z;
    2.0 * (w * h + w * d + h * d)
}

/// **Compute the center point of an AABB**
///
/// This function calculates the geometric center of an AABB.
///
/// ## **Mathematical Definition**
/// For AABB A:
/// ```text
/// Center(A) = (A.min + A.max) / 2
/// ```
///
/// # Arguments
/// * `aabb` - The AABB
///
/// # Returns
/// * `Point3<Real>` - The center point of the AABB
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_center;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 4.0, 6.0));
/// let center = aabb_center(&aabb);
/// assert_eq!(center, Point3::new(1.0, 2.0, 3.0));
/// ```
#[inline]
pub fn aabb_center(aabb: &Aabb) -> Point3<Real> {
    Point3::from((aabb.mins.coords + aabb.maxs.coords) * 0.5)
}

/// **Compute the extents (half-sizes) of an AABB**
///
/// This function calculates the extents (half the width, height, and depth) of an AABB.
///
/// # Arguments
/// * `aabb` - The AABB
///
/// # Returns
/// * `nalgebra::Vector3<Real>` - The extents of the AABB
#[inline]
pub fn aabb_extents(aabb: &Aabb) -> nalgebra::Vector3<Real> {
    (aabb.maxs - aabb.mins) * 0.5
}

/// **Check if two AABBs intersect**
///
/// This function tests whether two AABBs overlap in any way.
///
/// # Arguments
/// * `a` - First AABB
/// * `b` - Second AABB
///
/// # Returns
/// * `bool` - True if the AABBs intersect
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::aabb_intersects;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
/// let aabb2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
/// let aabb3 = Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
/// 
/// assert!(aabb_intersects(&aabb1, &aabb2));
/// assert!(!aabb_intersects(&aabb1, &aabb3));
/// ```
#[inline]
pub fn aabb_intersects(a: &Aabb, b: &Aabb) -> bool {
    a.maxs.x >= b.mins.x && a.mins.x <= b.maxs.x &&
    a.maxs.y >= b.mins.y && a.mins.y <= b.maxs.y &&
    a.maxs.z >= b.mins.z && a.mins.z <= b.maxs.z
}

/// **Expand an AABB by a uniform margin**
///
/// This function creates a new AABB that is expanded by the specified margin
/// in all directions.
///
/// # Arguments
/// * `aabb` - The original AABB
/// * `margin` - The margin to expand by
///
/// # Returns
/// * `Aabb` - The expanded AABB
#[inline]
pub fn aabb_expand(aabb: &Aabb, margin: Real) -> Aabb {
    let margin_vec = nalgebra::Vector3::new(margin, margin, margin);
    Aabb::new(
        Point3::from(aabb.mins.coords - margin_vec),
        Point3::from(aabb.maxs.coords + margin_vec),
    )
}

/// **Batch point containment testing with parallel processing**
///
/// Tests whether multiple points are contained within an AABB using intelligent
/// parallel processing for large datasets.
///
/// # Arguments
/// * `aabb` - The bounding box to test against
/// * `points` - Slice of points to test
///
/// # Returns
/// * `Vec<bool>` - Vector indicating containment for each point
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::batch_aabb_contains_points;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
/// let points = vec![
///     Point3::new(0.5, 0.5, 0.5),  // inside
///     Point3::new(1.5, 0.5, 0.5),  // outside
/// ];
/// let results = batch_aabb_contains_points(&aabb, &points);
/// assert_eq!(results, vec![true, false]);
/// ```
pub fn batch_aabb_contains_points(aabb: &Aabb, points: &[Point3<Real>]) -> Vec<bool> {
    #[cfg(feature = "parallel")]
    {
        if points.len() > 1000 {
            use rayon::prelude::*;

            // Use parallel processing for large datasets
            return points
                .par_iter()
                .map(|point| aabb_contains_point(aabb, point))
                .collect();
        }
    }

    // Sequential processing for smaller datasets or when parallel feature is disabled
    points
        .iter()
        .map(|point| aabb_contains_point(aabb, point))
        .collect()
}

/// **Batch AABB volume calculations with parallel processing**
///
/// Computes volumes for multiple AABBs using intelligent parallel processing
/// for large datasets.
///
/// # Arguments
/// * `aabbs` - Slice of AABBs to compute volumes for
///
/// # Returns
/// * `Vec<Real>` - Vector of volumes for each AABB
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::bounds::batch_aabb_volumes;
/// use csgrs::core::float_types::parry3d::bounding_volume::Aabb;
/// use nalgebra::Point3;
///
/// let aabbs = vec![
///     Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)),
///     Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0)),
/// ];
/// let volumes = batch_aabb_volumes(&aabbs);
/// assert_eq!(volumes, vec![1.0, 8.0]);
/// ```
pub fn batch_aabb_volumes(aabbs: &[Aabb]) -> Vec<Real> {
    #[cfg(feature = "parallel")]
    {
        if aabbs.len() > 1000 {
            use rayon::prelude::*;

            // Use parallel processing for large datasets
            return aabbs
                .par_iter()
                .map(|aabb| aabb_volume(aabb))
                .collect();
        }
    }

    // Sequential processing for smaller datasets or when parallel feature is disabled
    aabbs
        .iter()
        .map(|aabb| aabb_volume(aabb))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::float_types::EPSILON;

    #[test]
    fn test_aabb_union() {
        let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let aabb2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
        
        let union = aabb_union(&[aabb1, aabb2]).unwrap();
        assert_eq!(union.mins, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(union.maxs, Point3::new(1.5, 1.5, 1.5));
    }

    #[test]
    fn test_aabb_intersection() {
        let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let aabb2 = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
        
        let intersection = aabb_intersection(&aabb1, &aabb2).unwrap();
        assert_eq!(intersection.mins, Point3::new(1.0, 1.0, 1.0));
        assert_eq!(intersection.maxs, Point3::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn test_aabb_no_intersection() {
        let aabb1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let aabb2 = Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
        
        assert!(aabb_intersection(&aabb1, &aabb2).is_none());
    }

    #[test]
    fn test_aabb_contains_point() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        
        assert!(aabb_contains_point(&aabb, &Point3::new(0.5, 0.5, 0.5)));
        assert!(aabb_contains_point(&aabb, &Point3::new(0.0, 0.0, 0.0))); // boundary
        assert!(aabb_contains_point(&aabb, &Point3::new(1.0, 1.0, 1.0))); // boundary
        assert!(!aabb_contains_point(&aabb, &Point3::new(1.5, 0.5, 0.5)));
    }

    #[test]
    fn test_aabb_volume() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        let volume = aabb_volume(&aabb);
        assert!((volume - 24.0).abs() < EPSILON);
    }

    #[test]
    fn test_aabb_surface_area() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        let surface_area = aabb_surface_area(&aabb);
        assert!((surface_area - 52.0).abs() < EPSILON); // 2×(6+8+12) = 52
    }

    #[test]
    fn test_aabb_center() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 4.0, 6.0));
        let center = aabb_center(&aabb);
        assert_eq!(center, Point3::new(1.0, 2.0, 3.0));
    }
}
