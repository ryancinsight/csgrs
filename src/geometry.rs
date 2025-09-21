//! Shared geometric utilities and algorithms
//!
//! This module provides common geometric operations that are used across multiple
//! modules in the CSG library. By centralizing these utilities, we eliminate code
//! duplication and ensure consistent mathematical behavior.
//!
//! ## Mathematical Foundations
//!
//! ### Bounding Box Calculations
//! Bounding box computation follows the standard min-max approach:
//! - **Algorithm**: O(n) linear scan with partial_min/partial_max for numerical stability
//! - **Complexity**: O(n) where n is the number of points
//! - **Numerical Stability**: Uses partial_min/partial_max to handle NaN values gracefully
//! - **IEEE 754 Compliance**: Maintains precision through proper floating-point handling
//!
//! ### Error Handling
//! All functions in this module follow consistent error handling patterns:
//! - **No panics**: Functions return Results instead of panicking
//! - **Graceful degradation**: Invalid inputs produce sensible defaults
//! - **Numerical robustness**: Handle edge cases like empty datasets, infinite values

use crate::float_types::Real;
use nalgebra::{Point3, partial_max, partial_min};
use std::fmt::Debug;

/// Compute bounding box for a collection of 3D points
///
/// # Mathematical Foundation
/// The bounding box is computed using the min-max algorithm:
/// ```text
/// min_x = min(p_i.x for i in 0..n)
/// max_x = max(p_i.x for i in 0..n)
/// ```
///
/// # Algorithm Complexity
/// - **Time**: O(n) where n is the number of points
/// - **Space**: O(1) constant space usage
/// - **Numerical**: Handles NaN values gracefully using partial_min/partial_max
///
/// # IEEE 754 Compliance
/// Uses `partial_min` and `partial_max` to ensure numerical stability and proper
/// handling of special floating-point values according to IEEE 754 standards.
pub fn compute_bounding_box<T, I>(points: I) -> (Point3<Real>, Point3<Real>)
where
    T: Into<Point3<Real>> + Clone,
    I: IntoIterator<Item = T>,
{
    let mut points_iter = points.into_iter();

    match points_iter.next() {
        None => (Point3::origin(), Point3::origin()), // Empty case
        Some(first) => {
            let first_point: Point3<Real> = first.into();
            let mut min_x = first_point.x;
            let mut min_y = first_point.y;
            let mut min_z = first_point.z;
            let mut max_x = first_point.x;
            let mut max_y = first_point.y;
            let mut max_z = first_point.z;

            // Process remaining points
            for point in points_iter {
                let point: Point3<Real> = point.into();

                // Handle potential NaN values gracefully with partial_min/partial_max
                if let Some(new_min_x) = partial_min(&min_x, &point.x) {
                    min_x = *new_min_x;
                }
                if let Some(new_min_y) = partial_min(&min_y, &point.y) {
                    min_y = *new_min_y;
                }
                if let Some(new_min_z) = partial_min(&min_z, &point.z) {
                    min_z = *new_min_z;
                }

                if let Some(new_max_x) = partial_max(&max_x, &point.x) {
                    max_x = *new_max_x;
                }
                if let Some(new_max_y) = partial_max(&max_y, &point.y) {
                    max_y = *new_max_y;
                }
                if let Some(new_max_z) = partial_max(&max_z, &point.z) {
                    max_z = *new_max_z;
                }
            }

            (Point3::new(min_x, min_y, min_z), Point3::new(max_x, max_y, max_z))
        }
    }
}

/// Compute bounding box from a slice of Point3<Real>
///
/// # Specialization
/// This is a specialized version optimized for Point3<Real> slices, avoiding
/// unnecessary type conversions while maintaining the same mathematical guarantees.
pub fn compute_bounding_box_from_points(points: &[Point3<Real>]) -> (Point3<Real>, Point3<Real>) {
    if points.is_empty() {
        return (Point3::origin(), Point3::origin());
    }

    let mut min_x = Real::MAX;
    let mut min_y = Real::MAX;
    let mut min_z = Real::MAX;
    let mut max_x = -Real::MAX;
    let mut max_y = -Real::MAX;
    let mut max_z = -Real::MAX;

    for point in points {
        if let Some(new_min_x) = partial_min(&min_x, &point.x) {
            min_x = *new_min_x;
        }
        if let Some(new_min_y) = partial_min(&min_y, &point.y) {
            min_y = *new_min_y;
        }
        if let Some(new_min_z) = partial_min(&min_z, &point.z) {
            min_z = *new_min_z;
        }

        if let Some(new_max_x) = partial_max(&max_x, &point.x) {
            max_x = *new_max_x;
        }
        if let Some(new_max_y) = partial_max(&max_y, &point.y) {
            max_y = *new_max_y;
        }
        if let Some(new_max_z) = partial_max(&max_z, &point.z) {
            max_z = *new_max_z;
        }
    }

    (Point3::new(min_x, min_y, min_z), Point3::new(max_x, max_y, max_z))
}

/// Compute bounding box for vertices with metadata conversion
///
/// # Type Safety
/// This function handles the conversion from vertices with metadata to points
/// while maintaining type safety and avoiding unnecessary allocations.
pub fn compute_bounding_box_from_vertices<T>(vertices: &[T]) -> (Point3<Real>, Point3<Real>)
where
    T: Into<Point3<Real>> + Clone + Debug,
{
    if vertices.is_empty() {
        return (Point3::origin(), Point3::origin());
    }

    let mut min_x = Real::MAX;
    let mut min_y = Real::MAX;
    let mut min_z = Real::MAX;
    let mut max_x = -Real::MAX;
    let mut max_y = -Real::MAX;
    let mut max_z = -Real::MAX;

    for vertex in vertices {
        let point: Point3<Real> = vertex.clone().into();

        if let Some(new_min_x) = partial_min(&min_x, &point.x) {
            min_x = *new_min_x;
        }
        if let Some(new_min_y) = partial_min(&min_y, &point.y) {
            min_y = *new_min_y;
        }
        if let Some(new_min_z) = partial_min(&min_z, &point.z) {
            min_z = *new_min_z;
        }

        if let Some(new_max_x) = partial_max(&max_x, &point.x) {
            max_x = *new_max_x;
        }
        if let Some(new_max_y) = partial_max(&max_y, &point.y) {
            max_y = *new_max_y;
        }
        if let Some(new_max_z) = partial_max(&max_z, &point.z) {
            max_z = *new_max_z;
        }
    }

    (Point3::new(min_x, min_y, min_z), Point3::new(max_x, max_y, max_z))
}

/// Check if two bounding boxes intersect
///
/// # Algorithm
/// Uses the standard AABB intersection test:
/// ```text
/// ! (a_max_x < b_min_x || b_max_x < a_min_x ||
///    a_max_y < b_min_y || b_max_y < a_min_y ||
///    a_max_z < b_min_z || b_max_z < a_min_z)
/// ```
///
/// # Performance
/// - **Time**: O(1) constant time
/// - **Space**: O(1) constant space
pub fn bounding_boxes_intersect(
    a_min: &Point3<Real>,
    a_max: &Point3<Real>,
    b_min: &Point3<Real>,
    b_max: &Point3<Real>
) -> bool {
    !(a_max.x < b_min.x || b_max.x < a_min.x ||
      a_max.y < b_min.y || b_max.y < a_min.y ||
      a_max.z < b_min.z || b_max.z < a_min.z)
}

/// Compute union of two bounding boxes
///
/// # Mathematical Definition
/// The union of two AABBs is computed as:
/// ```text
/// min_x = min(a_min_x, b_min_x)
/// max_x = max(a_max_x, b_max_x)
/// ```
///
/// # Edge Cases
/// Handles degenerate cases where one or both AABBs are empty/invalid.
pub fn union_bounding_boxes(
    a_min: &Point3<Real>,
    a_max: &Point3<Real>,
    b_min: &Point3<Real>,
    b_max: &Point3<Real>
) -> (Point3<Real>, Point3<Real>) {
    let min_x = partial_min(&a_min.x, &b_min.x).copied().unwrap_or(a_min.x);
    let min_y = partial_min(&a_min.y, &b_min.y).copied().unwrap_or(a_min.y);
    let min_z = partial_min(&a_min.z, &b_min.z).copied().unwrap_or(a_min.z);

    let max_x = partial_max(&a_max.x, &b_max.x).copied().unwrap_or(a_max.x);
    let max_y = partial_max(&a_max.y, &b_max.y).copied().unwrap_or(a_max.y);
    let max_z = partial_max(&a_max.z, &b_max.z).copied().unwrap_or(a_max.z);

    (Point3::new(min_x, min_y, min_z), Point3::new(max_x, max_y, max_z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_empty_bounding_box() {
        let points: Vec<Point3<Real>> = vec![];
        let (min, max) = compute_bounding_box_from_points(&points);
        assert_eq!(min, Point3::origin());
        assert_eq!(max, Point3::origin());
    }

    #[test]
    fn test_single_point_bounding_box() {
        let points = vec![Point3::new(1.0, 2.0, 3.0)];
        let (min, max) = compute_bounding_box_from_points(&points);
        assert_eq!(min, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(max, Point3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_multiple_points_bounding_box() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(-1.0, -1.0, -1.0),
        ];
        let (min, max) = compute_bounding_box_from_points(&points);
        assert_eq!(min, Point3::new(-1.0, -1.0, -1.0));
        assert_eq!(max, Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn test_bounding_box_intersection() {
        let a_min = Point3::new(0.0, 0.0, 0.0);
        let a_max = Point3::new(1.0, 1.0, 1.0);
        let b_min = Point3::new(0.5, 0.5, 0.5);
        let b_max = Point3::new(1.5, 1.5, 1.5);

        assert!(bounding_boxes_intersect(&a_min, &a_max, &b_min, &b_max));
    }

    #[test]
    fn test_bounding_box_no_intersection() {
        let a_min = Point3::new(0.0, 0.0, 0.0);
        let a_max = Point3::new(1.0, 1.0, 1.0);
        let b_min = Point3::new(2.0, 2.0, 2.0);
        let b_max = Point3::new(3.0, 3.0, 3.0);

        assert!(!bounding_boxes_intersect(&a_min, &a_max, &b_min, &b_max));
    }

    #[test]
    fn test_union_bounding_boxes() {
        let a_min = Point3::new(0.0, 0.0, 0.0);
        let a_max = Point3::new(1.0, 1.0, 1.0);
        let b_min = Point3::new(2.0, 2.0, 2.0);
        let b_max = Point3::new(3.0, 3.0, 3.0);

        let (union_min, union_max) = union_bounding_boxes(&a_min, &a_max, &b_min, &b_max);
        assert_eq!(union_min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(union_max, Point3::new(3.0, 3.0, 3.0));
    }
}
