//! Shared geometric utilities for spatial data structures
//!
//! This module provides common geometric operations used across BSP trees, KD-trees,
//! and Octrees to eliminate code duplication and ensure consistent behavior.
//!
//! # Examples
//!
//! ## Polygon Operations
//!
//! ```rust
//! use csgrs::spatial::utils;
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create a test polygon
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//!
//! // Calculate polygon center
//! let center = utils::polygon_center(&polygon);
//! assert_eq!(center, Point3::new(0.5, 1.0/3.0, 0.0));
//!
//! // Calculate polygon bounds
//! let bounds = utils::polygon_bounds(&polygon);
//! assert!(bounds.is_some());
//! ```
//!
//! ## Bounds Operations
//!
//! ```rust
//! use csgrs::spatial::{utils, traits::Aabb};
//! use nalgebra::Point3;
//!
//! let bounds1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
//! let bounds2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
//!
//! // Calculate union of bounds
//! let union = utils::bounds_union(&[bounds1, bounds2]);
//! assert_eq!(union.min, Point3::new(0.0, 0.0, 0.0));
//! assert_eq!(union.max, Point3::new(1.5, 1.5, 1.5));
//!
//! // Calculate intersection
//! let intersection = utils::bounds_intersection(&bounds1, &bounds2);
//! assert!(intersection.is_some());
//! ```

use crate::geometry::{Polygon, Vertex};
use crate::core::float_types::Real;
use crate::spatial::traits::{Aabb, Ray};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Calculate the centroid (center point) of a polygon's vertices
///
/// This function computes the arithmetic mean of all vertex positions,
/// providing a simple but effective center point for spatial operations.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::polygon_center;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
///
/// let center = polygon_center(&polygon);
/// assert_eq!(center, Point3::new(1.0, 2.0/3.0, 0.0));
/// ```
#[inline]
pub fn polygon_center<S: Clone>(polygon: &Polygon<S>) -> Point3<Real> {
    let sum = polygon.vertices.iter()
        .fold(Point3::origin(), |acc, vertex| acc + vertex.pos.coords);
    Point3::from(sum.coords / polygon.vertices.len() as Real)
}

/// Calculate the axis-aligned bounding box for a polygon
///
/// Returns the smallest AABB that contains all vertices of the polygon.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::polygon_bounds;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(-1.0, -1.0, -1.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(2.0, 3.0, 4.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
///
/// let bounds = polygon_bounds(&polygon).unwrap();
/// assert_eq!(bounds.min, Point3::new(-1.0, -1.0, -1.0));
/// assert_eq!(bounds.max, Point3::new(2.0, 3.0, 4.0));
/// ```
#[inline]
pub fn polygon_bounds<S: Clone>(polygon: &Polygon<S>) -> Option<Aabb> {
    if polygon.vertices.is_empty() {
        return None;
    }

    let first_pos = polygon.vertices[0].pos;
    let mut min = first_pos;
    let mut max = first_pos;

    for vertex in &polygon.vertices[1..] {
        let pos = vertex.pos;
        min.x = min.x.min(pos.x);
        min.y = min.y.min(pos.y);
        min.z = min.z.min(pos.z);
        max.x = max.x.max(pos.x);
        max.y = max.y.max(pos.y);
        max.z = max.z.max(pos.z);
    }

    Some(Aabb::new(min, max))
}

/// Calculate the surface area of a polygon (simplified 3D implementation)
///
/// This uses a simplified calculation based on the cross product of edges.
/// For complex polygons, this may not be accurate.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::polygon_area;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create a unit square in the XY plane
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
///
/// let area = polygon_area(&polygon);
/// assert!((area - 1.0).abs() < 0.001); // Should be approximately 1.0
/// ```
#[inline]
pub fn polygon_area<S: Clone>(polygon: &Polygon<S>) -> Real {
    if polygon.vertices.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    let n = polygon.vertices.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let vi = polygon.vertices[i].pos;
        let vj = polygon.vertices[j].pos;
        
        // Cross product contribution to area
        area += vi.x * vj.y - vj.x * vi.y;
    }

    (area * 0.5).abs()
}

/// Test if a polygon intersects with an axis-aligned bounding box
///
/// This performs a conservative intersection test by checking if any vertex
/// is inside the bounds or if the polygon's bounding box intersects the query bounds.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::polygon_intersects_bounds, traits::Aabb};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(0.5, 0.5, 0.5), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.5, 0.5, 0.5), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 1.5, 0.5), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
///
/// let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
/// assert!(polygon_intersects_bounds(&polygon, &bounds));
/// ```
#[inline]
pub fn polygon_intersects_bounds<S: Clone>(polygon: &Polygon<S>, bounds: &Aabb) -> bool {
    // Check if any vertex is inside the bounds
    for vertex in &polygon.vertices {
        if bounds.contains_point(&vertex.pos) {
            return true;
        }
    }

    // Check if polygon bounding box intersects query bounds
    if let Some(poly_bounds) = polygon_bounds(polygon) {
        return poly_bounds.intersects(bounds);
    }

    false
}

/// Test if a point is inside a polygon (simplified 3D implementation)
///
/// This is a placeholder implementation that always returns false.
/// A proper 3D point-in-polygon test is complex and depends on polygon orientation.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::point_in_polygon;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
///
/// let point = Point3::new(0.5, 0.3, 0.0);
/// let inside = point_in_polygon(&point, &polygon);
/// // Currently always returns false (placeholder implementation)
/// assert!(!inside);
/// ```
#[inline]
pub fn point_in_polygon<S: Clone>(_point: &Point3<Real>, _polygon: &Polygon<S>) -> bool {
    // TODO: Implement proper 3D point-in-polygon test
    // This is complex and depends on polygon orientation and normal vectors
    // For now, return false as a placeholder
    false
}

/// Calculate the signed distance from a point to a plane
///
/// Returns positive distance if the point is on the front side of the plane,
/// negative if on the back side.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::point_to_plane_distance;
/// use csgrs::geometry::Plane;
/// use nalgebra::{Point3, Vector3};
///
/// let plane = Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.0); // XY plane at z=0
/// let point = Point3::new(1.0, 1.0, 2.0);
///
/// let distance = point_to_plane_distance(&point, &plane);
/// assert!((distance - 2.0).abs() < 0.001);
/// ```
#[inline]
pub fn point_to_plane_distance(point: &Point3<Real>, plane: &crate::geometry::Plane) -> Real {
    let normal = plane.normal();
    let offset = plane.offset();
    normal.dot(&point.coords) - offset
}

/// Find the closest point on a polygon to a query point (simplified implementation)
///
/// This implementation returns the polygon center as an approximation.
/// A proper implementation would project the point onto the polygon surface.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::closest_point_on_polygon;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
///
/// let query_point = Point3::new(5.0, 5.0, 5.0);
/// let closest = closest_point_on_polygon(&query_point, &polygon);
/// // Returns polygon center as approximation
/// assert_eq!(closest, Point3::new(1.0, 2.0/3.0, 0.0));
/// ```
#[inline]
pub fn closest_point_on_polygon<S: Clone>(_query_point: &Point3<Real>, polygon: &Polygon<S>) -> Point3<Real> {
    // TODO: Implement proper closest point projection
    // For now, return polygon center as approximation
    polygon_center(polygon)
}
