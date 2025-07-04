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
//! let union = utils::bounds_union(&[bounds1.clone(), bounds2.clone()]);
//! assert_eq!(union.min, Point3::new(0.0, 0.0, 0.0));
//! assert_eq!(union.max, Point3::new(1.5, 1.5, 1.5));
//!
//! // Calculate intersection
//! let intersection = utils::bounds_intersection(&bounds1, &bounds2);
//! assert!(intersection.is_some());
//! ```

use crate::geometry::Polygon;
use crate::core::float_types::Real;
use crate::spatial::traits::{Aabb, Ray};
use nalgebra::{Point3, Vector3};

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
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
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

    // Use fold() for efficient bounding box computation
    let (min, max) = polygon.vertices[1..]
        .iter()
        .map(|vertex| vertex.pos)
        .fold((first_pos, first_pos), |(mut min, mut max), pos| {
            min.x = min.x.min(pos.x);
            min.y = min.y.min(pos.y);
            min.z = min.z.min(pos.z);
            max.x = max.x.max(pos.x);
            max.y = max.y.max(pos.y);
            max.z = max.z.max(pos.z);
            (min, max)
        });

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

    let n = polygon.vertices.len();

    // Use enumerate() and fold() for efficient area calculation
    let area = (0..n)
        .map(|i| {
            let j = (i + 1) % n;
            let vi = polygon.vertices[i].pos;
            let vj = polygon.vertices[j].pos;

            // Cross product contribution to area
            vi.x * vj.y - vj.x * vi.y
        })
        .fold(0.0, |acc, contribution| acc + contribution);

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
    // Check if any vertex is inside the bounds using any() for early termination
    if polygon.vertices
        .iter()
        .any(|vertex| bounds.contains_point(&vertex.pos))
    {
        return true;
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
/// let plane = Plane::from_normal(Vector3::new(0.0, 0.0, 1.0), 0.0); // XY plane at z=0
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

/// Calculate the intersection of two axis-aligned bounding boxes
///
/// Returns the overlapping region if the boxes intersect, None otherwise.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::bounds_intersection, traits::Aabb};
/// use nalgebra::Point3;
///
/// let bounds1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
/// let bounds2 = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
///
/// let intersection = bounds_intersection(&bounds1, &bounds2).unwrap();
/// assert_eq!(intersection.min, Point3::new(1.0, 1.0, 1.0));
/// assert_eq!(intersection.max, Point3::new(2.0, 2.0, 2.0));
/// ```
#[inline]
pub fn bounds_intersection(a: &Aabb, b: &Aabb) -> Option<Aabb> {
    let min = Point3::new(
        a.min.x.max(b.min.x),
        a.min.y.max(b.min.y),
        a.min.z.max(b.min.z),
    );
    let max = Point3::new(
        a.max.x.min(b.max.x),
        a.max.y.min(b.max.y),
        a.max.z.min(b.max.z),
    );

    if min.x <= max.x && min.y <= max.y && min.z <= max.z {
        Some(Aabb::new(min, max))
    } else {
        None
    }
}

/// Calculate the union of multiple axis-aligned bounding boxes
///
/// Returns the smallest AABB that contains all input boxes.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::bounds_union, traits::Aabb};
/// use nalgebra::Point3;
///
/// let bounds = vec![
///     Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)),
///     Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0)),
///     Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(0.5, 0.5, 0.5)),
/// ];
///
/// let union = bounds_union(&bounds);
/// assert_eq!(union.min, Point3::new(-1.0, -1.0, -1.0));
/// assert_eq!(union.max, Point3::new(3.0, 3.0, 3.0));
/// ```
#[inline]
pub fn bounds_union(bounds: &[Aabb]) -> Aabb {
    if bounds.is_empty() {
        return Aabb::new(Point3::origin(), Point3::origin());
    }

    // Use fold() for efficient bounds merging with parallel processing for large datasets
    #[cfg(feature = "parallel")]
    let (min, max) = {
        if bounds.len() > 1000 {
            use rayon::prelude::*;

            bounds[1..]
                .par_iter()
                .fold(
                    || (bounds[0].min, bounds[0].max),
                    |(mut min, mut max), bound| {
                        min.x = min.x.min(bound.min.x);
                        min.y = min.y.min(bound.min.y);
                        min.z = min.z.min(bound.min.z);
                        max.x = max.x.max(bound.max.x);
                        max.y = max.y.max(bound.max.y);
                        max.z = max.z.max(bound.max.z);
                        (min, max)
                    }
                )
                .reduce(
                    || (bounds[0].min, bounds[0].max),
                    |(mut min1, mut max1), (min2, max2)| {
                        min1.x = min1.x.min(min2.x);
                        min1.y = min1.y.min(min2.y);
                        min1.z = min1.z.min(min2.z);
                        max1.x = max1.x.max(max2.x);
                        max1.y = max1.y.max(max2.y);
                        max1.z = max1.z.max(max2.z);
                        (min1, max1)
                    }
                )
        } else {
            bounds[1..]
                .iter()
                .fold((bounds[0].min, bounds[0].max), |(mut min, mut max), bound| {
                    min.x = min.x.min(bound.min.x);
                    min.y = min.y.min(bound.min.y);
                    min.z = min.z.min(bound.min.z);
                    max.x = max.x.max(bound.max.x);
                    max.y = max.y.max(bound.max.y);
                    max.z = max.z.max(bound.max.z);
                    (min, max)
                })
        }
    };

    #[cfg(not(feature = "parallel"))]
    let (min, max) = bounds[1..]
        .iter()
        .fold((bounds[0].min, bounds[0].max), |(mut min, mut max), bound| {
            min.x = min.x.min(bound.min.x);
            min.y = min.y.min(bound.min.y);
            min.z = min.z.min(bound.min.z);
            max.x = max.x.max(bound.max.x);
            max.y = max.y.max(bound.max.y);
            max.z = max.z.max(bound.max.z);
            (min, max)
        });

    Aabb::new(min, max)
}

/// Merge two axis-aligned bounding boxes
///
/// Returns the smallest AABB that contains both input boxes.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::merge_bounds, traits::Aabb};
/// use nalgebra::Point3;
///
/// let bounds1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
/// let bounds2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
///
/// let merged = merge_bounds(&bounds1, &bounds2);
/// assert_eq!(merged.min, Point3::new(0.0, 0.0, 0.0));
/// assert_eq!(merged.max, Point3::new(1.5, 1.5, 1.5));
/// ```
#[inline]
pub fn merge_bounds(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb::new(
        Point3::new(
            a.min.x.min(b.min.x),
            a.min.y.min(b.min.y),
            a.min.z.min(b.min.z),
        ),
        Point3::new(
            a.max.x.max(b.max.x),
            a.max.y.max(b.max.y),
            a.max.z.max(b.max.z),
        ),
    )
}

/// Test if an axis-aligned bounding box contains a point
///
/// Returns true if the point is inside or on the boundary of the box.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::bounds_contains_point, traits::Aabb};
/// use nalgebra::Point3;
///
/// let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
///
/// assert!(bounds_contains_point(&bounds, &Point3::new(1.0, 1.0, 1.0)));
/// assert!(bounds_contains_point(&bounds, &Point3::new(0.0, 0.0, 0.0))); // On boundary
/// assert!(!bounds_contains_point(&bounds, &Point3::new(3.0, 3.0, 3.0)));
/// ```
#[inline]
pub fn bounds_contains_point(bounds: &Aabb, point: &Point3<Real>) -> bool {
    point.x >= bounds.min.x && point.x <= bounds.max.x &&
    point.y >= bounds.min.y && point.y <= bounds.max.y &&
    point.z >= bounds.min.z && point.z <= bounds.max.z
}

/// Calculate the volume of an axis-aligned bounding box
///
/// Returns the 3D volume (width × height × depth) of the box.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::bounds_volume, traits::Aabb};
/// use nalgebra::Point3;
///
/// let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
/// let volume = bounds_volume(&bounds);
/// assert_eq!(volume, 24.0); // 2 × 3 × 4 = 24
/// ```
#[inline]
pub fn bounds_volume(bounds: &Aabb) -> Real {
    let size = bounds.size();
    size.x * size.y * size.z
}

/// Test if a ray intersects with an axis-aligned bounding box
///
/// Uses the slab method for efficient ray-AABB intersection testing.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::ray_intersects_aabb, traits::{Aabb, Ray}};
/// use nalgebra::{Point3, Vector3};
///
/// let ray = Ray {
///     origin: Point3::new(0.0, 0.0, 0.0),
///     direction: Vector3::new(1.0, 0.0, 0.0),
/// };
/// let bounds = Aabb::new(Point3::new(0.5, -0.5, -0.5), Point3::new(1.5, 0.5, 0.5));
///
/// assert!(ray_intersects_aabb(&ray, &bounds));
/// ```
#[inline]
pub fn ray_intersects_aabb(ray: &Ray, aabb: &Aabb) -> bool {
    let inv_dir = Vector3::new(
        1.0 / ray.direction.x,
        1.0 / ray.direction.y,
        1.0 / ray.direction.z,
    );

    let t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
    let t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;
    let t3 = (aabb.min.y - ray.origin.y) * inv_dir.y;
    let t4 = (aabb.max.y - ray.origin.y) * inv_dir.y;
    let t5 = (aabb.min.z - ray.origin.z) * inv_dir.z;
    let t6 = (aabb.max.z - ray.origin.z) * inv_dir.z;

    let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
    let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

    tmax >= 0.0 && tmin <= tmax
}

/// Calculate the distance along a ray to the intersection with an AABB
///
/// Returns the parameter t such that intersection_point = ray.origin + t * ray.direction.
/// Returns None if there is no intersection.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{utils::ray_aabb_intersection_distance, traits::{Aabb, Ray}};
/// use nalgebra::{Point3, Vector3};
///
/// let ray = Ray {
///     origin: Point3::new(0.0, 0.0, 0.0),
///     direction: Vector3::new(1.0, 0.0, 0.0),
/// };
/// let bounds = Aabb::new(Point3::new(1.0, -0.5, -0.5), Point3::new(2.0, 0.5, 0.5));
///
/// let distance = ray_aabb_intersection_distance(&ray, &bounds);
/// assert!(distance.is_some());
/// assert!((distance.unwrap() - 1.0).abs() < 0.001);
/// ```
#[inline]
pub fn ray_aabb_intersection_distance(ray: &Ray, aabb: &Aabb) -> Option<Real> {
    let inv_dir = Vector3::new(
        1.0 / ray.direction.x,
        1.0 / ray.direction.y,
        1.0 / ray.direction.z,
    );

    let t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
    let t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;
    let t3 = (aabb.min.y - ray.origin.y) * inv_dir.y;
    let t4 = (aabb.max.y - ray.origin.y) * inv_dir.y;
    let t5 = (aabb.min.z - ray.origin.z) * inv_dir.z;
    let t6 = (aabb.max.z - ray.origin.z) * inv_dir.z;

    let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
    let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

    if tmax >= 0.0 && tmin <= tmax {
        Some(if tmin >= 0.0 { tmin } else { tmax })
    } else {
        None
    }
}

/// Calculate the squared distance between two points
///
/// This is more efficient than calculating the actual distance when only
/// relative distances are needed (e.g., for nearest neighbor searches).
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::distance_squared;
/// use nalgebra::Point3;
///
/// let p1 = Point3::new(0.0, 0.0, 0.0);
/// let p2 = Point3::new(3.0, 4.0, 0.0);
///
/// let dist_sq = distance_squared(&p1, &p2);
/// assert_eq!(dist_sq, 25.0); // 3² + 4² = 9 + 16 = 25
/// ```
#[inline]
pub fn distance_squared(a: &Point3<Real>, b: &Point3<Real>) -> Real {
    let diff = b - a;
    diff.norm_squared()
}

/// Calculate the actual distance between two points
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::utils::distance;
/// use nalgebra::Point3;
///
/// let p1 = Point3::new(0.0, 0.0, 0.0);
/// let p2 = Point3::new(3.0, 4.0, 0.0);
///
/// let dist = distance(&p1, &p2);
/// assert_eq!(dist, 5.0); // sqrt(3² + 4²) = sqrt(25) = 5
/// ```
#[inline]
pub fn distance(a: &Point3<Real>, b: &Point3<Real>) -> Real {
    (b - a).norm()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Polygon, Vertex};
    use nalgebra::{Point3, Vector3};

    fn create_test_polygon() -> Polygon<i32> {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        Polygon::new(vertices, Some(1))
    }

    #[test]
    fn test_polygon_center() {
        let polygon = create_test_polygon();
        let center = polygon_center(&polygon);
        assert_eq!(center, Point3::new(1.0, 2.0/3.0, 0.0));
    }

    #[test]
    fn test_polygon_bounds() {
        let polygon = create_test_polygon();
        let bounds = polygon_bounds(&polygon).unwrap();
        assert_eq!(bounds.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(bounds.max, Point3::new(2.0, 2.0, 0.0));

        // Test single vertex polygon (minimum valid polygon)
        let single_vertex = vec![
            crate::geometry::Vertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
            crate::geometry::Vertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
            crate::geometry::Vertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let single_point_polygon: Polygon<i32> = Polygon::new(single_vertex, None);
        let single_bounds = polygon_bounds(&single_point_polygon).unwrap();
        assert_eq!(single_bounds.min, Point3::new(1.0, 1.0, 1.0));
        assert_eq!(single_bounds.max, Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn test_polygon_area() {
        // Create a unit square
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let square: Polygon<i32> = Polygon::new(vertices, None);
        let area = polygon_area(&square);
        assert!((area - 1.0).abs() < 0.001);

        // Test triangle
        let triangle = create_test_polygon();
        let triangle_area = polygon_area(&triangle);
        assert!(triangle_area > 0.0);
    }

    #[test]
    fn test_polygon_intersects_bounds() {
        let polygon = create_test_polygon();

        // Intersecting bounds
        let intersecting_bounds = Aabb::new(Point3::new(0.5, 0.5, -0.5), Point3::new(1.5, 1.5, 0.5));
        assert!(polygon_intersects_bounds(&polygon, &intersecting_bounds));

        // Non-intersecting bounds
        let non_intersecting_bounds = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));
        assert!(!polygon_intersects_bounds(&polygon, &non_intersecting_bounds));
    }

    #[test]
    fn test_point_in_polygon() {
        let polygon = create_test_polygon();
        let point = Point3::new(1.0, 1.0, 0.0);

        // Currently always returns false (placeholder implementation)
        assert!(!point_in_polygon(&point, &polygon));
    }

    #[test]
    fn test_bounds_intersection() {
        let bounds1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let bounds2 = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));

        let intersection = bounds_intersection(&bounds1, &bounds2).unwrap();
        assert_eq!(intersection.min, Point3::new(1.0, 1.0, 1.0));
        assert_eq!(intersection.max, Point3::new(2.0, 2.0, 2.0));

        // Non-intersecting bounds
        let bounds3 = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));
        assert!(bounds_intersection(&bounds1, &bounds3).is_none());
    }

    #[test]
    fn test_bounds_union() {
        let bounds = vec![
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0)),
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(0.5, 0.5, 0.5)),
        ];

        let union = bounds_union(&bounds);
        assert_eq!(union.min, Point3::new(-1.0, -1.0, -1.0));
        assert_eq!(union.max, Point3::new(3.0, 3.0, 3.0));

        // Test empty bounds
        let empty_union = bounds_union(&[]);
        assert_eq!(empty_union.min, Point3::origin());
        assert_eq!(empty_union.max, Point3::origin());
    }

    #[test]
    fn test_merge_bounds() {
        let bounds1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let bounds2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));

        let merged = merge_bounds(&bounds1, &bounds2);
        assert_eq!(merged.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(merged.max, Point3::new(1.5, 1.5, 1.5));
    }

    #[test]
    fn test_bounds_contains_point() {
        let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));

        assert!(bounds_contains_point(&bounds, &Point3::new(1.0, 1.0, 1.0)));
        assert!(bounds_contains_point(&bounds, &Point3::new(0.0, 0.0, 0.0))); // On boundary
        assert!(bounds_contains_point(&bounds, &Point3::new(2.0, 2.0, 2.0))); // On boundary
        assert!(!bounds_contains_point(&bounds, &Point3::new(3.0, 3.0, 3.0)));
        assert!(!bounds_contains_point(&bounds, &Point3::new(-1.0, 1.0, 1.0)));
    }

    #[test]
    fn test_bounds_volume() {
        let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 4.0));
        let volume = bounds_volume(&bounds);
        assert_eq!(volume, 24.0); // 2 × 3 × 4 = 24

        // Test unit cube
        let unit_cube = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert_eq!(bounds_volume(&unit_cube), 1.0);
    }

    #[test]
    fn test_ray_intersects_aabb() {
        let ray = Ray {
            origin: Point3::new(0.0, 0.0, 0.0),
            direction: Vector3::new(1.0, 0.0, 0.0),
        };
        let bounds = Aabb::new(Point3::new(0.5, -0.5, -0.5), Point3::new(1.5, 0.5, 0.5));

        assert!(ray_intersects_aabb(&ray, &bounds));

        // Non-intersecting ray
        let ray2 = Ray {
            origin: Point3::new(0.0, 0.0, 0.0),
            direction: Vector3::new(0.0, 1.0, 0.0),
        };
        let bounds2 = Aabb::new(Point3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0));
        assert!(!ray_intersects_aabb(&ray2, &bounds2));
    }

    #[test]
    fn test_distance_functions() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(3.0, 4.0, 0.0);

        let dist_sq = distance_squared(&p1, &p2);
        assert_eq!(dist_sq, 25.0); // 3² + 4² = 25

        let dist = distance(&p1, &p2);
        assert_eq!(dist, 5.0); // sqrt(25) = 5
    }
}
