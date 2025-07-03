//! **Spatial Geometry Types (The Skeleton)**
//!
//! This module defines core geometric types used throughout spatial data structures,
//! following Cathedral Engineering principles where geometric types represent the
//! "skeleton" that provides structural foundation for spatial operations.

use crate::core::float_types::Real;
use crate::geometry::Polygon;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// **Axis-Aligned Bounding Box (AABB)**
///
/// A fundamental geometric primitive for spatial data structures, representing
/// a rectangular box aligned with the coordinate axes.
///
/// ## **Mathematical Definition**
/// An AABB is defined by two points: minimum corner (min_x, min_y, min_z) and
/// maximum corner (max_x, max_y, max_z), such that all points within the box
/// satisfy: min ≤ point ≤ max (component-wise).
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::traits::geometry::Aabb;
/// use nalgebra::Point3;
///
/// let aabb = Aabb::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 1.0, 1.0)
/// );
/// 
/// assert!(aabb.contains_point(&Point3::new(0.5, 0.5, 0.5)));
/// assert!(!aabb.contains_point(&Point3::new(1.5, 0.5, 0.5)));
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Aabb {
    pub min: Point3<Real>,
    pub max: Point3<Real>,
}

impl Aabb {
    /// **Create a new AABB from min and max points**
    pub fn new(min: Point3<Real>, max: Point3<Real>) -> Self {
        Self { min, max }
    }

    /// **Check if this AABB contains a point**
    pub fn contains_point(&self, point: &Point3<Real>) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y &&
        point.z >= self.min.z && point.z <= self.max.z
    }

    /// **Check if this AABB intersects with another AABB**
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.max.x >= other.min.x && self.min.x <= other.max.x &&
        self.max.y >= other.min.y && self.min.y <= other.max.y &&
        self.max.z >= other.min.z && self.min.z <= other.max.z
    }

    /// **Compute the volume of this AABB**
    pub fn volume(&self) -> Real {
        let extents = self.max - self.min;
        extents.x * extents.y * extents.z
    }

    /// **Compute the center point of this AABB**
    pub fn center(&self) -> Point3<Real> {
        Point3::from((self.min.coords + self.max.coords) * 0.5)
    }

    /// **Expand this AABB to include another AABB**
    pub fn expand_to_include(&mut self, other: &Aabb) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.min.z = self.min.z.min(other.min.z);
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
        self.max.z = self.max.z.max(other.max.z);
    }

    /// **Get the size (extents) of this AABB**
    pub fn size(&self) -> Vector3<Real> {
        self.max - self.min
    }

    /// **Create AABB from a collection of polygons**
    pub fn from_polygons<S: Clone>(polygons: &[Polygon<S>]) -> Option<Aabb> {
        if polygons.is_empty() {
            return None;
        }

        let mut min = Point3::new(Real::INFINITY, Real::INFINITY, Real::INFINITY);
        let mut max = Point3::new(Real::NEG_INFINITY, Real::NEG_INFINITY, Real::NEG_INFINITY);

        for polygon in polygons {
            for vertex in &polygon.vertices {
                min.x = min.x.min(vertex.pos.x);
                min.y = min.y.min(vertex.pos.y);
                min.z = min.z.min(vertex.pos.z);
                max.x = max.x.max(vertex.pos.x);
                max.y = max.y.max(vertex.pos.y);
                max.z = max.z.max(vertex.pos.z);
            }
        }

        Some(Aabb::new(min, max))
    }
}

/// **Ray for spatial queries and intersection testing**
///
/// A ray is defined by an origin point and a direction vector, representing
/// a half-line extending infinitely in the direction from the origin.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::traits::geometry::Ray;
/// use nalgebra::{Point3, Vector3};
///
/// let ray = Ray::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Vector3::new(1.0, 0.0, 0.0)
/// );
/// 
/// let point_on_ray = ray.point_at(2.0);
/// assert_eq!(point_on_ray, Point3::new(2.0, 0.0, 0.0));
/// ```
#[derive(Debug, Clone)]
pub struct Ray {
    pub origin: Point3<Real>,
    pub direction: Vector3<Real>,
}

impl Ray {
    /// **Create a new ray**
    ///
    /// The direction vector will be automatically normalized.
    pub fn new(origin: Point3<Real>, direction: Vector3<Real>) -> Self {
        Self { 
            origin, 
            direction: direction.normalize() 
        }
    }

    /// **Get a point along the ray at parameter t**
    ///
    /// Returns: origin + t * direction
    pub fn point_at(&self, t: Real) -> Point3<Real> {
        self.origin + self.direction * t
    }

    /// **Test intersection with an AABB using the slab method**
    pub fn intersects_aabb(&self, aabb: &Aabb) -> bool {
        let inv_dir = Vector3::new(
            1.0 / self.direction.x,
            1.0 / self.direction.y,
            1.0 / self.direction.z,
        );

        let t1 = (aabb.min.x - self.origin.x) * inv_dir.x;
        let t2 = (aabb.max.x - self.origin.x) * inv_dir.x;
        let t3 = (aabb.min.y - self.origin.y) * inv_dir.y;
        let t4 = (aabb.max.y - self.origin.y) * inv_dir.y;
        let t5 = (aabb.min.z - self.origin.z) * inv_dir.z;
        let t6 = (aabb.max.z - self.origin.z) * inv_dir.z;

        let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
        let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

        tmax >= 0.0 && tmin <= tmax
    }
}

/// **Intersection result for ray-geometry queries**
///
/// Contains detailed information about a ray-geometry intersection,
/// including the intersection point, surface normal, and distance.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::traits::geometry::Intersection;
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
/// let intersection = Intersection {
///     distance: 1.0,
///     point: Point3::new(0.5, 0.5, 0.0),
///     normal: Vector3::new(0.0, 0.0, 1.0),
///     polygon,
/// };
/// ```
#[derive(Debug, Clone)]
pub struct Intersection<S: Clone> {
    pub distance: Real,
    pub point: Point3<Real>,
    pub normal: Vector3<Real>,
    pub polygon: Polygon<S>,
}

impl<S: Clone> Intersection<S> {
    /// **Create a new intersection record**
    pub fn new(
        distance: Real,
        point: Point3<Real>,
        normal: Vector3<Real>,
        polygon: Polygon<S>,
    ) -> Self {
        Self {
            distance,
            point,
            normal,
            polygon,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_creation() {
        let aabb = Aabb::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0)
        );
        
        assert_eq!(aabb.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(aabb.max, Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn test_aabb_contains_point() {
        let aabb = Aabb::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0)
        );
        
        assert!(aabb.contains_point(&Point3::new(0.5, 0.5, 0.5)));
        assert!(aabb.contains_point(&Point3::new(0.0, 0.0, 0.0))); // boundary
        assert!(aabb.contains_point(&Point3::new(1.0, 1.0, 1.0))); // boundary
        assert!(!aabb.contains_point(&Point3::new(1.5, 0.5, 0.5)));
    }

    #[test]
    fn test_aabb_intersects() {
        let aabb1 = Aabb::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0)
        );
        let aabb2 = Aabb::new(
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(1.5, 1.5, 1.5)
        );
        let aabb3 = Aabb::new(
            Point3::new(2.0, 2.0, 2.0),
            Point3::new(3.0, 3.0, 3.0)
        );
        
        assert!(aabb1.intersects(&aabb2));
        assert!(!aabb1.intersects(&aabb3));
    }

    #[test]
    fn test_ray_creation() {
        let ray = Ray::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0)
        );
        
        assert_eq!(ray.origin, Point3::new(0.0, 0.0, 0.0));
        assert!((ray.direction.norm() - 1.0).abs() < 1e-10); // Should be normalized
    }

    #[test]
    fn test_ray_point_at() {
        let ray = Ray::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0)
        );
        
        let point = ray.point_at(2.0);
        assert_eq!(point, Point3::new(2.0, 0.0, 0.0));
    }
}
