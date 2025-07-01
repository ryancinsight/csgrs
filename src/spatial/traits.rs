//! Common traits and interfaces for spatial data structures
//!
//! This module defines the unified trait system that enables polymorphic usage
//! of different spatial data structures (BSP, KD-tree, Octree) while maintaining
//! performance and type safety.

use crate::geometry::{Polygon, Vertex};
use crate::core::float_types::Real;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Axis-aligned bounding box for spatial queries
#[derive(Debug, Clone, PartialEq)]
pub struct Aabb {
    pub min: Point3<Real>,
    pub max: Point3<Real>,
}

impl Aabb {
    /// Create a new AABB from min and max points
    pub fn new(min: Point3<Real>, max: Point3<Real>) -> Self {
        Self { min, max }
    }

    /// Create an AABB that encompasses all given points
    pub fn from_points(points: &[Point3<Real>]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }

        let mut min = points[0];
        let mut max = points[0];

        for point in points.iter().skip(1) {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }

        Some(Self { min, max })
    }

    /// Create an AABB from a collection of polygons
    pub fn from_polygons<S: Clone>(polygons: &[Polygon<S>]) -> Option<Self> {
        let points: Vec<Point3<Real>> = polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();
        Self::from_points(&points)
    }

    /// Check if this AABB contains a point
    pub fn contains_point(&self, point: &Point3<Real>) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y &&
        point.z >= self.min.z && point.z <= self.max.z
    }

    /// Check if this AABB intersects with another AABB
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
        self.min.y <= other.max.y && self.max.y >= other.min.y &&
        self.min.z <= other.max.z && self.max.z >= other.min.z
    }

    /// Get the center point of the AABB
    pub fn center(&self) -> Point3<Real> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the size (dimensions) of the AABB
    pub fn size(&self) -> Vector3<Real> {
        self.max - self.min
    }

    /// Get the volume of the AABB
    pub fn volume(&self) -> Real {
        let size = self.size();
        size.x * size.y * size.z
    }
}

/// Ray for spatial intersection queries
#[derive(Debug, Clone)]
pub struct Ray {
    pub origin: Point3<Real>,
    pub direction: Vector3<Real>,
}

impl Ray {
    /// Create a new ray
    pub fn new(origin: Point3<Real>, direction: Vector3<Real>) -> Self {
        Self { origin, direction: direction.normalize() }
    }

    /// Get a point along the ray at parameter t
    pub fn point_at(&self, t: Real) -> Point3<Real> {
        self.origin + self.direction * t
    }
}

/// Result of a ray-geometry intersection
#[derive(Debug, Clone)]
pub struct Intersection<S: Clone> {
    pub distance: Real,
    pub point: Point3<Real>,
    pub normal: Vector3<Real>,
    pub polygon: Polygon<S>,
}

/// Common interface for all spatial data structures
pub trait SpatialIndex<S: Clone + Send + Sync + Debug> {
    /// Build the spatial structure from a collection of polygons
    fn build(polygons: &[Polygon<S>]) -> Self;

    /// Create an empty spatial structure
    fn new() -> Self;

    /// Get all polygons stored in the structure
    fn all_polygons(&self) -> Vec<Polygon<S>>;

    /// Find all polygons within a given bounding box
    fn query_range(&self, bounds: &Aabb) -> Vec<&Polygon<S>>;

    /// Find the nearest polygon to a given point
    fn nearest_neighbor(&self, point: &Point3<Real>) -> Option<&Polygon<S>>;

    /// Find all polygons intersected by a ray
    fn ray_intersections(&self, ray: &Ray) -> Vec<Intersection<S>>;

    /// Check if a point is inside the geometry represented by this structure
    fn contains_point(&self, point: &Point3<Real>) -> bool;

    /// Get the bounding box of all geometry in this structure
    fn bounding_box(&self) -> Option<Aabb>;

    /// Get statistics about the structure (depth, node count, etc.)
    fn statistics(&self) -> SpatialStatistics;
}

/// Statistics about a spatial data structure
#[derive(Debug, Clone)]
pub struct SpatialStatistics {
    pub node_count: usize,
    pub max_depth: usize,
    pub polygon_count: usize,
    pub memory_usage_bytes: usize,
}

/// Characteristics of a polygon dataset for structure selection
#[derive(Debug, Clone)]
pub struct DatasetCharacteristics {
    pub polygon_count: usize,
    pub bounding_box: Aabb,
    pub density: Real,  // polygons per unit volume
    pub aspect_ratio: Real,  // max dimension / min dimension
    pub spatial_distribution: SpatialDistribution,
}

/// Types of spatial distribution patterns
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialDistribution {
    Uniform,
    Clustered,
    Sparse,
    Linear,
    Planar,
}

/// Query type for optimal structure selection
#[derive(Debug, Clone, PartialEq)]
pub enum QueryType {
    BooleanOperations,
    PointLocation,
    NearestNeighbor,
    RangeQuery,
    RayIntersection,
    VolumeQuery,
}

/// Automatic spatial structure selector
pub struct SpatialStructureSelector;

impl SpatialStructureSelector {
    /// Analyze polygon dataset characteristics
    pub fn analyze_dataset<S: Clone>(polygons: &[Polygon<S>]) -> DatasetCharacteristics {
        let polygon_count = polygons.len();
        
        let bounding_box = Aabb::from_polygons(polygons)
            .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::origin()));
        
        let volume = bounding_box.volume();
        let density = if volume > 0.0 { polygon_count as Real / volume } else { 0.0 };
        
        let size = bounding_box.size();
        let max_dim = size.x.max(size.y).max(size.z);
        let min_dim = size.x.min(size.y).min(size.z);
        let aspect_ratio = if min_dim > 0.0 { max_dim / min_dim } else { Real::INFINITY };
        
        let spatial_distribution = Self::analyze_distribution(polygons, &bounding_box);
        
        DatasetCharacteristics {
            polygon_count,
            bounding_box,
            density,
            aspect_ratio,
            spatial_distribution,
        }
    }

    /// Recommend optimal spatial structure based on dataset and query type
    pub fn recommend_structure(
        characteristics: &DatasetCharacteristics,
        query_type: QueryType,
    ) -> SpatialStructureType {
        match query_type {
            QueryType::BooleanOperations => {
                // BSP trees are optimal for Boolean operations
                SpatialStructureType::Bsp
            },
            QueryType::PointLocation | QueryType::NearestNeighbor => {
                // KD-trees excel at point-based queries
                if characteristics.polygon_count < 1000 {
                    SpatialStructureType::Kdtree
                } else {
                    SpatialStructureType::Hybrid
                }
            },
            QueryType::RangeQuery | QueryType::VolumeQuery => {
                // Octrees are good for volume-based queries
                if characteristics.spatial_distribution == SpatialDistribution::Sparse {
                    SpatialStructureType::Octree
                } else {
                    SpatialStructureType::Kdtree
                }
            },
            QueryType::RayIntersection => {
                // Choose based on polygon density
                if characteristics.density > 10.0 {
                    SpatialStructureType::Bsp
                } else {
                    SpatialStructureType::Kdtree
                }
            },
        }
    }

    fn analyze_distribution<S: Clone>(polygons: &[Polygon<S>], bounds: &Aabb) -> SpatialDistribution {
        if polygons.is_empty() {
            return SpatialDistribution::Uniform;
        }

        // Simple heuristic based on polygon center distribution
        let centers: Vec<Point3<Real>> = polygons
            .iter()
            .map(|poly| {
                let sum = poly.vertices.iter().fold(Point3::origin(), |acc, v| acc + v.pos.coords);
                Point3::from(sum.coords / poly.vertices.len() as Real)
            })
            .collect();

        // Analyze clustering by checking variance
        let center_of_centers = centers.iter().fold(Point3::origin(), |acc, p| acc + p.coords) / centers.len() as Real;
        let variance = centers.iter()
            .map(|p| (p - center_of_centers).norm_squared())
            .sum::<Real>() / centers.len() as Real;

        let bounds_size = bounds.size().norm();
        let normalized_variance = variance / (bounds_size * bounds_size);

        if normalized_variance < 0.1 {
            SpatialDistribution::Clustered
        } else if normalized_variance > 0.5 {
            SpatialDistribution::Sparse
        } else {
            SpatialDistribution::Uniform
        }
    }
}

/// Types of spatial data structures
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialStructureType {
    Bsp,
    Kdtree,
    Octree,
    Hybrid,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_aabb_creation() {
        let min = Point3::new(0.0, 0.0, 0.0);
        let max = Point3::new(1.0, 1.0, 1.0);
        let aabb = Aabb::new(min, max);
        
        assert_eq!(aabb.min, min);
        assert_eq!(aabb.max, max);
        assert_eq!(aabb.volume(), 1.0);
    }

    #[test]
    fn test_aabb_contains_point() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        
        assert!(aabb.contains_point(&Point3::new(0.5, 0.5, 0.5)));
        assert!(!aabb.contains_point(&Point3::new(1.5, 0.5, 0.5)));
    }

    #[test]
    fn test_ray_creation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let direction = Vector3::new(1.0, 0.0, 0.0);
        let ray = Ray::new(origin, direction);
        
        assert_eq!(ray.origin, origin);
        assert_eq!(ray.direction, direction);
        assert_eq!(ray.point_at(1.0), Point3::new(1.0, 0.0, 0.0));
    }
}
