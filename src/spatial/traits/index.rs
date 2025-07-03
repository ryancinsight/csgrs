//! **Spatial Index Trait (The Soul)**
//!
//! This module defines the core trait interface for spatial data structures,
//! following Cathedral Engineering principles where traits represent the "soul"
//! that defines the fundamental contracts and behaviors.

use crate::core::float_types::Real;
use crate::geometry::Polygon;
use super::geometry::{Aabb, Ray, Intersection};
use nalgebra::Point3;
use std::fmt::Debug;

/// **Statistics about a spatial data structure**
#[derive(Debug, Clone)]
pub struct SpatialStatistics {
    pub node_count: usize,
    pub max_depth: usize,
    pub polygon_count: usize,
    pub memory_usage_bytes: usize,
}

impl Default for SpatialStatistics {
    fn default() -> Self {
        Self {
            node_count: 0,
            max_depth: 0,
            polygon_count: 0,
            memory_usage_bytes: 0,
        }
    }
}

/// **Characteristics of the dataset being indexed**
#[derive(Debug, Clone)]
pub struct DatasetCharacteristics {
    pub polygon_count: usize,
    pub total_vertices: usize,
    pub bounding_box: Option<Aabb>,
    pub density_estimate: f64,
}

/// **Primary trait for spatial data structures**
///
/// This trait defines the core interface that all spatial structures must implement,
/// providing a unified API for spatial queries and operations.
pub trait SpatialIndex<S: Clone + Debug + Send + Sync> {
    /// **Build a new spatial structure from polygons**
    fn build(polygons: &[Polygon<S>]) -> Self where Self: Sized;

    /// **Create a new empty spatial structure**
    fn new() -> Self where Self: Sized;

    /// **Get all polygons stored in this structure**
    fn all_polygons(&self) -> Vec<Polygon<S>>;

    /// **Query polygons within a bounding box**
    fn query_range(&self, bounds: &Aabb) -> Vec<&Polygon<S>>;

    /// **Find the nearest polygon to a point**
    fn nearest_neighbor(&self, point: &Point3<Real>) -> Option<&Polygon<S>>;

    /// **Find all ray-polygon intersections**
    fn ray_intersections(&self, ray: &Ray) -> Vec<Intersection<S>>;

    /// **Check if a point is contained within the structure**
    fn contains_point(&self, point: &Point3<Real>) -> bool;

    /// **Get statistics about this spatial structure**
    fn statistics(&self) -> SpatialStatistics;

    /// **Get the bounding box of all indexed geometry**
    fn bounding_box(&self) -> Option<Aabb>;

    /// **Check if the structure is empty**
    fn is_empty(&self) -> bool {
        self.all_polygons().is_empty()
    }

    /// **Get the number of polygons in the structure**
    fn polygon_count(&self) -> usize {
        self.all_polygons().len()
    }
}
