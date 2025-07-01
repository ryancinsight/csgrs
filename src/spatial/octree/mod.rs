//! Octree implementation for hierarchical 3D space subdivision
//!
//! This module provides a high-performance Octree implementation optimized for
//! 3D geometric operations, particularly volume queries, level-of-detail operations,
//! and sparse data handling. Octrees excel at hierarchical space subdivision and
//! adaptive refinement based on data density.
//!
//! # Module Structure
//!
//! - `core`: Core Octree node structure and basic operations
//! - `construction`: Tree building algorithms and adaptive refinement
//! - `queries`: Spatial query operations (volume queries, level-of-detail, etc.)

pub mod core;
pub mod construction;
pub mod queries;

// Re-export the main types for public API
pub use core::Node;

use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create an Octree from polygons using the optimal construction method
    pub fn from_polygons(polygons: &[crate::geometry::Polygon<S>]) -> Self {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 200 {
                // Use parallel construction for larger datasets
                return Self::from_polygons_parallel(polygons);
            }
        }

        // Default to sequential construction
        Self::from_polygons_sequential(polygons)
    }

    /// Automatically select the best query method based on input size
    pub fn volume_query_auto(&self, bounds: &crate::spatial::traits::Aabb) 
        -> Vec<&crate::geometry::Polygon<S>> {
        #[cfg(feature = "parallel")]
        {
            if self.polygon_count() > 500 {
                return self.volume_query_parallel(bounds);
            }
        }

        self.volume_query(bounds)
    }

    /// Automatically select the best level-of-detail method
    pub fn level_of_detail_auto(&self, viewpoint: &nalgebra::Point3<crate::core::float_types::Real>, 
                                detail_threshold: crate::core::float_types::Real) 
        -> Vec<&crate::geometry::Polygon<S>> {
        #[cfg(feature = "parallel")]
        {
            if self.polygon_count() > 500 {
                return self.level_of_detail_parallel(viewpoint, detail_threshold);
            }
        }

        self.level_of_detail(viewpoint, detail_threshold)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Polygon, Vertex};
    use crate::spatial::traits::Aabb;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_octree_basic_functionality() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let octree = Node::from_polygons(&polygons);
        assert!(!octree.all_polygons().is_empty());
    }

    #[test]
    fn test_octree_volume_query() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let octree = Node::from_polygons(&polygons);
        let query_bounds = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(1.5, 1.5, 0.5));
        let results = octree.volume_query(&query_bounds);
        assert!(!results.is_empty());
    }

    #[test]
    fn test_octree_level_of_detail() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let octree = Node::from_polygons(&polygons);
        let viewpoint = Point3::new(0.0, 0.0, 5.0);
        let detail_threshold = 2.0;
        let results = octree.level_of_detail(&viewpoint, detail_threshold);
        assert!(!results.is_empty());
    }

    #[test]
    fn test_auto_methods_fallback() {
        let octree = Node::<i32>::new();
        
        // Test auto methods with empty tree
        let query_bounds = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let results = octree.volume_query_auto(&query_bounds);
        assert!(results.is_empty());
        
        let viewpoint = Point3::new(0.0, 0.0, 0.0);
        let lod_results = octree.level_of_detail_auto(&viewpoint, 1.0);
        assert!(lod_results.is_empty());
    }
}
