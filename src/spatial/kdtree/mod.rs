//! KD-tree (K-Dimensional tree) implementation for spatial indexing
//!
//! This module provides a high-performance KD-tree implementation optimized for
//! 3D geometric operations, particularly point location, nearest neighbor searches,
//! and range queries. KD-trees excel at spatial queries involving points and
//! axis-aligned regions.
//!
//! # Module Structure
//!
//! - `core`: Core KD-tree node structure and basic operations
//! - `construction`: Tree building algorithms and optimization strategies
//! - `queries`: Spatial query operations (nearest neighbor, range queries, etc.)

pub mod core;
pub mod construction;
pub mod queries;

// Re-export the main types for public API
pub use core::Node;

use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create a KD-tree from polygons using the optimal construction method
    pub fn from_polygons(polygons: &[crate::geometry::Polygon<S>]) -> Self {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 500 {
                // Use parallel construction for larger datasets
                return Self::from_polygons_parallel(polygons);
            }
        }

        // Default to sequential construction
        Self::from_polygons_sequential(polygons)
    }

    /// Automatically select the best query method based on input size
    pub fn nearest_neighbor_auto(&self, point: &nalgebra::Point3<crate::core::float_types::Real>) 
        -> Option<&crate::geometry::Polygon<S>> {
        #[cfg(feature = "parallel")]
        {
            if self.polygon_count() > 1000 {
                return self.nearest_neighbor_parallel(point);
            }
        }

        self.nearest_neighbor(point)
    }

    /// Automatically select the best range query method
    pub fn range_query_auto(&self, bounds: &crate::spatial::traits::Aabb) 
        -> Vec<&crate::geometry::Polygon<S>> {
        #[cfg(feature = "parallel")]
        {
            if self.polygon_count() > 1000 {
                return self.range_query_parallel(bounds);
            }
        }

        self.range_query(bounds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Polygon, Vertex};
    use crate::spatial::traits::Aabb;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_kdtree_basic_functionality() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let kdtree = Node::from_polygons(&polygons);
        assert!(!kdtree.all_polygons().is_empty());
    }

    #[test]
    fn test_kdtree_nearest_neighbor() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let kdtree = Node::from_polygons(&polygons);
        let query_point = Point3::new(0.5, 0.3, 0.0);
        let nearest = kdtree.nearest_neighbor(&query_point);
        assert!(nearest.is_some());
    }

    #[test]
    fn test_kdtree_range_query() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let kdtree = Node::from_polygons(&polygons);
        let query_bounds = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(1.5, 1.5, 0.5));
        let results = kdtree.range_query(&query_bounds);
        assert!(!results.is_empty());
    }

    #[test]
    fn test_auto_methods_fallback() {
        let kdtree = Node::<i32>::new();
        let query_point = Point3::new(0.0, 0.0, 0.0);
        
        // Test auto methods with empty tree
        let nearest = kdtree.nearest_neighbor_auto(&query_point);
        assert!(nearest.is_none());
        
        let query_bounds = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let results = kdtree.range_query_auto(&query_bounds);
        assert!(results.is_empty());
    }
}
