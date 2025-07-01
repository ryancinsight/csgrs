//! BSP (Binary Space Partitioning) tree implementation
//!
//! This module provides a modular BSP tree implementation split across focused modules:
//! - `core`: Basic Node structure and fundamental operations
//! - `operations`: Non-parallel complex operations (clip, build, slice)
//! - `parallel`: Parallel implementations for better performance (requires "parallel" feature)

pub mod core;
pub mod operations;

#[cfg(feature = "parallel")]
pub mod parallel;

// Re-export the main types for public API
pub use core::Node;

use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {

}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Polygon, Vertex};
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_modular_bsp_basic_functionality() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let node = Node::from_polygons(&polygons);
        assert!(!node.all_polygons().is_empty());
    }

    #[test]
    fn test_auto_methods_fallback() {
        let node = Node::<i32>::new();
        let empty_polygons = vec![];

        // Test auto methods with empty input
        let result = node.clip_polygons_auto(&empty_polygons);
        assert!(result.is_empty());
    }
}
