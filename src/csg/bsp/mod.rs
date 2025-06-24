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

use crate::geometry::{Plane, Polygon, Vertex};
use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Creates a new BSP node from polygons using the appropriate build method
    /// 
    /// This function automatically chooses between parallel and non-parallel implementations
    /// based on feature availability and input size.
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 100 { // Use parallel for larger datasets
                let mut node = Self::new();
                node.build_parallel(polygons);
                return node;
            }
        }
        
        // Default to non-parallel implementation
        let mut node = Self::new();
        node.build(polygons);
        node
    }

    /// Clips polygons using the appropriate method based on feature availability
    pub fn clip_polygons_auto(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 50 {
                return self.clip_polygons_parallel(polygons);
            }
        }
        
        self.clip_polygons(polygons)
    }
    // End of Selection
    /// Clips to another BSP tree using the appropriate method
    pub fn clip_to_auto(&mut self, bsp: &Node<S>) {
        #[cfg(feature = "parallel")]
        {
            if self.polygons.len() > 50 {
                self.clip_to_parallel(bsp);
                return;
            }
        }
        
        self.clip_to(bsp);
    }

    /// Slices the BSP tree using the appropriate method
    pub fn slice_auto(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        #[cfg(feature = "parallel")]
        {
            if self.polygons.len() > 50 {
                return self.slice_parallel(slicing_plane);
            }
        }
        
        self.slice(slicing_plane)
    }
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