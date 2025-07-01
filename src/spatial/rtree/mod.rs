//! R-tree spatial data structure implementation
//!
//! This module provides an R-tree implementation optimized for bounding box queries
//! and dynamic spatial indexing. R-trees are particularly well-suited for:
//!
//! - Dynamic insertion and deletion without full tree reconstruction
//! - Efficient bounding box range queries with polygon overlap detection
//! - Real-time spatial indexing for GIS, collision detection, and spatial databases
//!
//! # Examples
//!
//! ## Basic Usage
//!
//! ```rust
//! use csgrs::spatial::rtree::Node;
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create test polygons
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//! let polygons = vec![polygon];
//!
//! // Build R-tree
//! let rtree = Node::from_polygons(&polygons);
//! assert_eq!(rtree.all_polygons().len(), 1);
//! ```
//!
//! ## Dynamic Operations
//!
//! ```rust
//! use csgrs::spatial::rtree::Node;
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create empty R-tree
//! let mut rtree: Node<i32> = Node::new();
//!
//! // Insert polygons dynamically
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//!
//! rtree.insert(polygon);
//! assert_eq!(rtree.polygon_count(), 1);
//! ```
//!
//! ## Range Queries
//!
//! ```rust
//! use csgrs::spatial::{rtree::Node, traits::Aabb};
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create and populate R-tree
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.5, 0.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.5, 0.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 1.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//! let rtree = Node::from_polygons(&vec![polygon]);
//!
//! // Perform range query
//! let query_bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
//! let results = rtree.query_range(&query_bounds);
//! assert!(results.len() > 0);
//! ```

pub mod core;
pub mod operations;
pub mod construction;
pub mod config;

// Re-export the main types for public API
pub use core::Node;
pub use config::{RTreeConfig, SplitAlgorithm};
