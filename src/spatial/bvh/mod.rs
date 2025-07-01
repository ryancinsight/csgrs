//! BVH (Bounding Volume Hierarchy) spatial data structure implementation
//!
//! This module provides a BVH implementation optimized for ray tracing, collision detection,
//! and dynamic object management. BVHs are particularly well-suited for:
//!
//! - Ray tracing and ray casting operations with optimal traversal performance
//! - Real-time collision detection with dynamic object updates
//! - Rendering applications requiring efficient ray-primitive intersection
//! - Game engines with moving objects and frequent spatial queries
//!
//! # Examples
//!
//! ## Basic Usage
//!
//! ```rust
//! use csgrs::spatial::{bvh::Node, SpatialIndex};
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
//! // Build BVH
//! let bvh = Node::from_polygons(&polygons);
//! assert_eq!(bvh.all_polygons().len(), 1);
//! ```
//!
//! ## Ray Tracing Operations
//!
//! ```rust
//! use csgrs::spatial::{bvh::Node, traits::Ray, SpatialIndex};
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create BVH for ray tracing
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//! let bvh = Node::from_polygons(&vec![polygon]);
//!
//! // Cast ray through scene
//! let ray = Ray {
//!     origin: Point3::new(-1.0, 0.5, 0.0),
//!     direction: Vector3::new(1.0, 0.0, 0.0),
//! };
//! let intersections = bvh.ray_intersections(&ray);
//! // Process ray-triangle intersections for rendering
//! ```
//!
//! ## Dynamic Object Management
//!
//! ```rust
//! use csgrs::spatial::bvh::Node;
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create empty BVH for dynamic scene
//! let mut bvh: Node<i32> = Node::new();
//!
//! // Add objects dynamically
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//!
//! bvh.insert_object(polygon);
//! assert_eq!(bvh.polygon_count(), 1);
//! ```
//!
//! ## SAH Construction
//!
//! ```rust
//! use csgrs::spatial::{bvh::{Node, BVHConfig, ConstructionAlgorithm}, SpatialIndex};
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Configure BVH for optimal ray tracing performance
//! let config = BVHConfig {
//!     construction_algorithm: ConstructionAlgorithm::SAH,
//!     max_polygons_per_leaf: 4,
//!     sah_traversal_cost: 1.0,
//!     sah_intersection_cost: 1.0,
//!     ..Default::default()
//! };
//!
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//! let polygons = vec![polygon];
//!
//! let bvh = Node::from_polygons_with_config(&polygons, &config);
//! assert_eq!(bvh.all_polygons().len(), 1);
//! ```

pub mod core;
pub mod operations;
pub mod construction;
pub mod config;

// Re-export the main types for public API
pub use core::Node;
pub use config::{BVHConfig, ConstructionAlgorithm};
