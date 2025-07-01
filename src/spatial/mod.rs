//! Spatial data structures and algorithms module
//!
//! This module provides spatial data structures and algorithms for geometric operations,
//! including Binary Space Partitioning (BSP) trees and other spatial indexing structures.
//! 
//! Following the Cathedral Engineering principles, this module serves as a dedicated
//! architectural space for spatial data structures, separate from CSG operations to
//! maintain modularity and single responsibility.
//!
//! # Modules
//!
//! - [`bsp`]: Binary Space Partitioning tree implementation for spatial subdivision
//!
//! # Design Philosophy
//!
//! The spatial module follows the Law of Sacred Spaces by organizing spatial data
//! structures as independent components that can be composed and reused across
//! different geometric operations. This separation allows for:
//!
//! - **Modularity**: Spatial structures can be used independently of CSG operations
//! - **Reusability**: BSP trees and other structures can serve multiple purposes
//! - **Maintainability**: Clear separation of concerns between spatial indexing and geometric operations
//! - **Extensibility**: Easy addition of new spatial data structures (kdtree, octree, etc.)

pub mod bsp;

// Re-export the main BSP types for convenience
pub use bsp::Node as BspNode;
