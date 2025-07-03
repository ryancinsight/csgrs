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
//! - [`kdtree`]: K-Dimensional tree implementation for point queries and nearest neighbor searches
//! - [`octree`]: Octree implementation for hierarchical 3D space subdivision and level-of-detail operations
//! - [`traits`]: Common interfaces and utilities for all spatial data structures
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
pub mod bvh;
pub mod kdtree;
pub mod octree;
pub mod rtree;
pub mod traits;
pub mod utils;

// Re-export the main types for convenience
pub use bsp::Node as BspNode;
pub use kdtree::Node as KdTreeNode;
pub use octree::Node as OctreeNode;

// Re-export common traits and utilities from the new modular structure
pub use traits::{
    SpatialError, SpatialResult, Aabb, Ray, Intersection, SpatialIndex, SpatialStatistics,
    DatasetCharacteristics, SpatialConfig, QueryType, SpatialDistribution, SpatialStructureType,
    SpatialStructureFactory, SpatialStructureSelector, SpatialBenchmark, BenchmarkResults,
    PerformanceMatrix, StructurePerformance, PerformanceRating
};
