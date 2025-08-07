//! Sparse Voxel Octree-Embedded BSP Trees
//!
//! This module implements a sparse voxel octree (SVO) approach where BSP trees are embedded
//! within octree nodes for efficient CSG operations. Unlike dense octrees, this implementation
//! only allocates memory for voxels containing geometry, providing superior memory efficiency
//! for sparse geometries common in CAD applications.
//!
//! ## Key Features
//!
//! - **Sparse Representation**: Memory usage scales with geometric complexity, not spatial volume
//! - **Embedded BSP**: Each octree node contains a BSP subtree for local geometry
//! - **CSG Compatibility**: Full implementation of the CSG trait
//! - **Zero-Copy Operations**: Leverages Rust's iterator system for efficient processing
//! - **Unified BSP Logic**: Single implementation with feature-gated parallelism
//!
//! ## Architecture
//!
//! The implementation follows research-proven techniques for octree-embedded BSP trees,
//! providing robust CSG operations while maintaining spatial efficiency through sparse
//! voxel allocation.



#[cfg(feature = "parallel")]
use rayon::prelude::*;

pub mod svo_node;
pub mod svo_mesh;
pub mod bsp_unified;
pub mod precision;
pub mod conversion;

pub use svo_node::SvoNode;
pub use svo_mesh::SvoMesh;
pub use precision::PrecisionConfig;

// Constants moved to individual modules where they're used