//! IndexedMesh module - Memory-efficient mesh representation with vertex deduplication
//!
//! This module provides the IndexedMesh data structure and all related functionality
//! organized into focused submodules for maintainability and modularity.

pub mod adjacency;
pub mod conversion;
pub mod core;
pub mod csg;
pub mod deduplication;
pub mod operations;
pub mod shapes;
pub mod topology;

// Re-export core types for convenience
pub use core::{AdjacencyInfo, IndexedFace, IndexedMetadata, IndexedMesh, DEDUP_EPSILON};
