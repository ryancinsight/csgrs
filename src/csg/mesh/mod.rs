//! Mesh processing, analysis, and manipulation functionalities.
//!
//! This module provides a suite of tools for working with CSG meshes, including:
//! - **Connectivity analysis**: Building robust adjacency graphs.
//! - **Quality metrics**: Assessing triangle and mesh quality.
//! - **Manifold checking**: Verifying mesh integrity.
//! - **Mesh operations**: Smoothing, refinement, and tessellation.

pub mod connectivity;
pub mod quality;
pub mod manifold;
pub mod ops;

// Re-export key types for easier access
pub use self::connectivity::VertexIndexMap;
pub use self::quality::{TriangleQuality, MeshQualityMetrics}; 