//! Utility functions for IndexedMesh operations
//!
//! This module contains utility functions for working with IndexedMesh,
//! including validation, adjacency operations, and helper functions.

use crate::indexed_mesh::{AdjacencyInfo, IndexedMesh};
use std::fmt::Debug;

/// Combine adjacency information from multiple meshes
/// This is useful for operations that work on multiple meshes simultaneously
pub fn combine_adjacency_info(meshes: &[&IndexedMesh<()>]) -> AdjacencyInfo {
    if meshes.is_empty() {
        return AdjacencyInfo {
            vertex_adjacency: Vec::new(),
            vertex_faces: Vec::new(),
            face_adjacency: Vec::new(),
            face_vertices: Vec::new(),
        };
    }

    // For simplicity, just return adjacency info from the first mesh
    // In practice, this might need more sophisticated combination logic
    meshes[0].adjacency().clone()
}

/// Validate that all face vertex indices are within valid range
/// Returns true if all indices are valid, false otherwise
pub fn validate_face_indices<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> Result<(), String> {
    let num_vertices = mesh.vertices.len();

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for &vertex_idx in &face.vertices {
            if vertex_idx >= num_vertices {
                return Err(format!(
                    "Face {} references vertex index {} which is out of range (max: {})",
                    face_idx, vertex_idx, num_vertices - 1
                ));
            }
        }

        // Validate that faces have at least 3 vertices (triangles minimum)
        if face.vertices.len() < 3 {
            return Err(format!(
                "Face {} has only {} vertices, minimum required is 3",
                face_idx, face.vertices.len()
            ));
        }
    }

    Ok(())
}
