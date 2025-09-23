//! Transformation operations for IndexedMesh
//!
//! This module provides geometric transformations (translation, rotation, scaling)
//! and mesh operations like inversion for IndexedMesh.

use crate::float_types::Real;
use crate::indexed_mesh::IndexedMesh;
use crate::mesh::vertex::Vertex;
use nalgebra::{Matrix4, Point3};
use std::fmt::Debug;

/// Apply affine transformation to IndexedMesh
pub fn transform<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
    transform_matrix: &Matrix4<Real>,
) -> IndexedMesh<S> {
    let mut result = mesh.clone();

    // Transform all vertices
    for vertex in &mut result.vertices {
        // Apply transformation to position
        let new_pos = transform_matrix.transform_point(&vertex.pos);
        vertex.pos = new_pos;

        // Transform normal if present (using inverse transpose for correct normal transformation)
        if let Some(normal) = vertex.normal {
            // Extract 3x3 rotation/scale/skew part of the matrix
            let rotation_matrix = transform_matrix.fixed_view::<3, 3>(0, 0);

            // For normals, we need the inverse transpose of the transformation matrix
            // This preserves the perpendicularity of normals after transformation
            if let Some(inv_transpose) = rotation_matrix.try_inverse() {
                let transformed_normal = inv_transpose.transpose() * normal;
                // Renormalize to ensure unit length (compensates for any scaling in the matrix)
                vertex.normal = Some(transformed_normal.normalize());
            } else {
                // Fallback: just transform the normal without inverse transpose
                // This happens when the transformation matrix is not invertible
                let transformed_normal = rotation_matrix * normal;
                vertex.normal = Some(transformed_normal.normalize());
            }
        }
    }

    // Clear cached data that depends on vertex positions
    result.adjacency = std::sync::OnceLock::new();
    result.bounding_box = std::sync::OnceLock::new();

    result
}

/// Invert face winding order of IndexedMesh
/// This flips the mesh "inside-out" by reversing the vertex order of each face
pub fn inverse<S: Clone + Send + Sync + Debug>(mesh: &IndexedMesh<S>) -> IndexedMesh<S> {
    let mut result = mesh.clone();

    // Reverse vertex order for each face to flip winding
    for face in &mut result.faces {
        face.vertices.reverse();
        // Flip normal if it exists
        if let Some(ref mut normal) = face.normal {
            *normal = -*normal;
        }
    }

    // Flip vertex normals to match the inverted faces
    for vertex in &mut result.vertices {
        if let Some(ref mut normal) = vertex.normal {
            *normal = -*normal;
        }
    }

    result
}
