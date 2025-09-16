//! Conversion utilities for IndexedMesh to external formats and integrations

use super::IndexedMesh;
use std::fmt::Debug;

#[cfg(any(feature = "f64", feature = "f32"))]
use crate::float_types::{
    parry3d::shape::{Shape, TriMesh},
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape,
    },
};
#[cfg(any(feature = "f64", feature = "f32"))]
use nalgebra::{Point3, Quaternion, Unit};

impl<S: Clone + Send + Sync + Debug> IndexedMesh<S> {
    /// Convert an IndexedMesh into a Bevy `Mesh`.
    ///
    /// This method triangulates faces as needed and converts the indexed representation
    /// into Bevy's vertex buffer format. Vertex deduplication is preserved in the conversion.
    ///
    /// # Returns
    /// A Bevy Mesh with triangulated geometry and proper vertex attributes
    ///
    /// # Performance
    /// This method triangulates faces on-demand and may allocate additional memory
    /// for the Bevy vertex buffers. The conversion preserves the memory efficiency
    /// of the indexed representation while creating the required vertex buffers.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh};
        use wgpu_types::PrimitiveTopology;

        // Prepare buffers - we'll expand indexed vertices to direct vertex buffers for Bevy
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::new();

        let mut current_index = 0u32;

        // Process each face, triangulating as needed
        for face in &self.faces {
            let face_vertices = &face.vertices;

            // Triangulate the face (ear clipping for polygons > 3 vertices)
            if face_vertices.len() < 3 {
                // Skip degenerate faces
                continue;
            } else if face_vertices.len() == 3 {
                // Triangle - add directly
                for &vertex_idx in face_vertices {
                    let vertex = &self.vertices[vertex_idx];
                    positions_32.push([
                        vertex.pos.x as f32,
                        vertex.pos.y as f32,
                        vertex.pos.z as f32,
                    ]);
                    normals_32.push([
                        vertex.normal.x as f32,
                        vertex.normal.y as f32,
                        vertex.normal.z as f32,
                    ]);
                }

                indices.push(current_index);
                indices.push(current_index + 1);
                indices.push(current_index + 2);
                current_index += 3;
            } else {
                // Polygon with > 3 vertices - triangulate using fan triangulation from first vertex
                // For a quad [0,1,2,3], this creates triangles [0,1,2] and [0,2,3]

                // Add all vertices for this face (fan triangulation duplicates the first vertex)
                for &vertex_idx in face_vertices {
                    let vertex = &self.vertices[vertex_idx];
                    positions_32.push([
                        vertex.pos.x as f32,
                        vertex.pos.y as f32,
                        vertex.pos.z as f32,
                    ]);
                    normals_32.push([
                        vertex.normal.x as f32,
                        vertex.normal.y as f32,
                        vertex.normal.z as f32,
                    ]);
                }

                // Create triangles using fan triangulation
                for i in 1..face_vertices.len() - 1 {
                    indices.push(current_index); // First vertex of fan
                    indices.push(current_index + i as u32); // Current vertex
                    indices.push(current_index + (i + 1) as u32); // Next vertex
                }

                current_index += face_vertices.len() as u32;
            }
        }

        // Create the Bevy mesh
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert vertex attributes
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }

    /// Convert the faces in this IndexedMesh to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any face has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    #[cfg(any(feature = "f64", feature = "f32"))]
    pub fn to_rapier_shape(&self) -> Option<SharedShape> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok().map(SharedShape::new)
    }

    /// Convert the faces in this IndexedMesh to a Parry `TriMesh`.
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any face has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    #[cfg(any(feature = "f64", feature = "f32"))]
    pub fn to_trimesh(&self) -> Option<TriMesh> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok()
    }

    /// Approximate mass properties using Rapier.
    #[cfg(any(feature = "f64", feature = "f32"))]
    pub fn mass_properties(
        &self,
        density: crate::float_types::Real,
    ) -> Option<(
        crate::float_types::Real,
        Point3<crate::float_types::Real>,
        Unit<Quaternion<crate::float_types::Real>>,
    )> {
        self.to_trimesh().map(|trimesh| {
            let mp = trimesh.mass_properties(density);
            (
                mp.mass(),
                mp.local_com,                     // a Point3<Real>
                mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
            )
        })
    }

    /// Create a Rapier rigid body + collider from this IndexedMesh, using
    /// an axis-angle `rotation` in 3D (the vector's length is the
    /// rotation in radians, and its direction is the axis).
    #[cfg(any(feature = "f64", feature = "f32"))]
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: nalgebra::Vector3<crate::float_types::Real>,
        rotation: nalgebra::Vector3<crate::float_types::Real>, /* rotation axis scaled by angle (radians) */
        density: crate::float_types::Real,
    ) -> Option<RigidBodyHandle> {
        self.to_rapier_shape().map(|shape| {
            // Build a Rapier RigidBody
            let rb = RigidBodyBuilder::dynamic()
                .translation(translation)
                // Now `rotation(...)` expects an axis-angle Vector3.
                .rotation(rotation)
                .build();
            let rb_handle = rb_set.insert(rb);

            // Build the collider
            let coll = ColliderBuilder::new(shape).density(density).build();
            co_set.insert_with_parent(coll, rb_handle, rb_set);

            rb_handle
        })
    }

    /// Get vertices and triangle indices for physics/collision detection.
    /// This triangulates faces as needed and returns vertex positions and triangle indices.
    #[cfg(any(feature = "f64", feature = "f32"))]
    fn get_vertices_and_indices(
        &self,
    ) -> (Vec<Point3<crate::float_types::Real>>, Vec<[u32; 3]>) {
        // For IndexedMesh, we need to triangulate faces and expand to vertex positions
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut vertex_map = std::collections::HashMap::new();

        // Process each face, triangulating as needed
        for face in &self.faces {
            let face_vertices = &face.vertices;

            if face_vertices.len() < 3 {
                continue; // Skip degenerate faces
            }

            // Collect vertex positions for this face
            let face_positions: Vec<Point3<crate::float_types::Real>> = face_vertices
                .iter()
                .map(|&idx| self.vertices[idx].pos)
                .collect();

            if face_vertices.len() == 3 {
                // Triangle - add directly
                for &vertex_idx in face_vertices {
                    let pos = self.vertices[vertex_idx].pos;
                    let global_idx = vertex_map.len() as u32;
                    vertex_map.insert(vertex_idx, global_idx);
                    vertices.push(pos);
                }

                let base_idx = (vertices.len() - 3) as u32;
                indices.push([base_idx, base_idx + 1, base_idx + 2]);
            } else {
                // Polygon with > 3 vertices - triangulate using fan triangulation
                // Add all vertices for this face
                let base_idx = vertices.len() as u32;
                vertices.extend(face_positions);

                // Create triangles using fan triangulation from first vertex
                for i in 1..face_vertices.len() - 1 {
                    indices.push([
                        base_idx,                  // First vertex of fan
                        base_idx + i as u32,       // Current vertex
                        base_idx + (i + 1) as u32, // Next vertex
                    ]);
                }
            }
        }

        (vertices, indices)
    }
}
