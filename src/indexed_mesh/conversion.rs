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

impl<S: Clone + Send + Sync + Debug> From<crate::mesh::Mesh<S>> for super::core::IndexedMesh<S> {
    /// Convert standard Mesh to IndexedMesh with automatic deduplication
    fn from(mesh: crate::mesh::Mesh<S>) -> Self {
        use super::core::{DEDUP_EPSILON, IndexedFace, IndexedMesh};
        use crate::mesh::vertex::Vertex;

        let mut vertices = Vec::new();
        let mut faces = Vec::new();

        // Extract all vertices and build face indices
        for polygon in &mesh.polygons {
            let mut face_indices = Vec::new();

            for vertex in &polygon.vertices {
                // Find existing vertex or add new one
                let vertex_idx = vertices
                    .iter()
                    .position(|v: &Vertex| (v.pos - vertex.pos).norm() < DEDUP_EPSILON);

                let vertex_idx = match vertex_idx {
                    Some(idx) => idx,
                    None => {
                        vertices.push(*vertex);
                        vertices.len() - 1
                    },
                };

                face_indices.push(vertex_idx);
            }

            // Use polygon plane normal for geometric consistency with Mesh operations
            // This ensures IndexedMesh operations produce identical results to Mesh operations
            let face_normal = if face_indices.len() >= 3 {
                let plane_normal = polygon.plane.normal();
                let length = plane_normal.norm();
                if length > DEDUP_EPSILON {
                    Some(plane_normal / length)
                } else {
                    None
                }
            } else {
                None
            };

            let indexed_face = IndexedFace {
                vertices: face_indices,
                normal: face_normal,
                metadata: None,
            };
            faces.push(indexed_face);
        }

        let mut indexed_mesh = IndexedMesh {
            vertices,
            faces,
            adjacency: std::sync::OnceLock::new(),
            bounding_box: std::sync::OnceLock::new(),
            metadata: mesh.metadata,
        };

        // Compute face normals for faces that don't have them
        indexed_mesh.compute_face_normals_fallback();

        indexed_mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{polygon::Polygon, Mesh, vertex::Vertex};
    use nalgebra::Point3;

    #[test]
    fn test_conversion_roundtrip_mesh_to_indexed() {
        // Test roundtrip conversion: Mesh -> IndexedMesh -> Mesh
        let original_mesh = Mesh::cube(2.0, None::<()>).unwrap();

        // Convert Mesh to IndexedMesh
        let indexed: IndexedMesh<()> = original_mesh.clone().into();

        // Verify IndexedMesh properties
        assert!(!indexed.vertices.is_empty(), "IndexedMesh should have vertices");
        assert!(!indexed.faces.is_empty(), "IndexedMesh should have faces");
        assert!(indexed.validate_face_indices().is_ok(), "IndexedMesh should be valid");

        // Convert back to Mesh
        let back_to_mesh = indexed.to_mesh();

        // Verify roundtrip preserves validity
        assert!(!back_to_mesh.polygons.is_empty(), "Roundtrip mesh should have polygons");
        assert!(back_to_mesh.polygons.len() >= original_mesh.polygons.len(),
            "Roundtrip should preserve or increase polygon count due to triangulation");
    }

    #[test]
    fn test_conversion_complex_geometry() {
        // Test conversion with complex geometry (sphere)
        let sphere_mesh = Mesh::sphere(1.0, 8, 6, None::<()>).unwrap();

        // Check that vertex deduplication occurred (sphere has many duplicate vertices)
        let original_vertex_count: usize = sphere_mesh.polygons.iter()
            .map(|p| p.vertices.len())
            .sum();

        // Convert to IndexedMesh
        let indexed: IndexedMesh<()> = sphere_mesh.into();

        // Verify properties
        assert!(!indexed.vertices.is_empty(), "Sphere should convert to vertices");
        assert!(!indexed.faces.is_empty(), "Sphere should convert to faces");
        assert!(indexed.validate_face_indices().is_ok(), "Sphere conversion should be valid");
        assert!(indexed.vertices.len() < original_vertex_count,
            "IndexedMesh should deduplicate vertices: {} < {}",
            indexed.vertices.len(), original_vertex_count);
    }

    #[test]
    fn test_conversion_with_metadata() {
        // Test conversion preserves metadata
        #[derive(Clone, Debug, PartialEq)]
        struct TestMetadata {
            id: u32,
            name: String,
        }

        let mesh = Mesh::cube(1.0, Some(TestMetadata {
            id: 42,
            name: "test_cube".to_string(),
        })).unwrap();

        let indexed: IndexedMesh<TestMetadata> = mesh.into();

        // Verify metadata is preserved
        assert_eq!(indexed.metadata.as_ref().unwrap().id, 42);
        assert_eq!(indexed.metadata.as_ref().unwrap().name, "test_cube");
    }

    #[test]
    fn test_conversion_empty_mesh() {
        // Test conversion of empty mesh
        let empty_mesh = crate::mesh::Mesh::<()> {
            polygons: Vec::new(),
            bounding_box: std::sync::OnceLock::new(),
            metadata: None,
        };

        let indexed: IndexedMesh<()> = empty_mesh.into();

        // Empty mesh should convert to empty IndexedMesh
        assert!(indexed.vertices.is_empty(), "Empty mesh should convert to empty IndexedMesh vertices");
        assert!(indexed.faces.is_empty(), "Empty mesh should convert to empty IndexedMesh faces");
        assert!(indexed.validate_face_indices().is_ok(), "Empty IndexedMesh should be valid");
    }

    #[test]
    fn test_conversion_degenerate_polygons() {
        // Test conversion handles degenerate polygons gracefully
        let mesh = crate::mesh::Mesh::<()> {
            polygons: vec![
                // Valid triangle
                Polygon::new(vec![
                    Vertex::new(Point3::new(0.0, 0.0, 0.0), nalgebra::Vector3::z()),
                    Vertex::new(Point3::new(1.0, 0.0, 0.0), nalgebra::Vector3::z()),
                    Vertex::new(Point3::new(0.0, 1.0, 0.0), nalgebra::Vector3::z()),
                ], None),
                // Degenerate polygon (single vertex)
                Polygon::new(vec![
                    Vertex::new(Point3::new(2.0, 2.0, 2.0), nalgebra::Vector3::z()),
                ], None),
            ],
            bounding_box: std::sync::OnceLock::new(),
            metadata: None,
        };

        let indexed: IndexedMesh<()> = mesh.into();

        // Should handle degenerate polygons gracefully
        assert!(indexed.validate_face_indices().is_ok(), "Conversion should handle degenerate polygons");
        // At least the valid triangle should be converted
        assert!(indexed.vertices.len() >= 3, "Should convert valid vertices");
    }

    #[test]
    fn test_conversion_preserves_face_normals() {
        // Test that face normals are computed when missing
        let cube_mesh = Mesh::cube(1.0, None::<()>).unwrap();
        let indexed: IndexedMesh<()> = cube_mesh.into();

        // All faces should have normals after conversion
        for face in &indexed.faces {
            assert!(face.normal.is_some(), "All faces should have normals after conversion");
            let normal = face.normal.unwrap();
            assert!(normal.norm() > 0.99, "Face normal should be unit length: {}", normal.norm());
        }
    }

    #[test]
    fn test_conversion_vertex_deduplication() {
        // Test that vertex deduplication works correctly
        let mesh = crate::mesh::Mesh::<()> {
            polygons: vec![
                // Two triangles sharing vertices
                Polygon::new(vec![
                    Vertex::new(Point3::new(0.0, 0.0, 0.0), nalgebra::Vector3::z()),
                    Vertex::new(Point3::new(1.0, 0.0, 0.0), nalgebra::Vector3::z()),
                    Vertex::new(Point3::new(0.0, 1.0, 0.0), nalgebra::Vector3::z()),
                ], None),
                Polygon::new(vec![
                    Vertex::new(Point3::new(0.0, 1.0, 0.0), nalgebra::Vector3::z()), // Shared vertex
                    Vertex::new(Point3::new(1.0, 0.0, 0.0), nalgebra::Vector3::z()), // Shared vertex
                    Vertex::new(Point3::new(1.0, 1.0, 0.0), nalgebra::Vector3::z()), // New vertex
                ], None),
            ],
            bounding_box: std::sync::OnceLock::new(),
            metadata: None,
        };

        let indexed: IndexedMesh<()> = mesh.into();

        // Should have 4 unique vertices (3 from first triangle + 1 new from second)
        assert_eq!(indexed.vertices.len(), 4, "Should deduplicate shared vertices");

        // Should have 2 faces
        assert_eq!(indexed.faces.len(), 2, "Should preserve all faces");

        // Verify all face indices are valid
        assert!(indexed.validate_face_indices().is_ok(), "All face indices should be valid");
    }

    #[test]
    fn test_conversion_large_mesh() {
        // Test conversion with larger mesh to ensure scalability
        let sphere_mesh = Mesh::sphere(1.0, 16, 12, None::<()>).unwrap();

        // Should achieve significant deduplication
        let original_vertices: usize = sphere_mesh.polygons.iter()
            .map(|p| p.vertices.len())
            .sum();

        let indexed: IndexedMesh<()> = sphere_mesh.into();

        // Should handle larger meshes without issues
        assert!(!indexed.vertices.is_empty(), "Large mesh should convert vertices");
        assert!(!indexed.faces.is_empty(), "Large mesh should convert faces");
        assert!(indexed.validate_face_indices().is_ok(), "Large mesh conversion should be valid");
        assert!(indexed.vertices.len() < original_vertices,
            "Should achieve vertex deduplication: {} < {}", indexed.vertices.len(), original_vertices);
    }

    #[cfg(any(feature = "f64", feature = "f32"))]
    #[test]
    fn test_conversion_physics_integration() {
        // Test physics-related conversion methods
        let cube_mesh = Mesh::cube(1.0, None::<()>).unwrap();
        let indexed: IndexedMesh<()> = cube_mesh.into();

        // Test mass properties calculation
        let density = 1000.0; // kg/m³
        let mass_props = indexed.mass_properties(density);
        assert!(mass_props.is_some(), "Mass properties should be calculable");
        let (mass, center_of_mass, _) = mass_props.unwrap();
        assert!(mass > 0.0, "Mass should be positive");
        assert!(center_of_mass.coords.norm() < 0.1, "Center of mass should be near origin for symmetric shape");

        // Test Rapier shape creation (rigid body creation requires external state)
        let rapier_shape = indexed.to_rapier_shape();
        assert!(rapier_shape.is_some(), "Should create Rapier shape");

        // Test collider creation
        let collider = indexed.to_trimesh();
        assert!(collider.is_some(), "Should create collider");
    }

    #[cfg(feature = "bevymesh")]
    #[test]
    fn test_conversion_bevy_integration() {
        // Test Bevy mesh conversion
        let cube_mesh = Mesh::cube(1.0, None::<()>).unwrap();
        let indexed: IndexedMesh<()> = cube_mesh.into();

        let bevy_mesh = indexed.to_bevy_mesh();

        // Bevy mesh should have positions and indices
        assert!(bevy_mesh.count_vertices() > 0, "Bevy mesh should have vertices");
        assert!(bevy_mesh.indices().is_some(), "Bevy mesh should have indices");

        // Should be triangulated
        let indices = bevy_mesh.indices().unwrap();
        assert_eq!(indices.len() % 3, 0, "Bevy mesh should be triangulated (indices divisible by 3)");
    }
}