//! Core IndexedMesh data structures and basic implementations
//!
//! This module contains the fundamental data structures and basic implementations
//! for IndexedMesh, maintaining modularity and separation of concerns.

use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::{fmt::Debug, sync::OnceLock};

/// Vertex deduplication precision for floating-point comparison
pub const DEDUP_EPSILON: Real = 1e-8;

/// Face representation using vertex indices
#[derive(Clone, Debug, PartialEq)]
pub struct IndexedFace {
    /// Indices into the vertex array
    pub vertices: Vec<usize>,
    /// Optional normal vector for the face
    pub normal: Option<Vector3<Real>>,
    /// Optional metadata for the face
    pub metadata: Option<IndexedMetadata>,
}

/// Metadata associated with indexed mesh elements
#[derive(Clone, Debug, PartialEq)]
pub enum IndexedMetadata {
    /// Face-level metadata
    Face(String),
    /// Vertex-level metadata
    Vertex(String),
    /// Edge-level metadata
    Edge(String),
}

/// Adjacency information for efficient connectivity queries
#[derive(Clone, Debug)]
pub struct AdjacencyInfo {
    /// Maps vertex index to list of adjacent vertex indices
    pub vertex_adjacency: Vec<Vec<usize>>,
    /// Maps vertex index to list of face indices that contain this vertex
    pub vertex_faces: Vec<Vec<usize>>,
    /// Maps face index to list of adjacent face indices (sharing edges)
    pub face_adjacency: Vec<Vec<usize>>,
    /// Maps face index to list of vertex indices in this face
    pub face_vertices: Vec<Vec<usize>>,
}

/// Core IndexedMesh data structure with vertex deduplication and face indexing
#[derive(Clone, Debug)]
pub struct IndexedMesh<S: Clone + Send + Sync + Debug> {
    /// Deduplicated vertices - each vertex appears exactly once
    pub vertices: Vec<Vertex>,
    /// Faces represented as indices into the vertices array
    pub faces: Vec<IndexedFace>,
    /// Pre-computed adjacency information for efficient queries
    pub adjacency: OnceLock<AdjacencyInfo>,
    /// Lazily calculated AABB that spans all vertices
    pub bounding_box: OnceLock<Aabb>,
    /// Optional mesh-level metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> Default for IndexedMesh<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> IndexedMesh<S> {
    /// Create a new empty IndexedMesh
    pub const fn new() -> Self {
        IndexedMesh {
            vertices: Vec::new(),
            faces: Vec::new(),
            adjacency: OnceLock::new(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Create IndexedMesh from vertices and faces with automatic deduplication
    pub fn from_vertices_and_faces(
        vertices: Vec<Point3<Real>>,
        faces: Vec<Vec<usize>>,
        metadata: Option<S>,
    ) -> Self {
        use super::deduplication;

        let mut mesh = Self::new();
        mesh.metadata = metadata;

        // Convert points to vertices
        let vertex_objects: Vec<Vertex> = vertices
            .into_iter()
            .map(|pos| {
                // For now, use computed normals later
                Vertex::new(pos, Vector3::z())
            })
            .collect();

        // Deduplicate vertices and remap face indices
        let (deduplicated_vertices, index_map) =
            deduplication::deduplicate_vertices(&vertex_objects, DEDUP_EPSILON);

        // Remap face indices and create indexed faces
        let mut indexed_faces = Vec::new();
        for face_indices in faces {
            let remapped_indices: Vec<usize> = face_indices
                .iter()
                .map(|&idx| *index_map.get(&idx).unwrap_or(&0))
                .collect();

            let indexed_face = IndexedFace {
                vertices: remapped_indices,
                normal: None, // Will be computed later
                metadata: None,
            };
            indexed_faces.push(indexed_face);
        }

        mesh.vertices = deduplicated_vertices;
        mesh.faces = indexed_faces;

        mesh
    }

    /// Compute normals only for faces that don't already have them
    pub fn compute_face_normals_fallback(&mut self) {
        // First pass: collect which faces need normals computed
        let faces_to_update: Vec<(usize, Option<Vector3<Real>>)> = self
            .faces
            .iter()
            .enumerate()
            .filter_map(|(idx, face)| {
                if face.normal.is_none() && face.vertices.len() >= 3 {
                    Some((idx, self.compute_face_normal(&face.vertices)))
                } else {
                    None
                }
            })
            .collect();

        // Second pass: update the faces
        for (face_idx, normal) in faces_to_update {
            if let Some(face) = self.faces.get_mut(face_idx) {
                face.normal = normal;
            }
        }
    }

    /// Compute normal for a single face using robust triangulation
    pub fn compute_face_normal(&self, vertex_indices: &[usize]) -> Option<Vector3<Real>> {
        if vertex_indices.len() < 3 {
            return None;
        }

        // For triangular faces, use simple cross product method (most reliable)
        if vertex_indices.len() == 3 {
            let v0 = self.vertices.get(vertex_indices[0])?.pos;
            let v1 = self.vertices.get(vertex_indices[1])?.pos;
            let v2 = self.vertices.get(vertex_indices[2])?.pos;

            // Compute vectors from first vertex
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;

            // Cross product gives normal
            let normal = edge1.cross(&edge2);

            let length = normal.norm();
            if length > DEDUP_EPSILON {
                return Some(normal / length);
            } else {
                return None;
            }
        }

        // For polygons with more than 3 vertices, use Newell's method
        // This is more numerically stable for complex polygons
        let mut normal: Vector3<Real> = Vector3::zeros();

        for i in 0..vertex_indices.len() {
            let current = self.vertices.get(vertex_indices[i])?;
            let next = self
                .vertices
                .get(vertex_indices[(i + 1) % vertex_indices.len()])?;

            // Newell's method: only uses coordinate differences, very robust
            normal.x += (current.pos.y - next.pos.y) * (current.pos.z + next.pos.z);
            normal.y += (current.pos.z - next.pos.z) * (current.pos.x + next.pos.x);
            normal.z += (current.pos.x - next.pos.x) * (current.pos.y + next.pos.y);
        }

        let length = normal.norm();
        if length > DEDUP_EPSILON {
            Some(normal / length)
        } else {
            None
        }
    }

    /// Triangulate a face into triangles (fan triangulation)
    pub fn triangulate_face(&self, face_vertices: &[usize]) -> Vec<Vec<usize>> {
        if face_vertices.len() < 3 {
            return Vec::new();
        }

        let mut triangles = Vec::new();

        // Simple fan triangulation from first vertex
        for i in 1..face_vertices.len() - 1 {
            triangles.push(vec![face_vertices[0], face_vertices[i], face_vertices[i + 1]]);
        }

        triangles
    }

    /// Validate that all face indices are within vertex bounds
    pub fn validate_face_indices(&self) -> Result<(), String> {
        for (face_idx, face) in self.faces.iter().enumerate() {
            for &vertex_idx in &face.vertices {
                if vertex_idx >= self.vertices.len() {
                    return Err(format!(
                        "Face {} references vertex index {} but only {} vertices exist",
                        face_idx,
                        vertex_idx,
                        self.vertices.len()
                    ));
                }
            }
        }
        Ok(())
    }

    /// Get reference to adjacency information, computing it if necessary
    pub fn adjacency(&self) -> &AdjacencyInfo {
        self.adjacency.get_or_init(|| {
            super::adjacency::compute_adjacency(self)
        })
    }

    /// Check if the mesh is manifold
    pub fn is_manifold(&self) -> bool {
        super::adjacency::analyze_manifold(self).is_manifold
    }

    /// Get faces adjacent to a given face
    pub fn get_face_adjacency(&self, face_idx: usize) -> Option<&Vec<usize>> {
        self.adjacency().face_adjacency.get(face_idx)
    }

    /// Get vertices adjacent to a given vertex
    pub fn get_vertex_adjacency(&self, vertex_idx: usize) -> Option<&Vec<usize>> {
        self.adjacency().vertex_adjacency.get(vertex_idx)
    }

    /// Get faces that contain a given vertex
    pub fn get_vertex_faces(&self, vertex_idx: usize) -> Option<&Vec<usize>> {
        self.adjacency().vertex_faces.get(vertex_idx)
    }

    /// Get vertices that belong to a given face
    pub fn get_face_vertices(&self, face_idx: usize) -> Option<&Vec<usize>> {
        if face_idx < self.faces.len() {
            Some(&self.faces[face_idx].vertices)
        } else {
            None
        }
    }

    /// Convert IndexedMesh to standard Mesh representation
    pub fn to_mesh(&self) -> crate::mesh::Mesh<S> {
        use crate::mesh::{Mesh, polygon::Polygon};
        use crate::mesh::plane::Plane;

        let polygons = self.faces.iter().map(|face| {
            let vertices: Vec<_> = face.vertices.iter()
                .map(|&idx| self.vertices[idx])
                .collect();

            // Use stored face normal if available, otherwise compute from vertices
            let plane = if let Some(face_normal) = face.normal {
                // Compute offset as -normal · point for point on plane
                let offset = -face_normal.dot(&vertices[0].pos.coords);
                Plane::from_normal(face_normal, offset)
            } else if vertices.len() >= 3 {
                Plane::from_vertices(vertices.clone())
            } else {
                // Fallback plane for degenerate faces
                Plane::from_normal(Vector3::z(), 0.0)
            };

            Polygon {
                vertices,
                plane,
                bounding_box: std::sync::OnceLock::new(),
                metadata: None, // IndexedMesh face metadata is different type
            }
        }).collect();

        Mesh {
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Create IndexedMesh from vertices with normals and faces
    pub fn from_vertices_with_normals_and_faces(
        vertices: Vec<(Point3<Real>, Vector3<Real>)>,
        faces: Vec<Vec<usize>>,
        metadata: Option<S>,
    ) -> Self {
        let mut mesh = Self::new();
        mesh.metadata = metadata;

        // Convert to Vertex structs and deduplicate
        let vertex_structs: Vec<crate::mesh::vertex::Vertex> = vertices
            .into_iter()
            .map(|(pos, normal)| crate::mesh::vertex::Vertex::new(pos, normal))
            .collect();

        // Deduplicate vertices and remap face indices
        let (deduplicated_vertices, index_map) =
            super::deduplication::deduplicate_vertices(&vertex_structs, DEDUP_EPSILON);

        // Remap face indices and create indexed faces with preserved normals
        let mut indexed_faces = Vec::new();
        for face_indices in faces {
            let remapped_indices: Vec<usize> = face_indices
                .iter()
                .map(|&idx| *index_map.get(&idx).unwrap_or(&0))
                .collect();

            let indexed_face = IndexedFace {
                vertices: remapped_indices,
                normal: None, // Will be computed from vertex normals if needed
                metadata: None,
            };
            indexed_faces.push(indexed_face);
        }

        mesh.vertices = deduplicated_vertices;
        mesh.faces = indexed_faces;

        mesh
    }
}
