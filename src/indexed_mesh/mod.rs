//! `IndexedMesh` struct and implementations of the `CSGOps` trait for `IndexedMesh`
//!
//! IndexedMesh provides memory-efficient mesh representation through vertex deduplication
//! and face indexing. This module implements all CSG operations with automatic vertex
//! deduplication to minimize memory usage while maintaining topological consistency.

use crate::float_types::{
    Real,
    parry3d::bounding_volume::Aabb,
};
use crate::mesh::{polygon::Polygon, vertex::Vertex};
use crate::traits::CSG;
use nalgebra::{Matrix4, Point3, Vector3, partial_max, partial_min};
use std::{cmp::PartialEq, fmt::Debug, sync::OnceLock};

// I/O functionality is now consolidated in the main io module for SSOT compliance

pub mod adjacency;
pub mod deduplication;
pub mod operations;
pub mod shapes;
pub mod topology;

/// Vertex deduplication precision for floating-point comparison
const DEDUP_EPSILON: Real = 1e-8;

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
        let (deduplicated_vertices, index_map) = deduplication::deduplicate_vertices(&vertex_objects, DEDUP_EPSILON);

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

        // Compute face normals
        mesh.compute_face_normals();

        mesh
    }

    /// Compute normals for all faces based on vertex positions
    fn compute_face_normals(&mut self) {
        let face_normals: Vec<Option<Vector3<Real>>> = self.faces
            .iter()
            .map(|face| {
                if face.vertices.len() >= 3 {
                    self.compute_face_normal(&face.vertices)
                } else {
                    None
                }
            })
            .collect();

        for (face, normal) in self.faces.iter_mut().zip(face_normals) {
            face.normal = normal;
        }
    }

    /// Compute normal for a single face
    pub fn compute_face_normal(&self, vertex_indices: &[usize]) -> Option<Vector3<Real>> {
        if vertex_indices.len() < 3 {
            return None;
        }

        // Use Newell's method for robust normal computation
        let mut normal: Vector3<Real> = Vector3::zeros();

        for i in 0..vertex_indices.len() {
            let current = self.vertices.get(vertex_indices[i])?;
            let next = self.vertices.get(vertex_indices[(i + 1) % vertex_indices.len()])?;

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
            triangles.push(vec![
                face_vertices[0],
                face_vertices[i],
                face_vertices[i + 1],
            ]);
        }

        triangles
    }

    /// Validate that all face indices are within vertex bounds
    pub fn validate_face_indices(&self) -> Result<(), String> {
        let vertex_count = self.vertices.len();

        for (face_idx, face) in self.faces.iter().enumerate() {
            for &vertex_idx in &face.vertices {
                if vertex_idx >= vertex_count {
                    return Err(format!(
                        "Face {} references vertex index {} but only {} vertices exist",
                        face_idx, vertex_idx, vertex_count
                    ));
                }
            }

            if face.vertices.len() < 3 {
                return Err(format!(
                    "Face {} has only {} vertices (minimum 3 required)",
                    face_idx, face.vertices.len()
                ));
            }
        }

        Ok(())
    }

    /// Check if the mesh is manifold (each edge has exactly two faces)
    pub fn is_manifold(&self) -> bool {
        use crate::indexed_mesh::adjacency::analyze_manifold;
        let analysis = analyze_manifold(self);
        analysis.is_manifold
    }

    /// Get adjacency information, computing it lazily if needed
    pub fn adjacency(&self) -> &AdjacencyInfo {
        self.adjacency.get_or_init(|| self.compute_adjacency())
    }

    /// Compute adjacency information for the mesh
    fn compute_adjacency(&self) -> AdjacencyInfo {
        let mut vertex_adjacency = vec![Vec::new(); self.vertices.len()];
        let mut vertex_faces = vec![Vec::new(); self.vertices.len()];
        let mut face_adjacency = vec![Vec::new(); self.faces.len()];
        let mut face_vertices = vec![Vec::new(); self.faces.len()];

        // Build vertex-to-face and face-to-vertex mappings
        for (face_idx, face) in self.faces.iter().enumerate() {
            face_vertices[face_idx] = face.vertices.clone();
            for &vertex_idx in &face.vertices {
                if vertex_idx < vertex_faces.len() {
                    vertex_faces[vertex_idx].push(face_idx);
                }
            }
        }

        // Build vertex adjacency (vertices sharing faces)
        for (vertex_idx, faces) in vertex_faces.iter().enumerate() {
            let mut adjacent_vertices = std::collections::HashSet::new();
            for &face_idx in faces {
                if let Some(face) = self.faces.get(face_idx) {
                    for &other_vertex in &face.vertices {
                        if other_vertex != vertex_idx {
                            adjacent_vertices.insert(other_vertex);
                        }
                    }
                }
            }
            vertex_adjacency[vertex_idx] = adjacent_vertices.into_iter().collect();
        }

        // Build face adjacency (faces sharing edges)
        for i in 0..self.faces.len() {
            let mut adjacent_faces = std::collections::HashSet::new();
            let face_i = &self.faces[i];

            for j in (i + 1)..self.faces.len() {
                let face_j = &self.faces[j];

                // Check if faces share an edge (two common vertices)
                let common_vertices: std::collections::HashSet<_> =
                    face_i.vertices.iter().collect();
                let common_count = face_j.vertices.iter()
                    .filter(|v| common_vertices.contains(v))
                    .count();

                if common_count >= 2 {
                    adjacent_faces.insert(j);
                    // Also add reverse adjacency
                    if face_adjacency.len() > j {
                        face_adjacency[j].push(i);
                    }
                }
            }

            face_adjacency[i] = adjacent_faces.into_iter().collect();
        }

        AdjacencyInfo {
            vertex_adjacency,
            vertex_faces,
            face_adjacency,
            face_vertices,
        }
    }

    /// Query vertex adjacency - get all vertices connected to the given vertex
    pub fn get_vertex_adjacency(&self, vertex_idx: usize) -> Option<&[usize]> {
        self.adjacency().vertex_adjacency.get(vertex_idx)
            .map(|v| v.as_slice())
    }

    /// Query face adjacency - get all faces adjacent to the given face
    pub fn get_face_adjacency(&self, face_idx: usize) -> Option<&[usize]> {
        self.adjacency().face_adjacency.get(face_idx)
            .map(|v| v.as_slice())
    }

    /// Get all faces containing a specific vertex
    pub fn get_vertex_faces(&self, vertex_idx: usize) -> Option<&[usize]> {
        self.adjacency().vertex_faces.get(vertex_idx)
            .map(|v| v.as_slice())
    }

    /// Get all vertices in a specific face
    pub fn get_face_vertices(&self, face_idx: usize) -> Option<&[usize]> {
        self.adjacency().face_vertices.get(face_idx)
            .map(|v| v.as_slice())
    }

    /// Convert IndexedMesh to standard Mesh representation
    pub fn to_mesh(&self) -> crate::mesh::Mesh<S> {
        let mut polygons = Vec::new();

        for face in &self.faces {
            let mut vertices = Vec::new();
            for &vertex_idx in &face.vertices {
                if let Some(vertex) = self.vertices.get(vertex_idx) {
                    vertices.push(*vertex);
                }
            }

            if vertices.len() >= 3 {
                let polygon = Polygon::new(vertices, self.metadata.clone());
                polygons.push(polygon);
            }
        }

        crate::mesh::Mesh {
            polygons,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> From<crate::mesh::Mesh<S>> for IndexedMesh<S> {
    /// Convert standard Mesh to IndexedMesh with automatic deduplication
    fn from(mesh: crate::mesh::Mesh<S>) -> Self {
        let mut vertices = Vec::new();
        let mut faces = Vec::new();

        // Extract all vertices and build face indices
        for polygon in &mesh.polygons {
            let mut face_indices = Vec::new();

            for vertex in &polygon.vertices {
                // Find existing vertex or add new one
                let vertex_idx = vertices.iter().position(|v: &Vertex| {
                    (v.pos - vertex.pos).norm() < DEDUP_EPSILON
                });

                let vertex_idx = match vertex_idx {
                    Some(idx) => idx,
                    None => {
                        vertices.push(*vertex);
                        vertices.len() - 1
                    }
                };

                face_indices.push(vertex_idx);
            }

            let indexed_face = IndexedFace {
                vertices: face_indices,
                normal: None,
                metadata: None,
            };
            faces.push(indexed_face);
        }

        let mut indexed_mesh = IndexedMesh {
            vertices,
            faces,
            adjacency: OnceLock::new(),
            bounding_box: OnceLock::new(),
            metadata: mesh.metadata,
        };

        // Compute face normals
        indexed_mesh.compute_face_normals();

        indexed_mesh
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for IndexedMesh<S> {
    /// Returns a new empty IndexedMesh
    fn new() -> Self {
        IndexedMesh::new()
    }

    /// Return a new IndexedMesh representing union of the two IndexedMeshes
    fn union(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        operations::union(self, other)
    }

    /// Return a new IndexedMesh representing difference of the two IndexedMeshes
    fn difference(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        operations::difference(self, other)
    }

    /// Return a new IndexedMesh representing intersection of the two IndexedMeshes
    fn intersection(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        operations::intersection(self, other)
    }

    /// Return a new IndexedMesh representing XOR of the two IndexedMeshes
    fn xor(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        operations::xor(self, other)
    }

    /// Apply transformation to IndexedMesh
    fn transform(&self, matrix: &Matrix4<Real>) -> IndexedMesh<S> {
        operations::transform(self, matrix)
    }

    /// Invert the IndexedMesh (flip inside vs. outside)
    fn inverse(&self) -> IndexedMesh<S> {
        operations::inverse(self)
    }

    /// Returns bounding box of the IndexedMesh
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            if self.vertices.is_empty() {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            for vertex in &self.vertices {
                let pos = vertex.pos;

                if let Some(new_min_x) = partial_min(&min_x, &pos.x) {
                    min_x = *new_min_x;
                }
                if let Some(new_min_y) = partial_min(&min_y, &pos.y) {
                    min_y = *new_min_y;
                }
                if let Some(new_min_z) = partial_min(&min_z, &pos.z) {
                    min_z = *new_min_z;
                }

                if let Some(new_max_x) = partial_max(&max_x, &pos.x) {
                    max_x = *new_max_x;
                }
                if let Some(new_max_y) = partial_max(&max_y, &pos.y) {
                    max_y = *new_max_y;
                }
                if let Some(new_max_z) = partial_max(&max_z, &pos.z) {
                    max_z = *new_max_z;
                }
            }

            if min_x > max_x {
                Aabb::new(Point3::origin(), Point3::origin())
            } else {
                let mins = Point3::new(min_x, min_y, min_z);
                let maxs = Point3::new(max_x, max_y, max_z);
                Aabb::new(mins, maxs)
            }
        })
    }

    /// Invalidate cached bounding box
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
        self.adjacency = OnceLock::new(); // Adjacency may also be invalid
    }
}

// Re-export key types for external use
pub use self::adjacency::*;
pub use self::deduplication::*;
pub use self::operations::*;
pub use self::shapes::*;
pub use self::topology::*;
