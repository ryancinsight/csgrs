//! Vertex deduplication algorithms for IndexedMesh
//!
//! This module provides efficient algorithms for removing duplicate vertices
//! from mesh data while maintaining topological consistency.

use crate::float_types::Real;
use crate::mesh::vertex::Vertex;
use nalgebra::Point3;
use std::collections::HashMap;

/// Deduplicate vertices using spatial hashing for efficiency
///
/// Returns deduplicated vertices and a mapping from original indices to new indices
pub fn deduplicate_vertices(
    vertices: &[Vertex],
    epsilon: Real,
) -> (Vec<Vertex>, HashMap<usize, usize>) {
    let mut deduplicated = Vec::new();
    let mut index_map = HashMap::new();
    let mut spatial_map = HashMap::new();

    for (original_idx, vertex) in vertices.iter().enumerate() {
        // Create spatial key for hashing
        let key = spatial_key(&vertex.pos, epsilon);

        if let Some(&existing_idx) = spatial_map.get(&key) {
            // Vertex already exists, map to existing index
            index_map.insert(original_idx, existing_idx);
        } else {
            // New vertex, add it and create mapping
            let new_idx = deduplicated.len();
            deduplicated.push(*vertex);
            spatial_map.insert(key, new_idx);
            index_map.insert(original_idx, new_idx);
        }
    }

    (deduplicated, index_map)
}

/// Create spatial hash key for vertex deduplication
fn spatial_key(pos: &Point3<Real>, epsilon: Real) -> (i64, i64, i64) {
    let scale = 1.0 / epsilon;
    (
        (pos.x * scale).round() as i64,
        (pos.y * scale).round() as i64,
        (pos.z * scale).round() as i64,
    )
}

/// Deduplicate vertices with exact comparison (no epsilon)
///
/// This is faster for cases where vertices are expected to be exactly identical
pub fn deduplicate_vertices_exact(
    vertices: &[Vertex],
) -> (Vec<Vertex>, HashMap<usize, usize>) {
    let mut deduplicated = Vec::new();
    let mut index_map = HashMap::new();
    let mut vertex_map = HashMap::new();

    for (original_idx, vertex) in vertices.iter().enumerate() {
        if let Some(&existing_idx) = vertex_map.get(vertex) {
            // Vertex already exists, map to existing index
            index_map.insert(original_idx, existing_idx);
        } else {
            // New vertex, add it and create mapping
            let new_idx = deduplicated.len();
            deduplicated.push(*vertex);
            vertex_map.insert(*vertex, new_idx);
            index_map.insert(original_idx, new_idx);
        }
    }

    (deduplicated, index_map)
}

/// Statistics about vertex deduplication process
#[derive(Debug, Clone)]
pub struct DeduplicationStats {
    /// Number of vertices before deduplication
    pub original_count: usize,
    /// Number of vertices after deduplication
    pub deduplicated_count: usize,
    /// Number of duplicate vertices removed
    pub duplicates_removed: usize,
    /// Memory savings percentage (0.0 to 1.0)
    pub memory_savings: f64,
}

impl DeduplicationStats {
    /// Create statistics from deduplication results
    pub fn new(original_count: usize, deduplicated_count: usize) -> Self {
        let duplicates_removed = original_count.saturating_sub(deduplicated_count);
        let memory_savings = if original_count > 0 {
            duplicates_removed as f64 / original_count as f64
        } else {
            0.0
        };

        Self {
            original_count,
            deduplicated_count,
            duplicates_removed,
            memory_savings,
        }
    }
}

/// Deduplicate vertices and return statistics
pub fn deduplicate_with_stats(
    vertices: &[Vertex],
    epsilon: Real,
) -> (Vec<Vertex>, HashMap<usize, usize>, DeduplicationStats) {
    let original_count = vertices.len();
    let (deduplicated, index_map) = deduplicate_vertices(vertices, epsilon);
    let stats = DeduplicationStats::new(original_count, deduplicated.len());

    (deduplicated, index_map, stats)
}

/// Merge duplicate vertices in an existing mesh
///
/// This function identifies vertices that are within epsilon distance
/// and merges them, updating all face indices accordingly
pub fn merge_duplicate_vertices(
    vertices: &mut Vec<Vertex>,
    faces: &mut [Vec<usize>],
    epsilon: Real,
) -> DeduplicationStats {
    let _original_count = vertices.len();

    // Create temporary copies for deduplication
    let temp_vertices = vertices.clone();
    let (deduplicated, index_map, stats) = deduplicate_with_stats(&temp_vertices, epsilon);

    // Update vertices
    *vertices = deduplicated;

    // Update face indices
    for face in faces.iter_mut() {
        for vertex_idx in face.iter_mut() {
            if let Some(&new_idx) = index_map.get(vertex_idx) {
                *vertex_idx = new_idx;
            }
        }
    }

    stats
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::vertex::Vertex;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_deduplicate_identical_vertices() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // duplicate
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        ];

        let (deduplicated, index_map) = deduplicate_vertices_exact(&vertices);

        assert_eq!(deduplicated.len(), 2);
        assert_eq!(*index_map.get(&0).unwrap(), 0);
        assert_eq!(*index_map.get(&1).unwrap(), 0); // mapped to first vertex
        assert_eq!(*index_map.get(&2).unwrap(), 1);
    }

    #[test]
    fn test_deduplicate_with_epsilon() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.000000001, 0.0, 0.0), Vector3::z()), // within epsilon
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        ];

        let (deduplicated, index_map) = deduplicate_vertices(&vertices, 1e-8);

        assert_eq!(deduplicated.len(), 2);
        assert_eq!(*index_map.get(&0).unwrap(), 0);
        assert_eq!(*index_map.get(&1).unwrap(), 0); // mapped to first vertex
        assert_eq!(*index_map.get(&2).unwrap(), 1);
    }

    #[test]
    fn test_deduplication_stats() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // duplicate
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // another duplicate
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        ];

        let (_deduplicated, _, stats) = deduplicate_with_stats(&vertices, 1e-8);

        assert_eq!(stats.original_count, 4);
        assert_eq!(stats.deduplicated_count, 2);
        assert_eq!(stats.duplicates_removed, 2);
        assert_eq!(stats.memory_savings, 0.5);
    }
}
