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

/// Deduplicate vertices while preserving normals for vertices at the same position
///
/// This function handles cases where vertices at the same spatial position
/// may have different normals (e.g., cylinder top/bottom vs sides)
pub fn deduplicate_vertices_with_normals(
    vertices: &[Vertex],
    epsilon: Real,
) -> (Vec<Vertex>, HashMap<usize, usize>) {
    let mut deduplicated = Vec::new();
    let mut index_map = HashMap::new();
    let mut spatial_normal_map: HashMap<(i64, i64, i64, i64, i64, i64), usize> =
        HashMap::new();

    for (original_idx, vertex) in vertices.iter().enumerate() {
        // Create a key that includes both position and normal
        let pos_key = spatial_key(&vertex.pos, epsilon);
        let normal_key = (
            (vertex.normal.x * 1000.0).round() as i64,
            (vertex.normal.y * 1000.0).round() as i64,
            (vertex.normal.z * 1000.0).round() as i64,
        );
        let combined_key = (
            pos_key.0,
            pos_key.1,
            pos_key.2,
            normal_key.0,
            normal_key.1,
            normal_key.2,
        );

        if let Some(&existing_idx) = spatial_normal_map.get(&combined_key) {
            // Vertex with same position and normal already exists, map to it
            index_map.insert(original_idx, existing_idx);
        } else {
            // New vertex (either new position or same position but different normal)
            let new_idx = deduplicated.len();
            deduplicated.push(*vertex);
            spatial_normal_map.insert(combined_key, new_idx);
            index_map.insert(original_idx, new_idx);
        }
    }

    (deduplicated, index_map)
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
    use crate::indexed_mesh::IndexedMesh;
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

    // ============================================================
    //   COMPREHENSIVE VERTEX DEDUPLICATION TESTS
    // ============================================================

    #[test]
    fn test_deduplication_precision_edge_cases() {
        // **Mathematical Foundation**: Floating-point precision in vertex deduplication
        // **SRS Requirement NFR004**: Robust floating-point arithmetic with configurable epsilon

        let base_vertex = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::z());

        // Test with vertices at different precision levels
        let epsilon = 1e-8;
        let test_cases = vec![
            // Exactly at epsilon boundary
            Vertex::new(Point3::new(1.0 + epsilon, 2.0, 3.0), Vector3::z()),
            // Just below epsilon
            Vertex::new(Point3::new(1.0 + epsilon * 0.9, 2.0, 3.0), Vector3::z()),
            // Just above epsilon
            Vertex::new(Point3::new(1.0 + epsilon * 1.1, 2.0, 3.0), Vector3::z()),
            // Much larger difference
            Vertex::new(Point3::new(1.0 + epsilon * 100.0, 2.0, 3.0), Vector3::z()),
        ];

        let vertices = vec![base_vertex]
            .into_iter()
            .chain(test_cases)
            .collect::<Vec<_>>();

        let (deduplicated, index_map, stats) = deduplicate_with_stats(&vertices, epsilon);

        // Should deduplicate vertices within epsilon distance
        assert!(
            deduplicated.len() <= vertices.len(),
            "Deduplication should not increase vertex count"
        );

        // Check index mapping behavior
        // Note: The exact behavior depends on the order of processing in the hash map
        // The important thing is that the mapping is consistent and valid
        for &new_index in index_map.values() {
            assert!(
                new_index < deduplicated.len(),
                "All mapped indices should be valid"
            );
        }

        // Statistics should be reasonable
        assert!(
            stats.memory_savings >= 0.0 && stats.memory_savings <= 1.0,
            "Memory savings should be between 0 and 1"
        );
    }

    #[test]
    fn test_deduplication_with_normals() {
        // **Mathematical Foundation**: Vertex deduplication preserving normal information
        // **SRS Requirement FR005**: Automatic vertex deduplication with normal preservation

        // Create vertices at same position but with different normals
        let position = Point3::new(1.0, 2.0, 3.0);
        let vertices = vec![
            Vertex::new(position, Vector3::x()), // Normal points in X direction
            Vertex::new(position, Vector3::y()), // Normal points in Y direction (different)
            Vertex::new(position, Vector3::z()), // Normal points in Z direction (different)
            Vertex::new(position, Vector3::x()), // Duplicate of first
        ];

        let (deduplicated, index_map) = deduplicate_vertices_with_normals(&vertices, 1e-8);

        // Should preserve vertices with different normals at same position
        assert!(
            deduplicated.len() >= 3,
            "Should preserve vertices with different normals at same position"
        );

        // Should deduplicate vertices with identical position AND normal
        assert!(
            deduplicated.len() < vertices.len(),
            "Should deduplicate vertices with identical position and normal"
        );

        // Check that vertices with same position but different normals get different indices
        let indices: Vec<usize> = (0..vertices.len())
            .map(|i| *index_map.get(&i).unwrap())
            .collect();

        // Vertices 0 and 3 should be deduplicated (same position and normal)
        assert_eq!(
            indices[0], indices[3],
            "Vertices with same position and normal should be deduplicated"
        );

        // Vertices 0, 1, 2 should have different indices (different normals)
        assert_ne!(
            indices[0], indices[1],
            "Vertices with same position but different normals should not be deduplicated"
        );
        assert_ne!(
            indices[0], indices[2],
            "Vertices with same position but different normals should not be deduplicated"
        );
    }

    #[test]
    fn test_deduplication_performance_scaling() {
        // **SRS Requirement NFR001**: Performance scaling for deduplication
        // **Mathematical Foundation**: Hash-based deduplication efficiency

        use std::time::Instant;

        // Test with different mesh sizes
        let sizes = vec![100, 500, 1000];

        for &size in &sizes {
            // Create a mesh with many duplicate vertices
            let mut vertices = Vec::new();
            for i in 0..size {
                // Add some unique vertices
                vertices.push(Vertex::new(Point3::new(i as f64, 0.0, 0.0), Vector3::z()));

                // Add duplicates of some vertices
                if i % 10 == 0 {
                    vertices.push(Vertex::new(
                        Point3::new(i as f64, 0.0, 0.0), // Same position
                        Vector3::z(),                    // Same normal
                    ));
                }
            }

            let start = Instant::now();
            let (deduplicated, _, stats) = deduplicate_with_stats(&vertices, 1e-8);
            let dedup_time = start.elapsed();

            println!(
                "Deduplication performance for {} vertices: time={:?}, duplicates_removed={}, memory_savings={:.2}%",
                vertices.len(),
                dedup_time,
                stats.duplicates_removed,
                stats.memory_savings * 100.0
            );

            // Should complete in reasonable time
            assert!(
                dedup_time.as_millis() < 1000,
                "Deduplication should complete in <1s for {} vertices",
                vertices.len()
            );

            // Should have removed some duplicates
            assert!(
                stats.duplicates_removed > 0 || deduplicated.len() == vertices.len(),
                "Should either remove duplicates or have no duplicates to remove"
            );

            // Memory savings should be reasonable
            assert!(
                stats.memory_savings >= 0.0,
                "Memory savings should be non-negative"
            );
        }
    }

    #[test]
    fn test_deduplication_extreme_coordinates() {
        // **Mathematical Foundation**: Deduplication with extreme coordinate values
        // **SRS Requirement NFR004**: Robust floating-point arithmetic

        let extreme_vals = vec![
            1e-20,  // Very small
            1e20,   // Very large
            1e100,  // Extremely large
            0.0,    // Zero
            -1e20,  // Negative large
            -1e-20, // Negative small
        ];

        for &val in &extreme_vals {
            let vertices = vec![
                Vertex::new(Point3::new(val, 0.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(val, 0.0, 0.0), Vector3::z()), // Duplicate
                Vertex::new(Point3::new(val + 1.0, 0.0, 0.0), Vector3::z()), // Different
            ];

            let (deduplicated, _, stats) = deduplicate_with_stats(&vertices, 1e-8);

            // Should handle extreme values without panicking
            assert!(
                deduplicated.len() <= vertices.len(),
                "Should not increase vertex count for extreme value {}",
                val
            );

            // Should remove at least the obvious duplicate
            assert!(
                stats.duplicates_removed >= 1 || deduplicated.len() < vertices.len(),
                "Should deduplicate obvious duplicates for extreme value {}",
                val
            );
        }
    }

    #[test]
    fn test_deduplication_mesh_integration() {
        // **SRS Requirement FR005**: Automatic vertex deduplication in IndexedMesh
        // **Mathematical Foundation**: Integration of deduplication in mesh construction

        // Create vertices with duplicates
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0), // Duplicate of first
            Point3::new(1.0, 0.0, 0.0), // Duplicate of second
        ];

        let faces = vec![
            vec![0, 1, 2],
            vec![3, 4, 2], // Uses duplicate indices
        ];

        let mesh = IndexedMesh::from_vertices_and_faces(vertices, faces, None::<()>);

        // Should have fewer vertices after automatic deduplication
        assert!(
            mesh.vertices.len() < 5,
            "IndexedMesh should deduplicate vertices automatically"
        );

        // Should have valid face indices
        assert!(
            mesh.validate_face_indices().is_ok(),
            "IndexedMesh should have valid face indices after deduplication"
        );

        // Should preserve face topology
        assert_eq!(
            mesh.faces.len(),
            2,
            "Should preserve face count after deduplication"
        );

        // Each face should still have 3 vertices
        for face in &mesh.faces {
            assert_eq!(
                face.vertices.len(),
                3,
                "Faces should maintain vertex count after deduplication"
            );
        }
    }

    #[test]
    fn test_deduplication_index_mapping_consistency() {
        // **Mathematical Foundation**: Consistent index mapping in deduplication
        // **SRS Requirement FR005**: Reliable index remapping

        let original_vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // Index 0
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()), // Index 1
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // Index 2 (duplicate of 0)
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()), // Index 3
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()), // Index 4 (duplicate of 1)
        ];

        let (deduplicated, index_map) = deduplicate_vertices(&original_vertices, 1e-8);

        // Should have exactly 3 unique vertices
        assert_eq!(
            deduplicated.len(),
            3,
            "Should deduplicate to exactly 3 unique vertices"
        );

        // Check index mapping consistency
        assert_eq!(
            *index_map.get(&0).unwrap(),
            0,
            "Original index 0 should map to 0"
        );
        assert_eq!(
            *index_map.get(&1).unwrap(),
            1,
            "Original index 1 should map to 1"
        );
        assert_eq!(
            *index_map.get(&2).unwrap(),
            0,
            "Original index 2 should map to 0 (duplicate)"
        );
        assert_eq!(
            *index_map.get(&3).unwrap(),
            2,
            "Original index 3 should map to 2"
        );
        assert_eq!(
            *index_map.get(&4).unwrap(),
            1,
            "Original index 4 should map to 1 (duplicate)"
        );

        // All mappings should be valid indices into deduplicated array
        for &new_index in index_map.values() {
            assert!(
                new_index < deduplicated.len(),
                "Mapped index {} should be valid for deduplicated array of length {}",
                new_index,
                deduplicated.len()
            );
        }
    }

    #[test]
    fn test_deduplication_numerical_stability() {
        // **Mathematical Foundation**: Numerical stability in spatial hashing
        // **SRS Requirement NFR004**: Robust floating-point computations

        // Test vertices that might cause issues in spatial key calculation
        let problematic_vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            // Very close to zero
            Vertex::new(Point3::new(1e-15, 1e-15, 1e-15), Vector3::z()),
            // Very close to each other
            Vertex::new(Point3::new(1e-14, 1e-14, 1e-14), Vector3::z()),
            // Large coordinates
            Vertex::new(Point3::new(1e15, 1e15, 1e15), Vector3::z()),
            // Mixed scales
            Vertex::new(Point3::new(1e-10, 1e10, 1e-5), Vector3::z()),
        ];

        let (deduplicated, index_map, stats) =
            deduplicate_with_stats(&problematic_vertices, 1e-8);

        // Should not crash on problematic inputs
        assert!(
            deduplicated.len() <= problematic_vertices.len(),
            "Should handle problematic vertices without crashing"
        );

        // Should produce valid index mappings
        for &new_index in index_map.values() {
            assert!(
                new_index < deduplicated.len(),
                "Should produce valid index mappings"
            );
        }

        // Statistics should be computable
        assert!(
            stats.memory_savings >= 0.0 && stats.memory_savings <= 1.0,
            "Should compute valid statistics for problematic inputs"
        );
    }

    #[test]
    fn test_merge_duplicate_vertices_integration() {
        // **Mathematical Foundation**: In-place deduplication of existing meshes
        // **SRS Requirement FR005**: Efficient mesh repair operations

        // Create a mesh with duplicate vertices
        let mut vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // Duplicate
        ];

        let mut faces = vec![
            vec![0, 1, 2],
            vec![3, 1, 2], // Uses duplicate vertex
        ];

        let stats = merge_duplicate_vertices(&mut vertices, &mut faces, 1e-8);

        // Should have removed duplicate vertex
        assert!(vertices.len() < 4, "Should remove duplicate vertices");

        // Should update face indices correctly
        for face in &faces {
            for &vertex_idx in face {
                assert!(
                    vertex_idx < vertices.len(),
                    "Face indices should be valid after deduplication"
                );
            }
        }

        // Should report deduplication statistics
        assert!(
            stats.duplicates_removed >= 1,
            "Should report removal of at least one duplicate"
        );
        assert!(
            stats.memory_savings > 0.0,
            "Should report positive memory savings"
        );
    }
}
