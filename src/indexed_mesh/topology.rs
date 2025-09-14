//! Topology analysis and repair for IndexedMesh
//!
//! This module provides algorithms for analyzing and repairing mesh topology,
//! including hole detection, manifold correction, and topological validation.

use crate::indexed_mesh::{IndexedFace, IndexedMesh};
use std::collections::{HashMap, HashSet};
use std::fmt::Debug;

/// Result of topology validation
#[derive(Debug, Clone)]
pub struct TopologyValidation {
    /// Whether the mesh has valid topology
    pub is_valid: bool,
    /// List of validation errors
    pub errors: Vec<TopologyError>,
    /// Number of holes detected
    pub hole_count: usize,
    /// List of non-manifold vertices
    pub non_manifold_vertices: Vec<usize>,
}

/// Types of topology errors
#[derive(Debug, Clone)]
pub enum TopologyError {
    /// Face with invalid vertex indices
    InvalidFaceIndices { face_idx: usize, vertex_idx: usize },
    /// Face with too few vertices
    DegenerateFace { face_idx: usize, vertex_count: usize },
    /// Non-manifold edge
    NonManifoldEdge { vertex1: usize, vertex2: usize, face_count: usize },
    /// Hole in the mesh
    HoleDetected { boundary_loop: Vec<usize> },
}

/// Validate mesh topology
pub fn validate_topology<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> TopologyValidation {
    let mut errors = Vec::new();
    let mut edge_face_map = HashMap::new();
    let mut non_manifold_vertices = HashSet::new();

    // Check face validity
    for (face_idx, face) in mesh.faces.iter().enumerate() {
        // Check vertex count
        if face.vertices.len() < 3 {
            errors.push(TopologyError::DegenerateFace {
                face_idx,
                vertex_count: face.vertices.len(),
            });
            continue;
        }

        // Check vertex indices
        for &vertex_idx in &face.vertices {
            if vertex_idx >= mesh.vertices.len() {
                errors.push(TopologyError::InvalidFaceIndices {
                    face_idx,
                    vertex_idx,
                });
            }
        }

        // Build edge-to-face mapping
        let vertices = &face.vertices;
        for i in 0..vertices.len() {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % vertices.len()];
            let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            edge_face_map.entry(edge).or_insert(Vec::new()).push(face_idx);
        }
    }

    // Check for non-manifold edges
    for (edge, faces) in &edge_face_map {
        if faces.len() > 2 {
            errors.push(TopologyError::NonManifoldEdge {
                vertex1: edge.0,
                vertex2: edge.1,
                face_count: faces.len(),
            });
            non_manifold_vertices.insert(edge.0);
            non_manifold_vertices.insert(edge.1);
        }
    }

    // Find boundary edges (potential holes)
    let boundary_edges: Vec<_> = edge_face_map
        .iter()
        .filter(|(_, faces)| faces.len() == 1)
        .map(|(edge, _)| *edge)
        .collect();

    let hole_count = boundary_edges.len();

    // Extract boundary loops (holes)
    let boundary_loops = extract_boundary_loops_from_edges(&boundary_edges);
    for boundary_loop in boundary_loops {
        if boundary_loop.len() > 2 {
            errors.push(TopologyError::HoleDetected { boundary_loop });
        }
    }

    TopologyValidation {
        is_valid: errors.is_empty(),
        errors,
        hole_count,
        non_manifold_vertices: non_manifold_vertices.into_iter().collect(),
    }
}

/// Extract boundary loops from boundary edges
fn extract_boundary_loops_from_edges(boundary_edges: &[(usize, usize)]) -> Vec<Vec<usize>> {
    let mut edge_map = HashMap::new();
    for &(v1, v2) in boundary_edges {
        edge_map.entry(v1).or_insert(Vec::new()).push(v2);
        edge_map.entry(v2).or_insert(Vec::new()).push(v1);
    }

    let mut loops = Vec::new();
    let mut visited = HashSet::new();

    for &start_vertex in edge_map.keys() {
        if visited.contains(&start_vertex) {
            continue;
        }

        let mut loop_vertices = Vec::new();
        let mut current = start_vertex;
        let mut prev = None;

        loop {
            visited.insert(current);
            loop_vertices.push(current);

            if let Some(neighbors) = edge_map.get(&current) {
                let next = neighbors
                    .iter()
                    .find(|&&v| Some(v) != prev)
                    .copied();

                if let Some(next_vertex) = next {
                    prev = Some(current);
                    current = next_vertex;

                    if current == start_vertex && loop_vertices.len() > 2 {
                        break;
                    }
                } else {
                    break;
                }
            } else {
                break;
            }
        }

        if loop_vertices.len() > 2 {
            loops.push(loop_vertices);
        }
    }

    loops
}

/// Attempt to repair mesh topology
pub fn repair_topology<S: Clone + Send + Sync + Debug>(
    mesh: &mut IndexedMesh<S>,
) -> Vec<String> {
    let mut repairs = Vec::new();

    // Validate current topology
    let validation = validate_topology(mesh);

    if validation.is_valid {
        repairs.push("Mesh topology is already valid".to_string());
        return repairs;
    }

    // Remove degenerate faces
    let original_face_count = mesh.faces.len();
    mesh.faces.retain(|face| face.vertices.len() >= 3);
    let removed_faces = original_face_count - mesh.faces.len();
    if removed_faces > 0 {
        repairs.push(format!("Removed {} degenerate faces", removed_faces));
    }

    // Fix invalid vertex indices
    for face in &mut mesh.faces {
        face.vertices.retain(|&vertex_idx| vertex_idx < mesh.vertices.len());
    }

    // Recompute adjacency after repairs
    mesh.adjacency = std::sync::OnceLock::new();
    mesh.bounding_box = std::sync::OnceLock::new();

    repairs.push("Recomputed adjacency information".to_string());

    repairs
}

/// Fill holes in the mesh using simple triangulation
pub fn fill_holes<S: Clone + Send + Sync + Debug>(
    mesh: &mut IndexedMesh<S>,
) -> Vec<String> {
    let mut fills = Vec::new();

    let validation = validate_topology(mesh);
    let mut new_faces = Vec::new();

    for error in &validation.errors {
        if let TopologyError::HoleDetected { boundary_loop } = error {
            if boundary_loop.len() == 3 {
                // Simple triangle hole
                let face = IndexedFace {
                    vertices: boundary_loop.clone(),
                    normal: None,
                    metadata: None,
                };
                new_faces.push(face);
                fills.push(format!("Filled triangular hole: {:?}", boundary_loop));
            } else if boundary_loop.len() == 4 {
                // Quad hole - triangulate
                let face1 = IndexedFace {
                    vertices: vec![boundary_loop[0], boundary_loop[1], boundary_loop[2]],
                    normal: None,
                    metadata: None,
                };
                let face2 = IndexedFace {
                    vertices: vec![boundary_loop[0], boundary_loop[2], boundary_loop[3]],
                    normal: None,
                    metadata: None,
                };
                new_faces.push(face1);
                new_faces.push(face2);
                fills.push(format!("Filled quadrilateral hole: {:?}", boundary_loop));
            } else {
                fills.push(format!("Cannot fill complex hole with {} vertices", boundary_loop.len()));
            }
        }
    }

    if !new_faces.is_empty() {
        mesh.faces.extend(new_faces);
        mesh.adjacency = std::sync::OnceLock::new();
        mesh.bounding_box = std::sync::OnceLock::new();
    }

    fills
}

/// Check if mesh is watertight (no holes)
pub fn is_watertight<S: Clone + Send + Sync + Debug>(mesh: &IndexedMesh<S>) -> bool {
    let boundary_edges = super::adjacency::find_boundary_edges(mesh);
    boundary_edges.is_empty()
}

/// Calculate genus of the mesh (number of holes in topology)
pub fn calculate_genus<S: Clone + Send + Sync + Debug>(mesh: &IndexedMesh<S>) -> Option<i32> {
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return None;
    }

    let stats = super::adjacency::MeshStatistics::analyze(mesh);

    if !stats.is_manifold {
        return None; // Genus undefined for non-manifold meshes
    }

    // Euler-Poincaré formula: V - E + F = 2(1 - g) for closed surfaces
    // g = genus (number of holes)
    let euler_char = stats.euler_characteristic;
    let genus = 1 - (euler_char / 2);

    Some(genus)
}

/// Separate mesh into connected components
pub fn separate_components<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> Vec<IndexedMesh<S>> {
    let components = super::adjacency::find_connected_components(mesh);

    components
        .into_iter()
        .map(|component_vertices| {
            // Find faces that only use vertices from this component
            let mut component_faces = Vec::new();
            let mut vertex_map = HashMap::new();

            // Create vertex mapping for this component
            for (new_idx, &old_vertex_idx) in component_vertices.iter().enumerate() {
                vertex_map.insert(old_vertex_idx, new_idx);
            }

            // Find faces using only component vertices
            for face in &mesh.faces {
                let mut component_face_vertices = Vec::new();
                let mut all_vertices_in_component = true;

                for &vertex_idx in &face.vertices {
                    if let Some(&new_idx) = vertex_map.get(&vertex_idx) {
                        component_face_vertices.push(new_idx);
                    } else {
                        all_vertices_in_component = false;
                        break;
                    }
                }

                if all_vertices_in_component && component_face_vertices.len() >= 3 {
                    let new_face = IndexedFace {
                        vertices: component_face_vertices,
                        normal: face.normal,
                        metadata: face.metadata.clone(),
                    };
                    component_faces.push(new_face);
                }
            }

            // Extract vertices for this component
            let component_vertices_data: Vec<_> = component_vertices
                .iter()
                .map(|&idx| mesh.vertices[idx])
                .collect();

            IndexedMesh {
                vertices: component_vertices_data,
                faces: component_faces,
                adjacency: std::sync::OnceLock::new(),
                bounding_box: std::sync::OnceLock::new(),
                metadata: mesh.metadata.clone(),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use crate::mesh::vertex::Vertex;
    use nalgebra::{Point3, Vector3};
    use super::*;
    use crate::indexed_mesh::shapes;

    #[test]
    fn test_valid_cube_topology() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let validation = validate_topology(&cube);

        assert!(validation.is_valid, "Cube should have valid topology");
        assert_eq!(validation.errors.len(), 0, "Cube should have no topology errors");
        assert_eq!(validation.hole_count, 0, "Cube should have no holes");
        assert_eq!(validation.non_manifold_vertices.len(), 0, "Cube should have no non-manifold vertices");
    }

    #[test]
    fn test_watertight_cube() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        assert!(is_watertight(&cube), "Cube should be watertight");
    }

    #[test]
    fn test_cube_genus() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let genus = calculate_genus(&cube);
        assert_eq!(genus, Some(0), "Cube should have genus 0 (sphere-like topology)");
    }

    #[test]
    fn test_cube_components() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let components = separate_components(&cube);

        assert_eq!(components.len(), 1, "Cube should have 1 connected component");
        assert_eq!(components[0].vertices.len(), 8, "Component should have 8 vertices");
        assert_eq!(components[0].faces.len(), 6, "Component should have 6 faces");
    }

    #[test]
    fn test_topology_repair() {
        let mut mesh: IndexedMesh<()> = IndexedMesh::new();

        // Add some valid vertices
        mesh.vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ];

        // Add a degenerate face (only 2 vertices)
        mesh.faces = vec![
            IndexedFace {
                vertices: vec![0, 1, 2],
                normal: None,
                metadata: None,
            },
            IndexedFace {
                vertices: vec![0, 1], // Degenerate
                normal: None,
                metadata: None,
            },
        ];

        let repairs = repair_topology(&mut mesh);

        assert_eq!(mesh.faces.len(), 1, "Should have removed degenerate face");
        assert!(repairs.len() > 0, "Should report repairs performed");
    }
}
