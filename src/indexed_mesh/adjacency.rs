//! Adjacency analysis for IndexedMesh
//!
//! This module provides efficient algorithms for analyzing mesh connectivity,
//! including vertex adjacency, face adjacency, and topological queries.

use crate::indexed_mesh::IndexedMesh;
use std::{collections::HashSet, fmt::Debug};

/// Analyze manifold properties of the mesh
#[derive(Debug, Clone)]
pub struct ManifoldAnalysis {
    /// Whether the mesh is manifold (each edge has exactly two faces)
    pub is_manifold: bool,
    /// Number of non-manifold edges
    pub non_manifold_edges: usize,
    /// Number of boundary edges (edges with only one face)
    pub boundary_edges: usize,
    /// List of non-manifold vertices (vertices with non-manifold edges)
    pub non_manifold_vertices: Vec<usize>,
}

/// Analyze mesh topology and connectivity
pub fn analyze_manifold<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> ManifoldAnalysis {
    let mut edge_face_count = std::collections::HashMap::new();

    // Count how many faces each edge belongs to
    for (face_idx, face) in mesh.faces.iter().enumerate() {
        let vertices = &face.vertices;
        for i in 0..vertices.len() {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % vertices.len()];

            // Create canonical edge representation (smaller index first)
            let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            edge_face_count.entry(edge).or_insert(Vec::new()).push(face_idx);
        }
    }

    let mut non_manifold_edges = 0;
    let mut boundary_edges = 0;
    let mut non_manifold_vertices = HashSet::new();

    for (edge, faces) in &edge_face_count {
        match faces.len() {
            0 => {} // Should not happen
            1 => boundary_edges += 1,
            2 => {} // Manifold edge
            _ => {
                // Non-manifold edge
                non_manifold_edges += 1;
                non_manifold_vertices.insert(edge.0);
                non_manifold_vertices.insert(edge.1);
            }
        }
    }

    ManifoldAnalysis {
        is_manifold: non_manifold_edges == 0,
        non_manifold_edges,
        boundary_edges,
        non_manifold_vertices: non_manifold_vertices.into_iter().collect(),
    }
}

/// Find boundary edges (edges belonging to only one face)
pub fn find_boundary_edges<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> Vec<(usize, usize)> {
    let mut edge_face_count = std::collections::HashMap::new();

    // Count faces per edge
    for face in &mesh.faces {
        let vertices = &face.vertices;
        for i in 0..vertices.len() {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % vertices.len()];
            let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            *edge_face_count.entry(edge).or_insert(0) += 1;
        }
    }

    // Find edges with only one face
    edge_face_count
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(edge, _)| edge)
        .collect()
}

/// Extract boundary loops from the mesh
pub fn extract_boundary_loops<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> Vec<Vec<usize>> {
    let boundary_edges = find_boundary_edges(mesh);
    let mut visited = HashSet::new();
    let mut loops = Vec::new();

    // Build edge adjacency map for boundary edges
    let mut edge_map = std::collections::HashMap::new();
    for &(v1, v2) in &boundary_edges {
        edge_map.entry(v1).or_insert(Vec::new()).push(v2);
        edge_map.entry(v2).or_insert(Vec::new()).push(v1);
    }

    // Find loops
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

            // Find next vertex
            if let Some(neighbors) = edge_map.get(&current) {
                let next = neighbors
                    .iter()
                    .find(|&&v| Some(v) != prev)
                    .copied();

                if let Some(next_vertex) = next {
                    prev = Some(current);
                    current = next_vertex;

                    // Check if we've completed the loop
                    if current == start_vertex && loop_vertices.len() > 2 {
                        break;
                    }
                } else {
                    break; // Dead end
                }
            } else {
                break; // No neighbors
            }
        }

        if loop_vertices.len() > 2 {
            loops.push(loop_vertices);
        }
    }

    loops
}

/// Find connected components in the mesh
pub fn find_connected_components<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
) -> Vec<Vec<usize>> {
    let adjacency = mesh.adjacency();
    let mut visited = HashSet::new();
    let mut components = Vec::new();

    for start_vertex in 0..mesh.vertices.len() {
        if visited.contains(&start_vertex) {
            continue;
        }

        // DFS to find connected component
        let mut component = Vec::new();
        let mut stack = vec![start_vertex];

        while let Some(vertex) = stack.pop() {
            if visited.insert(vertex) {
                component.push(vertex);

                // Add unvisited neighbors
                if let Some(neighbors) = adjacency.vertex_adjacency.get(vertex) {
                    for &neighbor in neighbors {
                        if !visited.contains(&neighbor) {
                            stack.push(neighbor);
                        }
                    }
                }
            }
        }

        if !component.is_empty() {
            components.push(component);
        }
    }

    components
}

/// Calculate mesh statistics
#[derive(Debug, Clone)]
pub struct MeshStatistics {
    /// Number of vertices
    pub vertex_count: usize,
    /// Number of faces
    pub face_count: usize,
    /// Number of edges
    pub edge_count: usize,
    /// Number of boundary edges
    pub boundary_edge_count: usize,
    /// Number of connected components
    pub component_count: usize,
    /// Whether the mesh is manifold
    pub is_manifold: bool,
    /// Average face size
    pub average_face_size: f64,
    /// Euler characteristic (V - E + F)
    pub euler_characteristic: i32,
}

impl MeshStatistics {
    /// Calculate comprehensive mesh statistics
    pub fn analyze<S: Clone + Send + Sync + Debug>(mesh: &IndexedMesh<S>) -> Self {
        let vertex_count = mesh.vertices.len();
        let face_count = mesh.faces.len();

        // Count unique edges
        let mut edges = HashSet::new();
        for face in &mesh.faces {
            let vertices = &face.vertices;
            for i in 0..vertices.len() {
                let v1 = vertices[i];
                let v2 = vertices[(i + 1) % vertices.len()];
                let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
                edges.insert(edge);
            }
        }
        let edge_count = edges.len();

        // Boundary analysis
        let boundary_edges = find_boundary_edges(mesh);
        let boundary_edge_count = boundary_edges.len();

        // Connected components
        let components = find_connected_components(mesh);
        let component_count = components.len();

        // Manifold check
        let manifold_analysis = analyze_manifold(mesh);
        let is_manifold = manifold_analysis.is_manifold;

        // Average face size
        let total_vertices: usize = mesh.faces.iter().map(|f| f.vertices.len()).sum();
        let average_face_size = if face_count > 0 {
            total_vertices as f64 / face_count as f64
        } else {
            0.0
        };

        // Euler characteristic
        let euler_characteristic = vertex_count as i32 - edge_count as i32 + face_count as i32;

        Self {
            vertex_count,
            face_count,
            edge_count,
            boundary_edge_count,
            component_count,
            is_manifold,
            average_face_size,
            euler_characteristic,
        }
    }
}

/// Find faces that share edges with a given face
pub fn find_adjacent_faces<S: Clone + Send + Sync + Debug>(
    mesh: &IndexedMesh<S>,
    face_idx: usize,
) -> Vec<usize> {
    if face_idx >= mesh.faces.len() {
        return Vec::new();
    }

    let face = &mesh.faces[face_idx];
    let mut adjacent_faces = HashSet::new();

    // Check each edge of the face
    for i in 0..face.vertices.len() {
        let v1 = face.vertices[i];
        let v2 = face.vertices[(i + 1) % face.vertices.len()];
        let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };

        // Find other faces that share this edge
        for (other_face_idx, other_face) in mesh.faces.iter().enumerate() {
            if other_face_idx == face_idx {
                continue;
            }

            let other_vertices = &other_face.vertices;
            for j in 0..other_vertices.len() {
                let ov1 = other_vertices[j];
                let ov2 = other_vertices[(j + 1) % other_vertices.len()];
                let other_edge = if ov1 < ov2 { (ov1, ov2) } else { (ov2, ov1) };

                if edge == other_edge {
                    adjacent_faces.insert(other_face_idx);
                }
            }
        }
    }

    adjacent_faces.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::indexed_mesh::shapes;

    #[test]
    fn test_cube_manifold_analysis() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let analysis = analyze_manifold(&cube);

        assert!(analysis.is_manifold, "Cube should be manifold");
        assert_eq!(analysis.non_manifold_edges, 0, "Cube should have no non-manifold edges");
        assert_eq!(analysis.boundary_edges, 0, "Closed cube should have no boundary edges");
        assert_eq!(analysis.non_manifold_vertices.len(), 0, "Cube should have no non-manifold vertices");
    }

    #[test]
    fn test_cube_boundary_edges() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let boundary_edges = find_boundary_edges(&cube);

        // Closed cube should have no boundary edges
        assert_eq!(boundary_edges.len(), 0);
    }

    #[test]
    fn test_cube_statistics() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let stats = MeshStatistics::analyze(&cube);

        assert_eq!(stats.vertex_count, 8);
        assert_eq!(stats.face_count, 6);
        assert_eq!(stats.edge_count, 12); // Cube has 12 edges
        assert_eq!(stats.component_count, 1); // Single connected component
        assert!(stats.is_manifold);
        assert_eq!(stats.euler_characteristic, 2); // V - E + F = 8 - 12 + 6 = 2
    }

    #[test]
    fn test_cube_face_adjacency() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);

        // Each face of a cube should be adjacent to 4 other faces
        for face_idx in 0..6 {
            let adjacent = find_adjacent_faces(&cube, face_idx);
            assert_eq!(adjacent.len(), 4, "Face {} should be adjacent to 4 faces", face_idx);
        }
    }

    #[test]
    fn test_connected_components() {
        let cube: IndexedMesh<()> = shapes::cube(2.0, None);
        let components = find_connected_components(&cube);

        assert_eq!(components.len(), 1, "Cube should have 1 connected component");
        assert_eq!(components[0].len(), 8, "Component should have all 8 vertices");
    }
}
