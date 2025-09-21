//! Fractal geometry generation
//!
//! This module provides algorithms for generating fractal geometry,
//! including recursive subdivision and fractal patterns.

use crate::indexed_mesh::{IndexedMesh, IndexedFace};
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

/// Generate a Koch snowflake fractal
pub fn koch_snowflake(iterations: usize, size: f64) -> IndexedMesh<()> {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // Start with equilateral triangle
    let height = size * (3.0_f64).sqrt() / 2.0;
    vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()));
    vertices.push(Vertex::new(Point3::new(size, 0.0, 0.0), Vector3::z()));
    vertices.push(Vertex::new(Point3::new(size/2.0, height, 0.0), Vector3::z()));

    faces.push(IndexedFace {
        vertices: vec![0, 1, 2],
    });

    // Apply Koch curve iterations (simplified for triangle)
    for _ in 0..iterations {
        let mut new_faces = Vec::new();

        for face in &faces {
            if face.vertices.len() == 3 {
                // Subdivide triangle into 4 smaller triangles (Koch-like)
                let v0 = vertices[face.vertices[0]].position;
                let v1 = vertices[face.vertices[1]].position;
                let v2 = vertices[face.vertices[2]].position;

                // Add midpoints
                let m01 = (v0 + v1) / 2.0;
                let m12 = (v1 + v2) / 2.0;
                let m20 = (v2 + v0) / 2.0;

                let idx0 = vertices.len();
                vertices.push(Vertex::new(m01, Vector3::z()));
                let idx1 = vertices.len();
                vertices.push(Vertex::new(m12, Vector3::z()));
                let idx2 = vertices.len();
                vertices.push(Vertex::new(m20, Vector3::z()));

                // Create 4 new triangles
                new_faces.push(IndexedFace {
                    vertices: vec![face.vertices[0], idx0, idx2],
                });
                new_faces.push(IndexedFace {
                    vertices: vec![idx0, face.vertices[1], idx1],
                });
                new_faces.push(IndexedFace {
                    vertices: vec![idx2, idx1, face.vertices[2]],
                });
                new_faces.push(IndexedFace {
                    vertices: vec![idx0, idx1, idx2],
                });
            }
        }

        faces = new_faces;
    }

    IndexedMesh::from_vertices_and_faces(vertices, faces, Some(()))
}

/// Generate a Sierpinski tetrahedron fractal
pub fn sierpinski_tetrahedron(iterations: usize, size: f64) -> IndexedMesh<()> {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // Create initial tetrahedron
    let h = size * (2.0_f64).sqrt() / 3.0;
    let r = size * (3.0_f64).sqrt() / 3.0;

    vertices.push(Vertex::new(Point3::new(0.0, 0.0, h), Vector3::y()));
    vertices.push(Vertex::new(Point3::new(-r, 0.0, -h/3.0), Vector3::y()));
    vertices.push(Vertex::new(Point3::new(r, 0.0, -h/3.0), Vector3::y()));
    vertices.push(Vertex::new(Point3::new(0.0, size * 2.0/3.0, -h/3.0), Vector3::y()));

    faces.push(IndexedFace { vertices: vec![0, 1, 2] });
    faces.push(IndexedFace { vertices: vec![0, 2, 3] });
    faces.push(IndexedFace { vertices: vec![0, 3, 1] });
    faces.push(IndexedFace { vertices: vec![1, 3, 2] });

    // Apply Sierpinski subdivision
    for _ in 0..iterations {
        let mut new_faces = Vec::new();

        for face in &faces {
            if face.vertices.len() == 3 {
                let v0 = vertices[face.vertices[0]].position;
                let v1 = vertices[face.vertices[1]].position;
                let v2 = vertices[face.vertices[2]].position;

                // Add midpoints
                let m01 = (v0 + v1) / 2.0;
                let m12 = (v1 + v2) / 2.0;
                let m20 = (v2 + v0) / 2.0;

                let idx0 = vertices.len();
                vertices.push(Vertex::new(m01, Vector3::y()));
                let idx1 = vertices.len();
                vertices.push(Vertex::new(m12, Vector3::y()));
                let idx2 = vertices.len();
                vertices.push(Vertex::new(m20, Vector3::y()));

                // Create 4 new triangles
                new_faces.push(IndexedFace {
                    vertices: vec![face.vertices[0], idx0, idx2],
                });
                new_faces.push(IndexedFace {
                    vertices: vec![face.vertices[1], idx1, idx0],
                });
                new_faces.push(IndexedFace {
                    vertices: vec![face.vertices[2], idx2, idx1],
                });
                // Center triangle is omitted for Sierpinski
            }
        }

        faces = new_faces;
    }

    IndexedMesh::from_vertices_and_faces(vertices, faces, Some(()))
}

/// Generate a fractal tree using recursive branching
pub fn fractal_tree(iterations: usize, length: f64, angle: f64) -> IndexedMesh<()> {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // Start with trunk
    vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()));
    vertices.push(Vertex::new(Point3::new(0.0, length, 0.0), Vector3::z()));

    generate_tree_branch(
        &mut vertices,
        &mut faces,
        0, // start vertex index
        1, // end vertex index
        length * 0.7, // branch length reduction
        angle,
        iterations,
        0.3, // thickness reduction
    );

    IndexedMesh::from_vertices_and_faces(vertices, faces, Some(()))
}

/// Recursively generate tree branches
fn generate_tree_branch(
    vertices: &mut Vec<Vertex>,
    faces: &mut Vec<IndexedFace>,
    start_idx: usize,
    end_idx: usize,
    length: f64,
    angle: f64,
    iterations: usize,
    thickness: f64,
) {
    if iterations == 0 || length < 0.01 {
        return;
    }

    let start_pos = vertices[start_idx].position;
    let end_pos = vertices[end_idx].position;
    let direction = (end_pos - start_pos).normalize();

    // Create branch points
    let branch1_pos = end_pos + rotate_around_z(direction * length, angle);
    let branch2_pos = end_pos + rotate_around_z(direction * length, -angle);

    let branch1_idx = vertices.len();
    vertices.push(Vertex::new(branch1_pos, Vector3::z()));

    let branch2_idx = vertices.len();
    vertices.push(Vertex::new(branch2_pos, Vector3::z()));

    // Add branch segments
    faces.push(IndexedFace {
        vertices: vec![end_idx, branch1_idx],
    });
    faces.push(IndexedFace {
        vertices: vec![end_idx, branch2_idx],
    });

    // Recurse
    generate_tree_branch(
        vertices, faces, end_idx, branch1_idx,
        length * 0.7, angle, iterations - 1, thickness * 0.8
    );

    generate_tree_branch(
        vertices, faces, end_idx, branch2_idx,
        length * 0.7, angle, iterations - 1, thickness * 0.8
    );
}

/// Rotate vector around Z axis
fn rotate_around_z(vector: Vector3<f64>, angle: f64) -> Vector3<f64> {
    let cos = angle.cos();
    let sin = angle.sin();
    Vector3::new(
        vector.x * cos - vector.y * sin,
        vector.x * sin + vector.y * cos,
        vector.z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_koch_snowflake() {
        let snowflake = koch_snowflake(2, 1.0);
        assert!(snowflake.vertices.len() > 3);
        assert!(snowflake.faces.len() > 1);
    }

    #[test]
    fn test_sierpinski_tetrahedron() {
        let tetra = sierpinski_tetrahedron(2, 1.0);
        assert!(tetra.vertices.len() > 4);
        assert!(tetra.faces.len() > 4);
    }

    #[test]
    fn test_fractal_tree() {
        let tree = fractal_tree(3, 1.0, std::f64::consts::PI / 6.0);
        assert!(tree.vertices.len() > 2);
        assert!(!tree.faces.is_empty());
    }
}
