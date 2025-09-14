//! IndexedMesh geometric primitives and shape constructors
//!
//! This module provides constructors for common geometric shapes as IndexedMesh,
//! with automatic vertex deduplication for optimal memory usage.

use crate::float_types::Real;
use crate::indexed_mesh::IndexedMesh;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::f64::consts::PI;
use std::fmt::Debug;

/// Create an indexed cube (hexahedron) with automatic vertex deduplication
pub fn cube<S: Clone + Send + Sync + Debug>(
    size: Real,
    metadata: Option<S>,
) -> IndexedMesh<S> {
    let half_size = size / 2.0;

    // Define vertices (8 corners of cube)
    let vertices = vec![
        Point3::new(-half_size, -half_size, -half_size), // 0: bottom-back-left
        Point3::new( half_size, -half_size, -half_size), // 1: bottom-back-right
        Point3::new( half_size,  half_size, -half_size), // 2: bottom-front-right
        Point3::new(-half_size,  half_size, -half_size), // 3: bottom-front-left
        Point3::new(-half_size, -half_size,  half_size), // 4: top-back-left
        Point3::new( half_size, -half_size,  half_size), // 5: top-back-right
        Point3::new( half_size,  half_size,  half_size), // 6: top-front-right
        Point3::new(-half_size,  half_size,  half_size), // 7: top-front-left
    ];

    // Define faces (6 faces, each with 4 vertices forming a quad)
    let faces = vec![
        vec![0, 1, 2, 3], // bottom face
        vec![4, 5, 6, 7], // top face
        vec![0, 1, 5, 4], // back face
        vec![3, 2, 6, 7], // front face
        vec![0, 3, 7, 4], // left face
        vec![1, 2, 6, 5], // right face
    ];

    // Convert to Vertex objects with appropriate face normals
    let _vertex_objects: Vec<Vertex> = vertices
        .iter()
        .map(|pos| {
            // For a cube, compute face-based normals instead of vertex normals
            // This is a simplification - proper face normals would be computed per face
            let center = Point3::origin();
            let normal = (pos - center).normalize();
            Vertex::new(*pos, normal)
        })
        .collect();

    // Create IndexedMesh with deduplication
    let mut mesh = IndexedMesh::from_vertices_and_faces(
        vertices,
        faces,
        metadata,
    );

    // Override normals with face-based normals for cube
    update_cube_normals(&mut mesh);

    mesh
}

/// Update normals for cube faces to be axis-aligned
fn update_cube_normals<S: Clone + Send + Sync + Debug>(mesh: &mut IndexedMesh<S>) {
    // Face normals for cube
    let face_normals = [
        Vector3::new(0.0, 0.0, -1.0), // bottom
        Vector3::new(0.0, 0.0,  1.0), // top
        Vector3::new(0.0, -1.0, 0.0), // back
        Vector3::new(0.0,  1.0, 0.0), // front
        Vector3::new(-1.0, 0.0, 0.0), // left
        Vector3::new( 1.0, 0.0, 0.0), // right
    ];

    for (i, face) in mesh.faces.iter_mut().enumerate() {
        if i < face_normals.len() {
            face.normal = Some(face_normals[i]);
        }
    }
}

/// Create an indexed sphere with automatic vertex deduplication
pub fn sphere<S: Clone + Send + Sync + Debug>(
    radius: Real,
    segments: usize,
    stacks: usize,
    metadata: Option<S>,
) -> IndexedMesh<S> {
    if segments < 3 || stacks < 2 {
        return IndexedMesh::new();
    }

    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // Generate vertices
    for stack in 0..=stacks {
        let phi = PI * (stack as Real) / (stacks as Real);

        for segment in 0..segments {
            let theta = 2.0 * PI * (segment as Real) / (segments as Real);

            let x = radius * phi.sin() * theta.cos();
            let y = radius * phi.sin() * theta.sin();
            let z = radius * phi.cos();

            let pos = Point3::new(x, y, z);
            let _normal = pos.coords.normalize();

            vertices.push(pos);
        }
    }

    // Generate faces
    for stack in 0..stacks {
        for segment in 0..segments {
            let current = stack * segments + segment;
            let next = stack * segments + (segment + 1) % segments;
            let below_current = (stack + 1) * segments + segment;
            let below_next = (stack + 1) * segments + (segment + 1) % segments;

            // Two triangles per quad
            faces.push(vec![current, below_current, below_next]);
            faces.push(vec![current, below_next, next]);
        }
    }

    IndexedMesh::from_vertices_and_faces(vertices, faces, metadata)
}

/// Create an indexed cylinder with automatic vertex deduplication
pub fn cylinder<S: Clone + Send + Sync + Debug>(
    radius: Real,
    height: Real,
    segments: usize,
    metadata: Option<S>,
) -> IndexedMesh<S> {
    if segments < 3 {
        return IndexedMesh::new();
    }

    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let half_height = height / 2.0;

    // Generate vertices
    // Bottom circle
    for i in 0..segments {
        let angle = 2.0 * PI * (i as Real) / (segments as Real);
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        vertices.push(Point3::new(x, y, -half_height));
    }

    // Top circle
    for i in 0..segments {
        let angle = 2.0 * PI * (i as Real) / (segments as Real);
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        vertices.push(Point3::new(x, y, half_height));
    }

    // Bottom center
    vertices.push(Point3::new(0.0, 0.0, -half_height));
    let bottom_center_idx = vertices.len() - 1;

    // Top center
    vertices.push(Point3::new(0.0, 0.0, half_height));
    let top_center_idx = vertices.len() - 1;

    // Side faces
    for i in 0..segments {
        let bottom_i = i;
        let top_i = i + segments;
        let bottom_next = (i + 1) % segments;
        let top_next = bottom_next + segments;

        faces.push(vec![bottom_i, top_i, top_next, bottom_next]);
    }

    // Bottom face
    for i in 0..segments {
        faces.push(vec![bottom_center_idx, (i + 1) % segments, i]);
    }

    // Top face
    for i in 0..segments {
        let top_i = i + segments;
        let top_next = ((i + 1) % segments) + segments;
        faces.push(vec![top_center_idx, top_i, top_next]);
    }

    // Convert to Vertex objects
    let _vertex_objects: Vec<Vertex> = vertices
        .iter()
        .enumerate()
        .map(|(i, pos)| {
            let normal = if i < segments {
                // Bottom vertices
                Vector3::new(0.0, 0.0, -1.0)
            } else if i < 2 * segments {
                // Top vertices
                Vector3::new(0.0, 0.0, 1.0)
            } else if i == bottom_center_idx {
                // Bottom center
                Vector3::new(0.0, 0.0, -1.0)
            } else if i == top_center_idx {
                // Top center
                Vector3::new(0.0, 0.0, 1.0)
            } else {
                // Side vertices - radial normal
                let radial_pos = Point3::new(pos.x, pos.y, 0.0);
                radial_pos.coords.normalize()
            };
            Vertex::new(*pos, normal)
        })
        .collect();

    IndexedMesh::from_vertices_and_faces(vertices, faces, metadata)
}

/// Create an indexed cuboid (rectangular box) with automatic vertex deduplication
pub fn cuboid<S: Clone + Send + Sync + Debug>(
    width: Real,
    length: Real,
    height: Real,
    metadata: Option<S>,
) -> IndexedMesh<S> {
    let half_width = width / 2.0;
    let half_length = length / 2.0;
    let half_height = height / 2.0;

    // Define vertices (8 corners of cuboid)
    let vertices = vec![
        Point3::new(-half_width, -half_length, -half_height), // 0: bottom-back-left
        Point3::new( half_width, -half_length, -half_height), // 1: bottom-back-right
        Point3::new( half_width,  half_length, -half_height), // 2: bottom-front-right
        Point3::new(-half_width,  half_length, -half_height), // 3: bottom-front-left
        Point3::new(-half_width, -half_length,  half_height), // 4: top-back-left
        Point3::new( half_width, -half_length,  half_height), // 5: top-back-right
        Point3::new( half_width,  half_length,  half_height), // 6: top-front-right
        Point3::new(-half_width,  half_length,  half_height), // 7: top-front-left
    ];

    // Define faces (same as cube)
    let faces = vec![
        vec![0, 1, 2, 3], // bottom face
        vec![4, 5, 6, 7], // top face
        vec![0, 1, 5, 4], // back face
        vec![3, 2, 6, 7], // front face
        vec![0, 3, 7, 4], // left face
        vec![1, 2, 6, 5], // right face
    ];

    IndexedMesh::from_vertices_and_faces(vertices, faces, metadata)
}

/// Create an indexed tetrahedron (4-faced polyhedron) with automatic vertex deduplication
pub fn tetrahedron<S: Clone + Send + Sync + Debug>(
    size: Real,
    metadata: Option<S>,
) -> IndexedMesh<S> {
    let a = size / (2.0 * 3.0_f64.sqrt());

    // Define vertices (4 vertices of tetrahedron)
    let vertices = vec![
        Point3::new( a,  a,  a), // 0
        Point3::new( a, -a, -a), // 1
        Point3::new(-a,  a, -a), // 2
        Point3::new(-a, -a,  a), // 3
    ];

    // Define faces (4 triangular faces)
    let faces = vec![
        vec![0, 1, 2], // face 0
        vec![0, 2, 3], // face 1
        vec![0, 3, 1], // face 2
        vec![1, 3, 2], // face 3
    ];

    IndexedMesh::from_vertices_and_faces(vertices, faces, metadata)
}

/// Create an indexed octahedron (8-faced polyhedron) with automatic vertex deduplication
pub fn octahedron<S: Clone + Send + Sync + Debug>(
    radius: Real,
    metadata: Option<S>,
) -> IndexedMesh<S> {
    // Define vertices (6 vertices of octahedron)
    let vertices = vec![
        Point3::new( radius,  0.0,    0.0),    // 0: +X
        Point3::new(-radius,  0.0,    0.0),    // 1: -X
        Point3::new( 0.0,    radius,  0.0),    // 2: +Y
        Point3::new( 0.0,   -radius,  0.0),    // 3: -Y
        Point3::new( 0.0,    0.0,    radius),  // 4: +Z
        Point3::new( 0.0,    0.0,   -radius),  // 5: -Z
    ];

    // Define faces (8 triangular faces)
    let faces = vec![
        vec![4, 0, 2], // top-front-right
        vec![4, 2, 1], // top-front-left
        vec![4, 1, 3], // top-back-left
        vec![4, 3, 0], // top-back-right
        vec![5, 2, 0], // bottom-front-right
        vec![5, 1, 2], // bottom-front-left
        vec![5, 3, 1], // bottom-back-left
        vec![5, 0, 3], // bottom-back-right
    ];

    IndexedMesh::from_vertices_and_faces(vertices, faces, metadata)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cube_creation() {
        let cube: IndexedMesh<()> = cube(2.0, None);
        assert_eq!(cube.vertices.len(), 8); // Cube has 8 vertices
        assert_eq!(cube.faces.len(), 6); // Cube has 6 faces
    }

    #[test]
    fn test_sphere_creation() {
        let sphere: IndexedMesh<()> = sphere(1.0, 8, 6, None);
        // Due to vertex deduplication, we may have fewer vertices than (stacks+1) * segments
        // The important thing is that we have a reasonable number of vertices and faces
        assert!(sphere.vertices.len() > 0, "Sphere should have vertices");
        assert!(sphere.faces.len() > 0, "Sphere should have faces");
        assert!(sphere.vertices.len() <= 7 * 8, "Should not exceed maximum possible vertices");
        assert_eq!(sphere.faces.len(), 6 * 8 * 2); // stacks * segments * 2 triangles per quad
    }

    #[test]
    fn test_cylinder_creation() {
        let cylinder: IndexedMesh<()> = cylinder(1.0, 2.0, 8, None);
        // 8 bottom + 8 top + 2 centers = 18 vertices
        assert_eq!(cylinder.vertices.len(), 18);
        // 8 sides + 8 bottom + 8 top = 24 faces
        assert_eq!(cylinder.faces.len(), 24);
    }

    #[test]
    fn test_cuboid_creation() {
        let cuboid: IndexedMesh<()> = cuboid(2.0, 3.0, 4.0, None);
        assert_eq!(cuboid.vertices.len(), 8); // Same as cube
        assert_eq!(cuboid.faces.len(), 6); // Same as cube
    }

    #[test]
    fn test_tetrahedron_creation() {
        let tetra: IndexedMesh<()> = tetrahedron(2.0, None);
        assert_eq!(tetra.vertices.len(), 4);
        assert_eq!(tetra.faces.len(), 4);
    }

    #[test]
    fn test_octahedron_creation() {
        let octa: IndexedMesh<()> = octahedron(1.0, None);
        assert_eq!(octa.vertices.len(), 6);
        assert_eq!(octa.faces.len(), 8);
    }

    #[test]
    fn test_invalid_sphere_parameters() {
        let invalid1: IndexedMesh<()> = sphere(1.0, 2, 6, None); // segments < 3
        let invalid2: IndexedMesh<()> = sphere(1.0, 8, 1, None); // stacks < 2

        assert_eq!(invalid1.vertices.len(), 0);
        assert_eq!(invalid2.vertices.len(), 0);
    }

    #[test]
    fn test_invalid_cylinder_parameters() {
        let invalid: IndexedMesh<()> = cylinder(1.0, 2.0, 2, None); // segments < 3
        assert_eq!(invalid.vertices.len(), 0);
    }
}
