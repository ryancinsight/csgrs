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
    let vertices = [
        Point3::new(-half_size, -half_size, -half_size), // 0: bottom-back-left
        Point3::new(half_size, -half_size, -half_size),  // 1: bottom-back-right
        Point3::new(half_size, half_size, -half_size),   // 2: bottom-front-right
        Point3::new(-half_size, half_size, -half_size),  // 3: bottom-front-left
        Point3::new(-half_size, -half_size, half_size),  // 4: top-back-left
        Point3::new(half_size, -half_size, half_size),   // 5: top-back-right
        Point3::new(half_size, half_size, half_size),    // 6: top-front-right
        Point3::new(-half_size, half_size, half_size),   // 7: top-front-left
    ];

    // Define faces (6 faces, each with 4 vertices forming a quad)
    // Vertex ordering ensures counter-clockwise winding when viewed from outside
    let faces = vec![
        vec![0, 3, 2, 1], // bottom face (counter-clockwise when viewed from below)
        vec![4, 5, 6, 7], // top face (counter-clockwise when viewed from above)
        vec![0, 1, 5, 4], // back face (counter-clockwise when viewed from back)
        vec![3, 7, 6, 2], // front face (counter-clockwise when viewed from front)
        vec![0, 4, 7, 3], // left face (counter-clockwise when viewed from left)
        vec![1, 2, 6, 5], // right face (counter-clockwise when viewed from right)
    ];

    // Convert to Vertex objects with appropriate face normals for flat shading
    let vertex_objects: Vec<Vertex> = vertices
        .into_iter()
        .map(|pos| {
            // For flat-shaded cube, assign vertex normals based on the primary face direction
            // Each vertex gets the normal of the "most significant" face it belongs to
            let normal = if pos.x.abs() >= pos.y.abs() && pos.x.abs() >= pos.z.abs() {
                // X is dominant - left/right face
                Vector3::new(pos.x.signum(), 0.0, 0.0)
            } else if pos.y.abs() >= pos.z.abs() {
                // Y is dominant - front/back face
                Vector3::new(0.0, pos.y.signum(), 0.0)
            } else {
                // Z is dominant - top/bottom face
                Vector3::new(0.0, 0.0, pos.z.signum())
            };
            Vertex::new(pos, normal)
        })
        .collect();

    // Convert vertex objects to (position, normal) pairs for the creation method
    let vertex_data: Vec<(Point3<Real>, Vector3<Real>)> = vertex_objects
        .into_iter()
        .map(|v| (v.pos, v.normal))
        .collect();

    // Create IndexedMesh with deduplication
    let mut mesh = IndexedMesh::from_vertices_with_normals_and_faces(
        vertex_data,
        faces.clone(),
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
        Vector3::new(0.0, 0.0, 1.0),  // top
        Vector3::new(0.0, -1.0, 0.0), // back
        Vector3::new(0.0, 1.0, 0.0),  // front
        Vector3::new(-1.0, 0.0, 0.0), // left
        Vector3::new(1.0, 0.0, 0.0),  // right
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

    // Top pole
    vertices.push(Point3::new(0.0, 0.0, radius));

    // Middle rings (stacks 1 to stacks-1)
    for stack in 1..stacks {
        let phi = PI * (stack as Real) / (stacks as Real);
        for segment in 0..segments {
            let theta = 2.0 * PI * (segment as Real) / (segments as Real);

            let x = radius * phi.sin() * theta.cos();
            let y = radius * phi.sin() * theta.sin();
            let z = radius * phi.cos();

            let pos = Point3::new(x, y, z);
            vertices.push(pos);
        }
    }

    // Bottom pole
    vertices.push(Point3::new(0.0, 0.0, -radius));

    // Generate faces with correct vertex indexing
    let top_pole_idx = 0;
    let bottom_pole_idx = vertices.len() - 1;

    // First ring (after top pole)
    let first_ring_start = 1; // After top pole

    // Middle rings (stacks 1 to stacks-1)
    // For stacks=N, we have N-1 rings between the poles
    let mut ring_starts = vec![first_ring_start];
    let mut vertex_idx = first_ring_start + segments;
    for _ in 1..(stacks - 1) {
        ring_starts.push(vertex_idx);
        vertex_idx += segments;
    }

    // Verify we have the right number of rings
    let expected_rings = stacks - 1;
    assert_eq!(
        ring_starts.len(),
        expected_rings,
        "Should have {} rings for {} stacks",
        expected_rings,
        stacks
    );

    // Handle top cap (triangles from top pole to first ring)
    for segment in 0..segments {
        let next_segment = (segment + 1) % segments;
        let v1 = first_ring_start + segment;
        let v2 = first_ring_start + next_segment;

        faces.push(vec![top_pole_idx, v1, v2]);
    }

    // Handle middle stacks (quads between rings)
    for ring_idx in 0..(ring_starts.len() - 1) {
        let current_ring_start = ring_starts[ring_idx];
        let next_ring_start = ring_starts[ring_idx + 1];

        for segment in 0..segments {
            let next_segment = (segment + 1) % segments;

            let current = current_ring_start + segment;
            let next = current_ring_start + next_segment;
            let below_current = next_ring_start + segment;
            let below_next = next_ring_start + next_segment;

            // Two triangles per quad
            faces.push(vec![current, below_current, below_next]);
            faces.push(vec![current, below_next, next]);
        }
    }

    // Handle bottom cap (triangles from last ring to bottom pole)
    let last_ring_start = *ring_starts
        .last()
        .expect("Ring starts should be populated for valid sphere parameters");
    for segment in 0..segments {
        let next_segment = (segment + 1) % segments;
        let v1 = last_ring_start + segment;
        let v2 = last_ring_start + next_segment;

        faces.push(vec![bottom_pole_idx, v2, v1]);
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

    // Side faces - wound counter-clockwise when viewed from outside
    for i in 0..segments {
        let bottom_i = i;
        let top_i = i + segments;
        let bottom_next = (i + 1) % segments;
        let top_next = ((i + 1) % segments) + segments;

        faces.push(vec![bottom_i, bottom_next, top_next, top_i]);
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

    // Use the regular constructor which will compute proper face normals
    let mut mesh = IndexedMesh::from_vertices_and_faces(vertices, faces, metadata);

    // Override face normals with correct values for cylinder
    update_cylinder_face_normals(&mut mesh, radius, segments);

    mesh
}

/// Update face normals for cylinder faces to be correct
fn update_cylinder_face_normals<S: Clone + Send + Sync + Debug>(
    mesh: &mut IndexedMesh<S>,
    radius: Real,
    segments: usize,
) {
    for (face_idx, face) in mesh.faces.iter_mut().enumerate() {
        if face.vertices.len() >= 3 {
            if face_idx < segments {
                // Side faces (0 to segments-1) - radial normals
                let vertex_idx = face.vertices[0];
                if vertex_idx < mesh.vertices.len() {
                    let pos = mesh.vertices[vertex_idx].pos;
                    let radial_normal =
                        Vector3::new(pos.x / radius, pos.y / radius, 0.0).normalize();
                    face.normal = Some(radial_normal);
                }
            } else if face_idx < segments + segments {
                // Bottom faces (segments to 2*segments-1) - face down
                face.normal = Some(Vector3::new(0.0, 0.0, -1.0));
            } else {
                // Top faces (2*segments to 3*segments-1) - face up
                face.normal = Some(Vector3::new(0.0, 0.0, 1.0));
            }
        }
    }
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
        Point3::new(half_width, -half_length, -half_height),  // 1: bottom-back-right
        Point3::new(half_width, half_length, -half_height),   // 2: bottom-front-right
        Point3::new(-half_width, half_length, -half_height),  // 3: bottom-front-left
        Point3::new(-half_width, -half_length, half_height),  // 4: top-back-left
        Point3::new(half_width, -half_length, half_height),   // 5: top-back-right
        Point3::new(half_width, half_length, half_height),    // 6: top-front-right
        Point3::new(-half_width, half_length, half_height),   // 7: top-front-left
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
        Point3::new(a, a, a),   // 0
        Point3::new(a, -a, -a), // 1
        Point3::new(-a, a, -a), // 2
        Point3::new(-a, -a, a), // 3
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
        Point3::new(radius, 0.0, 0.0),  // 0: +X
        Point3::new(-radius, 0.0, 0.0), // 1: -X
        Point3::new(0.0, radius, 0.0),  // 2: +Y
        Point3::new(0.0, -radius, 0.0), // 3: -Y
        Point3::new(0.0, 0.0, radius),  // 4: +Z
        Point3::new(0.0, 0.0, -radius), // 5: -Z
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
        // Corrected topology: 2 poles + (stacks-1) rings of segments vertices each
        let expected_vertices = 2 + (6 - 1) * 8; // 2 + 5*8 = 42
        // Corrected face count: top cap + bottom cap + middle sections
        // top/bottom caps: 2 * segments triangles each
        // middle sections: (stacks-2) sections × segments × 2 triangles each
        let expected_faces = 8 + 8 + (6 - 2) * 8 * 2; // 8 + 8 + 4*8*2 = 16 + 64 = 80

        assert!(!sphere.vertices.is_empty(), "Sphere should have vertices");
        assert!(!sphere.faces.is_empty(), "Sphere should have faces");
        assert_eq!(
            sphere.vertices.len(),
            expected_vertices,
            "Sphere should have correct vertex count"
        );
        assert_eq!(
            sphere.faces.len(),
            expected_faces,
            "Sphere should have correct face count"
        );

        // Sphere manifold issues have been fixed
        assert!(sphere.is_manifold(), "Sphere should be manifold");
    }

    #[test]
    fn test_sphere_manifold_debug() {
        // Debug the minimal failing case: sphere(1.0, 4, 4)
        println!("\n=== Debugging sphere(1.0, 4, 4) ===");
        let sphere: IndexedMesh<()> = sphere(1.0, 4, 4, None);

        println!("Vertices: {}", sphere.vertices.len());
        println!("Faces: {}", sphere.faces.len());

        // Print all faces to understand the topology
        println!("\nFace topology:");
        for (i, face) in sphere.faces.iter().enumerate() {
            println!("  Face {}: {:?}", i, face.vertices);
        }

        // Check edge connectivity
        let mut edge_face_count = std::collections::HashMap::new();
        for (face_idx, face) in sphere.faces.iter().enumerate() {
            let vertices = &face.vertices;
            for i in 0..vertices.len() {
                let v1 = vertices[i];
                let v2 = vertices[(i + 1) % vertices.len()];
                let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
                edge_face_count
                    .entry(edge)
                    .or_insert(Vec::new())
                    .push(face_idx);
            }
        }

        // Analyze edge sharing
        println!("\nEdge analysis:");
        let mut problematic_edges = Vec::new();
        for (edge, faces) in &edge_face_count {
            if faces.len() != 2 {
                println!(
                    "  Edge {:?}: {} faces (should be 2): {:?}",
                    edge,
                    faces.len(),
                    faces
                );
                problematic_edges.push(*edge);
            }
        }

        println!("\nProblematic edges: {}", problematic_edges.len());
        println!("Expected manifold: {}", problematic_edges.is_empty());

        // Check manifold status
        let is_manifold = sphere.is_manifold();
        println!("Manifold check result: {}", is_manifold);

        // Basic validation
        assert!(!sphere.vertices.is_empty(), "Sphere should have vertices");
        assert!(!sphere.faces.is_empty(), "Sphere should have faces");
    }

    #[test]
    fn test_sphere_simple() {
        // Test with minimal complexity to validate manifold generation
        let sphere: IndexedMesh<()> = sphere(1.0, 4, 4, None);
        let expected_vertices = 2 + (4 - 1) * 4; // 2 + 3*4 = 14
        // Corrected face count: top cap + bottom cap + middle sections
        let expected_faces = 4 + 4 + (4 - 2) * 4 * 2; // 4 + 4 + 2*4*2 = 8 + 16 = 24

        assert_eq!(sphere.vertices.len(), expected_vertices);
        assert_eq!(sphere.faces.len(), expected_faces);
        assert!(sphere.is_manifold(), "Simple sphere should be manifold");
    }

    #[test]
    fn test_sphere_normals_outward() {
        // Test that sphere face normals point outward
        let sphere: IndexedMesh<()> = sphere(1.0, 8, 4, None);

        for (face_idx, face) in sphere.faces.iter().enumerate() {
            if let Some(normal) = face.normal {
                // For each vertex in the face, check that normal points away from center
                for &vertex_idx in &face.vertices {
                    let vertex_pos = sphere.vertices[vertex_idx].pos.coords;

                    // Dot product of position vector with normal should be positive for outward normals
                    let dot_product = vertex_pos.dot(&normal);

                    // Allow for some numerical tolerance
                    assert!(
                        dot_product > -1e-10,
                        "Face {} normal points inward at vertex {} (dot product: {:.6})",
                        face_idx,
                        vertex_idx,
                        dot_product
                    );
                }
            }
        }
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
    fn test_cube_face_normals() {
        // **Mathematical Foundation**: Cube face normal validation
        // For a cube centered at origin with size 2.0 (half_size = 1.0),
        // face normals should be axis-aligned unit vectors

        let cube: IndexedMesh<()> = cube(2.0, None);

        // Cube should have 6 faces
        assert_eq!(cube.faces.len(), 6);

        // Expected face normals for axis-aligned cube
        let expected_normals = [
            Vector3::new(0.0, 0.0, -1.0), // bottom face (negative Z)
            Vector3::new(0.0, 0.0, 1.0),  // top face (positive Z)
            Vector3::new(0.0, -1.0, 0.0), // back face (negative Y)
            Vector3::new(0.0, 1.0, 0.0),  // front face (positive Y)
            Vector3::new(-1.0, 0.0, 0.0), // left face (negative X)
            Vector3::new(1.0, 0.0, 0.0),  // right face (positive X)
        ];

        // Validate each face normal
        for (i, face) in cube.faces.iter().enumerate() {
            assert!(face.normal.is_some(), "Face {} should have a normal", i);

            if let Some(normal) = face.normal {
                // Check that normal is unit length
                assert!(
                    (normal.norm() - 1.0).abs() < crate::float_types::EPSILON,
                    "Face {} normal should be unit length, got magnitude {}",
                    i,
                    normal.norm()
                );

                // Check that normal matches expected axis-aligned normal
                let expected = expected_normals[i];
                assert!(
                    (normal - expected).norm() < crate::float_types::EPSILON,
                    "Face {} normal should be {:?}, got {:?}",
                    i,
                    expected,
                    normal
                );
            }
        }
    }

    #[test]
    fn test_cube_vertex_normals() {
        // **Mathematical Foundation**: Cube vertex normal validation
        // For flat-shaded cube, vertex normals should be appropriate for their faces
        // Each vertex belongs to 3 faces and should have normals that support correct shading

        let cube: IndexedMesh<()> = cube(2.0, None);

        // Cube should have 8 vertices
        assert_eq!(cube.vertices.len(), 8);

        // For a cube with proper flat shading, vertex normals should be axis-aligned
        // Each vertex belongs to 3 faces, so vertex normals should match face normals
        let half_size = 1.0; // size = 2.0, so half_size = 1.0

        // Expected vertex positions and their corresponding face normals
        let vertex_expectations = [
            // Vertex 0: (-half_size, -half_size, -half_size) - bottom-back-left
            // Belongs to: bottom, back, left faces
            (
                Point3::new(-half_size, -half_size, -half_size),
                vec![
                    Vector3::new(0.0, 0.0, -1.0), // bottom
                    Vector3::new(0.0, -1.0, 0.0), // back
                    Vector3::new(-1.0, 0.0, 0.0), // left
                ],
            ),
            // Vertex 1: (half_size, -half_size, -half_size) - bottom-back-right
            // Belongs to: bottom, back, right faces
            (
                Point3::new(half_size, -half_size, -half_size),
                vec![
                    Vector3::new(0.0, 0.0, -1.0), // bottom
                    Vector3::new(0.0, -1.0, 0.0), // back
                    Vector3::new(1.0, 0.0, 0.0),  // right
                ],
            ),
            // Vertex 2: (half_size, half_size, -half_size) - bottom-front-right
            // Belongs to: bottom, front, right faces
            (
                Point3::new(half_size, half_size, -half_size),
                vec![
                    Vector3::new(0.0, 0.0, -1.0), // bottom
                    Vector3::new(0.0, 1.0, 0.0),  // front
                    Vector3::new(1.0, 0.0, 0.0),  // right
                ],
            ),
            // Vertex 3: (-half_size, half_size, -half_size) - bottom-front-left
            // Belongs to: bottom, front, left faces
            (
                Point3::new(-half_size, half_size, -half_size),
                vec![
                    Vector3::new(0.0, 0.0, -1.0), // bottom
                    Vector3::new(0.0, 1.0, 0.0),  // front
                    Vector3::new(-1.0, 0.0, 0.0), // left
                ],
            ),
            // Vertex 4: (-half_size, -half_size, half_size) - top-back-left
            // Belongs to: top, back, left faces
            (
                Point3::new(-half_size, -half_size, half_size),
                vec![
                    Vector3::new(0.0, 0.0, 1.0),  // top
                    Vector3::new(0.0, -1.0, 0.0), // back
                    Vector3::new(-1.0, 0.0, 0.0), // left
                ],
            ),
            // Vertex 5: (half_size, -half_size, half_size) - top-back-right
            // Belongs to: top, back, right faces
            (
                Point3::new(half_size, -half_size, half_size),
                vec![
                    Vector3::new(0.0, 0.0, 1.0),  // top
                    Vector3::new(0.0, -1.0, 0.0), // back
                    Vector3::new(1.0, 0.0, 0.0),  // right
                ],
            ),
            // Vertex 6: (half_size, half_size, half_size) - top-front-right
            // Belongs to: top, front, right faces
            (
                Point3::new(half_size, half_size, half_size),
                vec![
                    Vector3::new(0.0, 0.0, 1.0), // top
                    Vector3::new(0.0, 1.0, 0.0), // front
                    Vector3::new(1.0, 0.0, 0.0), // right
                ],
            ),
            // Vertex 7: (-half_size, half_size, half_size) - top-front-left
            // Belongs to: top, front, left faces
            (
                Point3::new(-half_size, half_size, half_size),
                vec![
                    Vector3::new(0.0, 0.0, 1.0),  // top
                    Vector3::new(0.0, 1.0, 0.0),  // front
                    Vector3::new(-1.0, 0.0, 0.0), // left
                ],
            ),
        ];

        // Validate vertex positions and normals
        for (i, vertex) in cube.vertices.iter().enumerate() {
            let (expected_pos, expected_face_normals) = &vertex_expectations[i];

            // Check vertex position
            assert!(
                (vertex.pos - expected_pos).norm() < crate::float_types::EPSILON,
                "Vertex {} position should be {:?}, got {:?}",
                i,
                expected_pos,
                vertex.pos
            );

            // Check vertex normal
            // For flat shading, vertex normal should be one of the face normals
            // (typically the normal of the primary face for that vertex)
            let vertex_normal = vertex.normal;

            // Check that vertex normal is unit length
            assert!(
                (vertex_normal.norm() - 1.0).abs() < crate::float_types::EPSILON,
                "Vertex {} normal should be unit length, got magnitude {}",
                i,
                vertex_normal.norm()
            );

            // Check that vertex normal matches one of the expected face normals
            let mut found_match = false;
            for expected_normal in expected_face_normals {
                if (vertex_normal - expected_normal).norm() < crate::float_types::EPSILON {
                    found_match = true;
                    break;
                }
            }

            assert!(
                found_match,
                "Vertex {} normal {:?} should match one of the expected face normals: {:?}",
                i, vertex_normal, expected_face_normals
            );
        }
    }

    #[test]
    fn test_cube_normal_consistency() {
        // **Mathematical Foundation**: Cube normal vector consistency
        // All normals should be unit length and properly oriented

        let cube: IndexedMesh<()> = cube(2.0, None);

        // Check all face normals
        for (i, face) in cube.faces.iter().enumerate() {
            if let Some(normal) = face.normal {
                // Face normal should be unit length
                assert!(
                    (normal.norm() - 1.0).abs() < crate::float_types::EPSILON,
                    "Face {} normal should be unit length, got {}",
                    i,
                    normal.norm()
                );

                // Face normal should be axis-aligned (one component is ±1, others are 0)
                let abs_normal = Vector3::new(normal.x.abs(), normal.y.abs(), normal.z.abs());
                let axis_sum = abs_normal.x + abs_normal.y + abs_normal.z;
                assert!(
                    (axis_sum - 1.0).abs() < crate::float_types::EPSILON,
                    "Face {} normal should be axis-aligned, got {:?} (sum of abs components: {})",
                    i,
                    normal,
                    axis_sum
                );
            } else {
                panic!("Face {} should have a normal", i);
            }
        }

        // Check all vertex normals
        for (i, vertex) in cube.vertices.iter().enumerate() {
            // Vertex normal should be unit length
            assert!(
                (vertex.normal.norm() - 1.0).abs() < crate::float_types::EPSILON,
                "Vertex {} normal should be unit length, got {}",
                i,
                vertex.normal.norm()
            );

            // Vertex normal should be axis-aligned (same as face normals)
            let abs_normal = Vector3::new(
                vertex.normal.x.abs(),
                vertex.normal.y.abs(),
                vertex.normal.z.abs(),
            );
            let axis_sum = abs_normal.x + abs_normal.y + abs_normal.z;
            assert!(
                (axis_sum - 1.0).abs() < crate::float_types::EPSILON,
                "Vertex {} normal should be axis-aligned, got {:?} (sum of abs components: {})",
                i,
                vertex.normal,
                axis_sum
            );
        }
    }

    #[test]
    fn test_cube_normal_orientation() {
        // **Mathematical Foundation**: Cube normal orientation validation
        // Normals should point outward from the cube surface

        let cube: IndexedMesh<()> = cube(2.0, None);
        let _half_size = 1.0;

        // For each face, check that the normal points outward
        for (i, face) in cube.faces.iter().enumerate() {
            if let Some(normal) = face.normal {
                // Get a vertex from this face to test orientation
                if !face.vertices.is_empty() {
                    let vertex_idx = face.vertices[0];
                    if vertex_idx < cube.vertices.len() {
                        let vertex_pos = cube.vertices[vertex_idx].pos;

                        // Compute vector from vertex to center
                        let to_center = Point3::origin() - vertex_pos;

                        // Dot product should be negative (normal points outward, to_center points inward)
                        let dot_product = normal.dot(&to_center);
                        assert!(
                            dot_product <= 0.0,
                            "Face {} normal should point outward (dot product <= 0), got dot product {} for normal {:?} and vertex {:?}",
                            i,
                            dot_product,
                            normal,
                            vertex_pos
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_cube_normal_edge_cases() {
        // **Mathematical Foundation**: Cube normal calculation edge cases
        // Test with different cube sizes and edge cases

        // Test with very small cube
        let small_cube: IndexedMesh<()> = cube(0.001, None);
        for face in &small_cube.faces {
            if let Some(normal) = face.normal {
                assert!((normal.norm() - 1.0).abs() < crate::float_types::EPSILON);
            }
        }

        // Test with large cube
        let large_cube: IndexedMesh<()> = cube(1000.0, None);
        for face in &large_cube.faces {
            if let Some(normal) = face.normal {
                assert!((normal.norm() - 1.0).abs() < crate::float_types::EPSILON);
            }
        }

        // Test with cube at origin (size = 2.0 is already at origin)
        let origin_cube: IndexedMesh<()> = cube(2.0, None);
        for vertex in &origin_cube.vertices {
            // For a cube centered at origin, all vertices should have coordinates with abs <= 1.0
            assert!(vertex.pos.x.abs() <= 1.0 + crate::float_types::EPSILON);
            assert!(vertex.pos.y.abs() <= 1.0 + crate::float_types::EPSILON);
            assert!(vertex.pos.z.abs() <= 1.0 + crate::float_types::EPSILON);
        }
    }

    #[test]
    fn test_cylinder_face_normals() {
        let cylinder: IndexedMesh<()> = cylinder(1.0, 2.0, 8, None);

        // Check that all faces have normals computed
        for (i, face) in cylinder.faces.iter().enumerate() {
            assert!(
                face.normal.is_some(),
                "Face {} should have a computed normal",
                i
            );
        }

        // Check specific face normal directions
        let segments = 8;

        // Side faces (0 to segments-1) should have radial normals
        for i in 0..segments {
            let face = &cylinder.faces[i];
            if let Some(normal) = face.normal {
                // For side faces, Z component should be approximately 0 (radial)
                assert!(
                    normal.z.abs() < 1e-6,
                    "Side face {} should have Z component near 0, got {}",
                    i,
                    normal.z
                );

                // Normal should be unit length
                assert!(
                    (normal.norm() - 1.0).abs() < 1e-6,
                    "Side face {} normal should be unit vector, got magnitude {}",
                    i,
                    normal.norm()
                );
            }
        }

        // Bottom faces (segments to 2*segments-1) should face down
        for i in segments..(2 * segments) {
            let face = &cylinder.faces[i];
            if let Some(normal) = face.normal {
                assert!(
                    (normal - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-6,
                    "Bottom face {} should have normal (0,0,-1), got {:?}",
                    i,
                    normal
                );
            }
        }

        // Top faces (2*segments to 3*segments-1) should face up
        for i in (2 * segments)..(3 * segments) {
            let face = &cylinder.faces[i];
            if let Some(normal) = face.normal {
                assert!(
                    (normal - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-6,
                    "Top face {} should have normal (0,0,1), got {:?}",
                    i,
                    normal
                );
            }
        }
    }

    #[test]
    fn test_invalid_cylinder_parameters() {
        let invalid: IndexedMesh<()> = cylinder(1.0, 2.0, 2, None); // segments < 3
        assert_eq!(invalid.vertices.len(), 0);
    }
}
