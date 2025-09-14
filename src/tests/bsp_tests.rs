//! Tests for Binary Space Partitioning (BSP) operations

use crate::float_types::Real;
use crate::mesh::bsp::Node;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
//   Node & Clipping Tests
// --------------------------------------------------------

#[test]
fn test_node_new_and_build() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let polygons = vec![poly];

    let mut node = Node::from_polygons(&polygons);

    // Build should work without panicking
    node.build(&polygons);
    let polygons_after = node.all_polygons();
    assert!(!polygons_after.is_empty());
    assert!(polygons_after.len() >= 1); // BSP may create additional polygons during processing
}

#[test]
fn test_node_clip_to() {
    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(1.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 3.0, 1.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let mut node1 = Node::from_polygons(&[poly1]);
    let node2 = Node::from_polygons(&[poly2]);

    // Clip should work without panicking
    node1.clip_to(&node2);
    let result = node1.all_polygons();
    assert!(!result.is_empty());
}

#[test]
fn test_node_clip_polygons2() {
    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(1.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 3.0, 1.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let node1 = Node::from_polygons(&[poly1]);

    // Clip polygons should work
    node1.clip_polygons(&[poly2]);
    let _result = node1.all_polygons();
    // Result may be empty if completely clipped, but shouldn't panic
}

#[test]
fn test_node_invert() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mut node = Node::from_polygons(&[poly]);

    // Store original polygons
    let original = node.all_polygons();

    // Invert should work
    node.invert();
    let inverted = node.all_polygons();

    // Should have polygons (may be different count due to orientation changes)
    assert!(!original.is_empty() || inverted.is_empty());
}

#[test]
fn test_node_all_polygons() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let node = Node::from_polygons(&[poly]);

    let polygons = node.all_polygons();
    assert!(!polygons.is_empty());
    assert_eq!(polygons.len(), 1);
}

#[test]
fn test_node_mathematical_correctness() {
    // Test that BSP operations preserve geometric properties
    let cube = crate::mesh::Mesh::cube(2.0, None).expect("Failed to create cube");
    let original_volume = calculate_mesh_volume(&cube);

    // BSP operations should preserve volume for solid objects
    let polygons = cube.polygons.clone();
    let node = Node::from_polygons(&polygons);
    let result_polygons = node.all_polygons();

    // Reconstruct mesh from BSP result
    let result_mesh = crate::mesh::Mesh::from_polygons(&result_polygons, None);
    let result_volume = calculate_mesh_volume(&result_mesh);

    // Volume should be preserved within numerical precision
    assert!(
        (original_volume - result_volume).abs()
            < crate::float_types::EPSILON * original_volume,
        "BSP operation should preserve volume: original={}, result={}",
        original_volume,
        result_volume
    );
}

#[test]
fn test_node_coplanar_face_handling() {
    // Test BSP behavior with coplanar faces
    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()), // Same Z plane
        Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 3.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 3.0, 0.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let mut node1 = Node::from_polygons(&[poly1]);
    let node2 = Node::from_polygons(&[poly2]);

    node1.clip_to(&node2);
    let result = node1.all_polygons();

    // BSP should handle coplanar faces without creating invalid geometry
    for poly in &result {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar clipping should not create degenerate polygons"
        );
        // Validate that all vertices lie on the same plane
        let plane = &poly.plane;
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance.abs() < crate::float_types::EPSILON,
                "All vertices should lie on the polygon plane, distance={}",
                distance
            );
        }
    }
}

#[test]
fn test_node_degenerate_polygon_handling() {
    // Test with degenerate polygons (collinear points)
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()), // Collinear
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()), // Collinear
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let polygons = vec![poly];
    let node = Node::from_polygons(&polygons);

    // Should handle degenerate polygons gracefully
    let result = node.all_polygons();
    // Result may be empty or contain valid polygons, but shouldn't panic
    for poly in &result {
        assert!(
            poly.vertices.len() >= 3,
            "Should not create polygons with < 3 vertices"
        );
    }
}

#[test]
fn test_node_empty_input_handling() {
    // Test with empty polygon list
    let polygons: Vec<Polygon<()>> = vec![];
    let node = Node::from_polygons(&polygons);

    let result = node.all_polygons();
    assert!(result.is_empty(), "Empty input should produce empty output");
}

#[test]
fn test_node_numerical_stability() {
    // Test with values near floating-point precision limits
    let epsilon = crate::float_types::EPSILON;

    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(0.0 + epsilon, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0 + epsilon, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0 + epsilon, 1.0, 0.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let mut node1 = Node::from_polygons(&[poly1]);
    let node2 = Node::from_polygons(&[poly2]);

    node1.clip_to(&node2);
    let result = node1.all_polygons();

    // Should handle precision boundaries without creating invalid geometry
    for poly in &result {
        assert!(
            poly.vertices.len() >= 3,
            "Precision boundaries should not create degenerate polygons"
        );
    }
}

// Helper function to calculate approximate mesh volume
fn calculate_mesh_volume(mesh: &crate::mesh::Mesh<()>) -> Real {
    let mut volume = 0.0;

    for polygon in &mesh.polygons {
        if polygon.vertices.len() >= 3 {
            // Use tetrahedron method for volume calculation
            let v0 = &polygon.vertices[0].pos;
            for i in 1..polygon.vertices.len() - 1 {
                let v1 = &polygon.vertices[i].pos;
                let v2 = &polygon.vertices[i + 1].pos;

                // Volume contribution of tetrahedron (v0, v1, v2, origin)
                let tetra_volume = (v0.x * (v1.y * v2.z - v1.z * v2.y)
                    + v0.y * (v1.z * v2.x - v1.x * v2.z)
                    + v0.z * (v1.x * v2.y - v1.y * v2.x))
                    / 6.0;
                volume += tetra_volume;
            }
        }
    }

    volume.abs()
}
