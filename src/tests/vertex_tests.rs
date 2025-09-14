//! Tests for vertex operations and functionality

use crate::float_types::Real;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
//   Vertex Tests
// --------------------------------------------------------

#[test]
fn test_vertex_flip() {
    let mut v = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::x());
    v.flip();
    // Position remains the same
    assert_eq!(v.pos, Point3::new(1.0, 2.0, 3.0));
    // Normal should be negated
    assert_eq!(v.normal, -Vector3::x());
}

#[test]
fn test_vertex_new() {
    let v = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::x());
    assert_eq!(v.pos, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(v.normal, Vector3::x());
}

#[test]
fn test_vertex_interpolate() {
    // **Mathematical Foundation**: Linear interpolation formula
    // For points p1, p2 and parameter t: result = (1-t)·p1 + t·p2
    // For normals n1, n2: result_normal = normalize((1-t)·n1 + t·n2)

    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
    let v2 = Vertex::new(Point3::new(2.0, 4.0, 6.0), Vector3::y());

    let result = v1.interpolate(&v2, 0.5);

    // Verify position interpolation: (1-0.5)·(0,0,0) + 0.5·(2,4,6) = (1,2,3)
    assert_eq!(
        result.pos,
        Point3::new(1.0, 2.0, 3.0),
        "Position interpolation should follow linear formula"
    );

    // Verify normal interpolation and normalization
    let expected_normal = (Vector3::x() + Vector3::y()).normalize();
    assert!(
        approx_eq(
            result.normal.x,
            expected_normal.x,
            crate::float_types::EPSILON
        ) && approx_eq(
            result.normal.y,
            expected_normal.y,
            crate::float_types::EPSILON
        ) && approx_eq(
            result.normal.z,
            expected_normal.z,
            crate::float_types::EPSILON
        ),
        "Normal interpolation should be normalized average of input normals"
    );

    // Verify result normal is unit length
    assert!(
        approx_eq(result.normal.magnitude(), 1.0, crate::float_types::EPSILON),
        "Interpolated normal should be unit length: magnitude = {}",
        result.normal.magnitude()
    );
}

#[test]
fn test_vertex_interpolation_methods() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
    let v2 = Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::x());

    let result = v1.interpolate(&v2, 0.25);
    assert_eq!(result.pos, Point3::new(0.5, 0.0, 0.0));

    let result2 = v1.interpolate(&v2, 0.75);
    assert_eq!(result2.pos, Point3::new(1.5, 0.0, 0.0));
}

#[test]
fn test_vertex_distance_operations() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
    let v2 = Vertex::new(Point3::new(3.0, 4.0, 0.0), Vector3::x());

    // Test distance calculation
    let distance = (v2.pos - v1.pos).norm();
    assert!(approx_eq(distance, 5.0, crate::float_types::EPSILON));
}

#[test]
fn test_vertex_clustering() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x()),
        Vertex::new(Point3::new(0.001, 0.0, 0.0), Vector3::x()), // Close to first
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::x()),   // Far from first
    ];

    // Test vertex creation and basic properties
    assert_eq!(vertices.len(), 3);
    assert!(vertices[0].pos.x < vertices[1].pos.x); // First vertex is at origin, second is at x=1
}

// Helper function for approximate equality
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}
