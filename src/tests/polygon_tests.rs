//! Tests for polygon operations and functionality

use crate::float_types::Real;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
//   Polygon Tests
// --------------------------------------------------------

#[test]
fn test_polygon_construction() {
    let v1 = Vertex::new(Point3::origin(), Vector3::y());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::y());
    let v3 = Vertex::new(Point3::new(1.0, 0.0, -1.0), Vector3::y());

    let poly: Polygon<()> = Polygon::new(vec![v1, v2, v3], None);
    assert_eq!(poly.vertices.len(), 3);
    // Plane should be defined by these three points. We expect a normal near ±Y.
    assert!(
        approx_eq(
            poly.plane.normal().dot(&Vector3::y()).abs(),
            1.0,
            crate::float_types::EPSILON
        ),
        "Expected plane normal to match ±Y"
    );
}

#[test]
fn test_polygon_new() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    assert_eq!(poly.vertices.len(), 3);
    assert!(poly.metadata.is_none());
}

#[test]
fn test_polygon_flip() {
    let v1 = Vertex::new(Point3::origin(), Vector3::z());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z());
    let v3 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z());

    let mut poly: Polygon<()> = Polygon::new(vec![v1, v2, v3], None);
    let original_normal = poly.plane.normal();

    poly.flip();

    // Normal should be negated after flip
    assert_eq!(poly.plane.normal(), -original_normal);
}

#[test]
fn test_polygon_recalc_plane_and_normals() {
    let v1 = Vertex::new(Point3::origin(), Vector3::z());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z());
    let v3 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z());

    let poly: Polygon<()> = Polygon::new(vec![v1, v2, v3], None);
    poly.calculate_new_normal();

    // All vertices should have the same normal as the polygon plane
    for vertex in &poly.vertices {
        assert_eq!(vertex.normal, poly.plane.normal());
    }
}

#[test]
fn test_polygon_subdivide_triangles() {
    let v1 = Vertex::new(Point3::origin(), Vector3::z());
    let v2 = Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z());
    let v3 = Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z());
    let v4 = Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z());

    let quad: Polygon<()> = Polygon::new(vec![v1, v2, v4, v3], None);

    // Subdivide once - quad becomes 2 triangles, each subdivides into 4 = 8 total
    let triangles = quad.subdivide_triangles(1.try_into().expect("Valid subdivision level"));
    assert_eq!(triangles.len(), 8);

    // Each triangle should have 3 vertices
    for triangle in triangles {
        assert_eq!(triangle.len(), 3);
    }
}

#[test]
fn test_polygon_triangulate() {
    let v1 = Vertex::new(Point3::origin(), Vector3::z());
    let v2 = Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z());
    let v3 = Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z());
    let v4 = Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z());

    let quad: Polygon<()> = Polygon::new(vec![v1, v2, v4, v3], None);

    // Triangulate quad - should create 2 triangles
    let triangles = quad.triangulate();
    assert_eq!(triangles.len(), 2);

    // Each triangle should have 3 vertices
    for triangle in triangles {
        assert_eq!(triangle.len(), 3);
    }
}

#[test]
fn test_polygon_metadata_custom_struct() {
    #[derive(Clone, Debug, PartialEq)]
    struct CustomMetadata {
        color: (u8, u8, u8),
        id: u32,
    }

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let metadata = CustomMetadata {
        color: (255, 0, 0),
        id: 42,
    };

    let poly: Polygon<CustomMetadata> = Polygon::new(vertices, Some(metadata.clone()));
    assert_eq!(poly.metadata, Some(metadata));
}

#[test]
fn test_polygon_metadata_integer() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<i32> = Polygon::new(vertices, Some(42));
    assert_eq!(poly.metadata, Some(42));
}

#[test]
fn test_polygon_metadata_string() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<String> = Polygon::new(vertices, Some("test".to_string()));
    assert_eq!(poly.metadata, Some("test".to_string()));
}

// Helper function for approximate equality
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}
