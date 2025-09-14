//! Tests for edge cases and numerical stability

use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::traits::CSG;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
//   Edge Case Tests: NaN, Infinity, and Numerical Stability
// --------------------------------------------------------

#[test]
fn test_nan_vertex_handling() {
    let vertices = vec![
        Vertex::new(Point3::new(Real::NAN, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mesh: Mesh<()> = Mesh::from_polygons(&[poly], None);

    // Should handle NaN gracefully without panicking
    let _bb = mesh.bounding_box();
    // Bounding box should handle NaN by using partial_min/max
}

#[test]
fn test_infinite_vertex_handling() {
    let vertices = vec![
        Vertex::new(Point3::new(Real::INFINITY, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mesh: Mesh<()> = Mesh::from_polygons(&[poly], None);

    // Should handle infinity gracefully
    let _bb = mesh.bounding_box();
}

#[test]
fn test_numerical_stability_extremes() {
    // **Mathematical Foundation**: Numerical stability with subnormal numbers
    // Test with very small numbers approaching subnormal range

    let tiny = 1e-10;
    let vertices = vec![
        Vertex::new(Point3::new(tiny, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(tiny + 1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(tiny, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mesh: Mesh<()> = Mesh::from_polygons(&[poly], None);

    let bb = mesh.bounding_box();

    // Verify bounding box calculation with tiny numbers
    assert!(
        bb.maxs.x > bb.mins.x,
        "Bounding box should have valid X range"
    );
    assert!(
        bb.maxs.y > bb.mins.y,
        "Bounding box should have valid Y range"
    );

    // Verify specific coordinate ranges are preserved
    assert!(
        approx_eq(bb.mins.x, tiny, tiny * 10.0),
        "Minimum X should match input tiny value"
    );
    assert!(
        approx_eq(bb.maxs.x, tiny + 1.0, (tiny + 1.0) * 1e-10),
        "Maximum X should match expected range"
    );
    assert!(
        approx_eq(bb.mins.y, 0.0, tiny * 10.0),
        "Minimum Y should be zero"
    );
    assert!(approx_eq(bb.maxs.y, 1.0, 1e-10), "Maximum Y should be one");
}

#[test]
fn test_overflow_underflow_operations() {
    // **Mathematical Foundation**: Numerical stability with large numbers
    // Test floating-point precision limits and overflow handling

    let large = 1e10;
    let vertices = vec![
        Vertex::new(Point3::new(large, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(large + 1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(large, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mesh: Mesh<()> = Mesh::from_polygons(&[poly], None);

    let bb = mesh.bounding_box();

    // Verify bounding box calculation with large numbers
    assert!(
        bb.maxs.x > bb.mins.x,
        "Bounding box should have valid X range with large numbers"
    );
    assert!(
        bb.maxs.y > bb.mins.y,
        "Bounding box should have valid Y range with large numbers"
    );

    // Verify specific coordinate ranges are preserved (within floating-point precision)
    let tolerance = large * crate::float_types::EPSILON * 1e3; // Allow for floating-point precision loss
    assert!(
        approx_eq(bb.mins.x, large, tolerance),
        "Minimum X should match large input value within precision tolerance"
    );
    assert!(
        approx_eq(bb.maxs.x, large + 1.0, tolerance),
        "Maximum X should match expected range within precision tolerance"
    );

    // Test with numbers approaching floating-point limits
    let near_max = Real::MAX / 1e3;
    let extreme_vertices = vec![
        Vertex::new(Point3::new(near_max, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(near_max + 1.0, 0.0, 0.0), Vector3::z()), // May overflow
        Vertex::new(Point3::new(near_max, 1.0, 0.0), Vector3::z()),
    ];

    let extreme_poly: Polygon<()> = Polygon::new(extreme_vertices, None);
    let extreme_mesh: Mesh<()> = Mesh::from_polygons(&[extreme_poly], None);

    // Should handle extreme values gracefully (may produce infinite/NaN results)
    let extreme_bb = extreme_mesh.bounding_box();
    // Just verify operation doesn't panic - specific bounds may be infinite
    assert!(
        extreme_bb.mins.x.is_finite() || extreme_bb.mins.x.is_infinite(),
        "Extreme value handling should produce finite or infinite bounds"
    );
}

#[test]
fn test_mathematical_correctness_validation() {
    // Test geometric invariants
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Volume should be conserved under rigid transformations
    let original_bb = cube.bounding_box();
    let translated = cube.translate(5.0, 3.0, 1.0);
    let translated_bb = translated.bounding_box();

    // Translation should preserve dimensions
    assert!(approx_eq(
        translated_bb.maxs.x - translated_bb.mins.x,
        original_bb.maxs.x - original_bb.mins.x,
        crate::float_types::EPSILON
    ));
    assert!(approx_eq(
        translated_bb.maxs.y - translated_bb.mins.y,
        original_bb.maxs.y - original_bb.mins.y,
        crate::float_types::EPSILON
    ));
    assert!(approx_eq(
        translated_bb.maxs.z - translated_bb.mins.z,
        original_bb.maxs.z - original_bb.mins.z,
        crate::float_types::EPSILON
    ));
}

#[test]
fn test_precision_dependent_boundary_cases() {
    // Test operations at the limits of floating-point precision
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

    let mesh1: Mesh<()> = Mesh::from_polygons(&[poly1], None);
    let mesh2: Mesh<()> = Mesh::from_polygons(&[poly2], None);

    // Operations should handle precision boundaries gracefully
    let union = mesh1.union(&mesh2);
    assert!(!union.polygons.is_empty());
}

#[test]
fn test_degenerate_geometry_edge_cases() {
    // Test with degenerate polygons (zero area)
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // Duplicate point
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // Duplicate point
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mesh: Mesh<()> = Mesh::from_polygons(&[poly], None);

    // Should handle degenerate geometry without panicking
    let _bb = mesh.bounding_box();
}

#[test]
fn test_empty_mesh_operations() {
    let empty: Mesh<()> = Mesh::new();
    let cube: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube");

    // Operations with empty mesh should not panic
    let _union = empty.union(&cube);
    let _diff = cube.difference(&empty);
    let _intersect = empty.intersection(&cube);

    // Results may be empty but should not panic
}

// Helper function for approximate equality
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}
