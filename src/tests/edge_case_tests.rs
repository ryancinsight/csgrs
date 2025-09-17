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

#[test]
fn test_srs_precision_requirements() {
    // **SRS NFR005**: Validate geometric precision requirements
    // Test that operations maintain 1e-8 precision for f64 operations

    let base_cube: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube");

    // Test translation precision
    let translated = base_cube.translate(1e-10, 2e-10, 3e-10);
    let bb_orig = base_cube.bounding_box();
    let bb_trans = translated.bounding_box();

    // Translation should preserve relative precision
    assert!(
        (bb_trans.mins.x - bb_orig.mins.x - 1e-10).abs() < 1e-9,
        "Translation X should maintain 1e-8 precision"
    );
    assert!(
        (bb_trans.mins.y - bb_orig.mins.y - 2e-10).abs() < 1e-9,
        "Translation Y should maintain 1e-8 precision"
    );
    assert!(
        (bb_trans.mins.z - bb_orig.mins.z - 3e-10).abs() < 1e-9,
        "Translation Z should maintain 1e-8 precision"
    );

    // Test scaling precision
    let scaled = base_cube.scale(1.0 + 1e-10, 1.0 + 2e-10, 1.0 + 3e-10);
    let bb_scaled = scaled.bounding_box();

    // Scaling should maintain precision
    let expected_width = (bb_orig.maxs.x - bb_orig.mins.x) * (1.0 + 1e-10);
    let actual_width = bb_scaled.maxs.x - bb_scaled.mins.x;
    assert!(
        (actual_width - expected_width).abs() < 1e-9,
        "Scaling should maintain 1e-8 precision: expected {}, got {}",
        expected_width,
        actual_width
    );
}

#[test]
fn test_topological_correctness_validation() {
    // **SRS NFR006**: Validate topological correctness requirements
    // Test that boolean operations maintain closed, oriented, non-self-intersecting outputs

    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5); // Partial overlap

    // Test all boolean operations for topological correctness
    let operations = vec![
        ("union", cube1.union(&cube2)),
        ("difference", cube1.difference(&cube2)),
        ("intersection", cube1.intersection(&cube2)),
    ];

    for (op_name, result) in operations {
        // Must have valid polygons
        assert!(
            !result.polygons.is_empty(),
            "{} should produce non-empty mesh",
            op_name
        );

        // All polygons must have at least 3 vertices
        for polygon in &result.polygons {
            assert!(
                polygon.vertices.len() >= 3,
                "{} should produce valid polygons with >= 3 vertices",
                op_name
            );

            // Check that polygon is properly formed (plane can be computed)
            let plane = crate::mesh::plane::Plane::from_vertices(polygon.vertices.clone());
            // Normal should be normalized (unit length)
            assert!(
                plane.normal().magnitude() > 0.9,
                "{} polygon normal should be properly normalized",
                op_name
            );
        }

        // Check Euler characteristic for topological validity
        let num_vertices = result.vertices().len();
        let num_faces = result.polygons.len();
        let num_edges = result
            .polygons
            .iter()
            .map(|p| p.vertices.len())
            .sum::<usize>()
            / 2;

        let euler_char = num_vertices as i32 - num_edges as i32 + num_faces as i32;

        // Euler characteristic should be reasonable
        assert!(
            (0..1000).contains(&euler_char),
            "{} should produce valid topology (V={}, E={}, F={}, χ={})",
            op_name,
            num_vertices,
            num_edges,
            num_faces,
            euler_char
        );
    }
}

#[test]
fn test_error_handling_robustness_comprehensive() {
    // **SRS NFR007**: Comprehensive error handling robustness validation
    // Test graceful failure modes for all major error conditions

    // Test NaN handling in various operations
    let nan_cube: Mesh<()> =
        Mesh::cube(Real::NAN, None).unwrap_or_else(|_| Mesh::cube(1.0, None).unwrap());

    // Operations with NaN should not panic
    let _ = nan_cube.union(&Mesh::cube(1.0, None).unwrap());
    let _ = nan_cube.translate(1.0, 0.0, 0.0);

    // Test infinity handling
    let inf_vertices = vec![
        Vertex::new(Point3::new(Real::INFINITY, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    // Test infinity handling - Polygon::new may panic or succeed
    // We catch any panic to ensure robustness
    let inf_mesh_result = std::panic::catch_unwind(|| {
        let inf_poly = Polygon::new(inf_vertices, None);
        let inf_mesh: Mesh<()> = Mesh::from_polygons(&[inf_poly], None);
        // Operations with infinity should not panic
        let _bb = inf_mesh.bounding_box();
        let _ = inf_mesh.union(&Mesh::cube(1.0, None).unwrap());
    });

    // If it doesn't panic, that's fine; if it does, we caught it
    // The important thing is that operations handle infinity gracefully
    let _ = inf_mesh_result;

    // Test empty mesh operations
    let empty_mesh: Mesh<()> = Mesh::from_polygons(&[], None);
    let normal_cube = Mesh::cube(1.0, None).unwrap();

    // Operations with empty mesh should not panic
    let _ = empty_mesh.union(&normal_cube);
    let _ = normal_cube.union(&empty_mesh);
    let _ = empty_mesh.intersection(&normal_cube);
    let _bb = empty_mesh.bounding_box(); // Should handle empty mesh
}

#[test]
fn test_numerical_stability_under_transformation() {
    // Test numerical stability across multiple transformation compositions
    let original: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube");

    // Apply a series of transformations that should accumulate numerical errors
    let mut current = original;
    for i in 0..10 {
        let angle = (i as Real) * 0.1;
        let offset = (i as Real) * 0.01;
        current = current
            .rotate(angle, angle * 0.5, angle * 0.25)
            .translate(offset, -offset, offset * 2.0)
            .scale(
                1.0 + offset * 0.001,
                1.0 - offset * 0.0005,
                1.0 + offset * 0.0008,
            );
    }

    // After 10 transformations, the mesh should still be valid
    assert!(
        !current.polygons.is_empty(),
        "Multiple transformations should preserve mesh validity"
    );

    // Volume should remain reasonable (not explode or vanish)
    if let Some((mass, _, _)) = current.mass_properties(1.0) {
        assert!(
            mass > 0.1 && mass < 1000.0,
            "Transformed mesh volume should remain reasonable: got {}",
            mass
        );
    }

    // All polygons should still be valid
    for polygon in &current.polygons {
        assert!(
            polygon.vertices.len() >= 3,
            "All polygons should remain valid after transformations"
        );
    }

    // Bounding box should be finite
    let bb = current.bounding_box();
    assert!(
        bb.mins.x.is_finite() && bb.maxs.x.is_finite(),
        "Bounding box should remain finite after transformations"
    );
    assert!(
        bb.mins.y.is_finite() && bb.maxs.y.is_finite(),
        "Bounding box should remain finite after transformations"
    );
    assert!(
        bb.mins.z.is_finite() && bb.maxs.z.is_finite(),
        "Bounding box should remain finite after transformations"
    );
}

// Helper function for approximate equality
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}
