//! Comprehensive tests for winding order and normal vector consistency
//!
//! This module validates that normal vectors are correctly oriented based on
//! polygon vertex winding order (clockwise vs counterclockwise) and that
//! winding is preserved through geometric operations.
//!
//! ## Mathematical Foundation
//!
//! ### Winding Order and Normal Orientation
//! For a polygon in 3D space, the winding order determines the normal direction:
//! - **Counterclockwise (CCW)** winding: Normal points outward (right-hand rule)
//! - **Clockwise (CW)** winding: Normal points inward (left-hand rule)
//!
//! ### Right-Hand Rule for Normal Calculation
//! For vertices A, B, C in CCW order:
//! ```text
//! n⃗ = (B - A) × (C - A)
//! ```
//! The normal direction follows the right-hand rule when curling fingers from A→B→C.
//!
//! ### Winding Detection Algorithm
//! The winding order can be determined by the sign of the normal's Z-component
//! when projected onto the XY plane:
//! - **Positive Z**: Counterclockwise (CCW)
//! - **Negative Z**: Clockwise (CW)
//!
//! ### Consistency Requirements
//! 1. **Normal Direction**: Must match winding order (right-hand rule)
//! 2. **Winding Preservation**: Operations must maintain consistent winding
//! 3. **Orientation Consistency**: Adjacent polygons must have consistent normal directions
//! 4. **Degenerate Handling**: Proper fallback for collinear/degenerate vertices

use crate::float_types::Real;
use crate::mesh::plane::Plane;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::traits::CSG;
use nalgebra::{Point3, Vector3};

// ============================================================
//   WINDING ORDER DETECTION AND VALIDATION
// ============================================================

#[test]
fn test_winding_order_detection_ccw() {
    // **Mathematical Foundation**: Counterclockwise winding detection
    // For CCW vertices A, B, C: (B-A) × (C-A) should have positive Z-component

    let vertices_ccw = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices_ccw);
    let normal = plane.normal();

    // CCW winding should produce positive Z-component
    assert!(
        normal.z > 0.0,
        "CCW winding should produce positive Z normal, got: {:?}",
        normal
    );

    // Verify magnitude is approximately 1
    assert!(
        approx_eq(normal.norm(), 1.0, crate::float_types::EPSILON),
        "Normal should be unit length, magnitude: {}",
        normal.norm()
    );
}

#[test]
fn test_winding_order_detection_cw() {
    // **Mathematical Foundation**: Clockwise winding detection
    // For CW vertices A, B, C: (B-A) × (C-A) should have negative Z-component

    let vertices_cw = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()), // Swapped order
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()), // Swapped order
    ];

    let plane = Plane::from_vertices(vertices_cw);
    let normal = plane.normal();

    // CW winding should produce negative Z-component
    assert!(
        normal.z < 0.0,
        "CW winding should produce negative Z normal, got: {:?}",
        normal
    );

    // Verify magnitude is approximately 1
    assert!(
        approx_eq(normal.norm(), 1.0, crate::float_types::EPSILON),
        "Normal should be unit length, magnitude: {}",
        normal.norm()
    );
}

#[test]
fn test_winding_consistency_triangle_variations() {
    // **Mathematical Foundation**: Winding consistency across triangle variations
    // All triangles with same winding should produce consistent normal directions

    let triangle_variations = [
        // Standard CCW triangle
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
        ],
        // Offset CCW triangle
        vec![
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 3.0, 0.0), Vector3::z()),
        ],
        // Different scale CCW triangle
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.5, 0.0), Vector3::z()),
        ],
    ];

    let mut first_normal: Option<Vector3<Real>> = None;
    for (i, vertices) in triangle_variations.iter().enumerate() {
        let plane = Plane::from_vertices(vertices.clone());
        let normal = plane.normal();

        // All should have positive Z (CCW winding)
        assert!(
            normal.z > 0.0,
            "Triangle variation {} should have CCW winding (positive Z), got: {:?}",
            i,
            normal
        );

        if let Some(first) = first_normal {
            // Normals should point in same general direction
            let dot_product = first.dot(&normal);
            assert!(
                dot_product > 0.9, // Allow for small numerical differences
                "Normals should be consistent across variations, dot product: {}",
                dot_product
            );
        } else {
            first_normal = Some(normal);
        }
    }
}

#[test]
fn test_winding_preservation_flip_operation() {
    // **Mathematical Foundation**: Winding preservation through flip operations
    // flip() should reverse both winding order and normal direction

    let vertices_ccw = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let mut polygon: Polygon<()> = Polygon::new(vertices_ccw, None);
    let original_normal = polygon.plane.normal();

    // Verify original is CCW
    assert!(
        original_normal.z > 0.0,
        "Original polygon should be CCW with positive Z normal"
    );

    // Flip the polygon
    polygon.flip();
    let flipped_normal = polygon.plane.normal();

    // Verify flipped is CW (negative Z)
    assert!(
        flipped_normal.z < 0.0,
        "Flipped polygon should be CW with negative Z normal"
    );

    // Verify normals are exact opposites
    let dot_product = original_normal.dot(&flipped_normal);
    assert!(
        approx_eq(dot_product, -1.0, crate::float_types::EPSILON),
        "Flipped normal should be exact opposite, dot product: {}",
        dot_product
    );

    // Verify vertex order is reversed
    assert_eq!(polygon.vertices[0].pos, Point3::new(0.0, 1.0, 0.0)); // Last becomes first
    assert_eq!(polygon.vertices[1].pos, Point3::new(1.0, 0.0, 0.0)); // Middle stays middle
    assert_eq!(polygon.vertices[2].pos, Point3::new(0.0, 0.0, 0.0)); // First becomes last
}

#[test]
fn test_winding_consistency_complex_polygon() {
    // **Mathematical Foundation**: Winding consistency for complex polygons
    // Complex polygons should maintain consistent winding throughout

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should be CCW with positive Z
    assert!(
        normal.z > 0.0,
        "Complex polygon should maintain CCW winding with positive Z normal, got: {:?}",
        normal
    );

    // Verify unit length
    assert!(
        approx_eq(normal.norm(), 1.0, crate::float_types::EPSILON),
        "Normal should be unit length, magnitude: {}",
        normal.norm()
    );
}

// ============================================================
//   NEWELL'S METHOD WINDING VALIDATION
// ============================================================

#[test]
fn test_newells_method_winding_consistency() {
    // **Mathematical Foundation**: Newell's method for polygon normal calculation
    // Newell's method should produce consistent results with cross product method

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);

    // Get plane normal (cross product method)
    let plane_normal = polygon.plane.normal();

    // Get Newell's method normal
    let newell_normal = polygon.calculate_new_normal();

    // Should be very close (allowing for numerical differences)
    let dot_product = plane_normal.dot(&newell_normal);
    assert!(
        approx_eq(dot_product, 1.0, crate::float_types::EPSILON * 10.0),
        "Newell's method should agree with cross product method, dot product: {}",
        dot_product
    );

    // Both should be unit length
    assert!(
        approx_eq(plane_normal.norm(), 1.0, crate::float_types::EPSILON),
        "Plane normal should be unit length"
    );
    assert!(
        approx_eq(newell_normal.norm(), 1.0, crate::float_types::EPSILON),
        "Newell's normal should be unit length"
    );
}

#[test]
fn test_newells_method_winding_robustness() {
    // **Mathematical Foundation**: Newell's method robustness to vertex ordering
    // Newell's method should handle various vertex orderings consistently

    let base_vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    ];

    // Test different rotations of the same polygon
    for rotation in 0..4 {
        let mut rotated_vertices = base_vertices.clone();
        rotated_vertices.rotate_left(rotation);

        let vertices: Vec<Vertex> = rotated_vertices
            .into_iter()
            .map(|pos| Vertex::new(pos, Vector3::z()))
            .collect();

        let polygon: Polygon<()> = Polygon::new(vertices, None);
        let newell_normal = polygon.calculate_new_normal();

        // Should always produce positive Z for CCW winding
        assert!(
            newell_normal.z > 0.0,
            "Newell's method should preserve winding for rotation {}, normal: {:?}",
            rotation,
            newell_normal
        );

        // Should be unit length
        assert!(
            approx_eq(newell_normal.norm(), 1.0, crate::float_types::EPSILON),
            "Newell's normal should be unit length for rotation {}: magnitude {}",
            rotation,
            newell_normal.norm()
        );
    }
}

// ============================================================
//   WINDING EDGE CASES AND ROBUSTNESS
// ============================================================

#[test]
fn test_winding_degenerate_collinear_vertices() {
    // **Mathematical Foundation**: Degenerate case handling
    // Collinear vertices should produce zero cross product and handle gracefully

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()), // Collinear
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()), // Collinear
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should handle gracefully (fallback behavior)
    assert!(
        normal.x.is_finite(),
        "Normal X should be finite for collinear vertices"
    );
    assert!(
        normal.y.is_finite(),
        "Normal Y should be finite for collinear vertices"
    );
    assert!(
        normal.z.is_finite(),
        "Normal Z should be finite for collinear vertices"
    );

    // Magnitude might not be exactly 1 for degenerate cases
    assert!(normal.norm().is_finite(), "Normal magnitude should be finite");
}

#[test]
fn test_winding_extreme_coordinates() {
    // **Mathematical Foundation**: Numerical stability with extreme coordinates
    // Winding detection should work with very large and very small coordinates

    let large_coord = 1e10;
    let small_coord = 1e-10;

    // Test with very large coordinates
    let large_vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(large_coord, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, large_coord, 0.0), Vector3::z()),
    ];

    let large_plane = Plane::from_vertices(large_vertices);
    let large_normal = large_plane.normal();

    assert!(
        large_normal.norm().is_finite(),
        "Large coordinate normal should be finite"
    );
    assert!(
        large_normal.z > 0.0,
        "Large coordinate triangle should maintain CCW winding"
    );

    // Test with very small coordinates
    let small_vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(small_coord, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, small_coord, 0.0), Vector3::z()),
    ];

    let small_plane = Plane::from_vertices(small_vertices);
    let small_normal = small_plane.normal();

    assert!(
        small_normal.norm().is_finite(),
        "Small coordinate normal should be finite"
    );
    assert!(
        small_normal.z > 0.0,
        "Small coordinate triangle should maintain CCW winding"
    );
}

#[test]
fn test_winding_precision_boundary() {
    // **Mathematical Foundation**: Precision boundary testing
    // Test winding detection near floating-point precision limits

    let epsilon = crate::float_types::EPSILON;

    // Test with coordinates near epsilon
    for i in 0..5 {
        let offset = epsilon * (10.0_f64).powi(i);
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0 + offset, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0 + offset, 0.0), Vector3::z()),
        ];

        let plane = Plane::from_vertices(vertices);
        let normal = plane.normal();

        // Should still produce valid CCW winding
        assert!(
            normal.z > 0.0,
            "Should maintain CCW winding near precision boundary {}",
            offset
        );
        assert!(
            normal.norm().is_finite(),
            "Normal should be finite near precision boundary {}",
            offset
        );
    }
}

// ============================================================
//   WINDING PRESERVATION IN GEOMETRIC OPERATIONS
// ============================================================

#[test]
fn test_winding_preservation_triangulation() {
    // **Mathematical Foundation**: Winding preservation in triangulation
    // Triangulated polygons should maintain consistent winding

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 3.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 3.0, 0.0), Vector3::z()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);
    let original_normal = polygon.plane.normal();

    let triangles = polygon.triangulate();

    // All triangles should have normals consistent with original winding
    for (i, triangle) in triangles.iter().enumerate() {
        let triangle_normal = (triangle[1].pos - triangle[0].pos)
            .cross(&(triangle[2].pos - triangle[0].pos))
            .normalize();

        let dot_product = original_normal.dot(&triangle_normal);
        assert!(
            dot_product > 0.9, // Allow for small numerical differences
            "Triangle {} should maintain winding consistency, dot product: {}",
            i,
            dot_product
        );
    }
}

#[test]
fn test_winding_consistency_mesh_operations() {
    // **Mathematical Foundation**: Winding consistency in mesh boolean operations
    // Boolean operations should preserve winding consistency where possible

    // Create two cubes with consistent winding
    let cube1 = crate::mesh::Mesh::<()>::cube(1.0, None).expect("Failed to create cube");
    let cube2 = crate::mesh::Mesh::<()>::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5);

    // Union operation
    let union_result = cube1.union(&cube2);

    // Check that all polygons maintain consistent winding
    for (i, polygon) in union_result.polygons.iter().enumerate() {
        let calculated_normal = polygon.calculate_new_normal();
        let plane_normal = polygon.plane.normal();

        let dot_product = calculated_normal.dot(&plane_normal);
        assert!(
            approx_eq(dot_product.abs(), 1.0, crate::float_types::EPSILON * 10.0),
            "Polygon {} should have consistent winding, dot product: {}",
            i,
            dot_product
        );
    }
}

// ============================================================
//   MATHEMATICAL VALIDATION OF WINDING FORMULAS
// ============================================================

#[test]
fn test_winding_mathematical_cross_product_formula() {
    // **Mathematical Foundation**: Cross product formula validation
    // Verify that n⃗ = (B - A) × (C - A) produces correct winding

    let a = Point3::new(1.0, 1.0, 0.0);
    let b = Point3::new(3.0, 1.0, 0.0);
    let c = Point3::new(2.0, 3.0, 0.0);

    let vertices = vec![
        Vertex::new(a, Vector3::z()),
        Vertex::new(b, Vector3::z()),
        Vertex::new(c, Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);

    // Manual cross product calculation
    let ab = b - a;
    let ac = c - a;
    let manual_normal = ab.cross(&ac).normalize();

    let plane_normal = plane.normal();

    // Should match exactly
    let dot_product = manual_normal.dot(&plane_normal);
    assert!(
        approx_eq(dot_product, 1.0, crate::float_types::EPSILON),
        "Cross product formula should match plane normal calculation, dot product: {}",
        dot_product
    );
}

#[test]
fn test_winding_right_hand_rule_validation() {
    // **Mathematical Foundation**: Right-hand rule validation
    // Verify that curling fingers from first to second to third vertex
    // produces thumb in normal direction

    let test_cases = [
        // CCW triangle in XY plane
        (
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
            ],
            1.0, // Expected Z component sign (positive for CCW)
        ),
        // CW triangle in XY plane
        (
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            ],
            -1.0, // Expected Z component sign (negative for CW)
        ),
    ];

    for (i, (vertices, expected_sign)) in test_cases.iter().enumerate() {
        let plane = Plane::from_vertices(vertices.clone());
        let normal = plane.normal();

        assert!(
            (normal.z * expected_sign) > 0.0,
            "Test case {} should follow right-hand rule, normal Z: {}, expected sign: {}",
            i,
            normal.z,
            expected_sign
        );
    }
}

#[test]
fn test_winding_normal_interpolation_consistency() {
    // **Mathematical Foundation**: Normal interpolation winding consistency
    // Interpolated normals should maintain winding consistency

    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)); // Up normal
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)); // Up normal

    let interpolated = v1.interpolate(&v2, 0.5);

    // Interpolated normal should maintain upward direction
    assert!(
        interpolated.normal.z > 0.5, // Should be mostly upward
        "Interpolated normal should maintain winding consistency, Z component: {}",
        interpolated.normal.z
    );

    // Should be unit length
    assert!(
        approx_eq(interpolated.normal.norm(), 1.0, crate::float_types::EPSILON),
        "Interpolated normal should be unit length: magnitude {}",
        interpolated.normal.norm()
    );
}

#[test]
fn test_winding_normal_complex_non_planar_polygon() {
    // **Mathematical Foundation**: Complex non-planar polygon winding
    // Tests winding detection for polygons that are not perfectly planar
    // Should still detect dominant winding direction despite minor deviations

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.1), Vector3::z()), // slight Z deviation
        Vertex::new(Point3::new(2.0, 2.0, -0.1), Vector3::z()), // slight Z deviation
        Vertex::new(Point3::new(0.0, 2.0, 0.05), Vector3::z()), // slight Z deviation
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should still detect CCW winding despite non-planarity
    assert!(
        normal.z > 0.0,
        "Non-planar polygon should still have positive Z normal for CCW winding, got {:?}",
        normal
    );
}

#[test]
fn test_winding_normal_self_intersecting_polygon() {
    // **Mathematical Foundation**: Self-intersecting polygon winding
    // Tests winding detection for polygons that cross themselves
    // The result depends on the specific triangulation and may vary

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.5, 0.0), Vector3::z()), // causes self-intersection
        Vertex::new(Point3::new(2.0, -1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, -2.0, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Self-intersecting polygons can produce unpredictable results
    // The important thing is that we get a reasonable unit normal
    assert!(
        normal.norm() > 0.9, // Should be close to unit length
        "Self-intersecting polygon should produce reasonable normal magnitude, got {:?}",
        normal
    );

    // The normal should be a valid unit vector
    assert!(
        (normal.norm() - 1.0).abs() < crate::float_types::EPSILON * 10.0,
        "Self-intersecting polygon normal should be unit length, got magnitude {}",
        normal.norm()
    );
}

#[test]
fn test_winding_normal_precision_boundary_subnormal() {
    // **Mathematical Foundation**: Precision boundary with subnormal numbers
    // Tests winding detection with extremely small coordinates approaching subnormal range
    // IEEE 754 subnormal numbers: |x| < 2^(-126) for f32, |x| < 2^(-1022) for f64

    let tiny = 1e-40; // Well into subnormal range for f64
    let vertices = vec![
        Vertex::new(Point3::new(tiny, tiny, 0.0), Vector3::z()),
        Vertex::new(Point3::new(-tiny, tiny, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, -tiny, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should still correctly detect CCW winding even with subnormal coordinates
    assert!(
        normal.z > 0.0,
        "Subnormal coordinate polygon should have positive Z normal for CCW winding, got {:?}",
        normal
    );

    // Normal should be unit length despite tiny input coordinates
    assert!(
        (normal.norm() - 1.0).abs() < crate::float_types::EPSILON * 10.0,
        "Normal should be unit length even with subnormal coordinates, got magnitude {}",
        normal.norm()
    );
}

#[test]
fn test_winding_normal_extreme_coordinates() {
    // **Mathematical Foundation**: Extreme coordinate values
    // Tests winding detection with very large coordinate values
    // Should handle precision loss and maintain correct winding detection

    let huge = 1e15;
    let vertices = vec![
        Vertex::new(Point3::new(huge, huge, 0.0), Vector3::z()),
        Vertex::new(Point3::new(-huge, huge, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, -huge, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should still correctly detect CCW winding even with extreme coordinates
    assert!(
        normal.z > 0.0,
        "Extreme coordinate polygon should have positive Z normal for CCW winding, got {:?}",
        normal
    );
}

#[test]
fn test_winding_normal_collinear_vertices_with_noise() {
    // **Mathematical Foundation**: Nearly collinear vertices with small perturbations
    // Tests robustness when vertices are almost but not quite collinear
    // Should still produce reasonable normal vectors

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.001, 0.0), Vector3::z()), // small Y perturbation
        Vertex::new(Point3::new(2.0, 0.002, 0.0), Vector3::z()), // small Y perturbation
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should produce a reasonable normal despite near-collinearity
    assert!(
        normal.norm() > 0.1, // Should not be near-zero
        "Near-collinear vertices should still produce reasonable normal, got {:?}",
        normal
    );

    // Z component should still be positive for this winding
    assert!(
        normal.z > 0.0,
        "Near-collinear vertices should maintain correct winding direction, got Z={}",
        normal.z
    );
}

#[test]
fn test_winding_normal_minimum_three_vertices() {
    // **Mathematical Foundation**: Minimum vertex count for winding detection
    // Tests that exactly 3 vertices produce correct winding detection
    // Any fewer vertices should be handled gracefully

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should correctly detect CCW winding for exactly 3 vertices
    assert!(
        normal.z > 0.0,
        "Exactly 3 vertices should produce correct CCW winding, got Z={}",
        normal.z
    );
}

#[test]
fn test_winding_normal_large_vertex_count() {
    // **Mathematical Foundation**: Large polygon vertex counts
    // Tests winding detection with many vertices (stress test)
    // Should maintain correctness and performance with large polygons

    let mut vertices = Vec::new();
    let num_vertices = 100; // Large polygon

    // Create a large CCW circle
    for i in 0..num_vertices {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (num_vertices as f64);
        let x = angle.cos();
        let y = angle.sin();
        vertices.push(Vertex::new(Point3::new(x, y, 0.0), Vector3::z()));
    }

    let plane = Plane::from_vertices(vertices);
    let normal = plane.normal();

    // Should correctly detect CCW winding even with many vertices
    assert!(
        normal.z > 0.0,
        "Large vertex count polygon should maintain correct CCW winding, got Z={}",
        normal.z
    );
}

#[test]
fn test_winding_normal_arbitrary_plane_orientation() {
    // **Mathematical Foundation**: Arbitrary plane orientations
    // Tests winding detection for polygons not in XY plane
    // Should correctly handle different plane orientations

    // Polygon in XZ plane (rotated around Y axis) - CCW when viewed from positive Y
    let vertices_xz = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::y()),
        Vertex::new(Point3::new(0.5, 0.0, 1.0), Vector3::y()), // Correct CCW order
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y()),
    ];

    let plane_xz = Plane::from_vertices(vertices_xz);
    let normal_xz = plane_xz.normal();

    // Should detect positive Y normal for this orientation (CCW in XZ plane)
    assert!(
        normal_xz.y > 0.0,
        "XZ plane polygon should have positive Y normal, got {:?}",
        normal_xz
    );

    // Polygon in YZ plane (rotated around X axis)
    let vertices_yz = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::x()),
        Vertex::new(Point3::new(0.0, 0.5, 1.0), Vector3::x()),
    ];

    let plane_yz = Plane::from_vertices(vertices_yz);
    let normal_yz = plane_yz.normal();

    // Should detect positive X normal for this orientation
    assert!(
        normal_yz.x > 0.0,
        "YZ plane polygon should have positive X normal, got {:?}",
        normal_yz
    );
}

#[test]
fn test_winding_normal_vertex_order_sensitivity() {
    // **Mathematical Foundation**: Vertex order sensitivity
    // Tests that winding detection is sensitive to vertex order
    // Reversing order should reverse the normal direction

    let ccw_vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
    ];

    let cw_vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()), // reversed order
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
    ];

    let ccw_plane = Plane::from_vertices(ccw_vertices);
    let cw_plane = Plane::from_vertices(cw_vertices);

    let ccw_normal = ccw_plane.normal();
    let cw_normal = cw_plane.normal();

    // CCW should have positive Z, CW should have negative Z
    assert!(
        ccw_normal.z > 0.0 && cw_normal.z < 0.0,
        "Vertex order should affect winding: CCW Z={}, CW Z={}",
        ccw_normal.z,
        cw_normal.z
    );

    // Normals should be equal in magnitude but opposite in direction
    assert!(
        (ccw_normal.norm() - cw_normal.norm()).abs() < crate::float_types::EPSILON,
        "CCW and CW normals should have equal magnitude: CCW={}, CW={}",
        ccw_normal.norm(),
        cw_normal.norm()
    );
}

// ============================================================
//   HELPER FUNCTIONS
// ============================================================

/// Approximate equality for floating-point comparisons
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}
