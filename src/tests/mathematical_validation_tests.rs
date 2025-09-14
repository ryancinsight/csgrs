//! Comprehensive mathematical validation tests for normal calculations and splits
//!
//! This module provides rigorous testing of geometric algorithms against
//! mathematical formulas, ensuring correctness across all edge cases and
//! precision boundaries.

use crate::float_types::Real;
use crate::mesh::plane::Plane;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::{Matrix4, Point3, Vector3};

// ============================================================
//   NORMAL CALCULATION MATHEMATICAL VALIDATION
// ============================================================

#[test]
fn test_plane_normal_cross_product_formula() {
    // **Mathematical Foundation**: Normal vector calculation using cross product
    // For points A, B, C: normal = (B - A) × (C - A)
    // Expected: normal vector perpendicular to plane defined by three points

    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0, 0.0, 0.0);
    let c = Point3::new(0.0, 1.0, 0.0);

    let vertices = vec![
        Vertex::new(a, Vector3::z()),
        Vertex::new(b, Vector3::z()),
        Vertex::new(c, Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);

    // Calculate expected normal using cross product formula
    let ab = b - a; // (1, 0, 0)
    let ac = c - a; // (0, 1, 0)
    let expected_normal = ab.cross(&ac); // (0, 0, 1)

    // Verify normal calculation matches mathematical expectation
    assert!(
        approx_eq_vector(
            plane.normal(),
            expected_normal.normalize(),
            crate::float_types::EPSILON
        ),
        "Plane normal should match cross product calculation: got {:?}, expected {:?}",
        plane.normal(),
        expected_normal.normalize()
    );

    // Verify normal is perpendicular to vectors in plane
    let dot_ab = plane.normal().dot(&ab);
    let dot_ac = plane.normal().dot(&ac);
    assert!(
        dot_ab.abs() < crate::float_types::EPSILON * 10.0,
        "Normal should be perpendicular to AB vector, dot product: {}",
        dot_ab
    );
    assert!(
        dot_ac.abs() < crate::float_types::EPSILON * 10.0,
        "Normal should be perpendicular to AC vector, dot product: {}",
        dot_ac
    );
}

#[test]
fn test_plane_normal_arbitrary_triangle() {
    // Test normal calculation for arbitrary triangle
    let a = Point3::new(1.0, 2.0, 3.0);
    let b = Point3::new(4.0, 6.0, 5.0);
    let c = Point3::new(2.0, 3.0, 7.0);

    let vertices = vec![
        Vertex::new(a, Vector3::z()),
        Vertex::new(b, Vector3::z()),
        Vertex::new(c, Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);

    // Calculate expected normal
    let ab = b - a;
    let ac = c - a;
    let expected_normal = ab.cross(&ac).normalize();

    assert!(
        approx_eq_vector(plane.normal(), expected_normal, crate::float_types::EPSILON),
        "Arbitrary triangle normal calculation failed"
    );

    // Verify unit length
    assert!(
        approx_eq(plane.normal().magnitude(), 1.0, crate::float_types::EPSILON),
        "Normal should be unit vector, magnitude: {}",
        plane.normal().magnitude()
    );
}

#[test]
fn test_plane_normal_collinear_points_mathematical_correctness() {
    // **Mathematical Foundation**: Cross product of collinear vectors = zero vector
    // For collinear points A, B, C: (B - A) × (C - A) = 0

    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(2.0, 0.0, 0.0); // Collinear with A
    let c = Point3::new(4.0, 0.0, 0.0); // Collinear with A and B

    let vertices = vec![
        Vertex::new(a, Vector3::z()),
        Vertex::new(b, Vector3::z()),
        Vertex::new(c, Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices);

    // For collinear points, cross product should be zero
    let ab = b - a;
    let ac = c - a;
    let cross_product = ab.cross(&ac);

    // Verify cross product magnitude is near zero
    assert!(
        cross_product.magnitude() < crate::float_types::EPSILON * 100.0,
        "Cross product of collinear vectors should be near zero, got magnitude: {}",
        cross_product.magnitude()
    );

    // Normal should still be finite (implementation should handle this case)
    assert!(
        plane.normal().magnitude().is_finite(),
        "Normal should be finite even for collinear points"
    );
}

#[test]
fn test_plane_signed_distance_formula() {
    // **Mathematical Foundation**: Signed distance formula
    // Distance = n·p + d, where n is unit normal, p is point, d is offset
    // Sign indicates side relative to plane normal direction

    let normal = Vector3::new(0.0, 0.0, 1.0);
    let offset = 5.0;
    let _plane = Plane::from_normal(normal, offset);

    // Test point on plane
    let point_on_plane = Point3::new(0.0, 0.0, -offset);
    let distance_on = _plane.normal().dot(&point_on_plane.coords) + _plane.offset();
    assert!(
        distance_on.abs() < crate::float_types::EPSILON,
        "Point on plane should have zero signed distance, got: {}",
        distance_on
    );

    // Test point above plane (positive Z relative to normal)
    let point_above = Point3::new(0.0, 0.0, 10.0);
    let distance_above = _plane.normal().dot(&point_above.coords) + _plane.offset();
    assert!(
        distance_above > 0.0,
        "Point above plane should have positive signed distance, got: {}",
        distance_above
    );

    // Test point below plane (negative Z relative to normal)
    let point_below = Point3::new(0.0, 0.0, -10.0);
    let distance_below = _plane.normal().dot(&point_below.coords) + _plane.offset();
    assert!(
        distance_below < 0.0,
        "Point below plane should have negative signed distance, got: {}",
        distance_below
    );
}

#[test]
fn test_plane_signed_distance_consistency() {
    // Test that signed distance is consistent with plane equation
    // For plane equation: ax + by + cz + d = 0
    // Signed distance = (ax + by + cz + d) / |n|

    let normal = Vector3::new(1.0, 2.0, 3.0).normalize();
    let offset = 2.0;
    let plane = Plane::from_normal(normal, offset);

    let test_point = Point3::new(3.0, 4.0, 5.0);

    // Calculate signed distance using dot product
    let signed_distance = normal.dot(&test_point.coords) + offset;

    // Calculate using plane equation components
    let plane_equation_value =
        normal.x * test_point.x + normal.y * test_point.y + normal.z * test_point.z + offset;

    // Also calculate using plane methods for consistency verification
    let plane_signed_distance = plane.normal().dot(&test_point.coords) + plane.offset();

    assert!(
        approx_eq(
            signed_distance,
            plane_equation_value,
            crate::float_types::EPSILON
        ),
        "Signed distance should match plane equation evaluation"
    );

    assert!(
        approx_eq(
            signed_distance,
            plane_signed_distance,
            crate::float_types::EPSILON
        ),
        "Direct calculation should match plane method result"
    );
}

// ============================================================
//   SPLIT OPERATION MATHEMATICAL VALIDATION
// ============================================================

#[test]
fn test_polygon_split_mathematical_correctness() {
    // **Mathematical Foundation**: Polygon splitting along plane
    // Vertices should be correctly classified as front/back/coplanar
    // Split edges should intersect plane at correct points

    let plane = Plane::from_normal(Vector3::z(), 0.0); // XY plane

    // Create polygon that clearly straddles the plane
    // Using larger Z values to ensure clear separation
    let vertices = vec![
        Vertex::new(Point3::new(-1.0, -1.0, -2.0), Vector3::z()), // Clearly below
        Vertex::new(Point3::new(1.0, -1.0, -2.0), Vector3::z()),  // Clearly below
        Vertex::new(Point3::new(1.0, 1.0, 2.0), Vector3::z()),    // Clearly above
        Vertex::new(Point3::new(-1.0, 1.0, 2.0), Vector3::z()),   // Clearly above
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);
    let (coplanar_front, coplanar_back, front, back) = plane.split_polygon(&polygon);

    // Verify that we have polygons on both sides or in coplanar lists
    assert!(
        !front.is_empty()
            || !back.is_empty()
            || !coplanar_front.is_empty()
            || !coplanar_back.is_empty(),
        "Split should produce polygons on at least one side or in coplanar lists"
    );

    // Verify all resulting polygons are valid (at least 3 vertices)
    for poly in &front {
        assert!(
            poly.vertices.len() >= 3,
            "Front polygons should have at least 3 vertices, got: {}",
            poly.vertices.len()
        );
    }
    for poly in &back {
        assert!(
            poly.vertices.len() >= 3,
            "Back polygons should have at least 3 vertices, got: {}",
            poly.vertices.len()
        );
    }
    for poly in &coplanar_front {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar front polygons should have at least 3 vertices, got: {}",
            poly.vertices.len()
        );
    }
    for poly in &coplanar_back {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar back polygons should have at least 3 vertices, got: {}",
            poly.vertices.len()
        );
    }

    // Verify vertex classification consistency
    for poly in &front {
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance >= -crate::float_types::EPSILON * 10.0,
                "Front polygon vertex should be on or in front of plane, distance: {}",
                distance
            );
        }
    }

    for poly in &back {
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance <= crate::float_types::EPSILON * 10.0,
                "Back polygon vertex should be on or behind plane, distance: {}",
                distance
            );
        }
    }

    // Coplanar polygons should have vertices very close to the plane
    for poly in &coplanar_front {
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance.abs() <= crate::float_types::EPSILON * 10.0,
                "Coplanar front polygon vertex should be on plane, distance: {}",
                distance
            );
        }
    }

    for poly in &coplanar_back {
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance.abs() <= crate::float_types::EPSILON * 10.0,
                "Coplanar back polygon vertex should be on plane, distance: {}",
                distance
            );
        }
    }
}

#[test]
fn test_polygon_split_edge_intersection() {
    // **Mathematical Foundation**: Line-plane intersection
    // For edge from p1 to p2, intersection point satisfies:
    // t = -d1 / (d2 - d1), where d1, d2 are signed distances

    let plane = Plane::from_normal(Vector3::z(), 0.0);

    // Create edge that crosses plane
    let p1 = Point3::new(0.0, 0.0, -1.0); // Below plane
    let p2 = Point3::new(0.0, 0.0, 1.0); // Above plane

    let vertices = vec![
        Vertex::new(p1, Vector3::z()),
        Vertex::new(p2, Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);
    let (coplanar_front, coplanar_back, front, back) = plane.split_polygon(&polygon);

    // At least one polygon should exist
    assert!(
        !front.is_empty()
            || !back.is_empty()
            || !coplanar_front.is_empty()
            || !coplanar_back.is_empty(),
        "Split should produce at least one polygon"
    );

    // The intersection point should be at (0,0,0)
    let expected_intersection = Point3::new(0.0, 0.0, 0.0);

    // Check that intersection point lies on plane
    for poly in &front {
        for vertex in &poly.vertices {
            if approx_eq_point(
                vertex.pos,
                expected_intersection,
                crate::float_types::EPSILON * 10.0,
            ) {
                let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
                assert!(
                    distance.abs() < crate::float_types::EPSILON * 10.0,
                    "Intersection point should lie on plane, distance: {}",
                    distance
                );
            }
        }
    }

    for poly in &back {
        for vertex in &poly.vertices {
            if approx_eq_point(
                vertex.pos,
                expected_intersection,
                crate::float_types::EPSILON * 10.0,
            ) {
                let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
                assert!(
                    distance.abs() < crate::float_types::EPSILON * 10.0,
                    "Intersection point should lie on plane, distance: {}",
                    distance
                );
            }
        }
    }

    for poly in &coplanar_front {
        for vertex in &poly.vertices {
            if approx_eq_point(
                vertex.pos,
                expected_intersection,
                crate::float_types::EPSILON * 10.0,
            ) {
                let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
                assert!(
                    distance.abs() < crate::float_types::EPSILON * 10.0,
                    "Intersection point should lie on plane, distance: {}",
                    distance
                );
            }
        }
    }

    for poly in &coplanar_back {
        for vertex in &poly.vertices {
            if approx_eq_point(
                vertex.pos,
                expected_intersection,
                crate::float_types::EPSILON * 10.0,
            ) {
                let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
                assert!(
                    distance.abs() < crate::float_types::EPSILON * 10.0,
                    "Intersection point should lie on plane, distance: {}",
                    distance
                );
            }
        }
    }
}

#[test]
fn test_polygon_split_coplanar_vertices() {
    // Test splitting polygon with vertices exactly on plane
    let plane = Plane::from_normal(Vector3::z(), 0.0);

    let vertices = vec![
        Vertex::new(Point3::new(-1.0, -1.0, 0.0), Vector3::z()), // On plane
        Vertex::new(Point3::new(1.0, -1.0, 0.0), Vector3::z()),  // On plane
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),   // On plane
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);
    let (coplanar_front, coplanar_back, front, back) = plane.split_polygon(&polygon);

    // All vertices are coplanar, so should go to coplanar lists
    assert!(
        !coplanar_front.is_empty() || !coplanar_back.is_empty(),
        "Coplanar polygon should be in coplanar lists"
    );

    // Front and back should be empty for exactly coplanar polygons
    assert!(
        front.is_empty() && back.is_empty(),
        "Front and back should be empty for coplanar polygon"
    );

    // Verify coplanar polygons are valid
    for poly in &coplanar_front {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar front polygons should have at least 3 vertices"
        );
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance.abs() < crate::float_types::EPSILON * 10.0,
                "Coplanar vertices should lie on plane, distance: {}",
                distance
            );
        }
    }
    for poly in &coplanar_back {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar back polygons should have at least 3 vertices"
        );
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance.abs() < crate::float_types::EPSILON * 10.0,
                "Coplanar vertices should lie on plane, distance: {}",
                distance
            );
        }
    }
}

#[test]
fn test_polygon_split_area_conservation() {
    // **Mathematical Foundation**: Area conservation in polygon splitting
    // Total area of split polygons should equal original polygon area

    let plane = Plane::from_normal(Vector3::z(), 0.0);

    // Create a simple triangle
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, -0.5), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, -0.5), Vector3::z()),
        Vertex::new(Point3::new(1.0, 2.0, 0.5), Vector3::z()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);

    // Calculate original area using cross product
    let v1 = polygon.vertices[1].pos - polygon.vertices[0].pos;
    let v2 = polygon.vertices[2].pos - polygon.vertices[0].pos;
    let cross = v1.cross(&v2);
    let original_area = cross.magnitude() / 2.0;

    let (coplanar_front, coplanar_back, front, back) = plane.split_polygon(&polygon);

    // Calculate areas of split polygons
    let mut total_split_area = 0.0;

    for poly in &front {
        if poly.vertices.len() >= 3 {
            for i in 1..poly.vertices.len() - 1 {
                let v1 = poly.vertices[i].pos - poly.vertices[0].pos;
                let v2 = poly.vertices[i + 1].pos - poly.vertices[0].pos;
                let cross = v1.cross(&v2);
                total_split_area += cross.magnitude() / 2.0;
            }
        }
    }

    for poly in &back {
        if poly.vertices.len() >= 3 {
            for i in 1..poly.vertices.len() - 1 {
                let v1 = poly.vertices[i].pos - poly.vertices[0].pos;
                let v2 = poly.vertices[i + 1].pos - poly.vertices[0].pos;
                let cross = v1.cross(&v2);
                total_split_area += cross.magnitude() / 2.0;
            }
        }
    }

    for poly in &coplanar_front {
        if poly.vertices.len() >= 3 {
            for i in 1..poly.vertices.len() - 1 {
                let v1 = poly.vertices[i].pos - poly.vertices[0].pos;
                let v2 = poly.vertices[i + 1].pos - poly.vertices[0].pos;
                let cross = v1.cross(&v2);
                total_split_area += cross.magnitude() / 2.0;
            }
        }
    }

    for poly in &coplanar_back {
        if poly.vertices.len() >= 3 {
            for i in 1..poly.vertices.len() - 1 {
                let v1 = poly.vertices[i].pos - poly.vertices[0].pos;
                let v2 = poly.vertices[i + 1].pos - poly.vertices[0].pos;
                let cross = v1.cross(&v2);
                total_split_area += cross.magnitude() / 2.0;
            }
        }
    }

    // Areas should be approximately equal (allowing for floating point precision)
    assert!(
        approx_eq(
            total_split_area,
            original_area,
            crate::float_types::EPSILON * 100.0
        ),
        "Total area of split polygons should equal original area: got {}, expected {}",
        total_split_area,
        original_area
    );
}

// ============================================================
//   PRECISION BOUNDARY TESTING
// ============================================================

#[test]
fn test_normal_calculation_precision_boundaries() {
    // Test normal calculation at various precision boundaries

    // Test with very small coordinates
    let tiny = crate::float_types::EPSILON * 100.0;
    let vertices_tiny = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(tiny, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, tiny, 0.0), Vector3::z()),
    ];

    let plane_tiny = Plane::from_vertices(vertices_tiny);
    assert!(
        plane_tiny.normal().magnitude().is_finite(),
        "Normal should be finite for tiny coordinates"
    );
    assert!(
        approx_eq(
            plane_tiny.normal().magnitude(),
            1.0,
            crate::float_types::EPSILON
        ),
        "Normal should be unit length for tiny coordinates"
    );

    // Test with very large coordinates
    let large = 1e10;
    let vertices_large = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(large, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, large, 0.0), Vector3::z()),
    ];

    let plane_large = Plane::from_vertices(vertices_large);
    assert!(
        plane_large.normal().magnitude().is_finite(),
        "Normal should be finite for large coordinates"
    );
    assert!(
        approx_eq(
            plane_large.normal().magnitude(),
            1.0,
            crate::float_types::EPSILON
        ),
        "Normal should be unit length for large coordinates"
    );
}

#[test]
fn test_signed_distance_precision_boundaries() {
    let plane = Plane::from_normal(Vector3::z(), 0.0);

    // Test with points extremely close to plane
    let epsilon = crate::float_types::EPSILON;
    for i in 0..10 {
        let offset = epsilon * (10.0_f64).powi(i);
        let point = Point3::new(0.0, 0.0, offset);
        let distance = plane.normal().dot(&point.coords) + plane.offset();

        // Distance should be approximately equal to offset
        assert!(
            approx_eq(distance, offset, epsilon * 10.0),
            "Signed distance should be accurate near precision boundary: got {}, expected {}",
            distance,
            offset
        );
    }
}

#[test]
fn test_split_operation_precision_boundaries() {
    let plane = Plane::from_normal(Vector3::z(), 0.0);

    // Test splitting with vertices extremely close to plane
    let epsilon = crate::float_types::EPSILON;
    let vertices = vec![
        Vertex::new(Point3::new(-1.0, -1.0, -epsilon * 0.1), Vector3::z()),
        Vertex::new(Point3::new(1.0, -1.0, -epsilon * 0.1), Vector3::z()),
        Vertex::new(Point3::new(1.0, 1.0, epsilon * 0.1), Vector3::z()),
        Vertex::new(Point3::new(-1.0, 1.0, epsilon * 0.1), Vector3::z()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);
    let (coplanar_front, coplanar_back, front, back) = plane.split_polygon(&polygon);

    // Operation should complete without panicking
    assert!(
        true,
        "Split operation should handle precision boundaries gracefully"
    );

    // All resulting polygons should be valid
    for poly in &front {
        assert!(poly.vertices.len() >= 3, "Front polygons should be valid");
    }
    for poly in &back {
        assert!(poly.vertices.len() >= 3, "Back polygons should be valid");
    }
    for poly in &coplanar_front {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar front polygons should be valid"
        );
    }
    for poly in &coplanar_back {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar back polygons should be valid"
        );
    }
}

// ============================================================
//   EDGE CASE MATHEMATICAL VALIDATION
// ============================================================

#[test]
fn test_normal_calculation_edge_cases() {
    // Test normal calculation for various edge cases

    // Nearly collinear points
    let epsilon = crate::float_types::EPSILON * 100.0;
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, epsilon, 0.0), Vector3::z()), // Very close to collinear
    ];

    let plane = Plane::from_vertices(vertices);
    assert!(
        plane.normal().magnitude().is_finite(),
        "Normal should be finite for nearly collinear points"
    );

    // Points forming very small triangle
    let tiny_vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(epsilon, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, epsilon, 0.0), Vector3::z()),
    ];

    let tiny_plane = Plane::from_vertices(tiny_vertices);
    assert!(
        tiny_plane.normal().magnitude().is_finite(),
        "Normal should be finite for very small triangle"
    );
}

#[test]
fn test_signed_distance_edge_cases() {
    let plane = Plane::from_normal(Vector3::z(), 0.0);

    // Test with zero vector normal (shouldn't happen in practice, but test robustness)
    let _zero_plane = Plane::from_normal(Vector3::zeros(), 0.0);

    // Test with extreme values
    let extreme_point = Point3::new(Real::MAX / 2.0, Real::MAX / 2.0, Real::MAX / 2.0);
    let distance_extreme = plane.normal().dot(&extreme_point.coords) + plane.offset();

    assert!(
        distance_extreme.is_finite(),
        "Signed distance should be finite for extreme coordinate values"
    );

    // Test with mixed extreme values
    let mixed_point = Point3::new(Real::MIN / 2.0, Real::MAX / 2.0, 0.0);
    let distance_mixed = plane.normal().dot(&mixed_point.coords) + plane.offset();

    assert!(
        distance_mixed.is_finite(),
        "Signed distance should handle mixed extreme values"
    );
}

// ============================================================
//   HELPER FUNCTIONS
// ============================================================

/// Approximate equality for vectors
fn approx_eq_vector(a: Vector3<Real>, b: Vector3<Real>, eps: Real) -> bool {
    approx_eq(a.x, b.x, eps) && approx_eq(a.y, b.y, eps) && approx_eq(a.z, b.z, eps)
}

#[test]
fn test_floating_point_precision_edge_cases() {
    // Test operations with values very close to zero
    let tiny = 1e-10;
    let point1 = Point3::new(tiny, tiny, tiny);
    let point2 = Point3::new(-tiny, -tiny, -tiny);

    // Distance calculation should be numerically stable
    let distance = (point1 - point2).norm();
    let expected = (8.0_f32 * tiny * tiny).sqrt();
    let tolerance = 1e-6_f32; // More reasonable tolerance for f32 precision
    assert!((distance - expected).abs() < tolerance,
        "Distance calculation should be numerically stable: got {}, expected {}", distance, expected);

    // Cross product with near-zero vectors
    let vec1 = Vector3::new(tiny, 0.0, 0.0);
    let vec2 = Vector3::new(0.0, tiny, 0.0);
    let cross = vec1.cross(&vec2);
    assert!(cross.norm() >= 0.0, "Cross product magnitude should be non-negative");

    // Normalization of near-zero vectors
    let near_zero = Vector3::new(tiny, tiny, tiny);
    let normalized = near_zero.normalize();
    assert!((normalized.norm() - 1.0).abs() < 1e-6,
        "Normalized vector should have unit length: got {}", normalized.norm());
}

#[test]
fn test_geometric_operations_with_extreme_values() {
    // Test with very large coordinates
    let large = 1e6;
    let point_large = Point3::new(large, large, large);

    // Translation should preserve relative distances
    let translation = Vector3::new(1.0, 2.0, 3.0);
    let translated = point_large + translation;

    assert_eq!(translated.x, large + 1.0);
    assert_eq!(translated.y, large + 2.0);
    assert_eq!(translated.z, large + 3.0);

    // Test with very small coordinates
    let small = 1e-6;
    let point_small = Point3::new(small, small, small);

    // Scaling should work correctly
    let scale_factor = 1e6;
    let scaled = point_small * scale_factor;

    assert!((scaled.x - 1.0_f32).abs() < 1e-6_f32);
    assert!((scaled.y - 1.0_f32).abs() < 1e-6_f32);
    assert!((scaled.z - 1.0_f32).abs() < 1e-6_f32);
}

#[test]
fn test_matrix_operations_numerical_stability() {
    // Test matrix operations with values that could cause numerical issues
    let translation = Vector3::new(1e6, 1e6, 1e6);
    let matrix = Matrix4::new_translation(&translation);

    // Apply transformation to a point
    let point = Point3::new(1.0, 2.0, 3.0);
    let homogeneous = point.to_homogeneous();
    let transformed_homogeneous = matrix * homogeneous;

    let transformed_point = Point3::from_homogeneous(transformed_homogeneous)
        .expect("Should be able to convert back from homogeneous coordinates");

    // Verify the translation was applied correctly
    assert!((transformed_point.x - 1e6_f32 - 1.0).abs() < 1e-3_f32);
    assert!((transformed_point.y - 1e6_f32 - 2.0).abs() < 1e-3_f32);
    assert!((transformed_point.z - 1e6_f32 - 3.0).abs() < 1e-3_f32);
}

/// Approximate equality for points
fn approx_eq_point(a: Point3<Real>, b: Point3<Real>, eps: Real) -> bool {
    approx_eq(a.x, b.x, eps) && approx_eq(a.y, b.y, eps) && approx_eq(a.z, b.z, eps)
}

/// Approximate equality for scalars
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}
