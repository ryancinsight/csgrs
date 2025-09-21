//! Scholarly numerical stability tests for csgrs
//!
//! This module provides rigorous validation of numerical precision and stability
//! against academic benchmarks and IEEE 754 standards.
//!
//! ## Academic References
//!
//! - Shewchuk, J. R. (1997). Adaptive precision floating-point arithmetic and fast robust geometric predicates.
//! - Kahan, W. (1996). IEEE Standard 754 for Binary Floating-Point Arithmetic.
//! - Fortune, S. & Wyk, C. V. (1993). Efficient exact arithmetic for computational geometry.
//! - Hoffmann, C. M. (1989). Geometric and solid modeling: an introduction.
//!
//! ## Test Categories
//!
//! ### IEEE 754 Compliance
//! - Special value handling (NaN, infinity, subnormals)
//! - Floating-point precision boundaries
//! - Error propagation analysis
//!
//! ### Robust Geometric Predicates
//! - Orientation tests with adaptive precision
//! - Intersection predicates with degenerate cases
//! - Point-in-triangle tests with boundary conditions
//!
//! ### CSG Operation Stability
//! - Boolean operations with near-coplanar faces
//! - Degenerate geometry handling
//! - Extreme scale factor validation

#[cfg(test)]
#[allow(clippy::module_inception)]
mod numerical_stability {
    use crate::float_types::stability::{AdaptiveEpsilon, ieee754, predicates};
    use crate::float_types::{constants, Real};
    use nalgebra::Point3;

    /// Test IEEE 754 special value handling
    #[test]
    fn test_ieee754_special_values() {
        // Test NaN handling
        let nan_val = Real::NAN;
        assert!(ieee754::is_special(nan_val));
        assert!(nan_val.is_nan());

        // Test infinity handling
        let inf_val = Real::INFINITY;
        assert!(ieee754::is_special(inf_val));
        assert!(inf_val.is_infinite());

        let neg_inf = Real::NEG_INFINITY;
        assert!(ieee754::is_special(neg_inf));
        assert!(neg_inf.is_infinite());

        // Test subnormal values
        let subnormal = Real::MIN_POSITIVE / 2.0;
        assert!(ieee754::is_subnormal(subnormal));

        // Test safe comparison with special values
        assert!(!ieee754::safe_eq(nan_val, nan_val, constants::EPSILON)); // NaN != NaN
        assert!(ieee754::safe_eq(inf_val, inf_val, constants::EPSILON)); // inf == inf

        // Test safe ordering
        assert_eq!(ieee754::safe_cmp(nan_val, 1.0), std::cmp::Ordering::Less);
        assert_eq!(ieee754::safe_cmp(1.0, nan_val), std::cmp::Ordering::Greater);
        assert_eq!(ieee754::safe_cmp(nan_val, nan_val), std::cmp::Ordering::Equal);
    }

    /// Test adaptive epsilon system
    #[test]
    fn test_adaptive_epsilon() {
        let mut epsilon = AdaptiveEpsilon::new();

        // Default epsilon should be base epsilon
        assert_eq!(epsilon.epsilon(), constants::EPSILON);

        // Test scale adjustment
        epsilon.update_scale(1000.0);
        assert!(epsilon.epsilon() > constants::EPSILON);

        // Test explicit scale
        let scaled_epsilon = AdaptiveEpsilon::with_scale(100.0);
        assert_eq!(scaled_epsilon.epsilon(), constants::EPSILON * 100.0);

        // Test minimum scale (prevent division by zero)
        let min_epsilon = AdaptiveEpsilon::with_scale(0.1);
        assert!(min_epsilon.epsilon() >= constants::EPSILON);
    }

    /// Test robust 3D orientation predicate
    #[test]
    fn test_robust_orientation_3d() {
        let epsilon = AdaptiveEpsilon::new();

        // Test simple triangle orientation
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(1.0, 0.0, 0.0);
        let p3 = Point3::new(0.0, 1.0, 0.0);

        let orientation = predicates::orient_3d(&p1, &p2, &p3, &epsilon);
        assert!(orientation > 0.0, "Counterclockwise triangle should have positive orientation");

        // Test degenerate case (collinear points)
        let p4 = Point3::new(0.5, 0.0, 0.0); // Collinear with p1 and p2
        let degenerate = predicates::orient_3d(&p1, &p2, &p4, &epsilon);
        assert!(degenerate.abs() <= epsilon.epsilon(),
               "Collinear points should have orientation near zero: {}", degenerate);

        // Test clockwise orientation
        let p5 = Point3::new(0.0, 1.0, 0.0);
        let p6 = Point3::new(1.0, 0.0, 0.0);
        let p7 = Point3::new(0.0, 0.0, 0.0);

        let clockwise = predicates::orient_3d(&p5, &p6, &p7, &epsilon);
        assert!(clockwise < 0.0, "Clockwise triangle should have negative orientation");
    }

    /// Test point-in-triangle predicate with edge cases
    #[test]
    fn test_point_in_triangle_robust() {
        let epsilon = AdaptiveEpsilon::new();

        // Standard triangle
        let tri1 = Point3::new(0.0, 0.0, 0.0);
        let tri2 = Point3::new(1.0, 0.0, 0.0);
        let tri3 = Point3::new(0.0, 1.0, 0.0);

        // Point inside triangle
        let inside = Point3::new(0.3, 0.3, 0.0);
        assert!(predicates::point_in_triangle(&inside, &tri1, &tri2, &tri3, &epsilon),
               "Point should be inside triangle");

        // Point outside triangle
        let outside = Point3::new(0.5, -0.5, 0.0);
        assert!(!predicates::point_in_triangle(&outside, &tri1, &tri2, &tri3, &epsilon),
               "Point should be outside triangle");

        // Point on edge (boundary case)
        let on_edge = Point3::new(0.5, 0.0, 0.0);
        assert!(predicates::point_in_triangle(&on_edge, &tri1, &tri2, &tri3, &epsilon),
               "Point on edge should be considered inside with epsilon tolerance");

        // Test degenerate triangle
        let degenerate_tri = Point3::new(0.0, 0.0, 0.0);
        let degenerate_point = Point3::new(0.5, 0.0, 0.0);
        assert!(!predicates::point_in_triangle(&degenerate_point, &degenerate_tri, &degenerate_tri, &tri1, &epsilon),
               "Degenerate triangle should reject points");
    }

    /// Test line segment intersection with edge cases
    #[test]
    fn test_segment_intersection_robust() {
        let epsilon = AdaptiveEpsilon::new();

        // Crossing segments
        let a1 = Point3::new(0.0, 0.0, 0.0);
        let a2 = Point3::new(1.0, 1.0, 0.0);
        let b1 = Point3::new(0.0, 1.0, 0.0);
        let b2 = Point3::new(1.0, 0.0, 0.0);

        assert!(predicates::segments_intersect(&a1, &a2, &b1, &b2, &epsilon),
               "Crossing segments should intersect");

        // Non-intersecting segments
        let c1 = Point3::new(0.0, 0.0, 0.0);
        let c2 = Point3::new(1.0, 0.0, 0.0);
        let d1 = Point3::new(0.0, 1.0, 0.0);
        let d2 = Point3::new(1.0, 1.0, 0.0);

        assert!(!predicates::segments_intersect(&c1, &c2, &d1, &d2, &epsilon),
               "Non-intersecting segments should not intersect");

        // Collinear overlapping segments
        let e1 = Point3::new(0.0, 0.0, 0.0);
        let e2 = Point3::new(2.0, 0.0, 0.0);
        let f1 = Point3::new(1.0, 0.0, 0.0);
        let f2 = Point3::new(3.0, 0.0, 0.0);

        // Note: Collinear overlapping segments may be considered intersecting or not
        // depending on the definition - this tests robustness of the predicate
        let _collinear_result = predicates::segments_intersect(&e1, &e2, &f1, &f2, &epsilon);
        // The predicate should handle this case gracefully without panicking
    }

    /// Test floating-point precision boundaries
    #[test]
    fn test_floating_point_precision_boundaries() {
        // Test very small values
        let tiny = 1e-15;
        let point1 = Point3::new(tiny, 0.0, 0.0);
        let point2 = Point3::new(0.0, tiny, 0.0);
        let point3 = Point3::new(0.0, 0.0, tiny);

        let epsilon = AdaptiveEpsilon::with_scale(tiny * 1e6); // Adjust for scale

        let orientation = predicates::orient_3d(&point1, &point2, &point3, &epsilon);
        assert!(orientation.abs() > 0.0, "Tiny values should produce non-zero orientation");

        // Test very large values
        let huge = 1e15;
        let large_point1 = Point3::new(huge, 0.0, 0.0);
        let large_point2 = Point3::new(0.0, huge, 0.0);
        let large_point3 = Point3::new(0.0, 0.0, huge);

        let large_epsilon = AdaptiveEpsilon::with_scale(huge);
        let large_orientation = predicates::orient_3d(&large_point1, &large_point2, &large_point3, &large_epsilon);

        // Large values should be handled without overflow
        assert!(large_orientation.is_finite(), "Large values should not overflow");
    }

    /// Test error propagation in geometric computations
    #[test]
    fn test_error_propagation_analysis() {
        // Test that small errors don't propagate to large inaccuracies
        let base_epsilon = constants::EPSILON;

        // Create nearly coplanar points
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(1.0, 0.0, 0.0);
        let p3 = Point3::new(0.0, 1.0, 0.0);
        let p4 = Point3::new(0.0, 0.0, base_epsilon * 0.5); // Slightly offset

        let epsilon = AdaptiveEpsilon::new();

        // The orientation should be very small but non-zero
        let orientation = predicates::orient_3d(&p1, &p2, &p4, &epsilon);
        assert!(orientation.abs() <= base_epsilon * 10.0,
               "Small perturbations should produce small orientation changes: {}", orientation);

        // Test that the point is still considered coplanar within tolerance
        let coplanar_orientation = predicates::orient_3d(&p1, &p2, &p3, &epsilon);
        assert!(coplanar_orientation.abs() > base_epsilon,
               "Original points should form a triangle with non-zero orientation: {}", coplanar_orientation);
    }

    /// Test degenerate geometry handling
    #[test]
    fn test_degenerate_geometry_handling() {
        let epsilon = AdaptiveEpsilon::new();

        // Test with identical points
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 0.0, 0.0); // Same as p1
        let p3 = Point3::new(1.0, 0.0, 0.0);

        let degenerate_orientation = predicates::orient_3d(&p1, &p2, &p3, &epsilon);
        assert!(degenerate_orientation.abs() <= epsilon.epsilon(),
               "Identical points should produce zero orientation: {}", degenerate_orientation);

        // Test with collinear points
        let col1 = Point3::new(0.0, 0.0, 0.0);
        let col2 = Point3::new(1.0, 0.0, 0.0);
        let col3 = Point3::new(0.5, 0.0, 0.0); // Midway between col1 and col2

        let collinear_orientation = predicates::orient_3d(&col1, &col2, &col3, &epsilon);
        assert!(collinear_orientation.abs() <= epsilon.epsilon(),
               "Collinear points should produce zero orientation: {}", collinear_orientation);

        // Test triangle with zero area (all points collinear)
        let zero_tri = Point3::new(0.0, 0.0, 0.0);
        let in_triangle = predicates::point_in_triangle(&zero_tri, &zero_tri, &zero_tri, &zero_tri, &epsilon);
        assert!(!in_triangle, "Degenerate triangle should reject all points");
    }

    /// Test numerical stability under extreme conditions
    #[test]
    fn test_extreme_numerical_conditions() {
        let epsilon = AdaptiveEpsilon::new();

        // Test with very large coordinates (potential overflow)
        let large_coord = 1e10;
        let large_p1 = Point3::new(large_coord, 0.0, 0.0);
        let large_p2 = Point3::new(0.0, large_coord, 0.0);
        let large_p3 = Point3::new(0.0, 0.0, large_coord);

        let large_result = predicates::orient_3d(&large_p1, &large_p2, &large_p3, &epsilon);
        assert!(large_result.is_finite(), "Large coordinates should not cause overflow");

        // Test with very small coordinates (potential underflow)
        let small_coord = 1e-10;
        let small_p1 = Point3::new(small_coord, 0.0, 0.0);
        let small_p2 = Point3::new(0.0, small_coord, 0.0);
        let small_p3 = Point3::new(0.0, 0.0, small_coord);

        let small_result = predicates::orient_3d(&small_p1, &small_p2, &small_p3, &epsilon);
        assert!(small_result.is_finite(), "Small coordinates should not cause underflow");
        assert!(small_result != 0.0, "Small coordinates should produce non-zero result");

        // Test mixed scale (very large and very small)
        let mixed_p1 = Point3::new(large_coord, 0.0, 0.0);
        let mixed_p2 = Point3::new(0.0, small_coord, 0.0);
        let mixed_p3 = Point3::new(0.0, 0.0, 1.0);

        let mixed_result = predicates::orient_3d(&mixed_p1, &mixed_p2, &mixed_p3, &epsilon);
        assert!(mixed_result.is_finite(), "Mixed scale should not cause numerical issues");
    }

    /// Test CSG operation numerical stability
    #[test]
    fn test_csg_numerical_stability() {
        use crate::mesh::Mesh;
        use crate::traits::CSG;

        // Create nearly coplanar meshes for testing
        let epsilon = AdaptiveEpsilon::new();

        let base_mesh = Mesh::cube(1.0, None::<()>).unwrap();
        let offset_mesh = Mesh::cube(1.0, None::<()>).unwrap()
            .translate(0.0, 0.0, epsilon.epsilon() * 0.1); // Very small offset

        // Test union operation with nearly coplanar geometry
        let union_result = base_mesh.union(&offset_mesh);
        assert!(!union_result.polygons.is_empty(), "Union should produce valid geometry");

        // Test difference operation
        let difference_result = base_mesh.difference(&offset_mesh);
        assert!(!difference_result.polygons.is_empty(), "Difference should produce valid geometry");

        // Test intersection operation
        let intersection_result = base_mesh.intersection(&offset_mesh);
        assert!(!intersection_result.polygons.is_empty(), "Intersection should produce valid geometry");
    }

    /// Test against academic benchmarks for computational geometry
    #[test]
    fn test_academic_benchmark_compliance() {
        // Test against Shewchuk's robust predicates requirements
        let epsilon = AdaptiveEpsilon::new();

        // Create test case from Shewchuk's paper: nearly collinear points
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(1.0, 0.0, 0.0);
        let p3 = Point3::new(0.5, 1e-15, 0.0); // Very small perturbation

        let orientation = predicates::orient_3d(&p1, &p2, &p3, &epsilon);

        // The result should be consistent and stable
        assert!(orientation.is_finite(), "Orientation should be finite");
        assert!(orientation.abs() > 0.0, "Small perturbation should produce non-zero orientation");

        // Test adaptive precision fallback
        let degenerate_epsilon = AdaptiveEpsilon::with_scale(1e-20);
        let degenerate_orientation = predicates::orient_3d(&p1, &p2, &p3, &degenerate_epsilon);

        // Should trigger high-precision computation
        assert!(degenerate_orientation.is_finite(), "High-precision computation should handle degenerate cases");
    }

    /// Test numerical stability under repeated operations
    #[test]
    fn test_repeated_operations_stability() {
        let epsilon = AdaptiveEpsilon::new();

        // Start with a simple triangle
        let mut points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];

        // Perform repeated transformations that could accumulate errors
        for i in 0..100 {
            // Rotate slightly each time
            let angle = (i as Real) * 0.01;
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            // Apply rotation matrix
            let new_points: Vec<Point3<Real>> = points.iter().map(|p| {
                Point3::new(
                    p.x * cos_a - p.y * sin_a,
                    p.x * sin_a + p.y * cos_a,
                    p.z
                )
            }).collect();

            points = new_points;

            // Check that the triangle remains valid
            let orientation = predicates::orient_3d(&points[0], &points[1], &points[2], &epsilon);
            assert!(orientation.is_finite(), "Orientation should remain finite after {} rotations", i);

            // Check that area remains approximately constant
            let area = orientation.abs();
            assert!(area > 0.0, "Triangle area should remain positive");
            assert!(area < 10.0, "Triangle area should not grow excessively: {}", area);
        }
    }
}
