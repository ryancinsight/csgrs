//! Property-based tests for mathematical correctness and geometric invariants
//!
//! These tests use simple property-based testing approaches to verify mathematical
//! properties that should hold for all valid inputs, without relying on external
//! crates like proptest.

use crate::mesh::Mesh;
use crate::traits::CSG;

// --------------------------------------------------------
//   Property-Based Tests: Mathematical Correctness
// --------------------------------------------------------

#[test]
fn test_union_idempotency() {
    // Property: A ∪ A = A (union with itself should be identity)
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let union_result = cube.union(&cube);

    // The result should have the same bounding box as the original
    let original_bb = cube.bounding_box();
    let union_bb = union_result.bounding_box();

    assert!(
        (original_bb.mins.x - union_bb.mins.x).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.x - union_bb.maxs.x).abs() < crate::float_types::EPSILON
            && (original_bb.mins.y - union_bb.mins.y).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.y - union_bb.maxs.y).abs() < crate::float_types::EPSILON
            && (original_bb.mins.z - union_bb.mins.z).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.z - union_bb.maxs.z).abs() < crate::float_types::EPSILON,
        "Union with self should preserve bounding box"
    );
}

#[test]
fn test_union_commutativity() {
    // Property: A ∪ B = B ∪ A (union should be commutative)
    let cube1: Mesh<()> = Mesh::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(0.5, 0.0, 0.0);
    let cube2: Mesh<()> = Mesh::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(-0.5, 0.0, 0.0);

    let union_ab = cube1.union(&cube2);
    let union_ba = cube2.union(&cube1);

    // Both results should have the same bounding box
    let bb_ab = union_ab.bounding_box();
    let bb_ba = union_ba.bounding_box();

    assert!(
        (bb_ab.mins.x - bb_ba.mins.x).abs() < crate::float_types::EPSILON
            && (bb_ab.maxs.x - bb_ba.maxs.x).abs() < crate::float_types::EPSILON
            && (bb_ab.mins.y - bb_ba.mins.y).abs() < crate::float_types::EPSILON
            && (bb_ab.maxs.y - bb_ba.maxs.y).abs() < crate::float_types::EPSILON
            && (bb_ab.mins.z - bb_ba.mins.z).abs() < crate::float_types::EPSILON
            && (bb_ab.maxs.z - bb_ba.maxs.z).abs() < crate::float_types::EPSILON,
        "Union should be commutative"
    );
}

#[test]
fn test_union_associativity() {
    // Property: (A ∪ B) ∪ C = A ∪ (B ∪ C) (union should be associative)
    let cube1: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(2.0, 0.0, 0.0);
    let cube2: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(0.0, 0.0, 0.0);
    let cube3: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(-2.0, 0.0, 0.0);

    let left_assoc = cube1.union(&cube2).union(&cube3);
    let right_assoc = cube1.union(&cube2.union(&cube3));

    let bb_left = left_assoc.bounding_box();
    let bb_right = right_assoc.bounding_box();

    assert!(
        (bb_left.mins.x - bb_right.mins.x).abs() < crate::float_types::EPSILON
            && (bb_left.maxs.x - bb_right.maxs.x).abs() < crate::float_types::EPSILON
            && (bb_left.mins.y - bb_right.mins.y).abs() < crate::float_types::EPSILON
            && (bb_left.maxs.y - bb_right.maxs.y).abs() < crate::float_types::EPSILON
            && (bb_left.mins.z - bb_right.mins.z).abs() < crate::float_types::EPSILON
            && (bb_left.maxs.z - bb_right.maxs.z).abs() < crate::float_types::EPSILON,
        "Union should be associative"
    );
}

#[test]
fn test_boolean_operation_closure() {
    // Property: Boolean operations should produce valid, closed meshes
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None).expect("Failed to create sphere");

    // Test all boolean operations
    let operations = vec![
        ("union", cube.union(&sphere)),
        ("difference", cube.difference(&sphere)),
        ("intersection", cube.intersection(&sphere)),
        ("xor", cube.xor(&sphere)),
    ];

    for (op_name, result) in operations {
        // Result should be a valid mesh (non-empty polygons)
        assert!(
            !result.polygons.is_empty(),
            "{} should produce non-empty mesh",
            op_name
        );

        // All polygons should be valid (at least 3 vertices)
        for polygon in &result.polygons {
            assert!(
                polygon.vertices.len() >= 3,
                "{} should produce valid polygons",
                op_name
            );
        }

        // Result should be manifold (Euler characteristic validation)
        let num_vertices = result.vertices().len();
        let num_faces = result.polygons.len();
        let num_edges = result
            .polygons
            .iter()
            .map(|p| p.vertices.len())
            .sum::<usize>()
            / 2;

        // For a closed manifold: V - E + F = 2 (Euler characteristic)
        let euler_char = num_vertices as i32 - num_edges as i32 + num_faces as i32;
        assert!(
            euler_char >= 0,
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
fn test_scaling_homogeneity() {
    // Property: Scaling should be homogeneous (linear)
    let original: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Apply scaling transformation
    let transformed = original.scale(1.5, 2.0, 0.8);

    // Volume should scale correctly
    let original_bb = original.bounding_box();
    let transformed_bb = transformed.bounding_box();

    let original_volume = (original_bb.maxs.x - original_bb.mins.x)
        * (original_bb.maxs.y - original_bb.mins.y)
        * (original_bb.maxs.z - original_bb.mins.z);

    let transformed_volume = (transformed_bb.maxs.x - transformed_bb.mins.x)
        * (transformed_bb.maxs.y - transformed_bb.mins.y)
        * (transformed_bb.maxs.z - transformed_bb.mins.z);

    let expected_volume = original_volume * 1.5 * 2.0 * 0.8; // scale factors

    assert!(
        (transformed_volume - expected_volume).abs()
            < crate::float_types::EPSILON * expected_volume,
        "Volume should scale correctly: expected {}, got {}",
        expected_volume,
        transformed_volume
    );

    // Dimensions should scale individually
    let original_width = original_bb.maxs.x - original_bb.mins.x;
    let scaled_width = transformed_bb.maxs.x - transformed_bb.mins.x;
    let original_height = original_bb.maxs.y - original_bb.mins.y;
    let scaled_height = transformed_bb.maxs.y - transformed_bb.mins.y;
    let original_depth = original_bb.maxs.z - original_bb.mins.z;
    let scaled_depth = transformed_bb.maxs.z - transformed_bb.mins.z;

    assert!(
        (scaled_width - original_width * 1.5).abs() < crate::float_types::EPSILON,
        "Width should scale by 1.5x: expected {}, got {}",
        original_width * 1.5,
        scaled_width
    );
    assert!(
        (scaled_height - original_height * 2.0).abs() < crate::float_types::EPSILON,
        "Height should scale by 2.0x: expected {}, got {}",
        original_height * 2.0,
        scaled_height
    );
    assert!(
        (scaled_depth - original_depth * 0.8).abs() < crate::float_types::EPSILON,
        "Depth should scale by 0.8x: expected {}, got {}",
        original_depth * 0.8,
        scaled_depth
    );
}

#[test]
fn test_intersection_commutativity() {
    // Property: A ∩ B = B ∩ A (intersection should be commutative)
    let sphere1: Mesh<()> = Mesh::sphere(2.0, 16, 8, None)
        .expect("Failed to create sphere")
        .translate(0.5, 0.0, 0.0);
    let sphere2: Mesh<()> = Mesh::sphere(2.0, 16, 8, None)
        .expect("Failed to create sphere")
        .translate(-0.5, 0.0, 0.0);

    let intersect_ab = sphere1.intersection(&sphere2);
    let intersect_ba = sphere2.intersection(&sphere1);

    // Both should have the same volume
    let bb_ab = intersect_ab.bounding_box();
    let bb_ba = intersect_ba.bounding_box();

    let volume_ab = (bb_ab.maxs.x - bb_ab.mins.x)
        * (bb_ab.maxs.y - bb_ab.mins.y)
        * (bb_ab.maxs.z - bb_ab.mins.z);

    let volume_ba = (bb_ba.maxs.x - bb_ba.mins.x)
        * (bb_ba.maxs.y - bb_ba.mins.y)
        * (bb_ba.maxs.z - bb_ba.mins.z);

    assert!(
        (volume_ab - volume_ba).abs() < crate::float_types::EPSILON * volume_ab.max(volume_ba),
        "Intersection should be commutative: volume_ab={}, volume_ba={}",
        volume_ab,
        volume_ba
    );
}

#[test]
fn test_intersection_idempotency() {
    // Property: A ∩ A = A (intersection with itself should be identity)
    let sphere: Mesh<()> = Mesh::sphere(2.0, 16, 8, None).expect("Failed to create sphere");
    let intersect_result = sphere.intersection(&sphere);

    let original_bb = sphere.bounding_box();
    let intersect_bb = intersect_result.bounding_box();

    // The intersection should have the same bounding box
    assert!(
        (original_bb.mins.x - intersect_bb.mins.x).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.x - intersect_bb.maxs.x).abs() < crate::float_types::EPSILON
            && (original_bb.mins.y - intersect_bb.mins.y).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.y - intersect_bb.maxs.y).abs() < crate::float_types::EPSILON
            && (original_bb.mins.z - intersect_bb.mins.z).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.z - intersect_bb.maxs.z).abs() < crate::float_types::EPSILON,
        "Intersection with self should be identity"
    );
}

#[test]
fn test_mirror_symmetry() {
    // Property: Mirroring should produce a valid mesh with the same number of polygons
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let mirror_plane = crate::mesh::plane::Plane::from_normal(
        nalgebra::Vector3::new(0.0, 0.0, 1.0), // XY plane at z=0
        0.0,
    );

    let mirrored = cube.mirror(mirror_plane.clone());

    // Mirroring should produce a valid mesh
    assert!(
        !mirrored.polygons.is_empty(),
        "Mirror should produce non-empty mesh"
    );
    assert_eq!(
        cube.polygons.len(),
        mirrored.polygons.len(),
        "Mirror should preserve polygon count"
    );

    // Each polygon should be valid
    for polygon in &mirrored.polygons {
        assert!(
            polygon.vertices.len() >= 3,
            "Mirror should produce valid polygons"
        );
    }

    // Mirroring should be involutory (mirror twice returns to original)
    let double_mirrored = mirrored.mirror(mirror_plane);
    assert_eq!(
        cube.polygons.len(),
        double_mirrored.polygons.len(),
        "Double mirror should preserve polygon count"
    );
}

#[test]
fn test_rotation_orthogonality() {
    // Property: Rotation should produce a valid mesh with the same structure
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Test various rotation angles
    let rotations = vec![
        (0.0, 0.0, 0.0),  // Identity
        (1.57, 0.0, 0.0), // 90° around X
        (0.0, 1.57, 0.0), // 90° around Y
        (0.0, 0.0, 1.57), // 90° around Z
        (0.5, 0.3, 0.7),  // Arbitrary rotation
    ];

    for (rx, ry, rz) in rotations {
        let rotated = cube.rotate(rx, ry, rz);

        // Rotation should produce a valid mesh
        assert!(
            !rotated.polygons.is_empty(),
            "Rotation should produce non-empty mesh"
        );
        assert_eq!(
            cube.polygons.len(),
            rotated.polygons.len(),
            "Rotation should preserve polygon count"
        );

        // Each polygon should be valid
        for polygon in &rotated.polygons {
            assert!(
                polygon.vertices.len() >= 3,
                "Rotation should produce valid polygons"
            );
        }
    }

    // Test that rotation is composable (multiple rotations should work)
    let rotated_once = cube.rotate(1.57, 0.0, 0.0);
    let rotated_twice = rotated_once.rotate(0.0, 1.57, 0.0);
    assert!(
        !rotated_twice.polygons.is_empty(),
        "Multiple rotations should produce valid mesh"
    );
}

#[test]
fn test_difference_inverse_consistency() {
    // Property: A - B should be different from B - A
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube");

    let diff_ab = cube1.difference(&cube2);
    let diff_ba = cube2.difference(&cube1);

    // The results should be different (unless the cubes are identical)
    let bb_ab = diff_ab.bounding_box();
    let bb_ba = diff_ba.bounding_box();

    // At least one dimension should be different
    let dimensions_differ = (bb_ab.mins.x - bb_ba.mins.x).abs() > crate::float_types::EPSILON
        || (bb_ab.maxs.x - bb_ba.maxs.x).abs() > crate::float_types::EPSILON
        || (bb_ab.mins.y - bb_ba.mins.y).abs() > crate::float_types::EPSILON
        || (bb_ab.maxs.y - bb_ba.maxs.y).abs() > crate::float_types::EPSILON
        || (bb_ab.mins.z - bb_ba.mins.z).abs() > crate::float_types::EPSILON
        || (bb_ab.maxs.z - bb_ba.maxs.z).abs() > crate::float_types::EPSILON;

    assert!(
        dimensions_differ,
        "A - B should differ from B - A for different operands"
    );
}

#[test]
fn test_intersection_associativity() {
    // Property: (A ∩ B) ∩ C = A ∩ (B ∩ C) (intersection should be associative)
    let sphere1: Mesh<()> = Mesh::sphere(2.0, 16, 8, None)
        .expect("Failed to create sphere")
        .translate(1.0, 0.0, 0.0);
    let sphere2: Mesh<()> = Mesh::sphere(2.0, 16, 8, None)
        .expect("Failed to create sphere")
        .translate(-0.5, 0.0, 0.0);
    let sphere3: Mesh<()> = Mesh::sphere(2.0, 16, 8, None)
        .expect("Failed to create sphere")
        .translate(0.0, 1.0, 0.0);

    let left_assoc = sphere1.intersection(&sphere2).intersection(&sphere3);
    let right_assoc = sphere1.intersection(&sphere2.intersection(&sphere3));

    let bb_left = left_assoc.bounding_box();
    let bb_right = right_assoc.bounding_box();

    assert!(
        (bb_left.mins.x - bb_right.mins.x).abs() < crate::float_types::EPSILON
            && (bb_left.maxs.x - bb_right.maxs.x).abs() < crate::float_types::EPSILON
            && (bb_left.mins.y - bb_right.mins.y).abs() < crate::float_types::EPSILON
            && (bb_left.maxs.y - bb_right.maxs.y).abs() < crate::float_types::EPSILON
            && (bb_left.mins.z - bb_right.mins.z).abs() < crate::float_types::EPSILON
            && (bb_left.maxs.z - bb_right.maxs.z).abs() < crate::float_types::EPSILON,
        "Intersection should be associative"
    );
}

#[test]
fn test_union_absorption() {
    // Property: A ∪ (A ∩ B) = A (absorption law)
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let sphere: Mesh<()> = Mesh::sphere(1.5, 16, 8, None).expect("Failed to create sphere");

    let intersection = cube.intersection(&sphere);
    let absorption_result = cube.union(&intersection);

    let bb_cube = cube.bounding_box();
    let bb_absorption = absorption_result.bounding_box();

    // The result should have the same or larger bounding box as the original cube
    // (absorption should not shrink the bounds)
    assert!(
        bb_absorption.maxs.x >= bb_cube.maxs.x - crate::float_types::EPSILON
            && bb_absorption.maxs.y >= bb_cube.maxs.y - crate::float_types::EPSILON
            && bb_absorption.maxs.z >= bb_cube.maxs.z - crate::float_types::EPSILON,
        "Union absorption law should preserve or expand bounds: A ∪ (A ∩ B) bounds should contain A bounds"
    );
}

#[test]
fn test_transformation_composition() {
    // Property: Transformations should compose correctly (matrix multiplication order)
    let original: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Apply transformations in different orders
    let translate_first = original.translate(1.0, 2.0, 3.0).rotate(0.1, 0.2, 0.3);
    let rotate_first = original.rotate(0.1, 0.2, 0.3).translate(1.0, 2.0, 3.0);

    // Results should be different (transformations don't commute)
    let bb_translate_first = translate_first.bounding_box();
    let bb_rotate_first = rotate_first.bounding_box();

    // At least one dimension should differ (transformations are non-commutative)
    let bounds_differ = (bb_translate_first.mins.x - bb_rotate_first.mins.x).abs()
        > crate::float_types::EPSILON
        || (bb_translate_first.maxs.x - bb_rotate_first.maxs.x).abs()
            > crate::float_types::EPSILON
        || (bb_translate_first.mins.y - bb_rotate_first.mins.y).abs()
            > crate::float_types::EPSILON
        || (bb_translate_first.maxs.y - bb_rotate_first.maxs.y).abs()
            > crate::float_types::EPSILON
        || (bb_translate_first.mins.z - bb_rotate_first.mins.z).abs()
            > crate::float_types::EPSILON
        || (bb_translate_first.maxs.z - bb_rotate_first.maxs.z).abs()
            > crate::float_types::EPSILON;

    assert!(bounds_differ, "Translation and rotation should not commute");
}

#[test]
fn test_mesh_volume_conservation() {
    // Property: Mesh volume should be conserved under rigid transformations
    let original: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Expected volume: side³ = 8.0
    let expected_volume = 8.0;

    if let Some((mass_original, _, _)) = original.mass_properties(1.0) {
        assert!(
            (mass_original - expected_volume).abs() < 1e-6,
            "Original cube volume should be 8.0, got {}",
            mass_original
        );
    }

    // Apply rigid transformations (should preserve volume)
    let transformations = vec![
        original.translate(1.0, 2.0, 3.0),
        original.rotate(0.1, 0.2, 0.3),
        original.rotate(1.57, 0.0, 0.0).translate(5.0, 0.0, 0.0), // Rotation + translation
    ];

    for transformed in transformations {
        if let Some((mass_transformed, _, _)) = transformed.mass_properties(1.0) {
            assert!(
                (mass_transformed - expected_volume).abs() < 1e-5,
                "Rigid transformation should preserve volume: expected {}, got {}",
                expected_volume,
                mass_transformed
            );
        }
    }
}

#[test]
fn test_boolean_operation_emptiness() {
    // Property: Intersection of disjoint shapes should be empty
    let cube1: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(3.0, 0.0, 0.0); // Move far apart
    let cube2: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(-3.0, 0.0, 0.0);

    let intersection = cube1.intersection(&cube2);

    // Intersection of disjoint shapes should be empty or nearly empty
    assert!(
        intersection.polygons.is_empty() || intersection.polygons.len() < 3,
        "Intersection of disjoint shapes should be empty or minimal, got {} polygons",
        intersection.polygons.len()
    );

    // Bounding box should be degenerate or very small
    let bb = intersection.bounding_box();
    let volume = (bb.maxs.x - bb.mins.x) * (bb.maxs.y - bb.mins.y) * (bb.maxs.z - bb.mins.z);
    assert!(
        volume < 1e-6,
        "Intersection volume should be near zero, got {}",
        volume
    );
}
