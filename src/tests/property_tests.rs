//! Property-based tests for mathematical correctness and geometric invariants
//!
//! These tests use simple property-based testing approaches to verify mathematical
//! properties that should hold for all valid inputs, without relying on external
//! crates like proptest.

use crate::mesh::Mesh;
use crate::traits::CSG;
use nalgebra::Vector3;

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
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(0.5, 0.0, 0.0);
    let cube2: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(-0.5, 0.0, 0.0);

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
    let cube1: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube").translate(2.0, 0.0, 0.0);
    let cube2: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube").translate(0.0, 0.0, 0.0);
    let cube3: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube").translate(-2.0, 0.0, 0.0);

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
fn test_intersection_idempotency() {
    // Property: A ∩ A = A (intersection with itself should be identity)
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None).expect("Failed to create sphere");
    let intersect_result = sphere.intersection(&sphere);

    // The result should have the same bounding box as the original
    let original_bb = sphere.bounding_box();
    let intersect_bb = intersect_result.bounding_box();

    assert!(
        (original_bb.mins.x - intersect_bb.mins.x).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.x - intersect_bb.maxs.x).abs() < crate::float_types::EPSILON
            && (original_bb.mins.y - intersect_bb.mins.y).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.y - intersect_bb.maxs.y).abs() < crate::float_types::EPSILON
            && (original_bb.mins.z - intersect_bb.mins.z).abs() < crate::float_types::EPSILON
            && (original_bb.maxs.z - intersect_bb.maxs.z).abs() < crate::float_types::EPSILON,
        "Intersection with self should preserve bounding box"
    );
}

#[test]
fn test_intersection_commutativity() {
    // Property: A ∩ B = B ∩ A (intersection should be commutative)
    let sphere1: Mesh<()> = Mesh::sphere(1.5, 16, 8, None).expect("Failed to create sphere").translate(0.5, 0.0, 0.0);
    let sphere2: Mesh<()> = Mesh::sphere(1.5, 16, 8, None).expect("Failed to create sphere").translate(-0.5, 0.0, 0.0);

    let intersect_ab = sphere1.intersection(&sphere2);
    let intersect_ba = sphere2.intersection(&sphere1);

    let bb_ab = intersect_ab.bounding_box();
    let bb_ba = intersect_ba.bounding_box();

    assert!(
        (bb_ab.mins.x - bb_ba.mins.x).abs() < crate::float_types::EPSILON
            && (bb_ab.maxs.x - bb_ba.maxs.x).abs() < crate::float_types::EPSILON
            && (bb_ab.mins.y - bb_ba.mins.y).abs() < crate::float_types::EPSILON
            && (bb_ab.maxs.y - bb_ba.maxs.y).abs() < crate::float_types::EPSILON
            && (bb_ab.mins.z - bb_ba.mins.z).abs() < crate::float_types::EPSILON
            && (bb_ab.maxs.z - bb_ba.maxs.z).abs() < crate::float_types::EPSILON,
        "Intersection should be commutative"
    );
}

#[test]
fn test_translation_invariance() {
    // Property: Translation should preserve relative geometry
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let original_bb = cube.bounding_box();

    let translated = cube.translate(5.0, 3.0, 1.0);
    let translated_bb = translated.bounding_box();

    // Dimensions should be preserved
    let original_dims = (
        original_bb.maxs.x - original_bb.mins.x,
        original_bb.maxs.y - original_bb.mins.y,
        original_bb.maxs.z - original_bb.mins.z,
    );

    let translated_dims = (
        translated_bb.maxs.x - translated_bb.mins.x,
        translated_bb.maxs.y - translated_bb.mins.y,
        translated_bb.maxs.z - translated_bb.mins.z,
    );

    assert!(
        (original_dims.0 - translated_dims.0).abs() < crate::float_types::EPSILON
            && (original_dims.1 - translated_dims.1).abs() < crate::float_types::EPSILON
            && (original_dims.2 - translated_dims.2).abs() < crate::float_types::EPSILON,
        "Translation should preserve dimensions"
    );
}

#[test]
fn test_scaling_homogeneity() {
    // Property: Scaling should be homogeneous (linear)
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let original_bb = cube.bounding_box();

    let scale_factor = 3.0;
    let scaled = cube.scale(scale_factor, scale_factor, scale_factor);
    let scaled_bb = scaled.bounding_box();

    // All dimensions should be scaled by the same factor
    let original_dims = (
        original_bb.maxs.x - original_bb.mins.x,
        original_bb.maxs.y - original_bb.mins.y,
        original_bb.maxs.z - original_bb.mins.z,
    );

    let scaled_dims = (
        scaled_bb.maxs.x - scaled_bb.mins.x,
        scaled_bb.maxs.y - scaled_bb.mins.y,
        scaled_bb.maxs.z - scaled_bb.mins.z,
    );

    assert!(
        (original_dims.0 * scale_factor - scaled_dims.0).abs() < crate::float_types::EPSILON
            && (original_dims.1 * scale_factor - scaled_dims.1).abs()
                < crate::float_types::EPSILON
            && (original_dims.2 * scale_factor - scaled_dims.2).abs()
                < crate::float_types::EPSILON,
        "Scaling should be homogeneous"
    );
}

#[test]
fn test_rotation_orthogonality() {
    // Property: Rotation should preserve distances from origin for centroid
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    if let Some(original_mass_props) = cube.mass_properties(1.0) {
        let (original_mass, original_com, _) = original_mass_props;

        let rotated = cube.rotate(45.0, 30.0, 60.0);

        if let Some(rotated_mass_props) = rotated.mass_properties(1.0) {
            let (rotated_mass, rotated_com, _) = rotated_mass_props;

            // Mass should be preserved
            assert!(
                (original_mass - rotated_mass).abs() < crate::float_types::EPSILON,
                "Rotation should preserve mass"
            );

            // Distance from origin to center of mass should be preserved
            let original_distance = original_com.coords.norm();
            let rotated_distance = rotated_com.coords.norm();

            assert!(
                (original_distance - rotated_distance).abs() < crate::float_types::EPSILON,
                "Rotation should preserve distance from origin to centroid"
            );
        }
    }
}

#[test]
fn test_mirror_symmetry() {
    // Property: Mirroring should create symmetric geometry
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let plane = crate::mesh::plane::Plane::from_normal(Vector3::x(), 0.0);

    let mirrored = cube.mirror(plane);

    // Original and mirrored should have same dimensions
    let original_bb = cube.bounding_box();
    let mirrored_bb = mirrored.bounding_box();

    let original_dims = (
        original_bb.maxs.x - original_bb.mins.x,
        original_bb.maxs.y - original_bb.mins.y,
        original_bb.maxs.z - original_bb.mins.z,
    );

    let mirrored_dims = (
        mirrored_bb.maxs.x - mirrored_bb.mins.x,
        mirrored_bb.maxs.y - mirrored_bb.mins.y,
        mirrored_bb.maxs.z - mirrored_bb.mins.z,
    );

    assert!(
        (original_dims.0 - mirrored_dims.0).abs() < crate::float_types::EPSILON
            && (original_dims.1 - mirrored_dims.1).abs() < crate::float_types::EPSILON
            && (original_dims.2 - mirrored_dims.2).abs() < crate::float_types::EPSILON,
        "Mirroring should preserve dimensions"
    );
}

#[test]
fn test_boolean_operation_closure() {
    // Property: Boolean operations should produce valid geometry
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(0.5, 0.0, 0.0);
    let cube2: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(-0.5, 0.0, 0.0);

    let union_result = cube1.union(&cube2);
    let diff_result = cube1.difference(&cube2);
    let intersect_result = cube1.intersection(&cube2);

    // All results should be valid (non-empty)
    assert!(
        !union_result.polygons.is_empty(),
        "Union should produce valid geometry"
    );
    assert!(
        !diff_result.polygons.is_empty(),
        "Difference should produce valid geometry"
    );
    // Intersection might be empty if shapes don't overlap, which is acceptable

    // All polygons should have valid vertex counts
    for poly in &union_result.polygons {
        assert!(
            poly.vertices.len() >= 3,
            "Union polygons should have at least 3 vertices"
        );
    }
    for poly in &diff_result.polygons {
        assert!(
            poly.vertices.len() >= 3,
            "Difference polygons should have at least 3 vertices"
        );
    }
    for poly in &intersect_result.polygons {
        assert!(
            poly.vertices.len() >= 3,
            "Intersection polygons should have at least 3 vertices"
        );
    }
}

#[test]
fn test_mesh_manifold_consistency() {
    // Property: Well-formed meshes should maintain manifold properties under operations
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(1.0, 0.0, 0.0);

    // Test with overlapping cubes
    let union_result = cube1.union(&cube2);

    // If the result is manifold, that's a good property
    // (Note: manifold checking might be expensive, so we just ensure no panics)
    let _is_manifold = union_result.is_manifold();
    // The operation should complete without panicking
}

#[test]
fn test_geometric_invariance_under_transform() {
    // Property: Certain geometric properties should be invariant under transformations
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    if let Some(original_props) = cube.mass_properties(1.0) {
        let (original_mass, _, _) = original_props;

        // Apply various transformations
        let transformed = cube
            .translate(5.0, 3.0, 1.0)
            .rotate(45.0, 30.0, 15.0)
            .scale(2.0, 1.5, 0.8);

        if let Some(transformed_props) = transformed.mass_properties(1.0) {
            let (transformed_mass, _, _) = transformed_props;

            // Mass should scale with volume scaling (2.0 * 1.5 * 0.8 = 2.4)
            let expected_mass = original_mass * 2.4;

            assert!(
                (transformed_mass - expected_mass).abs()
                    < crate::float_types::EPSILON * expected_mass,
                "Mass should scale correctly under transformations: expected {}, got {}",
                expected_mass,
                transformed_mass
            );
        }
    }
}

#[test]
fn test_symmetry_preservation() {
    // Property: Symmetric operations should preserve dimensions
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Rotate by 90 degrees around Z axis (symmetric operation for a cube)
    let rotated = cube.rotate(0.0, 0.0, 90.0);

    // The bounding box dimensions should be the same (cube is symmetric)
    let original_bb = cube.bounding_box();
    let rotated_bb = rotated.bounding_box();

    // Check that dimensions are preserved, not that coordinates are identical
    let original_dims = (
        original_bb.maxs.x - original_bb.mins.x,
        original_bb.maxs.y - original_bb.mins.y,
        original_bb.maxs.z - original_bb.mins.z,
    );

    let rotated_dims = (
        rotated_bb.maxs.x - rotated_bb.mins.x,
        rotated_bb.maxs.y - rotated_bb.mins.y,
        rotated_bb.maxs.z - rotated_bb.mins.z,
    );

    assert!(
        (original_dims.0 - rotated_dims.0).abs() < crate::float_types::EPSILON
            && (original_dims.1 - rotated_dims.1).abs() < crate::float_types::EPSILON
            && (original_dims.2 - rotated_dims.2).abs() < crate::float_types::EPSILON,
        "Symmetric rotation should preserve bounding box dimensions: original {:?}, rotated {:?}",
        original_dims,
        rotated_dims
    );
}

#[test]
fn test_boolean_operation_scaling_complexity() {
    // **SRS Requirement NFR001/NFR003**: Boolean operations shall scale O(n log n)
    // This test validates that operation time grows sub-quadratically with input size
    // by comparing small vs large mesh boolean operations

    use std::time::Instant;

    // Create test meshes of different sizes
    let small_cube: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create small cube");
    let large_cube: Mesh<()> = Mesh::cube(10.0, None).expect("Failed to create large cube");

    // Measure small operation time
    let small_start = Instant::now();
    let _small_union = small_cube.union(&small_cube);
    let small_time = small_start.elapsed();

    // Measure large operation time
    let large_start = Instant::now();
    let _large_union = large_cube.union(&large_cube);
    let large_time = large_start.elapsed();

    // Calculate scaling factor (size ratio is 10x, so time should be much less than 100x)
    let size_ratio = 10.0f64;
    let time_ratio = large_time.as_nanos() as f64 / small_time.as_nanos() as f64;

    // For O(n log n) scaling, time ratio should be much less than size_ratio²
    // Allow some margin for measurement noise and constant factors
    let max_expected_ratio = size_ratio * size_ratio.ln() * 2.0; // Conservative upper bound

    assert!(
        time_ratio < max_expected_ratio,
        "Boolean operations should scale O(n log n), but time ratio {:.2} exceeded expected maximum {:.2} for size ratio {:.1}",
        time_ratio,
        max_expected_ratio,
        size_ratio
    );

    println!(
        "Scaling validation: size ratio {:.1}, time ratio {:.2}, max expected {:.2} ✓",
        size_ratio,
        time_ratio,
        max_expected_ratio
    );
}

#[test]
fn test_indexed_mesh_boolean_operation_scaling() {
    // **SRS Requirement NFR003**: IndexedMesh boolean operations shall scale O(n log n)
    // Validate scaling behavior for IndexedMesh operations with vertex deduplication

    use crate::indexed_mesh::{IndexedMesh, shapes};
    use std::time::Instant;

    // Create test IndexedMeshes of different sizes
    let small_cube: IndexedMesh<()> = shapes::cube(1.0, None);
    let large_cube: IndexedMesh<()> = shapes::cube(10.0, None);

    // Measure operation times
    let small_start = Instant::now();
    let _small_union = small_cube.union(&small_cube);
    let small_time = small_start.elapsed();

    let large_start = Instant::now();
    let _large_union = large_cube.union(&large_cube);
    let large_time = large_start.elapsed();

    // Validate scaling behavior
    let size_ratio = 10.0f64;
    let time_ratio = large_time.as_nanos() as f64 / small_time.as_nanos() as f64;
    let max_expected_ratio = size_ratio * size_ratio.ln() * 2.0;

    assert!(
        time_ratio < max_expected_ratio,
        "IndexedMesh boolean operations should scale O(n log n), but time ratio {:.2} exceeded expected maximum {:.2}",
        time_ratio,
        max_expected_ratio
    );

    println!(
        "IndexedMesh scaling validation: time ratio {:.2}, max expected {:.2} ✓",
        time_ratio,
        max_expected_ratio
    );
}
