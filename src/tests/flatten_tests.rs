//! Tests for mesh flattening and 2D operations

use crate::mesh::Mesh;
use crate::sketch::Sketch;
use crate::traits::CSG;

// --------------------------------------------------------
//   Flatten and Union Tests
// --------------------------------------------------------

#[test]
fn test_flatten_cube() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let flattened = cube.flatten();

    // Flattened result should be a 2D sketch
    assert!(!flattened.geometry.is_empty());
}

#[test]
fn test_flatten_and_union_single_polygon() {
    let square: Sketch<()> = Sketch::square(2.0, None);
    let extruded = square.extrude(1.0);
    let flattened = extruded.flatten();

    assert!(!flattened.geometry.is_empty());
}

#[test]
fn test_flatten_and_union_two_disjoint_squares() {
    let square1: Sketch<()> = Sketch::square(1.0, None);
    let square2 = Sketch::square(1.0, None).translate(3.0, 0.0, 0.0);

    let extruded1 = square1.extrude(1.0);
    let extruded2 = square2.extrude(1.0);

    let union = extruded1.union(&extruded2);
    let flattened = union.flatten();

    assert!(!flattened.geometry.is_empty());
}

#[test]
fn test_flatten_and_union_two_overlapping_squares() {
    let square1: Sketch<()> = Sketch::square(2.0, None);
    let square2 = Sketch::square(2.0, None).translate(1.0, 0.0, 0.0);

    let extruded1 = square1.extrude(1.0);
    let extruded2 = square2.extrude(1.0);

    let union = extruded1.union(&extruded2);
    let flattened = union.flatten();

    assert!(!flattened.geometry.is_empty());
}

#[test]
fn test_flatten_and_union_near_xy_plane() {
    let square: Sketch<()> = Sketch::square(2.0, None);
    let extruded = square.extrude(1.0).translate(0.0, 0.0, 0.5); // Slightly above XY plane

    let flattened = extruded.flatten();
    assert!(!flattened.geometry.is_empty());
}

#[test]
fn test_flatten_and_union_collinear_edges() {
    let rect1: Sketch<()> = Sketch::rectangle(4.0, 2.0, None);
    let rect2: Sketch<()> = Sketch::rectangle(2.0, 4.0, None).translate(1.0, -1.0, 0.0);

    let extruded1 = rect1.extrude(1.0);
    let extruded2 = rect2.extrude(1.0);

    let union = extruded1.union(&extruded2);
    let flattened = union.flatten();

    assert!(!flattened.geometry.is_empty());
}

#[test]
fn test_flatten_and_union_debug() {
    let square1: Sketch<()> = Sketch::square(2.0, None);
    let square2 = Sketch::square(1.0, None).translate(0.5, 0.5, 0.0);

    let extruded1 = square1.extrude(1.0);
    let extruded2 = square2.extrude(1.0);

    let union = extruded1.union(&extruded2);
    let flattened = union.flatten();

    // Should handle complex unions without issues
    assert!(!flattened.geometry.is_empty());
}
