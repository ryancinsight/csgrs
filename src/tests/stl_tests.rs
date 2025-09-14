//! Tests for STL file format operations

use crate::mesh::Mesh;
use crate::traits::CSG;

// --------------------------------------------------------
//   CSG: STL Export
// --------------------------------------------------------

#[test]
#[cfg(feature = "stl-io")]
fn test_to_stl_ascii() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let stl_str = cube.to_stl_ascii("test_cube");
    // Basic checks
    assert!(stl_str.contains("solid test_cube"));
    assert!(stl_str.contains("endsolid test_cube"));
}

#[test]
#[cfg(feature = "stl-io")]
fn test_to_stl_binary() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let stl_bytes = cube.to_stl_binary("test_cube");
    assert!(stl_bytes.is_ok());
    let bytes = stl_bytes.expect("Binary STL export should succeed for valid mesh");
    assert!(!bytes.is_empty());
    // Binary STL should have at least 80 bytes header + 4 bytes triangle count
    assert!(bytes.len() >= 84);
}

#[test]
#[cfg(feature = "stl-io")]
fn test_csg_to_stl_and_from_stl_file() {
    let original_cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Export to binary STL
    let stl_data = original_cube
        .to_stl_binary("test_cube")
        .expect("Binary STL export should succeed");

    // Import from STL data
    let imported_cube: Mesh<()> =
        Mesh::from_stl(&stl_data, None).expect("STL import should succeed for valid data");

    // STL format stores triangles, so quads get triangulated during export
    // Cube: 6 quads → 12 triangles after roundtrip
    assert_eq!(original_cube.polygons.len(), 6); // Original has 6 quads
    assert_eq!(imported_cube.polygons.len(), 12); // Imported has 12 triangles

    // Should have same bounding box
    let orig_bb = original_cube.bounding_box();
    let import_bb = imported_cube.bounding_box();

    assert!((orig_bb.mins.x - import_bb.mins.x).abs() < crate::float_types::EPSILON);
    assert!((orig_bb.maxs.x - import_bb.maxs.x).abs() < crate::float_types::EPSILON);
    assert!((orig_bb.mins.y - import_bb.mins.y).abs() < crate::float_types::EPSILON);
    assert!((orig_bb.maxs.y - import_bb.maxs.y).abs() < crate::float_types::EPSILON);
    assert!((orig_bb.mins.z - import_bb.mins.z).abs() < crate::float_types::EPSILON);
    assert!((orig_bb.maxs.z - import_bb.maxs.z).abs() < crate::float_types::EPSILON);
}

#[test]
#[cfg(feature = "stl-io")]
fn test_empty_mesh_stl() {
    let empty_mesh: Mesh<()> = Mesh::new();
    let stl_str = empty_mesh.to_stl_ascii("empty");
    // Should still produce valid STL format
    assert!(stl_str.contains("solid empty"));
    assert!(stl_str.contains("endsolid empty"));
}
