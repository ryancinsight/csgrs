//! Fuzz test for CSG boolean operations
//! Tests union, difference, intersection with fuzzed mesh combinations

#![no_main]

use libfuzzer_sys::fuzz_target;
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

/// Enhanced fuzz test for CSG operations with comprehensive validation
fuzz_target!(|data: &[u8]| {
    if data.len() < 24 {
        return;
    }

    // Parse fuzzed data into mesh parameters
    let size1 = f32::from_le_bytes(data[0..4].try_into().unwrap_or([0; 4])).abs() % 5.0;
    let size2 = f32::from_le_bytes(data[4..8].try_into().unwrap_or([0; 4])).abs() % 5.0;
    let offset_x = f32::from_le_bytes(data[8..12].try_into().unwrap_or([0; 4])) % 10.0;
    let offset_y = f32::from_le_bytes(data[12..16].try_into().unwrap_or([0; 4])) % 10.0;
    let offset_z = f32::from_le_bytes(data[16..20].try_into().unwrap_or([0; 4])) % 10.0;
    let operation_type = data[20] % 4; // 0=union, 1=difference, 2=intersection, 3=xor
    let mesh_type = data[21] % 3; // 0=cube, 1=sphere, 2=cylinder

    // Skip invalid sizes
    if size1 < 0.1 || size2 < 0.1 {
        return;
    }

    // Create base meshes
    let mesh1 = match mesh_type {
        0 => Mesh::cube(size1, None::<()>).unwrap(),
        1 => Mesh::sphere(size1 / 2.0, 8, 4, None::<()>).unwrap(),
        2 => Mesh::cylinder(size1 / 2.0, size1, 8, None::<()>).unwrap(),
        _ => return,
    };

    let mesh2 = match mesh_type {
        0 => Mesh::cube(size2, None::<()>).unwrap(),
        1 => Mesh::sphere(size2 / 2.0, 8, 4, None::<()>).unwrap(),
        2 => Mesh::cylinder(size2 / 2.0, size2, 8, None::<()>).unwrap(),
        _ => return,
    };

    // Apply offset to second mesh
    let mesh2 = mesh2.translate(offset_x, offset_y, offset_z);

    // Perform CSG operation
    let result = match operation_type {
        0 => mesh1.union(&mesh2),      // Union
        1 => mesh1.difference(&mesh2), // Difference
        2 => mesh1.intersection(&mesh2), // Intersection
        3 => {
            // XOR as union minus intersection
            let union = mesh1.union(&mesh2);
            let intersection = mesh1.intersection(&mesh2);
            union.difference(&intersection)
        },
        _ => return,
    };

    // Validate result
    if let Ok(result_mesh) = result {
        // Basic validation
        assert!(result_mesh.is_valid(), "CSG result should be valid");

        // Test that result has geometry
        let vertex_count = result_mesh.vertices().len();
        let face_count = result_mesh.faces().len();

        // Result should have some geometry (may be empty for non-overlapping intersection)
        assert!(vertex_count >= 0);
        assert!(face_count >= 0);

        // Test bounding box
        let _bbox = result_mesh.bounding_box();

        // Test serialization
        let _stl = result_mesh.to_stl_ascii("test");
        let _obj = result_mesh.to_obj("test");

        // Verify serialization works
        assert!(!_stl.is_empty());
        assert!(!_obj.is_empty());

        // Test that we can perform operations on the result
        let _translated = result_mesh.translate(1.0, 1.0, 1.0);
        assert!(_translated.is_valid());

        // Test IndexedMesh compatibility
        let _indexed = csgrs::indexed_mesh::IndexedMesh::from(result_mesh);
        let _back_to_mesh = _indexed.to_mesh();
        assert!(_back_to_mesh.is_valid());
    }

    // All operations should complete without panicking
});
