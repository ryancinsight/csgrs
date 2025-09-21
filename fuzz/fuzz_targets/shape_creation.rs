#![no_main]

use libfuzzer_sys::fuzz_target;
use csgrs::mesh::Mesh;

/// Enhanced fuzz test for geometric shape creation with comprehensive edge case testing
/// Tests various shape constructors with fuzzed parameters including extreme values
fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }

    // Parse fuzzed data into shape parameters
    let size = f32::from_le_bytes(data[0..4].try_into().unwrap_or([0; 4])).abs() % 20.0;
    let height = f32::from_le_bytes(data[4..8].try_into().unwrap_or([0; 4])).abs() % 20.0;
    let radius = f32::from_le_bytes(data[8..12].try_into().unwrap_or([0; 4])).abs() % 10.0;
    let segments = (data[12] as u32 % 32) + 3; // 3-35 segments
    let shape_type = data[13] % 6; // 0=cube, 1=sphere, 2=cylinder, 3=torus, 4=cone, 5=pyramid

    // Skip invalid parameters
    if size < 0.1 || height < 0.1 || radius < 0.1 || segments < 3 {
        return;
    }

    // Test different shape creation functions
    let _mesh = match shape_type {
        0 => Mesh::cube(size, None::<()>),
        1 => Mesh::sphere(radius, segments as usize, segments as usize / 2, None::<()>),
        2 => Mesh::cylinder(radius, height, segments as usize, None::<()>),
        3 => {
            // Torus with fuzzed tube radius
            let tube_radius = f32::from_le_bytes(data[14..18].try_into().unwrap_or([0; 4])).abs() % radius;
            if tube_radius > 0.1 {
                Mesh::torus(radius, tube_radius, segments as usize, segments as usize, None::<()>)
            } else {
                return;
            }
        },
        4 => Mesh::cone(radius, height, segments as usize, None::<()>),
        5 => Mesh::pyramid(size, height, None::<()>),
        _ => return,
    };

    // Validate that mesh was created successfully
    if let Ok(mesh) = _mesh {
        // Test basic mesh properties
        let _vertex_count = mesh.vertices().len();
        let _face_count = mesh.faces().len();
        let _bbox = mesh.bounding_box();

        // Mesh should have valid geometry
        assert!(_vertex_count > 0);
        assert!(_face_count > 0);

        // Test mesh validity
        assert!(mesh.is_valid(), "Mesh should be valid after creation");

        // Test that we can perform operations on the mesh
        let _translated = mesh.translate(1.0, 1.0, 1.0);
        let _bbox2 = _translated.bounding_box();

        // Test serialization (basic smoke test)
        let _stl = mesh.to_stl_ascii("test");
        let _obj = mesh.to_obj("test");

        // Verify that serialized data is non-empty
        assert!(!_stl.is_empty());
        assert!(!_obj.is_empty());
    }

    // Result is implicitly validated by not panicking
});
