#![no_main]

use libfuzzer_sys::fuzz_target;
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

/// Fuzz test for mesh transformation operations
/// Tests translation, rotation, scaling, and complex transformations
fuzz_target!(|data: &[u8]| {
    if data.len() < 32 {
        return;
    }

    // Parse fuzzed data into transformation parameters
    let size = f32::from_le_bytes(data[0..4].try_into().unwrap_or([0; 4])).abs() % 10.0;
    let tx = f32::from_le_bytes(data[4..8].try_into().unwrap_or([0; 4])) % 50.0;
    let ty = f32::from_le_bytes(data[8..12].try_into().unwrap_or([0; 4])) % 50.0;
    let tz = f32::from_le_bytes(data[12..16].try_into().unwrap_or([0; 4])) % 50.0;
    let rx = f32::from_le_bytes(data[16..20].try_into().unwrap_or([0; 4])) % (2.0 * std::f32::consts::PI);
    let ry = f32::from_le_bytes(data[20..24].try_into().unwrap_or([0; 4])) % (2.0 * std::f32::consts::PI);
    let rz = f32::from_le_bytes(data[24..28].try_into().unwrap_or([0; 4])) % (2.0 * std::f32::consts::PI);
    let sx = (f32::from_le_bytes(data[28..32].try_into().unwrap_or([0; 4])).abs() % 5.0) + 0.1;
    let sy = (f32::from_le_bytes(data[32..36].try_into().unwrap_or([0; 4])).abs() % 5.0) + 0.1;
    let sz = (f32::from_le_bytes(data[36..40].try_into().unwrap_or([0; 4])).abs() % 5.0) + 0.1;

    // Skip invalid sizes
    if size < 0.1 || size > 100.0 {
        return;
    }

    // Create base mesh
    let mesh = match Mesh::cube(size, None::<()>) {
        Ok(m) => m,
        Err(_) => return,
    };

    // Test individual transformations
    let _translated = mesh.translate(tx, ty, tz);
    let _rotated = mesh.rotate(rx, ry, rz);
    let _scaled = mesh.scale(sx, sy, sz);

    // Test complex transformation chain
    let _complex = mesh
        .translate(tx, ty, tz)
        .rotate(rx, ry, rz)
        .scale(sx, sy, sz);

    // Test bounding box after transformations
    let _bbox = _complex.bounding_box();

    // All operations should complete without panicking
});
