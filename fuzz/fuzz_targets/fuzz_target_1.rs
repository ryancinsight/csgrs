#![no_main]

#[macro_use] extern crate libfuzzer_sys;
extern crate csgrs;

use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

/// Fuzz test for CSG boolean operations
/// Tests union, difference, and intersection operations with various inputs
fuzz_target!(|data: &[u8]| {
    if data.len() < 24 {
        return;
    }

    // Parse fuzzed data into geometric parameters
    let size1 = f32::from_le_bytes(data[0..4].try_into().unwrap_or([0; 4])).abs() % 10.0;
    let size2 = f32::from_le_bytes(data[4..8].try_into().unwrap_or([0; 4])).abs() % 10.0;
    let tx = f32::from_le_bytes(data[8..12].try_into().unwrap_or([0; 4])) % 5.0;
    let ty = f32::from_le_bytes(data[12..16].try_into().unwrap_or([0; 4])) % 5.0;
    let tz = f32::from_le_bytes(data[16..20].try_into().unwrap_or([0; 4])) % 5.0;
    let operation = data[20] % 3; // 0=union, 1=difference, 2=intersection

    // Skip invalid sizes
    if size1 < 0.1 || size2 < 0.1 || size1 > 100.0 || size2 > 100.0 {
        return;
    }

    // Create meshes with fuzzed parameters
    let mesh1 = match Mesh::cube(size1, None::<()> ) {
        Ok(m) => m,
        Err(_) => return,
    };

    let mesh2 = match Mesh::cube(size2, None::<()> ) {
        Ok(m) => m.translate(tx, ty, tz),
        Err(_) => return,
    };

    // Perform fuzzed operation
    let _result = match operation {
        0 => mesh1.union(&mesh2),
        1 => mesh1.difference(&mesh2),
        2 => mesh1.intersection(&mesh2),
        _ => return,
    };

    // Result is implicitly validated by not panicking
});
