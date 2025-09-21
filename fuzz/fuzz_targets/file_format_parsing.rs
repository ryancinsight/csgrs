//! Fuzz test for file format parsing
//! Tests STL, OBJ, and other format parsers with malformed and edge-case data

#![no_main]

use libfuzzer_sys::fuzz_target;

/// Fuzz test for file format parsing with comprehensive edge case handling
fuzz_target!(|data: &[u8]| {
    // Skip empty data
    if data.is_empty() {
        return;
    }

    // Test STL ASCII parsing
    let stl_content = String::from_utf8_lossy(data);
    if stl_content.contains("solid") || stl_content.contains("facet") {
        // This looks like STL ASCII format, try to parse it
        // We don't actually parse it here, just verify it doesn't crash
        let _ = csgrs::io::stl::Mesh::<()>::from_stl_ascii(data);
    }

    // Test STL binary parsing (if data looks like binary STL)
    if data.len() > 80 && data[0..5] == *b"solid" {
        // This might be binary STL, try to parse it
        let _ = csgrs::io::stl::Mesh::<()>::from_stl_binary(data);
    }

    // Test as generic byte data for robustness
    // The parsers should handle malformed data gracefully
    let _ = csgrs::io::stl::Mesh::<()>::from_stl_data(data);

    // Test OBJ parsing if data contains OBJ-like content
    let obj_content = String::from_utf8_lossy(data);
    if obj_content.contains("v ") || obj_content.contains("f ") {
        let _ = csgrs::io::obj::Mesh::<()>::from_obj_data(data);
    }

    // All parsing operations should complete without panicking
    // even with completely malformed data
});
