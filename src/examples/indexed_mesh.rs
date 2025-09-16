//! IndexedMesh examples demonstrating memory-efficient mesh operations and STL export
//!
//! This module showcases IndexedMesh functionality including:
//! - Automatic vertex deduplication
//! - Memory-efficient boolean operations
//! - Connectivity analysis and adjacency queries
//! - Optimized STL export with statistics

use crate::indexed_mesh::{IndexedMesh, shapes};
use crate::traits::CSG;
use std::fs;

/// Run comprehensive IndexedMesh demonstrations
pub fn run_indexed_mesh_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== IndexedMesh Comprehensive Demo ===\n");

    run_basic_shapes_demo()?;
    run_boolean_operations_demo()?;
    run_connectivity_demo()?;
    run_memory_optimization_demo()?;

    println!("\n=== IndexedMesh Demo Completed! ===");
    println!("STL files exported to 'stl/indexed_mesh/' directory");
    println!("Check the console output for memory optimization statistics");

    Ok(())
}

/// Demonstrate basic IndexedMesh shapes with STL export
pub fn run_basic_shapes_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Basic IndexedMesh Shapes ---\n");

    // Create basic shapes
    let cube: IndexedMesh<()> = shapes::cube(2.0, None);
    let sphere: IndexedMesh<()> = shapes::sphere(1.25, 16, 8, None);
    let cylinder: IndexedMesh<()> = shapes::cylinder(0.8, 3.0, 12, None);

    // Export each shape with statistics
    export_with_stats(&cube, "indexed_cube", "Basic cube shape")?;
    export_with_stats(&sphere, "indexed_sphere", "Sphere with vertex deduplication")?;
    export_with_stats(
        &cylinder,
        "indexed_cylinder",
        "Cylinder demonstrating optimization",
    )?;

    println!("Basic shapes exported successfully!\n");

    Ok(())
}

/// Demonstrate IndexedMesh boolean operations with STL export
pub fn run_boolean_operations_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- IndexedMesh Boolean Operations ---\n");

    // Create base shapes
    let cube: IndexedMesh<()> = shapes::cube(2.0, None);
    let sphere: IndexedMesh<()> = shapes::sphere(1.25, 16, 8, None);
    let cylinder: IndexedMesh<()> = shapes::cylinder(0.8, 3.0, 12, None);

    // Perform boolean operations
    let union_result = cube.union(&sphere);
    let difference_result = cube.difference(&sphere);
    let intersection_result = cube.intersection(&cylinder);
    let xor_result = sphere.xor(&cylinder);

    // Export results with statistics
    export_with_stats(&union_result, "indexed_union", "Cube union sphere")?;
    export_with_stats(&difference_result, "indexed_difference", "Cube minus sphere")?;
    export_with_stats(
        &intersection_result,
        "indexed_intersection",
        "Cube intersection cylinder",
    )?;
    export_with_stats(&xor_result, "indexed_xor", "Sphere XOR cylinder")?;

    println!("Boolean operations completed successfully!\n");

    Ok(())
}

/// Demonstrate IndexedMesh connectivity analysis
pub fn run_connectivity_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- IndexedMesh Connectivity Analysis ---\n");

    // Create a complex shape for connectivity analysis
    let cube: IndexedMesh<()> = shapes::cube(2.0, None);
    let sphere: IndexedMesh<()> = shapes::sphere(1.0, 12, 6, None);
    let complex_shape = cube.difference(&sphere);

    // Analyze connectivity
    println!("Complex shape analysis:");
    println!("  Vertices: {}", complex_shape.vertices.len());
    println!("  Faces: {}", complex_shape.faces.len());
    println!("  Is manifold: {}", complex_shape.is_manifold());

    // Check adjacency information
    if let Some(adjacent_faces) = complex_shape.get_face_adjacency(0) {
        println!("  Face 0 is adjacent to {} faces", adjacent_faces.len());
    }

    if let Some(adjacent_vertices) = complex_shape.get_vertex_adjacency(0) {
        println!(
            "  Vertex 0 is adjacent to {} vertices",
            adjacent_vertices.len()
        );
    }

    if let Some(vertex_faces) = complex_shape.get_vertex_faces(0) {
        println!("  Vertex 0 belongs to {} faces", vertex_faces.len());
    }

    // Export the complex shape
    export_with_stats(
        &complex_shape,
        "indexed_complex",
        "Complex shape for connectivity analysis",
    )?;

    println!("Connectivity analysis completed!\n");

    Ok(())
}

/// Demonstrate IndexedMesh memory optimization benefits
pub fn run_memory_optimization_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- IndexedMesh Memory Optimization ---\n");

    // Compare regular mesh vs indexed mesh for a complex boolean operation
    let cube1: IndexedMesh<()> = shapes::cube(2.0, None);
    let cube2: IndexedMesh<()> = shapes::cube(2.0, None).translate(1.0, 1.0, 1.0);

    // Create a complex operation
    let complex_operation = cube1.union(&cube2);

    // Show statistics
    let vertex_count = complex_operation.vertices.len();
    let face_count = complex_operation.faces.len();

    println!("Complex boolean operation results:");
    println!("  Vertices: {}", vertex_count);
    println!("  Faces: {}", face_count);
    println!("  Memory efficient: IndexedMesh automatically deduplicates vertices");

    // Export with detailed statistics
    export_with_stats(
        &complex_operation,
        "indexed_optimization",
        "Memory-optimized complex operation",
    )?;

    println!("Memory optimization demonstration completed!\n");

    Ok(())
}

/// Helper function to export IndexedMesh with statistics
fn export_with_stats(
    mesh: &IndexedMesh<()>,
    filename: &str,
    description: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    // Create output directory
    fs::create_dir_all("stl/indexed_mesh")?;

    // Export with statistics
    let (stl_content, stats) = mesh.to_stl_ascii_with_stats(filename);

    // Save STL file
    let filepath = format!("stl/indexed_mesh/{}.stl", filename);
    fs::write(&filepath, stl_content)?;

    // Display statistics
    println!("{} ({})", description, filename);
    println!("  File: {}", filepath);
    println!("  Original vertices: {}", stats.original_vertices);
    println!("  Deduplicated vertices: {}", stats.deduplicated_vertices);
    println!("  Faces: {}", stats.face_count);
    println!("  Memory savings: {:.1}%", stats.memory_savings * 100.0);
    println!("  Export successful: {}", stats.success);
    println!();

    Ok(())
}
